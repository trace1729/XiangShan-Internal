package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.{Redirect, RedirectLevel}
import utility.{CircularQueuePtr, HasCircularQueuePtrHelper, XSError, XSPerfAccumulate}

class TraceReaderIO(implicit p: Parameters) extends TraceBundle {
  val recv       = Flipped(Valid(new TraceRecvInfo()))
  val redirect   = Flipped(Valid(new Redirect()))
  val traceInsts = Output(Valid(Vec(trtl.TracePredictWidth, new TraceInstrBundle())))
  val pcMatch    = Flipped(new TracePCMatchBundle())
}

class TraceBufferPtr(size: Int) extends CircularQueuePtr[TraceBufferPtr](size)

class TraceReaderHelperWrapper(implicit p: Parameters) extends TraceModule {
  val io = IO(new Bundle {
    val enable          = Input(Bool())
    val instsReady      = Output(Bool())
    val insts           = Output(Vec(trtl.TraceFetchWidth, new TraceInstrInnerBundle()))
    val redirectValid   = Input(Bool())
    val redirectInstID  = Input(UInt(trtl.TraceInstIDWidth.W))
    val workingState    = Input(Bool())
  })

  val traceReaderHelper = Module(new TraceReaderHelper(trtl.TraceFetchWidth))
  traceReaderHelper.clock := clock
  traceReaderHelper.reset := reset
  traceReaderHelper.enable := io.enable && io.workingState

  val traceRedirectHelper = Module(new TraceRedirectHelper)
  traceRedirectHelper.clock := clock
  traceRedirectHelper.reset := reset
  traceRedirectHelper.enable := io.redirectValid && io.workingState
  traceRedirectHelper.InstID := io.redirectInstID

  io.insts.zipWithIndex.foreach { case (inst, i) =>
    inst.pcVA := traceReaderHelper.insts_pcVA(i)
    inst.pcPA := traceReaderHelper.insts_pcPA(i)
    inst.memoryAddrVA := traceReaderHelper.insts_memoryAddrVA(i)
    inst.memoryAddrPA := traceReaderHelper.insts_memoryAddrPA(i)
    inst.target := traceReaderHelper.insts_target(i)
    inst.inst := traceReaderHelper.insts_inst(i)
    inst.memoryType := traceReaderHelper.insts_memoryType(i)
    inst.memorySize := traceReaderHelper.insts_memorySize(i)
    inst.branchType := traceReaderHelper.insts_branchType(i)
    inst.branchTaken := traceReaderHelper.insts_branchTaken(i)
    inst.exception := traceReaderHelper.insts_exception(i)
    inst.fastSimulation := traceReaderHelper.insts_fastSimulation(i)
    inst.InstID := traceReaderHelper.insts_InstID(i)
  }
  io.instsReady := true.B
}

class TraceReader(implicit p: Parameters) extends TraceModule with HasCircularQueuePtrHelper {
  val io = IO(new TraceReaderIO())
  dontTouch(io)

  if (!env.TraceRTLMode) {
    io <> DontCare
  } else {
    val traceBuffer = RegInit(0.U.asTypeOf(Vec(trtl.TraceBufferSize, new TraceInstrBundle())))
    val traceReaderHelper = Module(new TraceReaderHelperWrapper())
    val enqPtr = RegInit(0.U.asTypeOf(new TraceBufferPtr(trtl.TraceBufferSize)))
    val deqPtr = RegInit(0.U.asTypeOf(new TraceBufferPtr(trtl.TraceBufferSize)))

    val workingState = RegInit(false.B)
    val startCount = RegInit(0.U(4.W))
    when (startCount < 10.U) {
      startCount := startCount + 1.U
    }
    when (startCount === 5.U) {
      workingState := true.B
    }

    val readTraceEnable = !isFull(enqPtr, deqPtr) &&
      hasFreeEntries(enqPtr, deqPtr) >= trtl.TraceFetchWidth.U &&
      workingState &&
      !io.redirect.valid
    val readTraceReady = traceReaderHelper.io.instsReady

    traceReaderHelper.io.enable := readTraceEnable
    traceReaderHelper.io.redirectValid := io.redirect.valid && workingState
    // Exceptions and synthetic force-jumps transfer control after the current
    // trace instruction even when the backend redirect flushes the ROB entry.
    val redirectFlushItself = RedirectLevel.flushItself(io.redirect.bits.level) &&
      !io.redirect.bits.traceInfo.isForceJump &&
      !io.redirect.bits.traceInfo.hasException
    traceReaderHelper.io.redirectInstID := Mux(
      redirectFlushItself,
      io.redirect.bits.traceInfo.InstID,
      io.redirect.bits.traceInfo.InstID + 1.U
    )(trtl.TraceInstIDWidth - 1, 0)
    traceReaderHelper.io.workingState := workingState

    val enqPtrVec = Wire(Vec(trtl.TraceFetchWidth, new TraceBufferPtr(trtl.TraceBufferSize)))
    enqPtrVec.zipWithIndex.foreach { case (ptr, i) =>
      ptr := enqPtr + i.U
    }

    when (readTraceEnable && readTraceReady) {
      (0 until trtl.TraceFetchWidth).foreach { i =>
        traceBuffer(enqPtrVec(i).value) := TraceInstrBundle(traceReaderHelper.io.insts(i))
      }
      enqPtr := enqPtr + trtl.TraceFetchWidth.U
    }

    when (io.recv.valid) {
      deqPtr := deqPtr + io.recv.bits.instNum
    }

    when (io.redirect.valid) {
      enqPtr := 0.U.asTypeOf(new TraceBufferPtr(trtl.TraceBufferSize))
      deqPtr := 0.U.asTypeOf(new TraceBufferPtr(trtl.TraceBufferSize))
      traceBuffer.foreach(_ := 0.U.asTypeOf(new TraceInstrBundle()))
    }

    // When S3 consumes the current packet, let S2 observe the following packet
    // in the same cycle.  The registered deqPtr advances at the clock edge, so
    // without this look-ahead TraceRTL has to insert a bubble between packets.
    val visibleDeqPtr = deqPtr + Mux(io.recv.valid, io.recv.bits.instNum, 0.U)
    io.traceInsts.bits.zipWithIndex.foreach { case (inst, i) =>
      inst := traceBuffer((visibleDeqPtr + i.U).value)
    }
    io.traceInsts.valid := distanceBetween(enqPtr, visibleDeqPtr) >= trtl.TracePredictWidth.U

    io.pcMatch.found := Cat(traceBuffer.map(_.pcVA === io.pcMatch.pcVA)).orR

    XSPerfAccumulate("TraceReaderValid", io.traceInsts.valid)
    XSPerfAccumulate("TraceReaderNotValid", !io.traceInsts.valid)

    XSError(!isFull(enqPtr, deqPtr) && enqPtr < deqPtr, "TraceReader enqPtr should not be before deqPtr")
    XSError(io.recv.valid && io.recv.bits.instNum > distanceBetween(enqPtr, deqPtr),
      "TraceReader should not read more instructions than buffered")

    for (i <- 0 until trtl.TracePredictWidth - 1) {
      XSError(io.traceInsts.valid && io.traceInsts.bits(i + 1).InstID =/= io.traceInsts.bits(i).InstID + 1.U,
        s"TraceReader InstID discontinuity at slot $i")
    }

    val firstInstCheck = RegInit(true.B)
    when (io.traceInsts.valid && firstInstCheck) {
      firstInstCheck := false.B
    }
    XSError(io.traceInsts.valid && firstInstCheck && io.traceInsts.bits(0).pcVA =/= 0x80000000L.U,
      "TraceReader first instruction PC is not 0x80000000")
    XSError(io.traceInsts.valid && firstInstCheck && io.traceInsts.bits(0).InstID =/= 1.U,
      "TraceReader first instruction InstID is not 1")
  }
}

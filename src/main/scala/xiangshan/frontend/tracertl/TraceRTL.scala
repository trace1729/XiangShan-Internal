package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.Redirect

class TraceFromIFU(implicit p: Parameters) extends TraceBundle {
  val ifuS2Flush      = Bool()
  val ifuS1Flush      = Bool()
  val ifuS1Fire       = Bool()
  val ifuS2Fire       = Bool()
  val ifuS2Valid      = Bool()
  val ifuWbEnable     = Bool()
  val ifuS1Valid      = Bool()
  val ifuS1AlignShift = UInt(2.W)
  val ifuS1PredInfo   = Input(new TracePredictInfo())
}

class TraceRTLIO(implicit p: Parameters) extends TraceBundle {
  val fromIFU = Input(new TraceFromIFU())
  val redirect = Input(new Bundle {
    val fromBackend = Valid(new Redirect())
    val fromIFUBPU  = Bool()
  })

  val checker           = Output(new TracePredCheckerFlatResp)
  val s2Block           = Output(Bool())
  val block             = Output(Bool())
  val s2CandidateCount  = Output(UInt(log2Ceil(IBufferEnqueueWidth + 1).W))
  val s3Decoded         = Output(Vec(IBufferEnqueueWidth, new TraceDecodedEntry))
}

class TraceRTL(implicit p: Parameters) extends TraceModule {
  val io = IO(new TraceRTLIO)
  dontTouch(io)

  private def shiftToIBuffer[T <: Data](data: Vec[T], shiftNum: UInt, default: T): Vec[T] = {
    require(shiftNum.getWidth == 2)
    val padded = VecInit.tabulate(IBufferEnqueueWidth) { i =>
      if (i < data.length) data(i) else 0.U.asTypeOf(default)
    }
    VecInit.tabulate(IBufferEnqueueWidth) { i =>
      MuxLookup(shiftNum, 0.U.asTypeOf(default))(Seq(
        0.U -> padded(i),
        1.U -> (if (i < 1) 0.U.asTypeOf(default) else padded(i - 1)),
        2.U -> (if (i < 2) 0.U.asTypeOf(default) else padded(i - 2)),
        3.U -> (if (i < 3) 0.U.asTypeOf(default) else padded(i - 3))
      ))
    }
  }

  if (env.TraceRTLMode) {
    val traceReader  = Module(new TraceReader)
    val traceDriver  = Module(new TraceDriver)
    val traceAligner = Module(new TraceAlignParallel)
    val preDecoder   = Module(new TracePreDecoder)
    val predChecker  = Module(new TracePredictChecker)
    val traceChecker = Module(new TraceChecker)

    traceReader.io.recv         := traceDriver.io.out.recv
    traceReader.io.redirect     := io.redirect.fromBackend
    traceReader.io.pcMatch.pcVA := io.fromIFU.ifuS1PredInfo.block(0).startAddr

    val pendingConcede2Bytes = RegInit(false.B)
    val nextConcede2Bytes = Wire(Bool())

    // TraceReader exposes the post-consumption packet combinationally when S3
    // fires. Keep the concede state in that same transaction domain: the next
    // S2 packet must see the state produced by the S3 packet being consumed,
    // not the value that will remain in the register until the rising edge.
    val visibleConcede2Bytes = Mux(io.fromIFU.ifuS2Fire, nextConcede2Bytes, pendingConcede2Bytes)
    val effectiveConcede2Bytes = visibleConcede2Bytes &&
      traceReader.io.traceInsts.valid &&
      traceReader.io.traceInsts.bits.head.pcVA === io.fromIFU.ifuS1PredInfo.block(0).startAddr - 2.U &&
      traceReader.io.traceInsts.bits.head.inst(1, 0) === 3.U

    traceAligner.io.debug_valid   := io.fromIFU.ifuS1Valid
    traceAligner.io.traceInsts    := traceReader.io.traceInsts
    traceAligner.io.predictInfo   := io.fromIFU.ifuS1PredInfo
    traceAligner.io.lastHalfValid := effectiveConcede2Bytes

    preDecoder.io.compact := traceAligner.io.result.compact

    val s2Block = !traceReader.io.traceInsts.valid ||
      (traceAligner.io.result.candidateCount === 0.U && !traceAligner.io.result.concede2Bytes)

    // Capture the same S2 transaction that sets IFU s3_valid at the rising edge.
    val s2ToS3 = io.fromIFU.ifuS1Fire && !io.fromIFU.ifuS1Flush

    val s3Decoded = RegEnable(preDecoder.io.out, s2ToS3)
    val s3ShiftNum = RegEnable(io.fromIFU.ifuS1AlignShift, 0.U(2.W), s2ToS3)
    val s3PredInfo = RegEnable(io.fromIFU.ifuS1PredInfo, s2ToS3)
    val s3TraceRange = RegEnable(traceAligner.io.result.position.traceRange, s2ToS3)
    val s3TraceForceJump = RegEnable(traceAligner.io.result.traceForceJump, s2ToS3)
    val s3DetectedConcede2Bytes = RegEnable(traceAligner.io.result.concede2Bytes, s2ToS3)
    val s3InheritedConcede2Bytes = RegEnable(effectiveConcede2Bytes, s2ToS3)
    val s3TraceBaseInstID = RegEnable(traceReader.io.traceInsts.bits.head.InstID, s2ToS3)

    // s2_fire && !s2_flush |=> wbEnable 
    predChecker.io.wbEnable      := io.fromIFU.ifuWbEnable
    predChecker.io.decoded       := s3Decoded
    predChecker.io.predictInfo   := s3PredInfo
    predChecker.io.traceRange    := s3TraceRange
    predChecker.io.concede2Bytes := s3InheritedConcede2Bytes

    traceChecker.io.decoded        := s3Decoded
    traceChecker.io.fixedValid     := predChecker.io.out.stage1Out.fixedTwoFetchRange
    traceChecker.io.traceForceJump := s3TraceForceJump

    traceDriver.io.fire          := io.fromIFU.ifuS2Fire
    traceDriver.io.decoded       := s3Decoded
    traceDriver.io.consumeValid  := traceChecker.io.consumeValid
    traceDriver.io.otherBlock    := false.B
    traceDriver.io.redirect      := io.redirect
    traceDriver.io.concede2Bytes := s3DetectedConcede2Bytes

    // A checker redirect retains only the prefix that actually fired into
    // IBuffer. A stalled checker packet has retained no trace instructions.
    val checkerRetainedInstNum = Mux(
      traceDriver.io.out.recv.valid,
      traceDriver.io.out.recv.bits.instNum,
      0.U
    )
    // checkerRedirect is produced one cycle after wbEnable. The checked S3
    // packet may fire either with wbEnable or one cycle later with the
    // redirect, so retain the first observed consumption across both cycles.
    val checkerBaseInstID = RegEnable(s3TraceBaseInstID, io.fromIFU.ifuWbEnable)
    val checkerRetainedAtWb = RegEnable(
      checkerRetainedInstNum,
      0.U(checkerRetainedInstNum.getWidth.W),
      io.fromIFU.ifuWbEnable
    )
    val checkerFiredAtWb = RegEnable(
      traceDriver.io.out.recv.valid,
      false.B,
      io.fromIFU.ifuWbEnable
    )
    val checkerFinalRetainedInstNum = Mux(
      checkerFiredAtWb,
      checkerRetainedAtWb,
      checkerRetainedInstNum
    )
    val checkerRewindInstID =
      (checkerBaseInstID + checkerFinalRetainedInstNum)(trtl.TraceInstIDWidth - 1, 0)
    // Ablation: IFU checker correction is already reflected by s3Fire/recv.
    // Keep the reader rewind path disabled to test whether the explicit second
    // redirect is responsible for the observed TraceReader deadlock.
    traceReader.io.ifuRedirect.valid := false.B
    traceReader.io.ifuRedirect.bits  := checkerRewindInstID

    // In a two-fetch transaction, block 0 may end with a taken CFI while block 1
    // contains only the first halfword of the target RVI.  That incomplete RVI
    // still has to be inherited by the next transaction even though the last
    // consumed instruction (in block 0) is a CFI.
    val concedeIsInSecondBlock = s3PredInfo.block(1).valid
    nextConcede2Bytes := s3DetectedConcede2Bytes &&
      (!traceDriver.io.out.endWithCFI || concedeIsInSecondBlock) &&
      !io.fromIFU.ifuS2Flush &&
      !io.fromIFU.ifuS1Flush

    val preserveConcedeOnInvalidTaken = pendingConcede2Bytes &&
      io.redirect.fromIFUBPU &&
      predChecker.io.out.stage2Out.checkerRedirect.valid &&
      predChecker.io.out.stage2Out.checkerRedirect.bits.invalidTaken
    val clearPendingConcede = io.fromIFU.ifuS2Flush || io.redirect.fromIFUBPU ||
      (io.fromIFU.ifuS1Flush && !io.fromIFU.ifuS2Valid)

    when(clearPendingConcede && !preserveConcedeOnInvalidTaken) {
      pendingConcede2Bytes := false.B
    }.elsewhen(io.fromIFU.ifuS2Fire) {
      pendingConcede2Bytes := nextConcede2Bytes
    }

    val s3DecodedForIFU = shiftToIBuffer(
      s3Decoded,
      s3ShiftNum,
      0.U.asTypeOf(new TraceDecodedEntry)
    )
    val fixedTwoFetchRangeForIFU = shiftToIBuffer(
      VecInit(predChecker.io.out.stage1Out.fixedTwoFetchRange.asBools),
      s3ShiftNum,
      false.B
    )
    val fixedTwoFetchTakenForIFU = shiftToIBuffer(
      VecInit(predChecker.io.out.stage1Out.fixedTwoFetchTaken.asBools),
      s3ShiftNum,
      false.B
    )

    io.checker           := predChecker.io.out
    io.checker.stage1Out.fixedTwoFetchRange := fixedTwoFetchRangeForIFU.asUInt
    io.checker.stage1Out.fixedTwoFetchTaken := fixedTwoFetchTakenForIFU.asUInt
    io.s2Block           := s2Block
    io.block             := traceDriver.io.out.block
    io.s2CandidateCount  := traceAligner.io.result.candidateCount
    io.s3Decoded         := s3DecodedForIFU
  } else {
    io <> DontCare
  }
}

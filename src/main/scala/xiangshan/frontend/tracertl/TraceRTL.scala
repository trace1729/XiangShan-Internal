package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.Redirect

class TraceS3FromIFU(implicit p: Parameters) extends TraceBundle {
  val valid          = Bool()
  val traceInsts     = Vec(trtl.TracePredictWidth, Valid(new TraceInstrBundle()))
  val predInfo       = new TracePredictInfo()
  val preDecode      = new TracePreDecodeFlatResp()
  val traceRange     = UInt(trtl.TracePredictWidth.W)
  val traceForceJump = Bool()
  val traceTaken2B   = Bool()
  val concede2Bytes  = Bool()
  val otherBlock     = Bool()
}

class TraceFromIFU(implicit p: Parameters) extends TraceBundle {
  val redirect      = Bool()
  val s2Flush       = Bool()
  val s2Fire        = Bool()
  val s3Fire        = Bool()
  val ibufferFire   = Bool()
  val s3Ready       = Bool()
  val wbEnable      = Bool()
  val valid         = Bool()
  val shiftNum      = UInt(2.W)
  val predInfo      = Input(new TracePredictInfo())
  val s3            = new TraceS3FromIFU()
}

class TraceRTLIO(implicit p: Parameters) extends TraceBundle {
  val fromIFU = Input(new TraceFromIFU())
  val redirect = Input(new Bundle {
    val fromBackend = Valid(new Redirect())
    val fromIFUBPU  = Bool()
  })

  val predecoder        = Output(new TracePreDecodeFlatResp)
  val checker           = Output(new TracePredCheckerFlatResp)
  val traceChecker      = Output(new TraceCheckerResp)
  val traceAlignInsts   = Output(Vec(trtl.TracePredictWidth, Valid(new TraceInstrBundle())))
  val traceRange        = Output(UInt(trtl.TracePredictWidth.W))
  val traceTaken2B      = Output(Bool())
  val concede2Bytes     = Output(Bool())
  val otherBlock        = Output(Bool())
  val s2Block           = Output(Bool())
  val block             = Output(Bool())
  val traceForceJump    = Output(Bool())
  val traceWrongPathEmu = Output(Bool())
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
    traceReader.io.pcMatch.pcVA := io.fromIFU.predInfo.block(0).startAddr

    val pendingConcede2Bytes = RegInit(false.B)
    val effectiveConcede2Bytes = pendingConcede2Bytes &&
      traceReader.io.traceInsts.valid &&
      traceReader.io.traceInsts.bits.head.pcVA === io.fromIFU.predInfo.block(0).startAddr - 2.U

    traceAligner.io.debug_valid   := io.fromIFU.valid
    traceAligner.io.traceInsts    := traceReader.io.traceInsts
    traceAligner.io.predictInfo   := io.fromIFU.predInfo
    traceAligner.io.lastHalfValid := effectiveConcede2Bytes

    preDecoder.io.compact := traceAligner.io.result.compact

    val s2Block = !traceReader.io.traceInsts.valid ||
      (traceAligner.io.result.candidateCount === 0.U && !traceAligner.io.result.concede2Bytes)

    // Capture the same S2 transaction that sets IFU s3_valid at the rising edge.
    val s2ToS3 = io.fromIFU.s2Fire && !io.fromIFU.s2Flush

    val s3Decoded = RegEnable(preDecoder.io.out, s2ToS3)
    val s3ShiftNum = RegEnable(io.fromIFU.shiftNum, 0.U(2.W), s2ToS3)
    val s3PredInfo = RegEnable(io.fromIFU.predInfo, s2ToS3)
    val s3TraceRange = RegEnable(traceAligner.io.result.position.traceRange, s2ToS3)
    val s3TraceForceJump = RegEnable(traceAligner.io.result.traceForceJump, s2ToS3)
    val s3DetectedConcede2Bytes = RegEnable(traceAligner.io.result.concede2Bytes, s2ToS3)
    val s3InheritedConcede2Bytes = RegEnable(effectiveConcede2Bytes, s2ToS3)

    // s2_fire && !s2_flush |=> wbEnable 
    predChecker.io.wbEnable      := io.fromIFU.wbEnable
    predChecker.io.decoded       := s3Decoded
    predChecker.io.predictInfo   := s3PredInfo
    predChecker.io.traceRange    := s3TraceRange
    predChecker.io.concede2Bytes := s3InheritedConcede2Bytes

    traceChecker.io.decoded        := s3Decoded
    traceChecker.io.fixedValid     := predChecker.io.out.stage1Out.fixedTwoFetchRange
    traceChecker.io.traceForceJump := s3TraceForceJump

    traceDriver.io.fire          := io.fromIFU.s3Fire
    traceDriver.io.decoded       := s3Decoded
    traceDriver.io.consumeValid  := traceChecker.io.consumeValid
    traceDriver.io.otherBlock    := false.B
    traceDriver.io.redirect      := io.redirect
    traceDriver.io.concede2Bytes := s3DetectedConcede2Bytes

    // In a two-fetch transaction, block 0 may end with a taken CFI while block 1
    // contains only the first halfword of the target RVI.  That incomplete RVI
    // still has to be inherited by the next transaction even though the last
    // consumed instruction (in block 0) is a CFI.
    val concedeIsInSecondBlock = s3PredInfo.block(1).valid
    val nextConcede2Bytes = s3DetectedConcede2Bytes &&
      (!traceDriver.io.out.endWithCFI || concedeIsInSecondBlock) &&
      !io.fromIFU.redirect &&
      !io.fromIFU.s2Flush

    val preserveConcedeOnInvalidTaken = pendingConcede2Bytes &&
      io.redirect.fromIFUBPU &&
      predChecker.io.out.stage2Out.checkerRedirect.valid &&
      predChecker.io.out.stage2Out.checkerRedirect.bits.invalidTaken
    val clearPendingConcede = io.fromIFU.redirect || io.redirect.fromIFUBPU ||
      (io.fromIFU.s2Flush && !io.fromIFU.s3.valid)

    when(clearPendingConcede && !preserveConcedeOnInvalidTaken) {
      pendingConcede2Bytes := false.B
    }.elsewhen(io.fromIFU.s3Fire) {
      pendingConcede2Bytes := nextConcede2Bytes
    }

    io.predecoder := 0.U.asTypeOf(io.predecoder)
    io.predecoder.pd.zipWithIndex.foreach { case (pd, i) =>
      pd := preDecoder.io.out(i).pd
    }
    io.predecoder.instr.zipWithIndex.foreach { case (inst, i) =>
      inst := preDecoder.io.out(i).compact.traceInfo.inst
    }
    io.predecoder.jumpOffset.zipWithIndex.foreach { case (offset, i) =>
      offset := preDecoder.io.out(i).jumpOffset
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
    io.traceChecker.traceRange := s3TraceRange
    io.traceAlignInsts.zipWithIndex.foreach { case (inst, i) =>
      inst.valid := traceAligner.io.result.compact(i).valid
      inst.bits := traceAligner.io.result.compact(i).traceInfo
    }
    io.traceRange        := traceAligner.io.result.position.traceRange
    io.traceTaken2B      := traceAligner.io.result.concede2Bytes
    io.concede2Bytes     := effectiveConcede2Bytes
    io.otherBlock        := !traceReader.io.traceInsts.valid
    io.s2Block           := s2Block
    io.block             := traceDriver.io.out.block
    io.traceForceJump    := traceAligner.io.result.traceForceJump
    io.traceWrongPathEmu := false.B
    io.s2CandidateCount  := traceAligner.io.result.candidateCount
    io.s3Decoded         := s3DecodedForIFU
  } else {
    io <> DontCare
  }
}

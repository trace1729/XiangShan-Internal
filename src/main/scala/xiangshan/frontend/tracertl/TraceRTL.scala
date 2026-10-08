package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.Redirect
import utility.{Constantin, XSPerfAccumulate, XSPerfHistogram}

class TraceFromIFU(implicit p: Parameters) extends TraceBundle {
  val ifuS2Flush      = Bool()
  val ifuS1Flush      = Bool()
  val ifuS1Fire       = Bool()
  val ifuS2Fire       = Bool()
  val ifuS2Valid      = Bool()
  val ifuWbEnable     = Bool()
  val ifuS1Valid      = Bool()
  val ifuS1AlignShift = UInt(2.W)
  val ifuS1NoTrace    = Bool() // uncache or ICache exception: the packet does not carry trace instructions
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
  val wrongPathEmu      = Output(new Bundle {
    val s2Active = Bool() // the IFU s1 packet is aligned as an emulated wrong-path packet
    val s3Packet = Bool() // the IFU s2 packet is an emulated wrong-path packet
  })
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
    val traceAlignerWrongPath = Module(new TraceAlignWrongPath)

    // Emulated wrong-path packets never consume the reader: after an IFU
    // redirect the head is already the right-path frontier, and a backend
    // redirect rewinds the reader anyway.
    val s3WrongPath = Wire(Bool())
    traceReader.io.recv.valid   := traceDriver.io.out.recv.valid && !s3WrongPath
    traceReader.io.recv.bits    := traceDriver.io.out.recv.bits
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

    // Capture the same S2 transaction that sets IFU s3_valid at the rising edge.
    val s2ToS3 = io.fromIFU.ifuS1Fire && !io.fromIFU.ifuS1Flush

    /* ** wrong-path emulation **
     * A pc mismatch in S2 means the predicted fetch block is off the trace.
     * Instead of blocking the IFU until the redirect, feed right-path trace
     * instructions tagged isWrongPath. The redirect that resolves the
     * mismatch comes from an older right-path instruction, so every
     * emulated instruction is younger than it and is flushed. After the
     * redirect the right path is fetched again from the rewound reader.
     */
    val wpEnable = if (trtl.TraceEnableWrongPathEmu) Constantin.createRecord("TraceWrongPathEmu", true) else false.B
    val wpState = RegInit(false.B)
    val wpOffset = RegInit(0.U(log2Ceil(trtl.TraceBufferSize + 1).W))
    val wpClear = io.redirect.fromBackend.valid || io.redirect.fromIFUBPU
    val alignMismatch = traceReader.io.traceInsts.valid &&
      traceAligner.io.result.candidateCount === 0.U && !traceAligner.io.result.concede2Bytes
    // The reader window is only the S2 frontier when no unconsumed older packet
    // is still held in S3; otherwise the mismatch is just S3 back-pressure.
    val s3Released = !io.fromIFU.ifuS2Valid || io.fromIFU.ifuS2Fire
    val wpConvergence = if (trtl.TraceWrongPathEmuWhenConvergence) traceReader.io.pcMatch.found else true.B
    val wpEnter = wpEnable && !wpState && io.fromIFU.ifuS1Valid && s3Released && alignMismatch &&
      !traceReader.io.traceInsts.bits.head.isFastSim && wpConvergence
    val wpActive = wpState || wpEnter

    traceReader.io.wpOffset := wpOffset
    traceAlignerWrongPath.io.traceInsts  := traceReader.io.wpInsts
    traceAlignerWrongPath.io.predictInfo := io.fromIFU.ifuS1PredInfo
    val wpCount = traceAlignerWrongPath.io.count

    when(wpClear) {
      wpState  := false.B
      wpOffset := 0.U
    }.otherwise {
      when(wpEnter) {
        wpState := true.B
      }
      when(wpActive && s2ToS3) {
        wpOffset := traceReader.io.wpBase + wpCount
      }
    }

    preDecoder.io.compact := Mux(wpActive, traceAlignerWrongPath.io.compact, traceAligner.io.result.compact)

    val s2Block = Mux(
      wpActive,
      // A wrong-path fetch block that is uncache or faults in the ICache would
      // leave the trace path; keep it blocked as without emulation.
      !traceReader.io.wpInsts.valid || wpCount === 0.U || io.fromIFU.ifuS1NoTrace,
      !traceReader.io.traceInsts.valid ||
        (traceAligner.io.result.candidateCount === 0.U && !traceAligner.io.result.concede2Bytes)
    )
    val s2CandidateCount = Mux(wpActive, wpCount, traceAligner.io.result.candidateCount)

    s3WrongPath := RegEnable(wpActive, false.B, s2ToS3)
    val s3Decoded = RegEnable(preDecoder.io.out, s2ToS3)
    val s3ShiftNum = RegEnable(io.fromIFU.ifuS1AlignShift, 0.U(2.W), s2ToS3)
    val s3PredInfo = RegEnable(io.fromIFU.ifuS1PredInfo, s2ToS3)
    val s3TraceRange = RegEnable(traceAligner.io.result.position.traceRange, s2ToS3)
    val s3TraceForceJump = RegEnable(traceAligner.io.result.traceForceJump && !wpActive, s2ToS3)
    val s3DetectedConcede2Bytes = RegEnable(traceAligner.io.result.concede2Bytes && !wpActive, s2ToS3)
    val s3InheritedConcede2Bytes = RegEnable(effectiveConcede2Bytes && !wpActive, s2ToS3)
    val s3TraceBaseInstID = RegEnable(traceReader.io.traceInsts.bits.head.InstID, s2ToS3)

    // s2_fire && !s2_flush |=> wbEnable 
    predChecker.io.wbEnable      := io.fromIFU.ifuWbEnable
    predChecker.io.decoded       := s3Decoded
    predChecker.io.predictInfo   := s3PredInfo
    predChecker.io.traceRange    := s3TraceRange
    predChecker.io.concede2Bytes := s3InheritedConcede2Bytes

    traceChecker.io.decoded        := s3Decoded
    // Emulated wrong-path packets are not checked against the prediction:
    // keep every placed instruction and never raise a checker redirect.
    val s3CompactValid = VecInit(s3Decoded.map(_.compact.valid)).asUInt
    val fixedTwoFetchRange = Mux(s3WrongPath, s3CompactValid, predChecker.io.out.stage1Out.fixedTwoFetchRange)
    val fixedTwoFetchTaken = Mux(s3WrongPath, 0.U, predChecker.io.out.stage1Out.fixedTwoFetchTaken)
    val checkerIsWrongPath = RegEnable(s3WrongPath, false.B, io.fromIFU.ifuWbEnable)
    val checkerRedirectValid = predChecker.io.out.stage2Out.checkerRedirect.valid && !checkerIsWrongPath

    traceChecker.io.fixedValid     := fixedTwoFetchRange
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
      checkerRedirectValid &&
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
      VecInit(fixedTwoFetchRange.asBools),
      s3ShiftNum,
      false.B
    )
    val fixedTwoFetchTakenForIFU = shiftToIBuffer(
      VecInit(fixedTwoFetchTaken.asBools),
      s3ShiftNum,
      false.B
    )

    io.checker           := predChecker.io.out
    io.checker.stage1Out.fixedTwoFetchRange := fixedTwoFetchRangeForIFU.asUInt
    io.checker.stage1Out.fixedTwoFetchTaken := fixedTwoFetchTakenForIFU.asUInt
    io.checker.stage2Out.checkerRedirect.valid := checkerRedirectValid
    when(checkerIsWrongPath) {
      io.checker.stage2Out.perfFaultType := 0.U.asTypeOf(io.checker.stage2Out.perfFaultType)
    }
    io.s2Block           := s2Block
    io.block             := traceDriver.io.out.block && !s3WrongPath
    io.s2CandidateCount  := s2CandidateCount
    io.s3Decoded         := s3DecodedForIFU
    io.wrongPathEmu.s2Active := wpActive
    io.wrongPathEmu.s3Packet := s3WrongPath

    /* ** perf ** */
    val s3EnqInstNum = PopCount(s3CompactValid)
    val wpEpisodeInsts = RegInit(0.U(32.W))
    val wpEpisodeCycles = RegInit(0.U(32.W))
    val wpExit = wpState && wpClear
    when(wpExit || !wpState) {
      wpEpisodeInsts  := 0.U
      wpEpisodeCycles := 0.U
    }.otherwise {
      wpEpisodeInsts  := wpEpisodeInsts + Mux(io.fromIFU.ifuS2Fire && s3WrongPath, s3EnqInstNum, 0.U)
      wpEpisodeCycles := wpEpisodeCycles + 1.U
    }
    XSPerfAccumulate("s1_block_reader_invalid", io.fromIFU.ifuS1Valid && !traceReader.io.traceInsts.valid)
    XSPerfAccumulate("s1_block_pc_mismatch", io.fromIFU.ifuS1Valid && alignMismatch && !wpActive)
    XSPerfAccumulate("s1_block_by_s3", io.fromIFU.ifuS1Valid && alignMismatch && !s3Released)
    XSPerfAccumulate("wpe_enter", wpEnter && !wpClear)
    XSPerfAccumulate("wpe_active_cycles", wpState)
    XSPerfAccumulate("wpe_exit_backend", wpExit && io.redirect.fromBackend.valid)
    XSPerfAccumulate("wpe_exit_ifu", wpExit && !io.redirect.fromBackend.valid)
    XSPerfAccumulate("wpe_s1_block", io.fromIFU.ifuS1Valid && wpActive && s2Block)
    XSPerfAccumulate("wpe_s1_packets", s2ToS3 && wpActive)
    XSPerfAccumulate("wpe_s1_insts", Mux(s2ToS3 && wpActive, wpCount, 0.U))
    XSPerfAccumulate("wpe_window_wrap", s2ToS3 && wpActive && wpOffset =/= 0.U && traceReader.io.wpBase === 0.U)
    XSPerfAccumulate("wpe_enq_packets", io.fromIFU.ifuS2Fire && s3WrongPath)
    XSPerfAccumulate("wpe_enq_insts", Mux(io.fromIFU.ifuS2Fire && s3WrongPath, s3EnqInstNum, 0.U))
    XSPerfAccumulate("wpe_checker_redirect_masked",
      predChecker.io.out.stage2Out.checkerRedirect.valid && checkerIsWrongPath)
    XSPerfHistogram("wpe_episode_enq_insts", wpEpisodeInsts, wpExit, 0, 256, 16)
    XSPerfHistogram("wpe_episode_cycles", wpEpisodeCycles, wpExit, 0, 128, 8)
  } else {
    io <> DontCare
  }
}

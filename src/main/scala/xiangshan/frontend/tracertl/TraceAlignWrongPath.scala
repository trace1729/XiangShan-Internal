package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters

class TraceAlignWrongPathIO(implicit p: Parameters) extends TraceBundle {
  val traceInsts  = Input(Valid(Vec(trtl.TracePredictWidth, new TraceInstrBundle())))
  val predictInfo = Input(new TracePredictInfo)
  val compact     = Output(Vec(IBufferEnqueueWidth, new TraceCompactEntry))
  val count       = Output(UInt(log2Ceil(IBufferEnqueueWidth + 1).W))
}

/** Lay consecutive trace instructions over the predicted (wrong-path) fetch
  * window as stand-in wrong-path instructions.
  *
  * The trace instructions keep their own pc/inst; only their FTQ placement
  * (selectBlock / instrEndOffset) follows the predicted blocks. Instructions
  * are packed from halfword 0 of block 0, then of block 1. An RVI that would
  * cross the end of block 0 moves to block 1 instead, so no wrong-path
  * instruction spans two FTQ entries. Packing stops at the first
  * instruction carrying an exception/force-jump or fast-simulation flag.
  */
class TraceAlignWrongPath(implicit p: Parameters) extends TraceModule {
  val io = IO(new TraceAlignWrongPathIO())

  private val width = trtl.TracePredictWidth
  private val posWidth = log2Ceil(width + 1)
  require(width <= IBufferEnqueueWidth)

  private val insts  = io.traceInsts.bits
  private val block0 = io.predictInfo.block(0)
  private val block1 = io.predictInfo.block(1)

  private val placed   = Wire(Vec(width, Bool()))
  private val inBlock1 = Wire(Vec(width, Bool()))
  private val endPos   = Wire(Vec(width, UInt(posWidth.W))) // local halfword position after this instruction

  for (i <- 0 until width) {
    val len        = Mux(insts(i).inst(1, 0) === 3.U, 2.U, 1.U)
    val prevPlaced = if (i == 0) io.traceInsts.valid else placed(i - 1)
    val prevBlock1 = if (i == 0) false.B else inBlock1(i - 1)
    val prevEnd    = if (i == 0) 0.U(posWidth.W) else endPos(i - 1)
    val allowed    = prevPlaced && !insts(i).hasException && !insts(i).isFastSim

    val fitsBlock0   = !prevBlock1 && block0.valid && prevEnd +& len <= block0.size
    val moveToBlock1 = !prevBlock1 && !fitsBlock0 && block1.valid && len <= block1.size
    val fitsBlock1   = prevBlock1 && prevEnd +& len <= block1.size

    placed(i)   := allowed && (fitsBlock0 || moveToBlock1 || fitsBlock1)
    inBlock1(i) := prevBlock1 || moveToBlock1
    endPos(i)   := Mux(moveToBlock1, len, prevEnd + len)
  }

  io.compact.zipWithIndex.foreach { case (entry, i) =>
    entry := 0.U.asTypeOf(new TraceCompactEntry)
    if (i < width) {
      entry.valid                 := placed(i)
      entry.traceInfo             := insts(i)
      entry.traceInfo.isWrongPath := true.B
      entry.instrEndOffset        := (endPos(i) - 1.U)(FetchBlockInstOffsetWidth - 1, 0)
      entry.selectBlock           := inBlock1(i)
    }
  }
  io.count := PopCount(placed)
}

package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.Redirect
import utility.ParallelPosteriorityMux

class TraceDriverIO(implicit p: Parameters) extends TraceBundle {
  val fire          = Input(Bool())
  val decoded       = Input(Vec(IBufferEnqueueWidth, new TraceDecodedEntry))
  val consumeValid  = Input(UInt(IBufferEnqueueWidth.W))
  val otherBlock    = Input(Bool())
  val concede2Bytes   = Input(Bool())
  val redirect      = Input(new Bundle {
    val fromBackend = Valid(new Redirect())
    val fromIFUBPU  = Bool()
  })
  val out = new TraceDriverOutput()
}

class TraceDriver(implicit p: Parameters) extends TraceModule {
  val io = IO(new TraceDriverIO())
  dontTouch(io)

  val compactValid = VecInit(io.decoded.map(_.compact.valid)).asUInt
  val finalValid = io.consumeValid & compactValid
  val recvInstNum = PopCount(finalValid)
  val pcMismatch = recvInstNum === 0.U

  io.out.block := io.otherBlock || (pcMismatch && !io.concede2Bytes)
  io.out.recv.valid := io.fire
  io.out.recv.bits.instNum := recvInstNum
  io.out.endWithCFI := Mux(
    finalValid.orR,
    ParallelPosteriorityMux(finalValid, io.decoded.map(entry =>
      (entry.compact.traceInfo.branchType =/= 0.U) && entry.compact.traceInfo.branchTaken(0)
    )),
    false.B
  )
}

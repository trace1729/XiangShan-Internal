package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utility.{CircularQueuePtr, HasCircularQueuePtrHelper, XSError}

class TraceDriveCollectBundle(implicit p: Parameters) extends TraceBundle {
  val pc = UInt(64.W)
  val inst = UInt(32.W)
}

class TraceDriveCollectorIO(implicit p: Parameters) extends TraceBundle {
  val in = Input(Vec(DecodeWidth, Valid(new TraceDriveCollectBundle())))
}

class TraceDrivePtr(Size: Int)(implicit p: Parameters) extends CircularQueuePtr[TraceDrivePtr](Size)

class TraceDriveCollector(implicit p: Parameters) extends TraceModule
  with HasCircularQueuePtrHelper {
  val io = IO(new TraceDriveCollectorIO)

  val traceDriveHelper = Module(new TraceDriveCollectorHelper(DecodeWidth))

  traceDriveHelper.clock := clock
  traceDriveHelper.reset := reset
  for (i <- 0 until DecodeWidth) {
    traceDriveHelper.enable(i) := io.in(i).valid
    traceDriveHelper.pc(i) := io.in(i).bits.pc
    traceDriveHelper.inst(i) := io.in(i).bits.inst
  }
}

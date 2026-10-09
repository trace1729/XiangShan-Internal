package xiangshan.frontend.tracertl

import chisel3._
import chisel3.experimental.ExtModule
import chisel3.util._
import difftest.DifftestModule.createCppExtModule
import org.chipsalliance.cde.config.Parameters

/** Combinational query of the C++ wrong-path oracle (trace_wrong_path.cpp).
  *
  * The outputs are a pure function of the inputs: the instructions found in
  * the trace at the predicted fetch PCs, placed in compact order. GSIM does
  * not support array ports on external modules, so every slot field is a
  * scalar port with an 8/16/32/64-bit C++ storage type.
  */
class TraceWrongPathHelper(slots: Int, vaddrBits: Int) extends ExtModule with HasExtModuleInline {
  val clock = IO(Input(Clock()))
  val reset = IO(Input(Reset()))

  private val inputSpec = Seq(
    "enable"   -> 8,
    "b0_valid" -> 8,
    "b0_start" -> 64,
    "b0_size"  -> 8,
    "b0_taken" -> 8,
    "b1_valid" -> 8,
    "b1_start" -> 64,
    "b1_size"  -> 8,
    "anchor"   -> 64
  )
  private val headOutputSpec = Seq("count" -> 8, "range" -> 32, "cfg" -> 8)
  // field name -> (C++ width, TraceWPSlot member)
  private val slotFieldSpec = Seq(
    "pcVA"   -> (64, "pc_va"),
    "pcPA"   -> (64, "pc_pa"),
    "memVA"  -> (64, "mem_va"),
    "memPA"  -> (64, "mem_pa"),
    "target" -> (64, "target"),
    "inst"   -> (32, "inst"),
    "meta"   -> (64, "meta")
  )
  private val slotFieldSel = Map(
    "pcVA" -> 0, "pcPA" -> 1, "memVA" -> 2, "memPA" -> 3, "target" -> 4, "inst" -> 5, "meta" -> 6
  )

  private val inPorts = inputSpec.map { case (name, w) => name -> IO(Input(UInt(w.W))).suggestName(name) }.toMap
  val enable   = inPorts("enable")
  val b0_valid = inPorts("b0_valid")
  val b0_start = inPorts("b0_start")
  val b0_size  = inPorts("b0_size")
  val b0_taken = inPorts("b0_taken")
  val b1_valid = inPorts("b1_valid")
  val b1_start = inPorts("b1_start")
  val b1_size  = inPorts("b1_size")
  val anchor   = inPorts("anchor")

  private val headPorts = headOutputSpec.map { case (name, w) => name -> IO(Output(UInt(w.W))).suggestName(name) }.toMap
  val count = headPorts("count")
  val range = headPorts("range")
  val cfg   = headPorts("cfg")

  // Declared field-major, matching the C++ parameter order below.
  private val slotPorts: Map[String, Seq[UInt]] = slotFieldSpec.map { case (name, (w, _)) =>
    name -> Seq.tabulate(slots)(i => IO(Output(UInt(w.W))).suggestName(s"insts_${i}_$name"))
  }.toMap
  def slot(name: String, i: Int): UInt = slotPorts(name)(i)

  private def cppType(w: Int): String = w match {
    case 8  => "uint8_t"
    case 16 => "uint16_t"
    case 32 => "uint32_t"
    case 64 => "uint64_t"
  }

  private val cppExtModule = {
    val params =
      Seq("uint8_t reset") ++
        inputSpec.map { case (name, w) => s"${cppType(w)} $name" } ++
        headOutputSpec.map { case (name, w) => s"${cppType(w)}& $name" } ++
        slotFieldSpec.flatMap { case (name, (w, _)) => (0 until slots).map(i => s"${cppType(w)}& insts_${i}_$name") }
    val copy = slotFieldSpec.flatMap { case (name, (_, member)) =>
      (0 until slots).map(i => s"    insts_${i}_$name = pkt.slots[$i].$member;")
    }.mkString("\n")
    val clear = slotFieldSpec.flatMap { case (name, _) =>
      (0 until slots).map(i => s"    insts_${i}_$name = 0;")
    }.mkString("\n")
    s"""
       |void TraceWrongPathHelper(
       |  ${params.mkString(",\n  ")}
       |) {
       |  cfg = reset ? 0 : trace_wp_cfg($vaddrBits);
       |  if (!reset && enable) {
       |    TraceWPRequest req = { b0_start, b1_start, anchor, b0_valid, b0_size, b0_taken,
       |      b1_valid, b1_size, $vaddrBits };
       |    const TraceWPPacket &pkt = trace_wp_packet(req);
       |    count = pkt.count;
       |    range = pkt.range;
       |$copy
       |  } else {
       |    count = 0;
       |    range = 0;
       |$clear
       |  }
       |}
       |""".stripMargin
  }
  createCppExtModule("TraceWrongPathHelper", cppExtModule, Some("\"tracertl.h\""))

  private def getVerilog: String = {
    val inDecl = inputSpec.map { case (name, w) => s"  input  [${w - 1}:0] $name," }.mkString("\n")
    val headDecl = headOutputSpec.map { case (name, w) => s"  output [${w - 1}:0] $name," }.mkString("\n")
    val slotDecl = slotFieldSpec.flatMap { case (name, (w, _)) =>
      (0 until slots).map(i => s"  output [${w - 1}:0] insts_${i}_$name,")
    }.mkString("\n")
    val query = "b0_valid, b0_start, b0_size, b0_taken, b1_valid, b1_start, b1_size, anchor"
    val on = "(!reset && enable[0])"
    val headAssign = Seq("count" -> 7, "range" -> 8).map { case (name, field) =>
      s"  assign $name = $on ? trace_wp_query($query, 8'd$vaddrBits, 8'd0, 8'd$field) : 0;"
    }.mkString("\n")
    val slotAssign = slotFieldSpec.flatMap { case (name, _) =>
      (0 until slots).map(i =>
        s"  assign insts_${i}_$name = $on ? trace_wp_query($query, 8'd$vaddrBits, 8'd$i, 8'd${slotFieldSel(name)}) : 0;"
      )
    }.mkString("\n")
    s"""
       |import "DPI-C" function longint trace_wp_query(
       |  input byte b0_valid, input longint b0_start, input byte b0_size, input byte b0_taken,
       |  input byte b1_valid, input longint b1_start, input byte b1_size,
       |  input longint anchor, input byte vaddr_bits, input byte slot, input byte field
       |);
       |
       |module TraceWrongPathHelper(
       |$inDecl
       |$headDecl
       |$slotDecl
       |  input  clock,
       |  input  reset
       |);
       |$headAssign
       |  assign cfg = reset ? 0 : trace_wp_query($query, 8'd$vaddrBits, 8'd0, 8'd9);
       |$slotAssign
       |endmodule
       |""".stripMargin
  }
  setInline(s"$desiredName.sv", getVerilog)
}

class TraceAlignRealWrongPathIO(implicit p: Parameters) extends TraceBundle {
  val enable      = Input(Bool())
  val anchorID    = Input(UInt(trtl.TraceInstIDWidth.W))
  val predictInfo = Input(new TracePredictInfo)
  val compact     = Output(Vec(IBufferEnqueueWidth, new TraceCompactEntry))
  val count       = Output(UInt(log2Ceil(IBufferEnqueueWidth + 1).W))
  val traceRange  = Output(UInt(trtl.TracePredictWidth.W))
  val ifuCheck    = Output(Bool())
}

/** Real wrong-path packets: the instructions at the predicted fetch PCs.
  *
  * Each predicted halfword address is looked up in the trace by the C++
  * oracle, which returns a dynamic instance of the static instruction there
  * (the latest one older than anchorID, the right-path frontier). Placement
  * follows the instructions' own PCs, so pc == traceInfo.pcVA and the
  * FTQ offsets match what an execution-driven frontend would produce.
  */
class TraceAlignRealWrongPath(implicit p: Parameters) extends TraceModule {
  val io = IO(new TraceAlignRealWrongPathIO())

  private val width = trtl.TracePredictWidth
  require(width <= IBufferEnqueueWidth && width <= 32)

  private val helper = Module(new TraceWrongPathHelper(width, VAddrBits))
  helper.clock := clock
  helper.reset := reset

  private val block0 = io.predictInfo.block(0)
  private val block1 = io.predictInfo.block(1)
  helper.enable   := io.enable
  helper.b0_valid := block0.valid
  helper.b0_start := block0.startAddr
  helper.b0_size  := block0.size
  helper.b0_taken := block0.ftqOffset.valid
  helper.b1_valid := block1.valid
  helper.b1_start := block1.startAddr
  helper.b1_size  := block1.size
  helper.anchor   := io.anchorID

  private def narrow(raw: UInt, w: Int): UInt = raw(w - 1, 0)

  io.compact.zipWithIndex.foreach { case (entry, i) =>
    entry := 0.U.asTypeOf(new TraceCompactEntry)
    if (i < width) {
      val meta = helper.slot("meta", i)
      val info = entry.traceInfo
      info.pcVA           := narrow(helper.slot("pcVA", i), info.pcVA.getWidth)
      info.pcPA           := narrow(helper.slot("pcPA", i), info.pcPA.getWidth)
      info.memoryAddrVA   := helper.slot("memVA", i)
      info.memoryAddrPA   := helper.slot("memPA", i)
      info.target         := narrow(helper.slot("target", i), info.target.getWidth)
      info.inst           := narrow(helper.slot("inst", i), info.inst.getWidth)
      info.memoryType     := meta(3, 0)
      info.memorySize     := meta(7, 4)
      info.branchType     := meta(15, 8)
      info.branchTaken    := meta(23, 16)
      info.exception      := meta(31, 24)
      info.fastSimulation := 0.U
      info.InstID         := io.anchorID
      info.isWrongPath    := true.B
      info.hasTriggeredExuRedirect := false.B
      entry.valid             := meta(42)
      entry.instrEndOffset    := meta(32 + FetchBlockInstOffsetWidth - 1, 32)
      entry.selectBlock       := meta(40)
      entry.isCrossBlockInstr := meta(41)
    }
  }
  io.count      := narrow(helper.count, io.count.getWidth)
  io.traceRange := narrow(helper.range, width)
  io.ifuCheck   := helper.cfg(0)
}

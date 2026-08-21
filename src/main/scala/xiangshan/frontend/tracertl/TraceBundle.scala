/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *          http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * ************************************************************************************* */
package xiangshan.frontend.tracertl

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.frontend.{FrontendBundle, FrontendModule, PreDecodeInfo, PrunedAddr}

class TraceBundle(implicit p: Parameters) extends FrontendBundle {
  def trtl = p(TraceRTLParamKey)
}
class TraceModule(implicit p: Parameters) extends FrontendModule {
  def trtl = p(TraceRTLParamKey)
}

class TraceInstrInnerBundle(implicit p: Parameters) extends Bundle {
  def trtl = p(TraceRTLParamKey)

  val pcVA = UInt(trtl.TraceVAddrWidth.W)
  val pcPA = UInt(trtl.TracePAddrWidth.W)
  val memoryAddrVA = UInt(trtl.TraceVAddrWidth.W)
  val memoryAddrPA = UInt(trtl.TracePAddrWidth.W)
  val target = UInt(trtl.TraceVAddrWidth.W)
  val inst = UInt(trtl.TraceInstCodeWidth.W)
  val memoryType = UInt(4.W)
  val memorySize = UInt(4.W)
  val branchType = UInt(8.W)
  val branchTaken = UInt(8.W)
  val exception = UInt(8.W)

  val fastSimulation = UInt(8.W)
  val InstID = UInt(trtl.TraceInstIDWidth.W)

  def isFastSim = fastSimulation(0) === 1.U
  def isTaken = branchTaken(0) === 1.U
  def isException = exception =/= 0.U

  def seqPC = pcVA + Mux(inst(1, 0) === 0x3.U, 4.U, 2.U)
  def nextPC = Mux((exception =/= 0.U) || branchTaken(0), target,
               Mux(inst(1,0) === 3.U, pcVA + 4.U, pcVA + 2.U))
  def nextEqSeq = nextPC === seqPC
}

object TraceInstrInnerBundle {
  def apply(pcVA: UInt, pcPA: UInt, memoryAddrVA: UInt, memoryAddrPA: UInt,
    target: UInt, inst: UInt, memoryType: UInt, memorySize: UInt,
    branchType: UInt, branchTaken: UInt,
    InstID: UInt)(implicit p: Parameters): TraceInstrInnerBundle = {

    def trtl = p(TraceRTLParamKey)

    val bundle = Wire(new TraceInstrInnerBundle)
    bundle.pcVA := pcVA
    bundle.pcPA := pcPA
    bundle.memoryAddrVA := memoryAddrVA
    bundle.memoryAddrPA := memoryAddrPA
    bundle.target := target
    bundle.inst := inst
    bundle.memoryType := memoryType
    bundle.memorySize := memorySize
    bundle.branchType := branchType
    bundle.branchTaken := branchTaken
    bundle.InstID := InstID
    bundle
  }

  def readRaw(raw: UInt)(implicit p: Parameters): TraceInstrInnerBundle = {
    val m = Wire(new TraceInstrInnerBundle)
    var offset = 0
    m.getElements.foreach( elt => {
      val width = elt.getWidth
      elt := raw(offset + width - 1, offset)
      offset += width
    })
    assert(offset == raw.getWidth,
      s"ERROR in TraceInstrInnerBundle, fromOuterRaw offset not match, expect ${raw.getWidth}, got ${offset}")

    m
  }
}

class TraceRecvInfo(implicit p: Parameters) extends TraceBundle {
  val instNum = UInt(log2Ceil(IBufferEnqueueWidth + 1).W)
}

class TraceInstrBundle(implicit p: Parameters) extends TraceInstrInnerBundle {
  val isWrongPath              = Bool()
  val hasTriggeredExuRedirect  = Bool()
  def hasException = exception =/= 0.U
  def isForceJump  = exception(7)
  def initMoreFromRaw() = {
    isWrongPath             := false.B
    hasTriggeredExuRedirect := false.B
  }
}

object TraceInstrBundle {
  def apply(rawInst: TraceInstrInnerBundle)
           (implicit p: Parameters): TraceInstrBundle = {
    val bundle = Wire(new TraceInstrBundle)
    rawInst.elements.foreach { case (name, elt) =>
      bundle.elements(name) := elt
    }
    bundle.initMoreFromRaw()
    bundle
  }
}

/** Trace instructions indexed by their halfword position in the prediction window. */
class TracePositionView(implicit p: Parameters) extends TraceBundle {
  val insts      = Vec(trtl.TracePredictWidth, Valid(new TraceInstrBundle))
  val traceRange = UInt(trtl.TracePredictWidth.W)
}

/** One instruction after removing holes from the position-indexed representation. */
class TraceCompactEntry(implicit p: Parameters) extends TraceBundle {
  val valid          = Bool()
  val traceInfo      = new TraceInstrBundle
  val instrEndOffset = UInt(FetchBlockInstOffsetWidth.W)
  val selectBlock    = Bool()
  val isPrevEndHalfRvi  = Bool()
  val isCrossBlockInstr = Bool()
}

/** A compact instruction enriched by TracePreDecoder. */
class TraceDecodedEntry(implicit p: Parameters) extends TraceBundle {
  val compact    = new TraceCompactEntry
  val pd         = new PreDecodeInfo
  val jumpOffset = PrunedAddr(VAddrBits)
}

/** Complete combinational result of TraceAlignParallel for one S2 packet. */
class TraceAlignResult(implicit p: Parameters) extends TraceBundle {
  val position       = new TracePositionView
  val compact        = Vec(IBufferEnqueueWidth, new TraceCompactEntry)
  val candidateCount = UInt(log2Ceil(IBufferEnqueueWidth + 1).W)
  val concede2Bytes  = Bool()
  val traceForceJump = Bool()
}

class TraceFetchBlockPredictInfo(implicit p: Parameters) extends TraceBundle {
  val valid         = Bool()
  val startAddr     = UInt(VAddrBits.W)
  val nextStartAddr = UInt(VAddrBits.W)
  val instRange     = UInt(trtl.TracePredictWidth.W)
  val size          = UInt(log2Ceil(trtl.TracePredictWidth + 1).W)
  val ftqOffset     = Valid(UInt(log2Ceil(trtl.TracePredictWidth).W))
}

class TracePredictInfo(implicit p: Parameters) extends TraceBundle {
  val block = Vec(FetchPorts, new TraceFetchBlockPredictInfo)
}

class TraceCheckerResp(implicit p: Parameters) extends TraceBundle {
  val traceRange = UInt(trtl.TracePredictWidth.W)
}

class TracePCMatchBundle(implicit p: Parameters) extends TraceBundle {
  val pcVA  = Output(UInt(VAddrBits.W))
  val found = Input(Bool())
}

class TraceDriverOutput(implicit p: Parameters) extends TraceBundle {
  val block      = Output(Bool())
  val recv       = ValidIO(new TraceRecvInfo())
  val endWithCFI = Output(Bool())
}

class TraceATSBundle(implicit p: Parameters) extends TraceBundle {
  val valid = Input(Bool())
  val vaddr = Input(UInt(64.W))
  val paddr = Output(UInt(64.W))
  val hit = Output(Bool())
}

class TraceAlignToIFUCutIO(implicit p: Parameters) extends TraceBundle {
  val debug_valid     = Input(Bool())
  val traceInsts      = Input(Valid(Vec(trtl.TracePredictWidth, new TraceInstrBundle())))
  val predictInfo     = Input(new TracePredictInfo)
  val lastHalfValid   = Input(Bool())
  val cutInsts        = Output(Vec(trtl.TracePredictWidth, Valid(new TraceInstrBundle)))
  val traceRange      = Output(UInt(trtl.TracePredictWidth.W))
  val pdValid         = Output(UInt(trtl.TracePredictWidth.W))
  val traceForceJump  = Output(Bool())
  val traceRangeTaken2B = Output(Bool())
  val instRangeTaken2B  = Output(Bool())
  val result             = Output(new TraceAlignResult)
}

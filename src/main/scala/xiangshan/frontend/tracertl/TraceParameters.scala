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

import org.chipsalliance.cde.config.{Field, Parameters}

case object TraceRTLParamKey extends Field[TraceRTLParameters]

case class TraceRTLParameters
(
  // Core widths
  TracePredictWidth: Int = 32,   // 64B IFU fetch block in 2-byte instruction slots
  TraceFetchWidth: Int = 32,     // batch read size from DPI-C per refill

  // Trace data field widths
  TraceVAddrWidth: Int = 50,     // virtual address width
  TracePAddrWidth: Int = 48,     // physical address width
  TraceInstCodeWidth: Int = 32,  // instruction code width
  TraceInstIDWidth: Int = 64,    // instruction ID width

  // Control flags
  TraceOverrideTarget: Boolean = true, // use trace target instead of PC+offset for branch targets
  TraceSoftL1TLB: Boolean = false,     // bypass L1 TLB misses with TraceRTL address translation
  TraceSoftL1TLBCheck: Boolean = false, // compare native L1 TLB hits against TraceRTL translation

  // Wrong-path emulation: on a trace pc mismatch, keep the IFU running with
  // right-path trace instructions tagged isWrongPath until a redirect.
  TraceEnableWrongPathEmu: Boolean = true,           // build the hardware; runtime switch is Constantin "TraceWrongPathEmu"
  TraceWrongPathEmuWhenConvergence: Boolean = false, // only enter when the fetch pc is found in the trace buffer
  TraceWrongPathEmuTrain: Boolean = false,           // let wrong-path BJU resolves train the BPU
  // Real wrong-path emulation: place, at the predicted fetch PCs, instances of
  // the static instructions found there in the trace (C++ oracle), and let
  // wrong-path BJU/JU resolve, train the BPU and redirect the frontend.
  // Runtime env: TRACERTL_WP_DISABLE, TRACERTL_WP_PAST_ONLY, TRACERTL_WP_IFU_CHECK.
  TraceWrongPathEmuReal: Boolean = true
) {
  def TraceBufferSize = TraceFetchWidth * 4 // 64-entry circular buffer
  def TraceWrongPathReal = TraceEnableWrongPathEmu && TraceWrongPathEmuReal
  // Real wrong-path instructions sit at their own fetch positions, so their
  // resolves are as meaningful to the BPU as in execution-driven simulation.
  def TraceWrongPathTrain = TraceWrongPathEmuTrain || TraceWrongPathReal
}

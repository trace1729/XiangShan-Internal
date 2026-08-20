# GSIM TraceRTL Integration

## Result

The GSIM workspace is `/nfs/home/gongkaichen/tracertl/OpenXiangshan-trace-verify-gsim`.
It was copied from `OpenXiangshan-trace-verify` without the `build` directory, as requested.

The working implementation is commit `ddb77f4de` (`fix(gsim): add TraceRTL helpers`).
The final clean build completed in 28m59.818s and produced a 63 MiB
`build/gsim-compile/emu`.

The requested CoreMark run completed with:

```text
Core-0 instrCnt = 3,196,820, cycleCnt = 1,058,640, IPC = 3.019742
Guest cycle spent: 1,058,644
Host time spent: 938,023ms
```

Full output is in `coremark_out_gsim`; `coremark_err_gsim` contains the normal
performance-counter dump and no hardware assertion failure.

## Reproduction

Create the sibling workspace without copying build products:

```bash
rsync -a --delete --delete-excluded --exclude=/build/ \
  /nfs/home/gongkaichen/tracertl/OpenXiangshan-trace-verify/ \
  /nfs/home/gongkaichen/tracertl/OpenXiangshan-trace-verify-gsim/
cd /nfs/home/gongkaichen/tracertl/OpenXiangshan-trace-verify-gsim
```

The generated build is about 7 GiB and exceeded the home quota. Keep Mill's
`out` directory in the repository because its VCS plugin must discover `.git`,
but place `build` on local scratch storage:

```bash
mkdir -p /tmp/OpenXiangshan-trace-verify-gsim-build
ln -s /tmp/OpenXiangshan-trace-verify-gsim-build build
```

The GSIM generation rule removes `build/gsim-compile/model` before regenerating
after a FIR change. GSIM does not delete obsolete numbered C++ partitions, so a
clean output directory is required to prevent stale definitions from reaching
the linker.

Build with the requested command:

```bash
export NOOP_HOME=$(pwd) && export NEMU_HOME=$(pwd)/ready-to-run && \
set -o pipefail && \
time make gsim GSIM=1 -j16 EMU_THREADS=8 TRACERTL_MODE=1 EMU_TRACE=1 \
  EMU_TRACE=fst WITH_DRAMSIM3=1 REMOTE=node039 EMU_OPTIMIZE=-O3 \
  2>&1 | tee make-emu.log
```

Run CoreMark with:

```bash
./build/emu --no-diff --gen-paddr \
  --tracertl-file ~/tracertl/nemu-internal/coremark.zstd \
  --enable-fork 2> coremark_err_gsim | tee coremark_out_gsim
```

## Failures And Fixes

1. The first link failed with undefined references to `TraceCollectorHelper`,
   `TraceDriveCollectorHelper`, and `TraceATSHelper`. These helpers had inline
   SystemVerilog DPI implementations but no GSIM C++ external-module bodies.
   `TraceHelper.scala` now emits all three through `createCppExtModule`.

2. The first ATS C++ wrapper returned translation results combinationally,
   while the SystemVerilog helper returned them one cycle later. CoreMark then
   failed at instruction 1 with `illegal MemAttr asserted: Device`. The address
   and hit registers now live in `TraceFakeMMU`, so both backends share the same
   one-cycle timing contract. CoreMark subsequently completed at IPC 3.019742.

3. A build failed with `No space left on device`. Moving `build` to `/tmp`
   fixed the quota issue. Moving `out` to `/tmp` does not work: Mill's
   VCS-version task runs under the resolved path and cannot find `.git`.

4. After the ATS change, GSIM emitted 379 partitions while 381 older files
   remained. The link failed with multiple definitions of `SSimTop::step()`.
   Cleaning `build/gsim-compile/model` before regeneration fixed it.

5. Commit `89e3005d` added `TraceSatpPpnHelper` and
   `TraceDynPageTableHelper` with sequential inline SystemVerilog DPI bodies
   but without GSIM C++ external-module bodies. GSIM stopped in `computeExtMod`
   with `Implement ME!` and `Assertion '0' failed`. Both helpers now use
   GSIM-supported combinational external-module bodies registered through
   `createCppExtModule`. The page-table helper also flattens its eight-word
   output vector because GSIM does not support array ports on external modules;
   cycle timing is implemented explicitly in the surrounding Chisel modules.

6. The original `gsim | tee gsim-gen-cpp.log` recipe returned `tee`'s status,
   hiding a GSIM abort and allowing stale objects to link into a new `emu`.
   Generation now captures stderr and runs the pipeline with `pipefail`, so an
   abort stops the build before compilation or linking.

The retained diagnostic logs are `make-emu-fail-link.log`,
`make-emu-fail-space.log`, and `make-emu-fail-stale-model.log`.

## Warnings

GSIM reports unsupported external/equality-derived JTAG debug clocks, generated
assertion strings with more format conversions than arguments, and narrowing of
the unused `Mem1R1WHelper` RAM-size argument. None prevented the successful
CoreMark run. These warnings should still be reevaluated for JTAG use or other
workloads.

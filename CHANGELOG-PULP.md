# Changelog
All notable changes to the `pulp` branch of this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased (pulp-v4)
### Added
- Configurations `cv64a6_imafdch_sv39_hpdcache_wb` (the testbench default),
  `cv64a6_imafdc_sv39_hpdcache_wb` (without H) and `cv64a6_splus` (dual issue)
- Performance probes from the issue stage (`perf_probes_o` of `cva6_top`)
- Testbench: Spike co-simulation and generated stimulus; benchmarks (CoreMark, Embench-IoT,
  RaiderSTREAM, Dhrystone, pointer chase, Lua) with expected cycles, sweeps and issue-slot accounting
  from the performance probes
- CI: benchmark check for pull requests, co-simulation in the GitLab pipeline

### Changed
- Rebase onto openhwfoundation:cva6:main#d808c36e7 (2026-11-17): `cva6_top` and a pipeline
  separate from the memory subsystem, the HPDcache as data and instruction cache, configurations under
  `config/target/`
- Bender sources follow `core/Flist.cva6`; bump axi 0.39.11, add obi 0.1.7
- FPU: cvfpu 532f0f8 of its develop branch, without the PULP div/sqrt unit
- OBI re-vendored at upstream v0.1.7 (openhwfoundation/cva6#3589)
- HPDcache from pulp-platform/cv-hpdcache `ez/bufferable`: the pinned HPDcache with the change the main
  flow applies to it as a patch
- SRAMs mapped onto `tc_sram`; FPGA targets use the same SRAMs
- Testbench: target `cv64a6_imafdch_sv39_hpdcache_wb`, instantiates `cva6_top`; CLINT 0.3.0 on APB
  and the platform registers in SystemRDL; the core's tuning knobs as testbench parameters;
  Verilator 5.052
- PMP NAPOT masks decoded when pmpaddr is written
- Frontend: a redirect's target is fetched from the registered next PC
- I$: a killed fetch is aborted, and fetching goes on while a killed fetch's refill is outstanding
- Stores complete on their grant with the HPDcache

### Removed
- CLIC and vCLIC, with their configurations
- The legacy write-through and write-back caches and the work built on them: self-invalidation coherence,
  their fixes and configurations
- fence.t
- RVV `acc_mmu` interface (absent on main)

### Fixed
- CSR: guest address on interrupts, pmpaddr NAPOT readback, U/VU-mode counter enables, stvec/vstvec
  alignment, per-register FIOM, flush after writing interrupt-related CSRs
- Interrupt priority across privilege modes
- MMU: PTW access-fault type, TLB hit assertion, TLB miss counter
- LSU: LR fault types, CBO permissions, AMO buffer overflow, full address in the non-idempotent load
  check, store response after a store done on its grant
- Frontend: fetch fault attributed to its own request
- Issue and execute: scalar crypto instruction bits from the port that issues to the unit, branch unit
  exception timing field
- Caches: undefined-data assertions, AMO response formatting
- RVFI: RV64 interrupt bit, VS-mode ecall, CSR zero-extension, sie/sip under H

## pulp-v3.0.0 - 2026-10-14
### Added
- Standalone testbench `test/` (Verilator, QuestaSim) with riscv-tests, riscv-hyp-tests and the RISC-V architectural tests
- RVFI instruction tracer for the testbench, with a Perfetto timeline from binary traces
- RVFI: branch mispredictions, virtualisation bit, hypervisor and CLIC CSRs
- Configurations `cv64a6_imafdchxhclic_sv39_wb`, `cv64a6_imafdchxhclic_sv39_hpdcache_wb` and `cv64a6_splus`
- `util/verible.sh`; CI: Slang elaboration of the PULP configurations

### Changed
- Rebase onto openhwgroup:cva6:master#81245a47 (2026-09-15)
- FPU: openhwgroup/cvfpu (3eb6afe)
- Bump axi 0.39.10, common_cells 1.40.0, tech_cells_generic 0.2.14; track `Bender.lock`
- Disable CV-X-IF in `cv64a6_imafdc_sv39*` and `cv64a6_imafdch_sv39*`
- Target `cv64a6_imafdchsclic_sv39_hpdcache` is now `cv64a6_imafdchsclic_sv39_hpdcache_wb`
- CVA6's built-in instruction trace can be left out with `CVA6_NO_LEGACY_TRACE`

### Fixed
- CSR: trap value and guest address on interrupts, pmpaddr NAPOT readback, U/VU-mode counter enables,
  stvec/vstvec alignment, per-register FIOM, flush after writing interrupt-related CSRs
- Interrupt priority across privilege modes; PMP check on instruction guest-page faults
- MMU: PTW access-fault type, TLB hit assertion, TLB miss counter; LSU: LR fault types, CBO permissions
- Caches: nbdcache miss count, user-bit selection, undefined-data assertions, load error from the WB/WT caches
- RVFI: per-access paddr, RV64 interrupt bit, VS-mode ecall, CSR zero-extension, sie/sip under H, pmpaddr,
  CSR views in CLIC mode
- CLIC: guest access to stvt/sintstatus, vstvt vector base
- fence.t no longer fires when cancelled; ID stage QuestaSim elaboration

### Known issues
- CLIC: selective hardware vectoring supports interrupt ids below 64; no xnxti or xscratchcsw
- fence.t is not supported with the HPDcache

## pulp-v2.0.0 - 2025-07-17
### Added
- vCLIC support

### Changed
- Rebase onto openhwgroup:cva6:master#2d411b2d (2025-02-20)
- Apply upstream patches adding superscalar configuration options
- Add HPDCache wrappers
- FPU: Upgrade to pulp-v0.2.3
- fence.t: Support 32-bit XLEN

### Fixed
- Apply upstream bugfixes mostly regarding H extension
- CLIC WFI resume condition and trap vector base

## pulp-v1.0.0 - 2024-02-25
### Changed
- Rebase onto openhwgroup:cva6:master#68f952b4 (2024-02-23)

## pulp-v0.4.3 - 2023-08-12
### Changed
- `tech_cells_generic`: Bump v0.2.12

## pulp-v0.4.2 - 2023-08-11
### Changed
- frontend: Do not speculate beyond xret to avoid spurious icache refills
- coherence: Pack Valid/Dirty SRAM

## pulp-v0.4.1 - 2023-08-03
### Fixed
- CLIC controller: Fix inferred latch

## pulp-v0.4.0 - 2023-07-25
### Added
- Self-invalidation coherence

### Changed
- Upgrade CLIC to v2.0.0
- Upgrade AXI RISC-V AMO adapter to v0.8.0

### Fixed
- Fix CLIC interrupt behaviour

## pulp-v0.3.1 - 2023-05-10
### Changed
- Upgrade FPU to pulp-v0.1.1

### Fixed
- Bender.yml: Fix file list for target cv64a6_imafdcsclic_sv39
- cva6_icache_axi_wrapper: Fix negative repetition multiplier for paddr width > AxiAddrWidth
- cva6_clic_controller: Resolve inferred latch by adding default case
- Bender.lock: Update

## pulp-v0.3.0 - 2023-05-03
### Added
- Initial Hypervisor extension (incl. regression tests)

### Changed
- Upgrade AXI to v0.39.0-beta.2

## pulp-v0.2.2 - 2023-04-27
### Fixed
- `Bender.yml`: Specify version for tech_cells_generic
- `Makefile`: Fix vopt for recent questasim versions

## pulp-v0.2.1 - 2023-04-17
### Fixed
- `Bender.yml`: Add cv64a6_imafdcsclic_sv39 target

## pulp-v0.2.0 - 2023-04-08
### Added
- CLIC support

### Fixed
- Verilator 5 compatibility
- `Bender.yml`: Update paths
- Minor tool compatibility issues

## pulp-v0.1.1 - 2023-03-25
### Fixed
- Typo in `Bender.yml`

## pulp-v0.1.0 - 2023-03-24
### Added
- `fence.t` instruction to support timing channel prevention

### Changed
- Bumped `master`
- Updated `Bender.yml`

### Fixed
- Questasim flow
- `icache_axi_wrapper`: Signal widths

// Copyright 2026 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Description: Self-contained RV32/RV64 IMAFDC + Zicsr disassembler used by the
//              CVA6 instruction tracer. Deliberately has no dependency on spike,
//              fesvr or any other external library: it decodes raw encodings and
//              formats them in the column layout of the legacy instr_tracer.

#ifndef CVA6TB_TRACER_RV_DISASM_H
#define CVA6TB_TRACER_RV_DISASM_H

#include <cstdint>
#include <string>

namespace cva6tb_tracer {

// Which register file an operand refers to.
enum reg_kind_t : uint8_t { REG_NONE = 0, REG_GPR = 1, REG_FPR = 2 };

struct reg_ref_t {
  reg_kind_t kind;
  uint8_t    idx;
};

// Everything the tracer needs to know about one instruction, derived purely
// from its encoding.
struct decoded_t {
  std::string text;      // mnemonic + operands, legacy column layout
  reg_ref_t   dst;       // destination register (kind == REG_NONE if none)
  reg_ref_t   src[3];    // source registers, in the order the legacy tracer printed them
  unsigned    n_src;
  bool        is_load;
  bool        is_store;
  bool        is_amo;
  bool        is_csr;
  bool        is_branch;  // conditional branch
  bool        is_jump;    // unconditional jump, direct or register
  uint16_t    csr_addr;  // valid if is_csr
  unsigned    mem_bytes; // access width in bytes, valid if is_load/is_store/is_amo
};

// Optional extensions whose encodings collide with something else, so the
// decoder cannot tell them apart on its own.
enum rv_ext_t : unsigned {
  RV_EXT_ZCMT = 1u << 0,  // cm.jt / cm.jalt, which share the c.fsdsp encoding
};

// Decode a single instruction. Compressed instructions are passed in the low
// 16 bits with the upper bits zeroed (as RVFI reports them). `xlen` is 32 or 64
// and only affects encodings that differ between RV32 and RV64 (c.jal/c.addiw,
// c.flw/c.ld, c.fsw/c.sd, c.flwsp/c.ldsp, c.fswsp/c.sdsp). `ext` is a mask of
// rv_ext_t; Zcb needs no flag because it only uses otherwise reserved encodings.
decoded_t rv_decode(uint32_t insn, int xlen, unsigned ext = 0);

// ABI register names ("ra", "sp", "a0", "ft0", ...).
const char *rv_gpr_name(unsigned idx);
const char *rv_fpr_name(unsigned idx);

// CSR name, or "0x___" for addresses we do not know.
std::string rv_csr_name(uint16_t addr);

// Architectural trap cause as a string, e.g. "ILLEGAL_INSTR". `cause` is the
// raw mcause/scause value: bit XLEN-1 set marks an interrupt.
std::string rv_cause_name(uint64_t cause, int xlen);

}  // namespace cva6tb_tracer

#endif  // CVA6TB_TRACER_RV_DISASM_H

/* Linker script for generated random programs.
 *
 * They are self-contained -- no startup code, no library -- so this only has to
 * place them in DRAM where the boot ROM jumps, and give the data and scratch
 * sections somewhere to live. See test/hw/cva6tb_pkg.sv for the memory map.
 *
 * SPDX-License-Identifier: Apache-2.0 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY { ram (rwx) : ORIGIN = 0x80000000, LENGTH = 0x10000000 }

SECTIONS {
  .text.init 0x80000000 : { *(.text.init) } > ram
  .text  . : { *(.text) *(.text.*) } > ram
  .data  . : { *(.data) *(.data.*) *(.sdata) *(.sdata.*) } > ram
  .bss   . : { *(.bss) *(.bss.*) *(.sbss) *(.sbss.*) *(COMMON) } > ram
  _end = .;
}

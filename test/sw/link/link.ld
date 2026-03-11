/* Copyright 2026 ETH Zurich and University of Bologna. */
/* Licensed under the Apache License, Version 2.0, see LICENSE for details. */
/* SPDX-License-Identifier: Apache-2.0 */

ENTRY(_start)

MEMORY {
  dram (rwx) : ORIGIN = 0x80000000, LENGTH = 8M
}

SECTIONS {

  /* Keep binaries lean */
  /DISCARD/ : { *(.riscv.attributes) *(.comment) }

  /* Global and stack pointer */
  __global_pointer$ = ADDR(.misc) + SIZEOF(.misc) / 2;
  __stack_pointer$  = ORIGIN(dram) + LENGTH(dram) - 8;

  .text : {
    *(.text._start)
    *(.text)
    *(.text.*)
  } > dram

  .misc : ALIGN(16) {
    *(.rodata)
    *(.rodata.*)
    *(.data)
    *(.data.*)
    *(.srodata)
    *(.srodata.*)
    *(.sdata)
    *(.sdata.*)
  } > dram

  .bss : ALIGN(16) {
    __bss_start = .;
    *(.bss)
    *(.bss.*)
    *(.sbss)
    *(.sbss.*)
    __bss_end = .;
  } > dram

  .bulk : ALIGN(16) {
    *(.bulk)
    *(.bulk.*)
  } > dram

  /* Relevant addresses */
  __base_regs    = 0x10000000;
  __base_console = 0x10001000;
  __base_clic    = 0x10040000;
  __base_dram    = ORIGIN(dram);
  __base_heap    = ADDR(.bulk) + SIZEOF(.bulk);

}

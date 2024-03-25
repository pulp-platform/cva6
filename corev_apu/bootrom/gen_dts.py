#!/usr/bin/env python3
# Copyright 2023 ETH Zurich, University of Bologna.
#
# Copyright and related rights are licensed under the Solderpad Hardware
# License, Version 0.51 (the "License"); you may not use this file except in
# compliance with the License.  You may obtain a copy of the License at
# http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
# or agreed to in writing, software, hardware and materials distributed under
# this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

from string import Template
import argparse
import os.path
import sys
import binascii


parser = argparse.ArgumentParser(description='Generate dts file')
parser.add_argument('ncores', metavar='ncores', nargs=1,
                   help='number of cores')

args = parser.parse_args()
ncores = int(args.ncores[0])

if ncores < 1:
    print("ncores must be 1 or greater - assuming 1...")
    ncores = 1

filename = "ariane.dts"

header = f"""\
// Copyright 2023 ETH Zurich, University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// File: {filename}
//
// Description: Auto-generated code
/dts-v1/;

/ {{
  #address-cells = <2>;
  #size-cells = <2>;
  compatible = "eth,ariane-bare-dev";
  model = "eth,ariane-bare";
  cpus {{
    #address-cells = <1>;
    #size-cells = <0>;
    timebase-frequency = <32768>; // 32.768 kHz
"""

cpus = ""
for c in range(ncores):
    cpus = cpus + "    CPU" + str(c) + ": cpu@" + str(c) + " {\n"
    cpus = cpus + "      clock-frequency = <50000000>; // 50 MHz\n"
    cpus = cpus + "      device_type = \"cpu\";\n"
    cpus = cpus + "      reg = <" + str(c) + ">;\n"
    cpus = cpus + "      status = \"okay\";\n"
    cpus = cpus + "      compatible = \"eth, ariane\", \"riscv\";\n"
    cpus = cpus + "      riscv,isa = \"rv64imafdc\";\n"
    cpus = cpus + "      mmu-type = \"riscv,sv39\";\n"
    cpus = cpus + "      tlb-split;\n"
    cpus = cpus + "      // HLIC - hart local interrupt controller\n"
    cpus = cpus + "      CPU" + str(c) + "_intc: interrupt-controller {\n"
    cpus = cpus + "        #interrupt-cells = <1>;\n"
    cpus = cpus + "        interrupt-controller;\n"
    cpus = cpus + "        compatible = \"riscv,cpu-intc\";\n"
    cpus = cpus + "      };\n"
    cpus = cpus + "    };\n"

footer = """\
  };
  memory@80000000 {
    device_type = "memory";
    reg = <0x0 0x80000000 0x0 0x10000000>;
  };
  soc {
    #address-cells = <2>;
    #size-cells = <2>;
    compatible = "eth,ariane-bare-soc", "simple-bus";
    ranges;
    clint@2000000 {
      compatible = "riscv,clint0";
      interrupts-extended = <&CPU0_intc 3 &CPU0_intc 7>;
      reg = <0x0 0x2000000 0x0 0xc0000>;
      reg-names = "control";
    };
    // PLIC needs to be disabeld for tandem verification
    // PLIC0: interrupt-controller@c000000 {
    //   #address-cells = <0>;
    //   #interrupt-cells = <1>;
    //   compatible = "sifive,plic-1.0.0", "riscv,plic0";
    //   interrupt-controller;
    //   interrupts-extended = <&CPU0_intc 11 &CPU0_intc 9>;
    //   reg = <0x0 0xc000000 0x0 0x4000000>;
    //   riscv,max-priority = <7>;
    //   riscv,ndev = <2>;
    // };
    // Specifying the interrupt controller in the devicetree is not necessary.
    // Furthermore, the IRQ 65535 will cause a `hwirq 0xffff is too large` during
    // Linux boot (occured with mainline linux 5.14.0).
    // debug-controller@0 {
    //   compatible = "riscv,debug-013";
    //   interrupts-extended = <&CPU0_intc 65535>;
    //   reg = <0x0 0x0 0x0 0x1000>;
    //   reg-names = "control";
    // };
    uart@10000000 {
      compatible = "ns16550a";
      reg = <0x0 0x10000000 0x0 0x1000>;
      clock-frequency = <50000000>;
      current-speed = <115200>;
      // interrupt-parent = <&PLIC0>;
      interrupts = <1>;
      reg-shift = <2>; // regs are spaced on 32 bit boundary
      reg-io-width = <4>; // only 32-bit access are supported
    };
    timer@18000000 {
      compatible = "pulp,apb_timer";
      interrupts = <0x00000004 0x00000005 0x00000006 0x00000007>;
      reg = <0x00000000 0x18000000 0x00000000 0x00001000>;
      // interrupt-parent = <&PLIC0>;
      reg-names = "control";
    };
  };
};
"""

""" Generate device tree file
"""
with open(filename, "w") as f:
    f.write(header)
    f.write(cpus)
    f.write(footer)
    f.close()

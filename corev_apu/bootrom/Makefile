bootrom_img = bootrom.img bootrom.sv

RISCV_GCC?=riscv64-unknown-elf-gcc
RISCV_OBJCOPY?=riscv64-unknown-elf-objcopy
DTB=ariane.dtb
DTS=ariane.dts
PYTHON=python

CORES ?= 1

all: $(bootrom_img)

%.img: %.bin
	dd if=$< of=$@ bs=128

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $< $@

%.elf: %.S linker.ld $(DTB)
	$(RISCV_GCC) -Tlinker.ld -march=rv32i -mabi=ilp32 $< -nostdlib -static -Wl,--no-gc-sections -o $@

%.dts:
	$(PYTHON) ./gen_dts.py $(CORES)

%.dtb: $(DTS)
	dtc -I dts $< -O dtb -o $@

%.sv: %.img
	$(PYTHON) ./gen_rom.py $<

clean:
	rm -f $(bootrom_img) $(DTB) $(DTS)

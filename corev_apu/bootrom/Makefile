bootrom_img = bootrom.img bootrom.sv

RISCV_GCC?=riscv-none-elf-gcc
RISCV_OBJCOPY?=riscv-none-elf-objcopy
DTB=ariane.dtb
PYTHON=python

all: $(bootrom_img)

%.img: %.bin
	dd if=$< of=$@ bs=128

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $< $@

%.elf: %.S linker.ld $(DTB)
	$(RISCV_GCC) -Tlinker.ld -march=rv32i_zicsr -mabi=ilp32 $< -nostdlib -static -Wl,--no-gc-sections -o $@

%.dtb: %.dts
	dtc -I dts $< -O dtb -o $@

%.sv: %.img
	$(PYTHON) ./gen_rom.py $<

clean:
	rm -f $(bootrom_img) $(DTB)

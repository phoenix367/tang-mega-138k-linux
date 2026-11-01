# Linux for the Gowin AE350 SoC on the Sipeed Tang Mega 138K Pro Dock

Linux 6.12 (rv32) for the Andes A25-based AE350 hard SoC inside the GW5AST-138B, with the
drivers for the custom cores of the
[tang-mega-138K-Pro-demo](https://github.com/phoenix367/tang-mega-138K-Pro-demo) gateware.

History: the first commit is a byte-identical import of the upstream `v6.12` tree (commit
`adc218676eef25575469234709c2d87185ca223a`), not upstream's history, because the full
kernel history cannot be pushed to GitHub in one piece. Everything board-specific is in one
commit on top of it; the same changes are kept as a patch series in the gateware
repository's `PATCHES/` directory with a longer write-up.

## What is changed

- **Soft atomics** (`arch/riscv/include/asm/soft_atomic.h`, `atomic.h`, `bitops.h`,
  `cmpxchg.h`, `futex.h`): the SoC has no coherency manager, so AMO and LR/SC to DRAM hang.
  Every read-modify-write is an interrupt-disabled plain load/store. Single hart only
  (`CONFIG_SMP=n`; SMP's qspinlock deadlocks with this scheme).
- **A25 L1 cache maintenance** (`arch/riscv/mm/cmo-andes-a25.c`): non-standard cache ops
  (writeback / invalidate) through the AndeStar V5 user CCTL CSRs, for non-coherent DMA.
  Gated on `andestech,a25`.
- **Page-table writeback** (`arch/riscv/include/asm/a25-ptwb.h`, hooks in `pgtable.h`,
  `pgalloc.h`, `mm/pgtable.c`): the A25 page-table walker is not cache-coherent, so every
  page-table store is written back to DRAM. Required with the gateware's overlapped DDR
  bridge mode.
- **Reset** (`arch/riscv/kernel/reset.c`): clean reboot / poweroff / halt through SBI.
- **Drivers** for the gateware cores, each `gowin,*` in the device tree:
  `drivers/net/ethernet/gowin/gw_tse.c` (gigabit MAC with the custom DMA engine),
  `drivers/video/fbdev/gowin-lcdfb.c` (800x480 RGB panel), `sound/drivers/gowin-pt8211.c`
  (ALSA, DMA or PIO), `drivers/hwmon/gowin-adc.c` (die temperature, rails),
  `drivers/hwmon/gowin-fan.c` (PWM + tacho), `drivers/mmc/host/gowin-sdhost.c` (native
  4-bit SD host), `drivers/spi/spi-gowin.c` (the earlier SPI-mode SD route, unused now).
- `arch/riscv/configs/tang_mega_138k_defconfig`: the working configuration. Its
  `CONFIG_INITRAMFS_SOURCE` points at the gateware repository's `linux/rootfs` and
  `linux/initramfs_nodes.list` by absolute path; adjust for your checkout.

## Build

```
LGNU=/mnt/data/xilinx/2025.2/gnu/riscv/linux_toolchain/lin32/bin/riscv32-amd-linux-gnu-
make ARCH=riscv CROSS_COMPILE=$LGNU tang_mega_138k_defconfig
make ARCH=riscv CROSS_COMPILE=$LGNU -j$(nproc) Image
```

The device tree (`linux/ae350-linux.dts`) and the rootfs live in the gateware repository.
`arch/riscv/boot/Image` is wrapped into an OpenSBI `fw_payload` (see the
tang-mega-138k-opensbi fork) and flashed at 0x800000; the payload must stay under 8 MB.

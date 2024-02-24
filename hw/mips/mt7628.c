/*
 * QEMU/mt7628 emulation
 *
 * Copyright (c) 2023 Lu Hui <luhux76@gmail.com>
 * some code from mipssim:
 * ./mipssim.c
 * Copyright (c) 2007 Thiemo Seufer
 * some code from linux kernel:
 * arch/mips/boot/dts/ralink/mt7628a.dtsi
 * arch/mips/ralink/bootrom.c
 * Copyright (C) 2013 John Crispin <john@phrozen.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "hw/clock.h"
#include "hw/block/flash.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/mips/bios.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/char/serial.h"
#include "hw/misc/unimp.h"
#include "hw/mips/mt7628.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/mt7628-spi.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"
#include "sysemu/reset.h"

/* interrupt table */
enum {
    IRQ_UART0 = 20,
    IRQ_UART1 = 21,
    IRQ_UART2 = 22,
    IRQ_EHCI = 18
};

/* Memory map */
const hwaddr mt7628_memmap[] = {
    [MT7628_DEV_DDR]          = 0x00000000,
    [MT7628_DEV_SYSCTRL]      = 0x10000000,
    [MT7628_DEV_INTC]         = 0x10000200,
    [MT7628_DEV_UART0]        = 0x10000C00,
    [MT7628_DEV_UART1]        = 0x10000D00,
    [MT7628_DEV_UART2]        = 0x10000E00,
    [MT7628_DEV_EHCI]         = 0x101C0000,
    [MT7628_DEV_OHCI]         = 0x101C1000,
    [MT7628_DEV_SPI]          = 0x10000B00,
    [MT7628_DEV_ETH]          = 0x10100000,
    [MT7628_DEV_BOOTROM]      = 0x10118000,
    [MT7628_DEV_FLASH_DIRECT] = 0x1C000000,
};

struct mt7628Unimplemented {
    const char *device_name;
    hwaddr base;
    hwaddr size;
} unimplemented[] = {
    { "timer",        0x10000100, 0xFF },
    { "memc",         0x10000300, 0xFF },
    { "rbus",         0x10000400, 0xFF },
    { "mips-cnt",     0x10000500, 0xFF },
    { "gpio",         0x10000600, 0xFF },
    { "spi-slave",    0x10000700, 0xFF },
    { "i2c",          0x10000900, 0xFF },
    { "i2s",          0x10000A00, 0xFF },
    { "hsuart0",      0x10000C20, 0x40 },
    { "hsuart1",      0x10000D20, 0x40 },
    { "hsuart2",      0x10000E20, 0x40 },
    { "rgctl",        0x10001000, 2 * KiB },
    { "pcm",          0x10002000, 2 * KiB },
    { "dma",          0x10002800, 2 * KiB },
    { "aes",          0x10004000, 4 * KiB },
    { "pwm",          0x10005000, 4 * KiB },
    { "usb-phy",      0x10120000, 32 * KiB },
    { "sdxc",         0x10130000, 32 * KiB },
    { "pcie",         0x10140000, 256 * KiB },
    { "wlan",         0x10300000, 1 * MiB },
    { "pcie-direct",  0x20000000, 256 * MiB },
};

static void mt7628_init(Object *obj)
{
    mt7628State *s = MT7628(obj);
    s->memmap = mt7628_memmap;

    object_initialize_child(obj, "sysctrl", &s->sysctrl, TYPE_MT7628_SYSCTRL);
    object_initialize_child(obj, "intc", &s->intc, TYPE_MT7628_INTC);
    object_initialize_child(obj, "spi", &s->spi, TYPE_MT7628_SPI);
    object_initialize_child(obj, "eth", &s->eth, TYPE_MT7628_ETH);
    if (machine_usb(current_machine)) {
        object_initialize_child(obj, "ehci", &s->ehci, TYPE_PLATFORM_EHCI);
        object_initialize_child(obj, "ohci", &s->ohci, TYPE_SYSBUS_OHCI);
    }
}

static void mt7628_realize(DeviceState *dev, Error **errp)
{
    mt7628State *s = MT7628(dev);
    int i;

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->intc), errp)) {
        return;
    }
    CPUMIPSState *env = &s->cpu->env;

    /* interrupt control */
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0, s->memmap[MT7628_DEV_INTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0,
                       env->irq[MT7628_CPU_IRQ_INTC]);
    qdev_pass_gpios(DEVICE(&s->intc), dev, NULL);

    /* system control */
    sysbus_realize(SYS_BUS_DEVICE(&s->sysctrl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sysctrl), 0,
                    s->memmap[MT7628_DEV_SYSCTRL]);

    /* spi master */
    mt7628SpiState *spi_bus = &s->spi;
    sysbus_realize(SYS_BUS_DEVICE(spi_bus), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(spi_bus), 0, s->memmap[MT7628_DEV_SPI]);
    sysbus_mmio_map(SYS_BUS_DEVICE(spi_bus), 1,
                    s->memmap[MT7628_DEV_FLASH_DIRECT]);

    /* spi nor flash, default attach a w25q128 (16 MiB) */
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    DeviceState *spi_flash;
    qemu_irq cs_line;
    if (dinfo) {
        spi_flash = qdev_new("w25q128");
        qdev_prop_set_drive(spi_flash, "drive", blk_by_legacy_dinfo(dinfo));
        qdev_realize_and_unref(spi_flash, BUS(spi_bus->spi), &error_fatal);
        cs_line = qdev_get_gpio_in_named(spi_flash, SSI_GPIO_CS, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(spi_bus), 0, cs_line);
    }


    /* UART x 3 */
    SerialMM *smm;
    for (i = 0; i < 3; i++) {
        smm = SERIAL_MM(qdev_new(TYPE_SERIAL_MM));
        qdev_prop_set_chr(DEVICE(smm), "chardev", serial_hd(i));
        qdev_prop_set_uint8(DEVICE(smm), "regshift", 2);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(smm), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(smm), 0,
                        s->memmap[MT7628_DEV_UART0 + i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0,
                       qdev_get_gpio_in(dev, IRQ_UART0 + i));
    }

    /* usb host */
    if (machine_usb(current_machine)) {
        sysbus_realize(SYS_BUS_DEVICE(&s->ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->ehci), 0,
                        s->memmap[MT7628_DEV_EHCI]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ehci), 0,
                           qdev_get_gpio_in(dev, IRQ_EHCI));
        sysbus_realize(SYS_BUS_DEVICE(&s->ohci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->ohci), 0,
                        s->memmap[MT7628_DEV_OHCI]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ohci), 0,
                           qdev_get_gpio_in(dev, IRQ_EHCI));
    } else {
        create_unimplemented_device("ehci", s->memmap[MT7628_DEV_EHCI],
                                    256 * KiB);
        create_unimplemented_device("ohci", s->memmap[MT7628_DEV_OHCI],
                                    256 * KiB);
    }

    /* ethernet */
    if (nd_table[0].used) {
        qemu_check_nic_model(&nd_table[0], TYPE_MT7628_ETH);
        qdev_set_nic_properties(DEVICE(&s->eth), &nd_table[0]);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->eth), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->eth), 0,
                    s->memmap[MT7628_DEV_ETH]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->eth), 0,
                       env->irq[MT7628_CPU_IRQ_ETH]);

    /* Unimplemented devices */
    for (i = 0; i < ARRAY_SIZE(unimplemented); i++) {
        create_unimplemented_device(unimplemented[i].device_name,
                                    unimplemented[i].base,
                                    unimplemented[i].size);
    }
}

static void mt7628_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = mt7628_realize;
    dc->user_creatable = false;
}

static const TypeInfo mt7628_type_info = {
    .name = TYPE_MT7628,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(mt7628State),
    .instance_init = mt7628_init,
    .class_init = mt7628_class_init,
};

static void mt7628_register_types(void)
{
    type_register_static(&mt7628_type_info);
}

type_init(mt7628_register_types);

/*
 * a board use mt7628
 */

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
} loaderparams;

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *) opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong) 1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

static uint64_t load_kernel(void)
{
    uint64_t entry, kernel_high;
    long kernel_size;

    kernel_size = load_elf(loaderparams.kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL,
                           &entry, NULL,
                           &kernel_high, NULL, 0, EM_MIPS, 1, 0);
    if (kernel_size < 0) {
        error_report("could not load kernel '%s': %s",
                     loaderparams.kernel_filename,
                     load_elf_strerror(kernel_size));
        exit(1);
    }
    return entry;
}

static void mt7628_board_init(MachineState *machine)
{
    const char *kernel_filename = machine->kernel_filename;
    mt7628State *mt7628;
    Clock *cpuclk;
    ResetData *reset_info;
    char *filename;

    /* CPU limit */
    if (strcmp(machine->cpu_type, MIPS_CPU_TYPE_NAME("24KEc")) != 0) {
        error_report("This board can only be used with 24KEc CPU");
        exit(1);
    }

    /* RAM limit */
    if (machine->ram_size > 256 * MiB) {
        error_report("mt7628: memory size must not exceed 256MiB");
        exit(1);
    }

    /* SOC */
    mt7628 = MT7628(object_new(TYPE_MT7628));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(mt7628));
    object_unref(OBJECT(mt7628));

    /* CPU Clock */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    if (mt7628->sysctrl.xtal_freq == 1) { /* 40 Mhz */
        clock_set_hz(cpuclk, 580000000);
    } else { /* 25 Mhz */
        clock_set_hz(cpuclk, 575000000);
    }

    /* CPU */
    mt7628->cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk);
    cpu_mips_irq_init_cpu(mt7628->cpu);
    cpu_mips_clock_init(mt7628->cpu);

    /* Mark mt7628 object realized */
    qdev_realize(DEVICE(mt7628), NULL, &error_abort);

    /* DDR */
    memory_region_add_subregion(get_system_memory(),
                                mt7628->memmap[MT7628_DEV_DDR],
                                machine->ram);
    /*
     * fill a empty memory region,
     * let uboot 'get_ram_size' function working.
     * in uboot source tree : common/memsize.c
     */
    if (machine->ram_size < 256 * MiB) {
        create_unimplemented_device("ddr(empty)",
                                    mt7628->memmap[MT7628_DEV_DDR] +
                                    machine->ram_size,
                                    (256 * MiB) - machine->ram_size);
    }

    /*
     * I hexdump the mainline u-boot-spl.bin,
     * begin of flash 4 bytes is a instruction use for jump.
     * NOTE: uboot-with-spl.bin will loop at ddr_calibrate.
     * TODO: emulate ddr_calibrate?.
     * No, you can simple skip it, use your hex editor.
     * qemu's DDR is always working, don't need care about it.
     */
    reset_info = g_new0(ResetData, 1);
    reset_info->cpu = mt7628->cpu;
    reset_info->vector = (target_long)(int32_t)
                         (mt7628->memmap[MT7628_DEV_FLASH_DIRECT] + 0x80000000);
    /*
     * bootrom
     * I found its definition in the Linux kernel source code.
     */
    memory_region_init_rom(&mt7628->bootrom, NULL, "mt7628.bootrom", 0x8000,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(),
                                mt7628->memmap[MT7628_DEV_BOOTROM],
                                &mt7628->bootrom);
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                              machine->firmware ?: BIOS_FILENAME);
    if (filename) {
        load_image_targphys(filename, mt7628->memmap[MT7628_DEV_BOOTROM],
                            0x8000);
        g_free(filename);
        reset_info->vector = (target_long)(int32_t)
                             (mt7628->memmap[MT7628_DEV_BOOTROM] + 0xA0000000);
    }
    /* Load kernel to RAM & goto kernel */
    if (kernel_filename) {
        loaderparams.ram_size = machine->ram_size;
        loaderparams.kernel_filename = kernel_filename;
        reset_info->vector = load_kernel();
    }
    qemu_register_reset(main_cpu_reset, reset_info);
}

static void mt7628_machine_init(MachineClass *mc)
{
    mc->desc = "mt7628 (24KEc)";
    mc->init = mt7628_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("24KEc");
    mc->default_ram_id = "mt7628.ram";
    mc->default_ram_size = 256 * MiB;
    mc->min_cpus = 1;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
}

DEFINE_MACHINE("mt7628", mt7628_machine_init)

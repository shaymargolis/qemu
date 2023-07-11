/*
 * QEMU/mipssim emulation
 *
 * Emulates a very simple machine model similar to the one used by the
 * proprietary MIPS emulator.
 *
 * Copyright (c) 2007 Thiemo Seufer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "hw/clock.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "hw/ssi/ssi.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/mips/bios.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"
#include "sysemu/reset.h"

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static uint64_t load_kernel(void)
{
    uint64_t entry, kernel_high, initrd_size;
    long kernel_size;
    ram_addr_t initrd_offset;
    int big_endian;

#if TARGET_BIG_ENDIAN
    big_endian = 1;
#else
    big_endian = 0;
#endif

    kernel_size = load_elf(loaderparams.kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL,
                           &entry, NULL,
                           &kernel_high, NULL, big_endian,
                           EM_MIPS, 1, 0);
    if (kernel_size < 0) {
        error_report("could not load kernel '%s': %s",
                     loaderparams.kernel_filename,
                     load_elf_strerror(kernel_size));
        exit(1);
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size(loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = ROUND_UP(kernel_high, INITRD_PAGE_SIZE);
            if (initrd_offset + initrd_size > loaderparams.ram_size) {
                error_report("memory too small for initial ram disk '%s'",
                             loaderparams.initrd_filename);
                exit(1);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                initrd_offset, loaderparams.ram_size - initrd_offset);
        }
        if (initrd_size == (target_ulong) -1) {
            error_report("could not load initial ram disk '%s'",
                         loaderparams.initrd_filename);
            exit(1);
        }
    }
    return entry;
}

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong)1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

// static void mipsnet_init(int base, qemu_irq irq, NICInfo *nd)
// {
//     DeviceState *dev;
//     SysBusDevice *s;

//     dev = qdev_new("mipsnet");
//     qdev_set_nic_properties(dev, nd);

//     s = SYS_BUS_DEVICE(dev);
//     sysbus_realize_and_unref(s, &error_fatal);
//     sysbus_connect_irq(s, 0, irq);
//     memory_region_add_subregion(get_system_io(),
//                                 base,
//                                 sysbus_mmio_get_region(s, 0));
// }

#define EW_DRIVER_NOT_WRITTEN (0)
#define EW_DRIVER_WROTE_SUCCESS (1)

static int ew_driver_state = EW_DRIVER_NOT_WRITTEN;

static uint64_t ew_driver_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    int val = ew_driver_state;
    if (val == EW_DRIVER_WROTE_SUCCESS) {
        ew_driver_state = EW_DRIVER_NOT_WRITTEN;
    }

    if (addr == 0xc4) {
        return val;
    }

    return -1;
}

static void ew_driver_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    if (addr == 0xc0) {
        ew_driver_state = EW_DRIVER_WROTE_SUCCESS;
    }
}

const MemoryRegionOps ew_driver_ops = {
    .read = ew_driver_read,
    .write = ew_driver_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

static void
mips_mipssim_init(MachineState *machine)
{
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    ram_addr_t ram_size = machine->ram_size;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *isa = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MemoryRegion *readonly_ram = g_new(MemoryRegion, 1);
    Clock *cpuclk;
    MIPSCPU *cpu;
    CPUMIPSState *env;
    ResetData *reset_info;
    int bios_size;

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
#ifdef TARGET_MIPS64
    clock_set_hz(cpuclk, 6000000); /* 6 MHz */
#else
    clock_set_hz(cpuclk, 12000000); /* 12 MHz */
#endif

    clock_set_hz(cpuclk, 575000000); /* 575 MHz */

    /* Init CPUs. */
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk);
    env = &cpu->env;

    reset_info = g_new0(ResetData, 1);
    reset_info->cpu = cpu;
    reset_info->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info);

    /* Allocate RAM. */
    memory_region_init_rom(bios, NULL, "mips_mipssim.bios", BIOS_SIZE,
                           &error_fatal);

    memory_region_add_subregion(address_space_mem, 0x00000000, machine->ram);

    memory_region_init_rom(readonly_ram, NULL, "RAM_READONLY", 0x10000000 - ram_size, &error_fatal);
    memory_region_add_subregion(address_space_mem, ram_size, readonly_ram);

    /* Map the BIOS / boot exception handler. */
    memory_region_add_subregion(address_space_mem, 0x1c030000LL, bios);
    /* Load a BIOS / boot exception handler image. */
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware ?: BIOS_FILENAME);
    if (filename) {
        bios_size = load_image_targphys(filename, 0x1c030000LL, BIOS_SIZE);
        g_free(filename);
    } else {
        bios_size = -1;
    }
    if ((bios_size < 0 || bios_size > BIOS_SIZE) &&
        machine->firmware && !qtest_enabled()) {
        /* Bail out if we have neither a kernel image nor boot vector code. */
        error_report("Could not load MIPS bios '%s'", machine->firmware);
        exit(1);
    } else {
        /* We have a boot vector start address. */
        env->active_tc.PC = (target_long)(int32_t)0xbc030000;
    }

    if (kernel_filename) {
        loaderparams.ram_size = machine->ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }

    /* Init CPU internal devices. */
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    /* Register 64 KB of ISA IO space at 0x1fd00000. */
    memory_region_init_alias(isa, NULL, "isa_mmio",
                             get_system_io(), 0, 0x000200000);
    // memory_region_add_subregion(get_system_memory(), 0x10000000, isa);

    MemoryRegion *test123 = g_new(MemoryRegion, 1);
    memory_region_init_ram(test123, NULL, "test123", 0xb00, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10000000LL, test123);

    MemoryRegion *more_ram = g_new(MemoryRegion, 1);
    memory_region_init_ram(more_ram, NULL, "more_ram", 0x300, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10000c00LL, more_ram);

    // void memory_region_init_io(MemoryRegion *mr,
    //                        Object *owner,
    //                        const MemoryRegionOps *ops,
    //                        void *opaque,
    //                        const char *name,
    //                        uint64_t size);

    MemoryRegion *test124 = g_new(MemoryRegion, 1);
    memory_region_init_io(test124, NULL, &ew_driver_ops, NULL, "", 0xf00);
    memory_region_add_subregion(get_system_memory(), 0x10110000LL, test124);

    MemoryRegion *eth_random = g_new(MemoryRegion, 1);
    memory_region_init_ram(eth_random, NULL, "eth_random", 0x1000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10100000LL, eth_random);

    // SPI reg = <0xb00 0x100>;

    DeviceState *dev = sysbus_create_simple("mt7621_spi", 0x10000b00, env->irq[1]);

    BusState *spi_bus;
    DeviceState *flash_dev;
    qemu_irq flash_cs;
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);

    spi_bus = qdev_get_child_bus(dev, "spi");

    flash_dev = qdev_new("w25q64");
    if (dinfo) {
        qdev_prop_set_drive_err(flash_dev, "drive",
                                blk_by_legacy_dinfo(dinfo), &error_fatal);
    }
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);

    flash_cs = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out_named(DEVICE(dev), "cs", 0, flash_cs);

    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, env->irq[1]);

    // PCI reg = <0x10140000 0x100
    //            0x10142000 0x100>;

    MemoryRegion *pci_region_1 = g_new(MemoryRegion, 1);
    memory_region_init_ram(pci_region_1, NULL, "pci_region_1", 0x10000, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x10140000LL, pci_region_1);

    // rom_add_blob_fixed("read", buff, 0, 0x10000400LL);

    serial_mm_init(get_system_memory(), 0x10000000LL + 0xc00, 2, env->irq[4],
                             115200, serial_hd(0), DEVICE_NATIVE_ENDIAN);

    env->active_tc.PC = (target_long)(int32_t)0xbc030000;
    reset_info->vector = (target_long)(int32_t)0xbc030000;

    /*
     * A single 16450 sits at offset 0x3f8. It is attached to
     * MIPS CPU INT2, which is interrupt 4.
     */
    // if (serial_hd(0)) {
    //     DeviceState *dev = qdev_new(TYPE_SERIAL_MM);

    //     qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    //     qdev_prop_set_uint8(dev, "regshift", 0);
    //     qdev_prop_set_uint8(dev, "endianness", DEVICE_LITTLE_ENDIAN);
    //     sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    //     sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, env->irq[4]);
    //     sysbus_add_io(SYS_BUS_DEVICE(dev), 0x3f8,
    //                   sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
    // }

    // if (nd_table[0].used)
    //     /* MIPSnet uses the MIPS CPU INT0, which is interrupt 2. */
    //     mipsnet_init(0x4200, env->irq[2], &nd_table[0]);
}

static void mips_mipssim_machine_init(MachineClass *mc)
{
    mc->desc = "MIPS MIPSsim platform";
    mc->init = mips_mipssim_init;
#ifdef TARGET_MIPS64
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("5Kf");
#else
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("24Kf");
#endif
    mc->default_ram_id = "mips_mipssim.ram";
}

DEFINE_MACHINE("mt7628an", mips_mipssim_machine_init)

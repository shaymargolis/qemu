/*
 * Mediatek mt7628 System Control emulation
 *
 * Copyright (C) 2023 Lu Hui <luhux76@gmail.com>
 *
 * some code from linux kernel:
 * arch/mips/include/asm/mach-ralink/mt7621.h
 * Copyright (C) 2015 John Crispin <john@phrozen.org>
 * arch/mips/ralink/mt7620.c
 * Copyright (C) 2008-2011 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 * Copyright (C) 2013 John Crispin <john@phrozen.org>
 * arch/mips/ralink/reset.c
 * Copyright (C) 2008-2009 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 * Copyright (C) 2013 John Crispin <john@phrozen.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/mt7628-sysctrl.h"
#include "sysemu/runstate.h"
#include "hw/qdev-properties.h"

/* System Control register offsets */
enum {
    REG_CHIP_NAME0 = 0x00,
    REG_CHIP_NAME1 = 0x04,
    REG_EFUSE_CFG = 0x08,
    REG_CHIP_REV = 0x0C,
    REG_SYS_CFG0 = 0x10,
    REG_ROM_STATUS = 0x28,
    REG_RST_CTRL = 0x34,
};

#define CHIP_REV_PKG_SHIFT 16
#define CHIP_REV_VER_SHIFT 8
#define CHIP_REV_ECO_SHIFT 0

#define XTAL_FREQ_SHIFT 6
#define CHIP_MODE_SHIFT 1
#define DRAM_TYPE_SHIFT 0

#define RST_SYSTEM_SHIFT 0

static uint64_t mt7628_sysctrl_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    const mt7628SysCtrlState *s = MT7628_SYSCTRL(opaque);
    uint32_t val = 0x0;
    switch (addr) {
    case REG_CHIP_NAME0:
        return s->chipname0;
    case REG_CHIP_NAME1:
        return s->chipname1;
    case REG_EFUSE_CFG:
        return s->efuse_cfg;
    case REG_CHIP_REV:
        val |= s->chip_pkg_id << CHIP_REV_PKG_SHIFT;
        val |= s->chip_ver_id << CHIP_REV_VER_SHIFT;
        val |= s->chip_eco_id << CHIP_REV_ECO_SHIFT;
        return val;
    case REG_SYS_CFG0:
        val |= s->chip_mode << CHIP_MODE_SHIFT;
        val |= s->dram_type << DRAM_TYPE_SHIFT;
        val |= s->xtal_freq << XTAL_FREQ_SHIFT;
        return val;
    case REG_ROM_STATUS:
        val = 0x1; /* always done */
        return val;
    case REG_RST_CTRL:
        val |= s->reset_system << RST_SYSTEM_SHIFT;
        return val;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: no impl addr: 0x%08lx\n",
                      __func__, addr);
        return 0x0;
    }
}
static void mt7628_sysctrl_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    mt7628SysCtrlState *s = MT7628_SYSCTRL(opaque);
    switch (addr) {
    case REG_SYS_CFG0:
        s->dram_type = test_bit(DRAM_TYPE_SHIFT, (void *)&val);
        s->chip_mode = test_bit(CHIP_MODE_SHIFT, (void *)&val);
        s->xtal_freq = test_bit(XTAL_FREQ_SHIFT, (void *)&val);
        break;
    case REG_RST_CTRL:
        s->reset_system = test_bit(RST_SYSTEM_SHIFT, (void *)&val);
        if (s->reset_system) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: no impl addr: 0x%08lx\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps mt7628_sysctrl_ops = {
    .read = mt7628_sysctrl_read,
    .write = mt7628_sysctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl.min_access_size = 4,
};

static void mt7628_sysctrl_reset(DeviceState *dev)
{
    mt7628SysCtrlState *s = MT7628_SYSCTRL(dev);
    /* Set default values for registers */
    memcpy(&s->chipname0, "MT76", 4);
    memcpy(&s->chipname1, "28  ", 4);
    s->reset_system = 0;
}

static void mt7628_sysctrl_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    mt7628SysCtrlState *s = MT7628_SYSCTRL(obj);

    /* Memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &mt7628_sysctrl_ops, s,
                          TYPE_MT7628_SYSCTRL, 0xFF);
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property mt7628_sysctrl_properties[] = {
    DEFINE_PROP_UINT32("efuse_cfg", mt7628SysCtrlState, efuse_cfg, 0x01010000),
    DEFINE_PROP_UINT8("chip_pkg_id", mt7628SysCtrlState, chip_pkg_id, 1),
    DEFINE_PROP_UINT8("chip_ver_id", mt7628SysCtrlState, chip_ver_id, 1),
    DEFINE_PROP_UINT8("chip_eco_id", mt7628SysCtrlState, chip_eco_id, 2),
    DEFINE_PROP_UINT8("chip_mode", mt7628SysCtrlState, chip_mode, 0b010),
    DEFINE_PROP_UINT8("dram_type", mt7628SysCtrlState, dram_type, 0),
    DEFINE_PROP_UINT8("xtal_freq", mt7628SysCtrlState, xtal_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription mt7628_sysctrl_vmstate = {
    .name = "mt7628-sysctrl",
    .version_id = 1,
    .minimum_version_id = 1,
};

static void mt7628_sysctrl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = mt7628_sysctrl_reset;
    dc->vmsd = &mt7628_sysctrl_vmstate;
    device_class_set_props(dc, mt7628_sysctrl_properties);
}

static const TypeInfo mt7628_sysctrl_info = {
    .name = TYPE_MT7628_SYSCTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = mt7628_sysctrl_init,
    .instance_size = sizeof(mt7628SysCtrlState),
    .class_init = mt7628_sysctrl_class_init,
};

static void mt7628_sysctrl_register(void)
{
    type_register_static(&mt7628_sysctrl_info);
}

type_init(mt7628_sysctrl_register)

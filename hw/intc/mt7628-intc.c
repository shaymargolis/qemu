/*
 * mt7628 interrupt controller device emulation
 *
 * Copyright (C) 2023 Lu Hui
 * Written by Lu Hui <luhux76@gmail.com>
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/intc/mt7628-intc.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/module.h"

static void mt7628_intc_update(mt7628intcState *s)
{
    int i;
    for (i = 0; i < 32; i++) {
        if (test_bit(i, (void *) &s->disable)) {
            clear_bit(i, (void *) &s->enable);
        }
    }
    qemu_set_irq(s->parent_irq, !!(s->enable));
}

static void mt7628_intc_set_irq(void *opaque, int irq, int level)
{
    mt7628intcState *s = opaque;

    if (irq >= 20 && irq <= 22) {
        // printf("TRYING TO USE UART!! %d (E %d, D %d)\n", level, test_bit(irq, (void *) &s->enable), test_bit(irq, (void *) &s->disable));
    }

    if (level) {
        set_bit(irq, (void *) &s->enable);
        clear_bit(irq, (void *) &s->disable);
    } else {
        clear_bit(irq, (void *) &s->enable);
        set_bit(irq, (void *) &s->disable);
    }
    mt7628_intc_update(s);
}

static uint64_t mt7628_intc_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    mt7628intcState *s = opaque;

    switch (offset) {
    case MT7628_INTC_REG_STATUS0:
        return s->enable;
    case MT7628_INTC_REG_TYPE:
        return s->type;
    case MT7628_INTC_REG_ENABLE:
        return s->enable;
    case MT7628_INTC_REG_DISABLE:
        return s->disable;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: not imp offset 0x%x\n", __func__, (int) offset);
        return 0x0;
        break;
    }

    return 0;
}

static void mt7628_intc_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    mt7628intcState *s = opaque;

    switch (offset) {
    case MT7628_INTC_REG_TYPE:
        s->type = value;
        break;
    case MT7628_INTC_REG_ENABLE:
        s->enable = value;
        break;
    case MT7628_INTC_REG_DISABLE:
        s->disable = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: not imp offset 0x%x\n", __func__, (int) offset);
        break;
    }

    mt7628_intc_update(s);
}

static const MemoryRegionOps mt7628_intc_ops = {
    .read = mt7628_intc_read,
    .write = mt7628_intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_mt7628_intc = {
    .name = "mt7628.intc",
    .version_id = 1,
    .minimum_version_id = 1,
};

static void mt7628_intc_init(Object *obj)
{
    mt7628intcState *s = MT7628_INTC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    qdev_init_gpio_in(DEVICE(dev), mt7628_intc_set_irq, 32);
    sysbus_init_irq(dev, &s->parent_irq);
    memory_region_init_io(&s->iomem, OBJECT(s), &mt7628_intc_ops, s,
                          TYPE_MT7628_INTC, MT7628_INTC_REGS_MAXADDR);
    sysbus_init_mmio(dev, &s->iomem);
}

static void mt7628_intc_reset(DeviceState *d)
{
    mt7628intcState *s = MT7628_INTC(d);

    s->type    = 0b00000000000000000000000000000000;
    s->enable  = 0b00000000000000000000000000000000;
    s->disable = 0b00000000000000000000000000000000;
}

static void mt7628_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = mt7628_intc_reset;
    dc->desc = "mt7628 interrupt control";
    dc->vmsd = &vmstate_mt7628_intc;
}

static const TypeInfo mt7628_intc_info = {
    .name = TYPE_MT7628_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mt7628intcState),
    .instance_init = mt7628_intc_init,
    .class_init = mt7628_intc_class_init,
};

static void mt7628_register_types(void)
{
    type_register_static(&mt7628_intc_info);
}

type_init(mt7628_register_types);

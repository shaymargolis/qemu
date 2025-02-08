/*
 * mt7628 timer device emulation
 *
 * Copyright XXX
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
#include "hw/timer/mt7628-timer.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define SYSTICK_FREQ        (50 * 1000)
// #define SYSTICK_FREQ        (50 * 1000 * 1000)
#define SYSTICK_INTERVAL_NS ((u32)(1e9/ SYSTICK_FREQ))

#define SYSTICK_CONFIG      (0x00)
#define SYSTICK_COMPARE     (0x04)
#define SYSTICK_COUNT       (0x08)


/* route systick irq to mips irq 7 instead of the r4k-timer */
#define CFG_EXT_STK_EN      (0x2)
/* enable the counter */
#define CFG_CNT_EN      (0x1)

// static bool systick_enabled = false;
// static u32 systick_config = 0;
// static u32 systick_compare = 0;
// static u32 systick_count = 0;

/* MIPS R4K timer */
static void rasystick_timer_update(mt7628TimerState *s)
{
    uint64_t now_ns, next_ns;

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    next_ns = now_ns + SYSTICK_INTERVAL_NS;
    // printf("Modding to %lu\n", next_ns);
    timer_mod(s->ra_systick, next_ns);
}

/* Expire the timer.  */
static void rasystick_timer_expire(mt7628TimerState *s)
{
    CPUMIPSState *env = &s->cpu->env;

    rasystick_timer_update(s);

    if (!s->enabled) {
        return;
    }

    s->count++;

    // if (s->count > s->compare) {
    //     s->count = s->compare;
    // }

    if (s->count < s->compare) {
        return;
    }

    env->CP0_Cause |= 1 << CP0Ca_TI;

    // printf("Raising irq\n");

    // qemu_irq_raise(env->irq[5]);
    qemu_irq_raise(env->irq[MT7628_CPU_IRQ_TIMER]);
}

static void rasystick_timer_cb(void *opaque)
{
    mt7628TimerState *s = opaque;
    rasystick_timer_expire(s);
}

// static void create_ra_systick(CPUMIPSState *env)
static void create_ra_systick(mt7628TimerState *s)
{
    // CPUMIPSState *env = &s->cpu->env;
    s->ra_systick = timer_new_ns(QEMU_CLOCK_VIRTUAL, &rasystick_timer_cb, s);
    printf("Create called! %016lx timer %016lx\n", (unsigned long)s, (unsigned long)s->ra_systick);
    rasystick_timer_cb(s);

    // sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0,
    //                    qdev_get_gpio_in(dev, 7));
}

static uint64_t ra_systick_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    mt7628TimerState *s = opaque;
    u32 val;

    switch(addr) {
    case SYSTICK_CONFIG:
        val = s->config;
        break;
    case SYSTICK_COUNT:
        val = s->count;
        break;
    case SYSTICK_COMPARE:
        val = s->compare;
        break;
    default:
        val = -1;
        break;
    }

    // printf("<ra_systick> Read 0x%08X %08X\n", (u32)addr, val);

    return val;
}

static void ra_systick_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    mt7628TimerState *s = opaque;
    CPUMIPSState *env = &s->cpu->env;
    u32 val = (u32)value;

    if (addr != SYSTICK_COMPARE) {
        printf("<ra_systick> Wrote 0x%08X %08X\n", (u32)addr, val);
    }

    switch(addr) {
    case SYSTICK_CONFIG:
        s->config = val;

        s->enabled = (s->config & CFG_CNT_EN) != 0;

        if (s->enabled) {
            timer_del(env->timer);
        }
        break;
    case SYSTICK_COUNT:
        // Disallow writing to the count
        // s->count = val;
        break;
    case SYSTICK_COMPARE:
        s->compare = val;
        // s->count = 0;
        qemu_irq_lower(env->irq[MT7628_CPU_IRQ_TIMER]);
        env->CP0_Cause &= ~(1 << CP0Ca_TI);
        break;
    default:
        val = -1;
        break;
    }
}

const MemoryRegionOps ra_systick_ops = {
    .read = ra_systick_read,
    .write = ra_systick_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_mt7628_timer = {
    .name = "mt7628.timer",
    .version_id = 1,
    .minimum_version_id = 1,
};

static void mt7628_timer_reset(DeviceState *d)
{
    mt7628TimerState *s = MT7628_TIMER(d);

    s->enabled = false;
    s->config = 0;
    s->compare = 0;
    s->count = 0;
}

static void mt7628_timer_init(Object *obj)
{
    mt7628TimerState *s = MT7628_TIMER(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &rasystick_timer_cb, s);
    printf("Create called! %016lx timer %016lx\n", (unsigned long)s, (unsigned long)s->ra_systick);
    rasystick_timer_cb(s);

    memory_region_init_io(&s->iomem, OBJECT(s), &ra_systick_ops, s,
                          "ra_systick", 0x00c);
    sysbus_init_mmio(dev, &s->iomem);

    mt7628_timer_reset(s);
}

static void mt7628_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = mt7628_timer_reset;
    dc->desc = "mt7628 interrupt control";
    dc->vmsd = &vmstate_mt7628_timer;
}

static const TypeInfo mt7628_timer_info = {
    .name = TYPE_MT7628_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mt7628TimerState),
    .instance_init = mt7628_timer_init,
    .class_init = mt7628_timer_class_init,
};

static void mt7628_register_types(void)
{
    type_register_static(&mt7628_timer_info);
}

type_init(mt7628_register_types);

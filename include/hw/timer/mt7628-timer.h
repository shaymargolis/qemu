#ifndef MT7628_TIMER_H
#define MT7628_TIMER_H

/*
 * some code from linux kernel:
 * arch/mips/ralink/irq.c
 * Copyright (C) 2009 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2013 John Crispin <john@phrozen.org>
 */

#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_MT7628_TIMER  "mt7628-timer"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628TimerState, MT7628_TIMER)



typedef unsigned int u32;
typedef unsigned char u8;


struct mt7628TimerState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq parent_irq;

    QEMUTimer *timer;
    bool enabled;
    u32 config;
    u32 compare;
    u32 count;
};

#endif

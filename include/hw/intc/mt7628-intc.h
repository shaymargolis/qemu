#ifndef MT7628_INTC_H
#define MT7628_INTC_H

/*
 * some code from linux kernel:
 * arch/mips/ralink/irq.c
 * Copyright (C) 2009 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2013 John Crispin <john@phrozen.org>
 */

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_MT7628_INTC  "mt7628-intc"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628intcState, MT7628_INTC)

#define MT7628_INTC_REG_STATUS0     (0x9c)
#define MT7628_INTC_REG_TYPE        (0x6c)
#define MT7628_INTC_REG_ENABLE      (0x80)
#define MT7628_INTC_REG_DISABLE     (0x78)

#define MT7628_INTC_REGS_MAXADDR (0xFF)

struct mt7628intcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq parent_irq;

    uint32_t type;
    uint32_t enable;
    uint32_t disable;
    /*priority setting here*/
};

#endif

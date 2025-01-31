/*
 * mt7628 spi master
 *
 * Copyright (c) 2023 Lu Hui <luhux76@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef HW_MT7628_SPI_H
#define HW_MT7628_SPI_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/units.h"

#define TYPE_MT7628_SPI "mt7628-spi"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628SpiState, MT7628_SPI)

#define MT7628_SPI_TX_BUFSIZE (36)
#define MT7628_SPI_RX_BUFSIZE (32)
#define MT7628_SPI_FLASH_WINDOW_SIZE (4 * MiB)

typedef struct mt7628SpiState mt7628SpiState;
typedef struct mt7628SpiFlash mt7628SpiFlash;

typedef struct mt7628SpiFlash {
    mt7628SpiState *spi;
    MemoryRegion direct_access;
} mt7628SpiFlash;

struct mt7628SpiState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    SSIBus *spi;

    /* real device only support two cs */
    bool cs_status[2];
    qemu_irq cs_lines[2];

    bool bufmode;     /* 0 disable morebuf, 1 enable morebuf */
    bool full_duplex; /* 0 half, 1 full   */
    bool lsb_first; /* 0 msb first, 1 lsb first */

    bool trans_busy;  /* 0 no task, 1 pending */
    bool trans_start; /* write 1 start transfer action */

    /* how many bits need transfer */
    uint16_t tx_bitcount; /* 0 ~ 256 */
    uint16_t rx_bitcount; /* 0 ~ 256 */
    uint8_t cmd_bitcount; /* 0 ~ 32 */

    uint32_t opcode;

    uint8_t dido[MT7628_SPI_RX_BUFSIZE];

    /* flash direct access */
    mt7628SpiFlash flash;
    uint8_t flash_read_cmd;
};

#endif

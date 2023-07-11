/*
 * Microsemi SmartFusion2 SPI
 *
 * Copyright (c) 2017 Subbaraya Sundeep <sundeep.lkml@gmail.com>
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

#ifndef HW_MT7621_SPI_H
#define HW_MT7621_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo32.h"
#include "qom/object.h"

#define TYPE_MT7621_SPI   "mt7621_spi"
OBJECT_DECLARE_SIMPLE_TYPE(MT7621SPIState, MT7621_SPI)

#define R_SPI_MAX             16

struct MT7621SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;

    qemu_irq cs_line;

    SSIBus *spi;

    Fifo8 rx_fifo;
    Fifo8 tx_fifo;

    int fifo_depth;
    uint32_t frame_count;
    bool enabled;

    uint32_t regs[R_SPI_MAX];
};

#endif /* HW_MT7621_SPI_H */

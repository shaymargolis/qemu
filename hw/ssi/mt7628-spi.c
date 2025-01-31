/*
 * mt7628 spi master emulation
 *
 * Copyright (c) 2023 Lu Hui <luhux76@gmail.com>
 * some code from linux kernel:
 * drivers/spi/spi-mt7621.c
 * Copyright (C) 2011 Sergiy <piratfm@gmail.com>
 * Copyright (C) 2011-2013 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2014-2015 Felix Fietkau <nbd@nbd.name>
 * some code from ./npcm7xx_fiu.c
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/mt7628-spi.h"

/*
 * TODO:
 * byte order select
 */

/* #define DEBUG_MT7628_SPI 1 */

#ifdef DEBUG_MT7628_SPI
#define DPRINTF(fmt, ...) \
do { printf("mt7628-spi: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

enum {
    REG_SPI_TRANS    = 0x00,
    REG_SPI_OPADDR   = 0x04,
    REG_SPI_DIDO0    = 0x08,
    REG_SPI_DIDO1    = 0x0C,
    REG_SPI_DIDO2    = 0x10,
    REG_SPI_DIDO3    = 0x14,
    REG_SPI_DIDO4    = 0x18,
    REG_SPI_DIDO5    = 0x1C,
    REG_SPI_DIDO6    = 0x20,
    REG_SPI_DIDO7    = 0x24,
    REG_SPI_MOREBUF  = 0x2C,
    REG_SPI_MASTER   = 0x28,
    REG_SPI_CS_POLAR = 0x38,
};

/* reg shift */
#define TRANS_START 8
#define TRANS_BUSY  16
#define MOREBUF_TXBITCNT    0
#define MOREBUF_RXBITCNT    12
#define MOREBUF_CMDBITCNT   24
#define MASTER_BUFMODE      2
#define MASTER_FULL_DUPLEX  10
#define MASTER_LSB_FIRST    2

static void mt7628_spi_update_select(mt7628SpiState *s)
{
    int i;
    for (i = 0; i < 2; i++) {
        if (s->cs_status[i]) {
            DPRINTF("%s: cs %d assert\n", __func__, i);
            qemu_irq_lower(s->cs_lines[i]);
        } else {
            DPRINTF("%s: cs %d deassert\n", __func__, i);
            qemu_irq_raise(s->cs_lines[i]);
        }
    }
}

static void mt7628_spi_transfer(mt7628SpiState *s)
{
    DPRINTF("mt7628_spi_transfer : %d %d\n", s->trans_start, s->trans_busy);
    DPRINTF("cmd_bitcount tx_bitcount rx_bitcount : %d %d %d\n", s->cmd_bitcount, s->tx_bitcount, s->rx_bitcount);
    if (s->trans_start == 0) {
        return;
    }
    if (s->trans_busy  == 1) {
        return;
    }
    s->trans_busy = 1;

    qemu_irq_lower(s->cs_lines[0]);
    qemu_irq_lower(s->cs_lines[1]);

    uint32_t *data_reg = (uint32_t *)s->dido;
    uint8_t txbuf[MT7628_SPI_TX_BUFSIZE];
    uint8_t *txbufp = txbuf;
    memset(txbuf, 0, MT7628_SPI_TX_BUFSIZE);

    uint32_t val = s->opcode;
    if (s->lsb_first) { /* || (s->cmd_bitcount / 8) == 4) { */
        /* The byte-order of the opcode is weird!    --- from linux kernel */
        val = bswap32(val);
    } else {
        /* The opcode is the LSB, but the address (1-3 bytes) is still reversed */
        val = (val & 0xFF) + ((__builtin_bswap32(val >> 8)));
        data_reg[0] = (data_reg[0] & 0xFF) + ((__builtin_bswap32(data_reg[0] >> 8)));
    }

    memcpy(txbufp, &val, s->cmd_bitcount / 8);
    txbufp += s->cmd_bitcount / 8;
    // memcpy(txbufp, &val, 1);
    // txbufp += 1 / 8;
    memcpy(txbufp, s->dido, s->tx_bitcount / 8);
    txbufp += s->tx_bitcount / 8;

    /* send opcode and data to slave */
    int tx_i = 0;
    while (tx_i < ((s->cmd_bitcount / 8) + (s->tx_bitcount / 8))) {
        ssi_transfer(s->spi, txbuf[tx_i]);
        DPRINTF("SEND TO SLAVE: %02X\n", txbuf[tx_i]);
        tx_i++;
    }
    /* recv data from slave */
    int rx_i = 0;
    while (rx_i < (s->rx_bitcount / 8)) {
        s->dido[rx_i] = ssi_transfer(s->spi, 0);
        DPRINTF("RECV FROM SLAVE: %02X\n", s->dido[rx_i]);
        rx_i++;
    }

    s->trans_busy = 0;
    s->trans_start = 0;

    qemu_irq_raise(s->cs_lines[0]);
    qemu_irq_raise(s->cs_lines[1]);
    return;
}

static void mt7628_spi_reset(DeviceState *d)
{
    mt7628SpiState *s = MT7628_SPI(d);
    s->trans_start = 0;
    s->trans_busy = 0;
    s->bufmode = 0;
    s->full_duplex = 0;
    s->lsb_first = 0;
    s->tx_bitcount = 0;
    s->rx_bitcount = 0;
    s->cmd_bitcount = 0;
    s->opcode = 0;
    memset(s->dido, 0x0, MT7628_SPI_RX_BUFSIZE);
    s->cs_status[0] = 0;
    s->cs_status[1] = 0;
    mt7628_spi_update_select(s);
    s->flash_read_cmd = 0x03;
}

static uint64_t mt7628_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    mt7628SpiState *s = opaque;
    uint32_t val = 0x0;
    uint32_t *data_reg = (uint32_t *)s->dido;
    switch (addr) {
    case REG_SPI_TRANS:
        /* trans_start is always return 0 */
        val |= s->trans_busy << TRANS_BUSY;
        val += ((s->rx_bitcount / 8) & 7) << 4;
        val += ((s->tx_bitcount / 8) & 7);

        return val;
    case REG_SPI_OPADDR:
        val = s->opcode;
        return val;
    case REG_SPI_DIDO0:
        return *(data_reg + 0);
    case REG_SPI_DIDO1:
        return *(data_reg + 1);
    case REG_SPI_DIDO2:
        return *(data_reg + 2);
    case REG_SPI_DIDO3:
        return *(data_reg + 3);
    case REG_SPI_DIDO4:
        return *(data_reg + 4);
    case REG_SPI_DIDO5:
        return *(data_reg + 5);
    case REG_SPI_DIDO6:
        return *(data_reg + 6);
    case REG_SPI_DIDO7:
        return *(data_reg + 7);
    case REG_SPI_MOREBUF:
        val |= s->tx_bitcount << MOREBUF_TXBITCNT;
        val |= s->rx_bitcount << MOREBUF_RXBITCNT;
        val |= s->cmd_bitcount << MOREBUF_CMDBITCNT;
        return val;
    case REG_SPI_MASTER:
        val |= s->bufmode << MASTER_BUFMODE;
        val |= s->full_duplex << MASTER_FULL_DUPLEX;
        val |= s->lsb_first << MASTER_LSB_FIRST;
        return val;
    case REG_SPI_CS_POLAR:
        val |= s->cs_status[0] << 0;
        val |= s->cs_status[1] << 1;
        return val;
    default:
        return 0x0;
    }
}

static void mt7628_spi_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    mt7628SpiState *s = opaque;
    uint32_t *data_reg = (uint32_t *)s->dido;
    switch (addr) {
    case REG_SPI_TRANS:
        /* trans_busy is RO */
        s->trans_start = test_bit(TRANS_START, (void *)&value);

        unsigned int value32 = (unsigned int)value;

        if (s->bufmode == 0 && s->trans_start == 0) {
            s->cmd_bitcount = 0;
            s->rx_bitcount  = ((value32 >> 4) & 7) * 8;
            s->tx_bitcount = (value32 & 7) * 8;
        }

        DPRINTF("[mt7628-spi] Got MT7621_SPI_TRANS %08X start %d Val while rebuf %d %d %d\n", value32, s->trans_start, s->cmd_bitcount, s->rx_bitcount, s->tx_bitcount);
        break;
    case REG_SPI_OPADDR:
        s->opcode = value;
        data_reg[0] = value;
        DPRINTF("[mt7628-spi] Got opcode %08X\n", (unsigned int)value);
        break;
    case REG_SPI_DIDO0:
        data_reg[0] = value;
        break;
    case REG_SPI_DIDO1:
        data_reg[1] = value;
        break;
    case REG_SPI_DIDO2:
        data_reg[2] = value;
        break;
    case REG_SPI_DIDO3:
        data_reg[3] = value;
        break;
    case REG_SPI_DIDO4:
        data_reg[4] = value;
        break;
    case REG_SPI_DIDO5:
        data_reg[5] = value;
        break;
    case REG_SPI_DIDO6:
        data_reg[6] = value;
        break;
    case REG_SPI_DIDO7:
        data_reg[7] = value;
        break;
    case REG_SPI_MOREBUF:
        s->tx_bitcount = extract32(value, MOREBUF_TXBITCNT, 9);
        s->rx_bitcount = extract32(value, MOREBUF_RXBITCNT, 9);
        s->cmd_bitcount = extract32(value, MOREBUF_CMDBITCNT, 6);
        break;
    case REG_SPI_MASTER:
        s->bufmode     = test_bit(MASTER_BUFMODE, (void *)&value);
        s->full_duplex = test_bit(MASTER_FULL_DUPLEX, (void *)&value);
        s->lsb_first   = test_bit(MASTER_LSB_FIRST, (void *)&value);
        break;
    case REG_SPI_CS_POLAR:
        s->cs_status[0] = test_bit(0, (void *)&value);
        s->cs_status[1] = test_bit(1, (void *)&value);
        mt7628_spi_update_select(s);
        break;
    default:
        break;
    }
    if (s->trans_start) {
        mt7628_spi_transfer(s);
    }
}

static uint64_t mt7628_spi_flash_read(void *opaque, hwaddr addr,
                                      unsigned int size)
{
    struct mt7628SpiFlash *f = opaque;
    mt7628SpiState *s = f->spi;
    uint64_t val = 0;
    uint32_t i;

    // DPRINTF("%s: read %x size %x\n", __func__, (uint32_t)addr, size);
    s->trans_busy = 1;
    s->cs_status[0] = 1;
    s->cs_status[1] = 0;
    mt7628_spi_update_select(s);

    ssi_transfer(s->spi, s->flash_read_cmd);
    ssi_transfer(s->spi, extract32(addr, 16, 8));
    ssi_transfer(s->spi, extract32(addr, 8, 8));
    ssi_transfer(s->spi, extract32(addr, 0, 8));

    for (i = 0; i < size; i++) {
        val = deposit64(val, 8 * i, 8, ssi_transfer(s->spi, 0));
    }

    s->cs_status[0] = 0;
    s->cs_status[1] = 0;
    mt7628_spi_update_select(s);
    s->trans_busy = 0;
    // DPRINTF("%s: read %x size %x\n", __func__, (uint32_t)addr, size);

    return val;
}

static void mt7628_spi_flash_write(void *opaque, hwaddr addr, uint64_t v,
                                   unsigned int size)
{
    /* write is not support */
    (void) opaque;
    (void) addr;
    (void) v;
    (void) size;
    return;
}

static const MemoryRegionOps mt7628_spi_ops = {
    .read = mt7628_spi_read,
    .write = mt7628_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static const MemoryRegionOps mt7628_spi_flash_ops = {
    .read = mt7628_spi_flash_read,
    .write = mt7628_spi_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static void mt7628_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    mt7628SpiState *s = MT7628_SPI(dev);

    s->spi = ssi_create_bus(dev, "spi");


    memory_region_init_io(&s->mmio, OBJECT(s), &mt7628_spi_ops, s,
                          TYPE_MT7628_SPI, 0xFF);
    sysbus_init_mmio(sbd, &s->mmio);

    mt7628SpiFlash *flash = &s->flash;
    flash->spi = s;
    memory_region_init_io(&flash->direct_access,
                          OBJECT(s), &mt7628_spi_flash_ops, &s->flash,
                          "flash-direct", MT7628_SPI_FLASH_WINDOW_SIZE);
    sysbus_init_mmio(sbd, &flash->direct_access);

    sysbus_init_irq(sbd, &s->cs_lines[0]);
    sysbus_init_irq(sbd, &s->cs_lines[1]);
}

static void mt7628_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = mt7628_spi_reset;
    dc->realize = mt7628_spi_realize;
}

static const TypeInfo mt7628_spi_info = {
    .name           = TYPE_MT7628_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(mt7628SpiState),
    .class_init     = mt7628_spi_class_init,
};

static void mt7628_spi_register_types(void)
{
    type_register_static(&mt7628_spi_info);
}

type_init(mt7628_spi_register_types)

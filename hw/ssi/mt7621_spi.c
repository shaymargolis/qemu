/*
 * QEMU model of the SiFive SPI Controller
 *
 * Copyright (c) 2021 Wind River Systems, Inc.
 *
 * Author:
 *   Bin Meng <bin.meng@windriver.com>
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
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/mt7621_spi.h"

#define DRIVER_NAME     "spi-mt7621"

/* in usec */
#define RALINK_SPI_WAIT_MAX_LOOP 2000

/* SPISTAT register bit field */
#define SPISTAT_BUSY        BIT(0)

#define MT7621_SPI_TRANS    0x00
#define SPITRANS_BUSY       BIT(16)

#define MT7621_SPI_OPCODE   0x04
#define MT7621_SPI_DATA0    0x08
#define MT7621_SPI_DATA1    0x0C
#define MT7621_SPI_DATA2    0x10
#define MT7621_SPI_DATA3    0x14
#define MT7621_SPI_DATA4    0x18
#define MT7621_SPI_DATA5    0x1C
#define MT7621_SPI_DATA6    0x20
#define MT7621_SPI_DATA7    0x24
#define SPI_CTL_TX_RX_CNT_MASK  0xff
#define SPI_CTL_START       BIT(8)

#define NUM_OF_DATA_REGISTERS 0x09

#define MT7621_SPI_MASTER   0x28
#define MASTER_MORE_BUFMODE BIT(2)
#define MASTER_FULL_DUPLEX  BIT(10)
#define MASTER_RS_CLK_SEL   GENMASK(27, 16)
#define MASTER_RS_CLK_SEL_SHIFT 16
#define MASTER_RS_SLAVE_SEL GENMASK(31, 29)

#define MT7621_SPI_MOREBUF  0x2c
#define MT7621_SPI_POLAR    0x38
#define MT7621_SPI_SPACE    0x3c

#define MT7621_CPHA     BIT(5)
#define MT7621_CPOL     BIT(4)
#define MT7621_LSB_FIRST    BIT(3)

#define MT7621_SPI_REG_NUM (30) // TBD

typedef uint32_t u32;


#define FIFO_CAPACITY   32

static char buffer[NUM_OF_DATA_REGISTERS * sizeof(u32)];
static bool g_more_buf_mode = false;
static u32 g_cmd_bit_cnt = 0;
static u32 g_miso_bit_cnt = 0;
static u32 g_mosi_bit_cnt = 0;
static u32 g_spi_addr_ext = 0;
static u32 g_spi_addr_size = 0;


static void g_print_bytes(const char *title, char *buff, int len)
{
    printf("%s: ", title);
    for (int i = 0; i < len; i++) {
        if (i% 0x10 == 0) {
            printf("\n%04x: ", i);
        }

        printf("%02X ", buff[i]);
    }

    printf("\n");
}

static void mt7621_spi_txfifo_reset(MT7621SPIState *s)
{
    fifo8_reset(&s->tx_fifo);

    // s->regs[R_TXDATA] &= ~TXDATA_FULL;
    // s->regs[R_IP] &= ~IP_TXWM;
}

static void mt7621_spi_rxfifo_reset(MT7621SPIState *s)
{
    fifo8_reset(&s->rx_fifo);

    // s->regs[R_RXDATA] |= RXDATA_EMPTY;
    // s->regs[R_IP] &= ~IP_RXWM;
}

static void mt7621_spi_update_cs(MT7621SPIState *s)
{
    // int i;

    // for (i = 0; i < s->num_cs; i++) {
    //     if (s->regs[R_CSDEF] & (1 << i)) {
    //         qemu_set_irq(s->cs_lines[i], !(s->regs[R_CSMODE]));
    //     }
    // }
}

static void mt7621_spi_update_irq(MT7621SPIState *s)
{
    // int level;

    // if (fifo8_num_used(&s->tx_fifo) < s->regs[R_TXMARK]) {
    //     s->regs[R_IP] |= IP_TXWM;
    // } else {
    //     s->regs[R_IP] &= ~IP_TXWM;
    // }

    // if (fifo8_num_used(&s->rx_fifo) > s->regs[R_RXMARK]) {
    //     s->regs[R_IP] |= IP_RXWM;
    // } else {
    //     s->regs[R_IP] &= ~IP_RXWM;
    // }

    // level = s->regs[R_IP] & s->regs[R_IE] ? 1 : 0;
    // qemu_set_irq(s->irq, level);
}

static void mt7621_spi_reset(DeviceState *d)
{
    MT7621SPIState *s = MT7621_SPI(d);

    // memset(s->regs, 0, sizeof(s->regs));

    // /* The reset value is high for all implemented CS pins */
    // s->regs[R_CSDEF] = (1 << s->num_cs) - 1;

    // /* Populate register with their default value */
    // s->regs[R_SCKDIV] = 0x03;
    // s->regs[R_DELAY0] = 0x1001;
    // s->regs[R_DELAY1] = 0x01;

    mt7621_spi_txfifo_reset(s);
    mt7621_spi_rxfifo_reset(s);

    mt7621_spi_update_cs(s);
    mt7621_spi_update_irq(s);
}

// static void mt7621_spi_flush_txfifo(MT7621SPIState *s)
// {
//     uint8_t tx;
//     uint8_t rx;

//     while (!fifo8_is_empty(&s->tx_fifo)) {
//         tx = fifo8_pop(&s->tx_fifo);
//         rx = ssi_transfer(s->spi, tx);

//         printf("Wrote %02X Got %02x\n", tx, rx);

//         if (!fifo8_is_full(&s->rx_fifo)) {
//             // if (!(s->regs[R_FMT] & FMT_DIR)) {
//                 fifo8_push(&s->rx_fifo, rx);
//             // }
//         }
//     }
// }

/*static bool mt7621_spi_is_bad_reg(hwaddr addr, bool allow_reserved)
{
    bool bad;

    switch (addr) {
    /\* reserved offsets *\/
    case 0x08:
    case 0x0C:
    case 0x1C:
    case 0x20:
    case 0x24:
    case 0x30:
    case 0x34:
    case 0x38:
    case 0x3C:
    case 0x44:
    case 0x58:
    case 0x5C:
    case 0x68:
    case 0x6C:
        bad = allow_reserved ? false : true;
        break;
    default:
        bad = false;
    }

    if (addr >= (MT7621_SPI_REG_NUM << 2)) {
        bad = true;
    }

    return bad;
}*/

static uint64_t mt7621_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    MT7621SPIState *s = opaque;
    uint32_t r;

    // if (mt7621_spi_is_bad_reg(addr, true)) {
    //     qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read at address 0x%"
    //                   HWADDR_PRIx "\n", __func__, addr);
    //     return 0;
    // }

    // printf("mt7621_spi: Reading %08X %08X\n", (u32)addr, size);

    u32 *buffer_i = (u32 *)buffer;

    // addr >>= 2;
    switch (addr) {
    case MT7621_SPI_TRANS:
        /* Always ready! */
        r = (g_spi_addr_ext << 24);
        r += (g_spi_addr_size & 3) << 19;
        r += ((g_miso_bit_cnt / 8) & 7) << 4;
        r += ((g_mosi_bit_cnt / 8) & 7);
        break;
    case MT7621_SPI_MOREBUF:
        r = (g_cmd_bit_cnt & (BIT(6)-1)) << 24;
        r += (g_miso_bit_cnt & (BIT(9)-1)) << 12;
        r += (g_mosi_bit_cnt & (BIT(9)-1));

        break;
    case MT7621_SPI_OPCODE:
        // r = __builtin_bswap32(buffer_i[0]);
        r = buffer_i[0];
        break;
    case MT7621_SPI_DATA0:
    case MT7621_SPI_DATA1:
    case MT7621_SPI_DATA2:
    case MT7621_SPI_DATA3:
    case MT7621_SPI_DATA4:
    case MT7621_SPI_DATA5:
    case MT7621_SPI_DATA6:
    case MT7621_SPI_DATA7:
        r = buffer_i[(addr - MT7621_SPI_DATA0) / 4 + 1];
        // r = __builtin_bswap32(buffer_i[(addr - MT7621_SPI_DATA0) / 4 + 1]);
        break;
    default:
        r = 0;
        break;
    }

    printf("mt7621_spi_read: Addr is 0x%08X val %08x\n", (u32)addr, r);

    mt7621_spi_update_irq(s);

    return r;
}

static void mt7621_spi_write(void *opaque, hwaddr addr,
                             uint64_t val64, unsigned int size)
{
    MT7621SPIState *s = opaque;
    uint32_t value = val64;

    // if (mt7621_spi_is_bad_reg(addr, false)) {
    //     qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write at addr=0x%"
    //                   HWADDR_PRIx " value=0x%x\n", __func__, addr, value);
    //     return;
    // }

    u32 *buffer_i = (u32 *)buffer;

    // addr >>= 2;
    printf("mt7621_spi_write: Addr is 0x%08X Val %08x\n", (u32)addr, value);
    switch (addr) {
        case MT7621_SPI_OPCODE:
            // buffer_i[0] = __builtin_bswap32(value);
            buffer_i[0] = value;
            break;
        case MT7621_SPI_DATA0:
        case MT7621_SPI_DATA1:
        case MT7621_SPI_DATA2:
        case MT7621_SPI_DATA3:
        case MT7621_SPI_DATA4:
        case MT7621_SPI_DATA5:
        case MT7621_SPI_DATA6:
        case MT7621_SPI_DATA7:
            buffer_i[(addr - MT7621_SPI_DATA0)/4 + 1] = value;
            // buffer_i[(addr - MT7621_SPI_DATA0)/4 + 1] = __builtin_bswap32(value);
            break;
        case MT7621_SPI_MOREBUF:
            printf("Got MT7621_SPI_MOREBUF %08X Val\n", value);
            g_cmd_bit_cnt = ((value >> 24) & (BIT(6) - 1)); // Opcode stuff
            g_miso_bit_cnt = ((value >> 12) & (BIT(9)-1)); // To read
            g_mosi_bit_cnt = (value & (BIT(9)-1)); // To Write

            break;
        case MT7621_SPI_MASTER:
            g_more_buf_mode = (value & BIT(2));
            break;
        case MT7621_SPI_TRANS:
            // Start a SPI transaction

            g_spi_addr_ext = (value >> 24);
            g_spi_addr_size = (value >> 19) & 3;

            if (!g_more_buf_mode && (value & SPI_CTL_START) == 0) {
                g_cmd_bit_cnt = 0;
                g_miso_bit_cnt = ((value >> 4) & 7) * 8;
                g_mosi_bit_cnt = (value & 7) * 8;
            }

            printf("Got MT7621_SPI_TRANS %08X Val while rebuf %d %d %d\n", value, g_mosi_bit_cnt, g_cmd_bit_cnt, g_miso_bit_cnt);

            if ((value & SPI_CTL_START) == 0) {
                break;
            }

            qemu_set_irq(s->cs_line, 0);

            // if (cmd_bit_cnt > 0) {
            //     // WTF?
            //     if (fifo8_is_full(&s->tx_fifo)) {
            //         break;
            //     }

            //     // First buffer byte is swapped
            //     fifo8_push(&s->tx_fifo, __builtin_bswap32(buffer[0]));
            // }

            int to_write = (g_mosi_bit_cnt / 8);
            if (g_more_buf_mode) {
                to_write += (g_cmd_bit_cnt / 8);
            }

            if (to_write > 0) {
                // Get data from registers and put it into the SPI 
                // device
                g_print_bytes("Writing", buffer, to_write);

                if (g_more_buf_mode) {
                    buffer_i[0] = __builtin_bswap32(buffer_i[0]);
                }

                for (int i = 0; i < to_write; i++)
                {
                    // if (fifo8_is_full(&s->tx_fifo)) {
                    //     break;
                    // }
                    // fifo8_push(&s->tx_fifo, buffer[i]);

                    unsigned char rx = ssi_transfer(s->spi, buffer[i]);
                    printf("Wrote %02X Got %02X\n", buffer[i], rx);
                }
                
                // mt7621_spi_flush_txfifo();

                memset(buffer, 0, sizeof(buffer));
            }

            g_mosi_bit_cnt = 0;
            g_cmd_bit_cnt = 0;

            int to_read = (g_miso_bit_cnt / 8);

            if (to_read > 0) {
                // Write data into the registers from the SPI
                // device
                memset(buffer, 0, sizeof(buffer));
                
                for (int i = 4; i < 4 + to_read; i++)
                {
                    // if (fifo8_is_empty(&s->rx_fifo)) {
                    //     break;
                    // }

                    buffer[i] = ssi_transfer(s->spi, 0);
                }

                g_print_bytes("Reading", buffer + 4, to_read);

                // buffer_i[0] = __builtin_bswap32(buffer[0]);
            }

            g_miso_bit_cnt = 0;

            qemu_set_irq(s->cs_line, 1);

            break;
    default:
        // s->regs[addr] = value;
        break;
    }

    mt7621_spi_update_irq(s);
}

static const MemoryRegionOps mt7621_spi_ops = {
    .read = mt7621_spi_read,
    .write = mt7621_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void mt7621_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    MT7621SPIState *s = MT7621_SPI(dev);
    // int i;

    s->spi = ssi_create_bus(dev, "spi");
    sysbus_init_irq(sbd, &s->irq);

    sysbus_init_irq(sbd, &s->cs_line);
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_line, "cs", 1);

    // s->cs_lines = g_new0(qemu_irq, s->num_cs);
    // for (i = 0; i < s->num_cs; i++) {
    //     sysbus_init_irq(sbd, &s->cs_lines[i]);
    // }

    memory_region_init_io(&s->mmio, OBJECT(s), &mt7621_spi_ops, s,
                          TYPE_MT7621_SPI, 0x100);
    sysbus_init_mmio(sbd, &s->mmio);

    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);
}

static Property mt7621_spi_properties[] = {
    // DEFINE_PROP_UINT32("num-cs", MT7621SPIState, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void mt7621_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, mt7621_spi_properties);
    dc->reset = mt7621_spi_reset;
    dc->realize = mt7621_spi_realize;
}

static const TypeInfo mt7621_spi_info = {
    .name           = TYPE_MT7621_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(MT7621SPIState),
    .class_init     = mt7621_spi_class_init,
};

static void mt7621_spi_register_types(void)
{
    type_register_static(&mt7621_spi_info);
}

type_init(mt7621_spi_register_types)

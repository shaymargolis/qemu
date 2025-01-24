/*
 * Mediatek mt7628 ethernet emulation
 *
 * Copyright (C) 2023 Lu Hui <luhux76@gmail.com>
 *
 * some code from uboot drivers/net/mt7628-eth.c:
 * Copyright (C) 2018 Stefan Roese <sr@denx.de>
 * some code from openwrt
 * target/linux/ramips/files/drivers/net/ethernet/ralink/mtk_eth_soc.c
 * Copyright (C) 2009-2015 John Crispin <blogic@openwrt.org>
 * Copyright (C) 2009-2015 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2013-2015 Michael Lee <igvtee@gmail.com>
 * some code from ./allwinner_emac.c:
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
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
#include "hw/irq.h"
#include "qemu/module.h"
#include "hw/net/mt7628-eth.h"
#include "sysemu/runstate.h"
#include "sysemu/dma.h"
#include "exec/address-spaces.h"

/*
 * note:
 * test on
 * mainline uboot 2022.10
 * NetBSD 9.0
 */

/* #define DEBUG_MT7628_ETH 1 */

#ifdef DEBUG_MT7628_ETH
#define DPRINTF(fmt, ...) \
do { printf("mt7628-eth: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

/* register offsets */
enum {
    REG_TX_BASE_PTR0 = 0x00800,
    REG_TX_MAX_CNT0  = 0x00804,
    REG_TX_CTX_IDX0  = 0x00808,
    REG_TX_DTX_IDX0  = 0x0080C,

    REG_RX_BASE_PTR0 = 0x00900,
    REG_RX_MAX_CNT0  = 0x00904,
    REG_RX_CTX_IDX0  = 0x00908,
    REG_RX_DTX_IDX0  = 0x0090C,

    REG_PDMA_GLO_CFG = 0x00A04,
    REG_INT_STATUS   = 0x00A20,
    REG_INT_MASK     = 0x00A28,

    REG_MAC_ADDR_L   = 0x00C0C,
    REG_MAC_ADDR_H   = 0x00C10,

    REG_TX_PACKETS   = 0x00D00,
    REG_TX_BYTES     = 0x00D04,
    REG_RX_PACKETS   = 0x00D08,
    REG_RX_BYTES     = 0x00D0C,

    REG_SWITCH_PCR0  = 0x100C0,
    REG_SWITCH_PCR1  = 0x100C4,
};

/* register shift */
#define PCR0_PHY_ADDR 0
#define PCR0_PHY_ADDR_REG 8
#define PCR0_WT_PHY_CMD 13
#define PCR0_RD_PHY_CMD 14
#define PCR0_WT_DATA 16

#define PCR1_WT_DONE 0
#define PCR1_RD_RDY  1
#define PCR1_RD_DATA 16

#define TX_DMA_ENABLE 0
#define RX_DMA_ENABLE 2
#define TX_DMA_STATUS 1
#define RX_DMA_STATUS 3

#define INT_RX_DONE 16
#define INT_TX_DONE 0

/* TODO: need dump from real board */
#define MT7628_PHYID1       0xBADB
#define MT7628_PHYID2       0xADBA

static void mt7628_eth_update_irq(mt7628EthState *s)
{
    uint32_t irq = s->irq_stat & s->irq_mask;
    // DPRINTF("irq stat: %08x\n", s->irq_stat);
    // DPRINTF("irq mask: %08x\n", s->irq_mask);
    qemu_set_irq(s->irq, !!irq);
}

static void mt7628_eth_mii_set_link(mt7628EthState *s, bool link_ok)
{
    if (link_ok) {
        s->bmsr |= MII_BMSR_LINK_ST | MII_BMSR_AN_COMP;
        s->anlpar |= MII_ANAR_TXFD | MII_ANAR_10FD | MII_ANAR_10 |
                       MII_ANAR_CSMACD;
    } else {
        s->bmsr &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
        s->anlpar = MII_ANAR_TX;
    }
}

static void mt7628_eth_mii_reset(mt7628EthState *s, bool link_ok)
{
    s->bmcr = MII_BMCR_FD | MII_BMCR_AUTOEN | MII_BMCR_SPEED;
    s->bmsr = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD | MII_BMSR_10T_FD |
                MII_BMSR_10T_HD | MII_BMSR_MFPS | MII_BMSR_AUTONEG;
    s->anar = MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD | MII_ANAR_10 |
                MII_ANAR_CSMACD;
    s->anlpar = MII_ANAR_TX;

    mt7628_eth_mii_set_link(s, link_ok);
}

static void mt7628_eth_mii_read(mt7628EthState *s)
{
    if (s->pcr_rd_phy_cmd == 0) {
        return;
    }
    switch (s->pcr_phy_addr_reg) {
    case MII_BMCR:
        s->pcr_rd_data = s->bmcr;
        break;
    case MII_BMSR:
        s->pcr_rd_data = s->bmsr;
        break;
    case MII_PHYID1:
        s->pcr_rd_data = MT7628_PHYID1;
        break;
    case MII_PHYID2:
        s->pcr_rd_data = MT7628_PHYID2;
        break;
    case MII_ANAR:
        s->pcr_rd_data = s->anar;
        break;
    case MII_ANLPAR:
        s->pcr_rd_data = s->anlpar;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: reg %x is not imp\n",
                       __func__, s->pcr_phy_addr_reg);
        break;
    }
    s->pcr_rd_phy_cmd = 0; /* transfer ok, reset this bit */
}

static void mt7628_eth_mii_write(mt7628EthState *s)
{
    if (s->pcr_wt_phy_cmd == 0) {
        return;
    }
    NetClientState *nc;
    uint16_t value = s->pcr_wt_data;
    switch (s->pcr_phy_addr_reg) {
    case MII_BMCR:
        if (value & MII_BMCR_RESET) {
            nc = qemu_get_queue(s->nic);
            mt7628_eth_mii_reset(s, !nc->link_down);
        } else {
            s->bmcr = value;
        }
        break;
    case MII_ANAR:
        s->anar = value;
        break;
    default:
         qemu_log_mask(LOG_UNIMP, "%s: reg %x is not imp\n",
                       __func__, s->pcr_phy_addr_reg);
         break;
    }
    s->pcr_wt_phy_cmd = 0; /* transfer ok, reset this bit */
}

#define PKT_LEN_SHIFT 16
#define PKT_DMA_DONE 31

#define GET_RING_ADDR(base, idx) ((base) +\
               (sizeof(struct mt7628_eth_ring) * idx))

static uint32_t mt7628_eth_dma_ring_prev(uint32_t idx, uint32_t max)
{
    int64_t idx64;
    idx64 = idx;
    idx64--;
    if (idx64 < 0) {
        idx = max - 1;
    } else {
        idx--;
    }
    return idx;
}

static uint32_t mt7628_eth_dma_ring_next(uint32_t idx, uint32_t max)
{
    int64_t idx64, max64;
    idx64 = idx;
    max64 = max;
    idx64++;
    if (idx64 >= max64) {
        idx = 0;
    } else {
        idx++;
    }
    return idx;
}

static void mt7628_eth_dma_ring_read(uint32_t base, uint32_t idx,
                                   struct mt7628_eth_ring *ring)
{
    uint32_t ring_addr = GET_RING_ADDR(base, idx);
    DPRINTF("%s: read dma ring from %08x\n", __func__, ring_addr);
    dma_memory_read(&address_space_memory, ring_addr, ring, sizeof(*ring),
                    MEMTXATTRS_UNSPECIFIED);
}

static void mt7628_eth_dma_ring_write(uint32_t base, uint32_t idx,
                                   struct mt7628_eth_ring *ring)
{
    uint32_t ring_addr = GET_RING_ADDR(base, idx);
    DPRINTF("%s: write dma ring to %08x\n", __func__, ring_addr);
    dma_memory_write(&address_space_memory, ring_addr, ring, sizeof(*ring),
                    MEMTXATTRS_UNSPECIFIED);
}

static bool mt7628_eth_can_send(mt7628EthState *s)
{
    if (s->tx_dma_enable == 0) {
        DPRINTF("%s: tx dma disable\n", __func__);
        return 0;
    }
    if (s->tx_dma_busy == 1) {
        DPRINTF("%s: tx dma busy\n", __func__);
        return 0;
    }
    if (s->tx_ring_base == 0) {
        DPRINTF("%s: tx dma ring addr base not setup\n", __func__);
        return 0;
    }
    if (s->tx_ring_maxi == 0) {
        DPRINTF("%s: tx dma ring range not setup\n", __func__);
        return 0;
    }
    if (s->tx_ring_dmai >= s->tx_ring_maxi) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: tx dma idx is out of range\n", __func__);
        return 0;
    }
    if (s->tx_ring_cpui >= s->tx_ring_maxi) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: tx cpu idx is out of range\n", __func__);
        return 0;
    }
    if (s->tx_ring_dmai == s->tx_ring_cpui) {
        s->tx_ring_dmai = mt7628_eth_dma_ring_prev(s->tx_ring_cpui,
                                                   s->tx_ring_maxi);
    }

    struct mt7628_eth_ring tx_desc;

    mt7628_eth_dma_ring_read(s->tx_ring_base, s->tx_ring_dmai,
                             &tx_desc);
    if (tx_desc.addr == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: pkt address is not setup\n", __func__);
        return 0;
    }
    if (test_bit(PKT_DMA_DONE, (void *)&tx_desc.info1)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: pkt is send ok, don't send again\n", __func__);
        return 0;
    }
    if (extract32(tx_desc.info1, PKT_LEN_SHIFT, 14) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: pkt length not setup\n", __func__);
        return 0;
    }

    return 1;
}
void print_bytes(const char *title, unsigned char *buff, int len);

void print_bytes(const char *title, unsigned char *buff, int len)
{
    DPRINTF("[%s for len %d]: ", title, len);
    for (int i = 0; i< len ; i++) {
        if (i%0x10 == 0) {
            DPRINTF("\n%08X: ", i);
        }

        DPRINTF("%02X  ", buff[i]);
    }

    DPRINTF("\n");
}

static ssize_t mt7628_eth_send(void *opaque)
{
    mt7628EthState *s = MT7628_ETH(opaque);
    NetClientState *nc = qemu_get_queue(s->nic);

    if (!mt7628_eth_can_send(s)) {
        return -1;
    }
    DPRINTF("%s: tx dma idx is : %d\n", __func__, s->tx_ring_dmai);
    DPRINTF("%s: tx cpu idx is : %d\n", __func__, s->tx_ring_cpui);

    /* start transfer */
    s->tx_dma_busy = 1;

    struct mt7628_eth_ring tx_desc;
    mt7628_eth_dma_ring_read(s->tx_ring_base, s->tx_ring_dmai, &tx_desc);
    uint16_t pkt_len;
    pkt_len = extract32(tx_desc.info1, PKT_LEN_SHIFT, 14);
    dma_memory_read(&address_space_memory, tx_desc.addr,
                    s->tx_buffer, pkt_len, MEMTXATTRS_UNSPECIFIED);
    print_bytes("mt7628_eth_send", s->tx_buffer, pkt_len);
    qemu_send_packet(nc, s->tx_buffer, pkt_len);
    DPRINTF("%s: pkt send ok, set dma done bit\n", __func__);
    set_bit(PKT_DMA_DONE, (void *)(&tx_desc.info1));
    mt7628_eth_dma_ring_write(s->tx_ring_base, s->tx_ring_dmai,
                             &tx_desc);
    s->tx_bytes += pkt_len;
    s->tx_packets += 1;
    DPRINTF("%s: tx bytes total %x\n", __func__, s->tx_bytes);
    DPRINTF("%s: tx packets total %x\n", __func__, s->tx_packets);

    /* interrupt */
    s->irq_stat |= (1 << INT_TX_DONE);
    mt7628_eth_update_irq(s);

    /* point to next */
    s->tx_ring_dmai = mt7628_eth_dma_ring_next(s->tx_ring_dmai,
                                               s->tx_ring_maxi);
    /* transfer end */
    s->tx_dma_busy = 0;
    return pkt_len;
}

static uint64_t mt7628_eth_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    mt7628EthState *s = MT7628_ETH(opaque);

    uint32_t val = 0x0;
    switch (addr) {
    case REG_TX_BASE_PTR0:
        val = s->tx_ring_base;
        return val;
    case REG_TX_MAX_CNT0:
        val = s->tx_ring_maxi;
        return val;
    case REG_TX_CTX_IDX0:
        val = s->tx_ring_cpui;
        return val;
    case REG_TX_DTX_IDX0:
        val = s->tx_ring_dmai;
        return val;
    case REG_RX_BASE_PTR0:
        val = s->rx_ring_base;
        return val;
    case REG_RX_MAX_CNT0:
        val = s->rx_ring_maxi;
        return val;
    case REG_RX_CTX_IDX0:
        val = s->rx_ring_cpui;
        return val;
    case REG_RX_DTX_IDX0:
        val = s->rx_ring_dmai;
        return val;
    case REG_PDMA_GLO_CFG:
        val |= s->rx_dma_enable << RX_DMA_ENABLE;
        val |= s->tx_dma_enable << TX_DMA_ENABLE;
        val |= s->rx_dma_busy << RX_DMA_STATUS;
        val |= s->tx_dma_busy << TX_DMA_STATUS;
        return val;
    case REG_INT_STATUS:
        return s->irq_stat;
    case REG_INT_MASK:
        return s->irq_mask;
    case REG_TX_BYTES:
        val = s->tx_bytes;
        return val;
    case REG_TX_PACKETS:
        val = s->tx_packets;
        return val;
    case REG_RX_BYTES:
        val = s->rx_bytes;
        return val;
    case REG_RX_PACKETS:
        val = s->rx_packets;
        return val;
    case REG_SWITCH_PCR0:
        val |= s->pcr_phy_addr     << PCR0_PHY_ADDR;
        val |= s->pcr_phy_addr_reg << PCR0_PHY_ADDR_REG;
        val |= s->pcr_wt_phy_cmd   << PCR0_WT_PHY_CMD;
        val |= s->pcr_rd_phy_cmd   << PCR0_RD_PHY_CMD;
        val |= s->pcr_wt_data      << PCR0_WT_DATA;
        return val;
    case REG_SWITCH_PCR1:
        s->pcr_wt_done = !s->pcr_wt_done; /* simple switch it */
        s->pcr_rd_ready = !s->pcr_rd_ready; /* simple switch it */
        val |= s->pcr_wt_done  << PCR1_WT_DONE;
        val |= s->pcr_rd_ready << PCR1_RD_RDY;
        val |= s->pcr_rd_data  << PCR1_RD_DATA;
        return val;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: addr %lx is not imp\n",
                      __func__, addr);
        return 0x0;
    }
}

static void mt7628_eth_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    mt7628EthState *s = opaque;
    switch (addr) {
    case REG_TX_BASE_PTR0:
        s->tx_ring_base = val;
        DPRINTF("%s: set tx ring base to %08x\n", __func__, s->tx_ring_base);
        break;
    case REG_TX_MAX_CNT0:
        s->tx_ring_maxi = val;
        DPRINTF("%s: set tx ring idx max to %08x\n", __func__,
                 s->tx_ring_maxi);
        break;
    case REG_TX_CTX_IDX0:
        s->tx_ring_cpui = val;
        DPRINTF("%s: set tx ring cpu idx to %08x\n", __func__,
                s->tx_ring_cpui);
        s->tx_ring_dmai = mt7628_eth_dma_ring_prev(s->tx_ring_cpui,
                                                   s->tx_ring_maxi);
        DPRINTF("%s: set tx ring dma idx from cpu idx %08x\n", __func__,
                s->tx_ring_dmai);
        mt7628_eth_send(s);
        break;
    case REG_RX_BASE_PTR0:
        s->rx_ring_base = val;
        DPRINTF("%s: set rx ring base to %08x\n", __func__, s->rx_ring_base);
        break;
    case REG_RX_MAX_CNT0:
        s->rx_ring_maxi = val;
        DPRINTF("%s: set rx ring idx max to %08x\n", __func__,
                s->rx_ring_maxi);
        break;
    case REG_RX_CTX_IDX0:
        s->rx_ring_cpui = val;
        DPRINTF("%s: set rx ring cpu idx to %08x\n", __func__,
                s->rx_ring_cpui);
        s->rx_ring_dmai = mt7628_eth_dma_ring_next(s->rx_ring_cpui,
                                                   s->rx_ring_maxi);
        break;
    case REG_RX_DTX_IDX0:
        s->rx_ring_dmai = val;
        DPRINTF("%s: set rx ring dma idx to %08x\n", __func__,
                s->rx_ring_dmai);
        break;
    case REG_MAC_ADDR_L:
        s->conf.macaddr.a[5] = extract32((uint32_t)val, 0, 8);
        s->conf.macaddr.a[4] = extract32((uint32_t)val, 8, 8);
        s->conf.macaddr.a[3] = extract32((uint32_t)val, 16, 8);
        s->conf.macaddr.a[2] = extract32((uint32_t)val, 24, 8);
        break;
    case REG_MAC_ADDR_H:
        s->conf.macaddr.a[1] = extract32((uint32_t)val, 0, 8);
        s->conf.macaddr.a[0] = extract32((uint32_t)val, 8, 8);
        DPRINTF("%s: set mac address to %02x-%02x-%02x-%02x-%02x-%02x\n",
                __func__,
                s->conf.macaddr.a[5], s->conf.macaddr.a[4],
                s->conf.macaddr.a[3], s->conf.macaddr.a[2],
                s->conf.macaddr.a[1], s->conf.macaddr.a[0]);
        break;
    case REG_PDMA_GLO_CFG:
        s->rx_dma_enable = test_bit(RX_DMA_ENABLE, (void *)&val);
        s->tx_dma_enable = test_bit(TX_DMA_ENABLE, (void *)&val);
        break;
    case REG_INT_STATUS:
        s->irq_stat = ~val;
        mt7628_eth_update_irq(s);
        break;
    case REG_INT_MASK:
        s->irq_mask = val;
        mt7628_eth_update_irq(s);
        break;
    case REG_SWITCH_PCR0:
        s->pcr_phy_addr = extract32(val, PCR0_PHY_ADDR, 5);
        s->pcr_phy_addr_reg = extract32(val, PCR0_PHY_ADDR_REG, 5);
        s->pcr_wt_phy_cmd = test_bit(PCR0_WT_PHY_CMD, (void *)&val);
        s->pcr_rd_phy_cmd = test_bit(PCR0_RD_PHY_CMD, (void *)&val);
        s->pcr_wt_data = extract32(val, PCR0_WT_DATA, 16);
        mt7628_eth_mii_read(s);
        mt7628_eth_mii_write(s);
        break;
    case REG_SWITCH_PCR1:
        s->pcr_wt_done  = test_bit(PCR1_WT_DONE, (void *)&val);
        s->pcr_rd_ready = test_bit(PCR1_RD_RDY, (void *)&val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: addr %lx is not imp\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps mt7628_eth_ops = {
    .read = mt7628_eth_read,
    .write = mt7628_eth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl.min_access_size = 4,
};

static void mt7628_eth_reset(DeviceState *dev)
{
    mt7628EthState *s = MT7628_ETH(dev);
    NetClientState *nc = qemu_get_queue(s->nic);

    s->bmcr = 0;
    s->bmsr = 0;
    s->anar = 0;
    s->anlpar = 0;
    mt7628_eth_mii_reset(s, !nc->link_down);

    s->irq_stat = 0;
    s->irq_mask = 0;

    s->rx_dma_enable = 0;
    s->tx_dma_enable = 0;
    s->rx_dma_busy = 0;
    s->tx_dma_busy = 0;

    s->rx_ring_base = 0;
    s->rx_ring_maxi = 0;
    s->rx_ring_cpui = 0;
    s->rx_ring_dmai = 0;

    s->tx_ring_base = 0;
    s->tx_ring_maxi = 0;
    s->tx_ring_cpui = 0;
    s->tx_ring_dmai = 0;
    memset(s->tx_buffer, 0x0, MT7628_ETH_PKT_MAX_LEN);

    s->tx_bytes = 0;
    s->tx_packets = 0;
    s->rx_bytes = 0;
    s->tx_packets = 0;

    s->pcr_wt_data = 0;
    s->pcr_rd_phy_cmd = 0;
    s->pcr_wt_phy_cmd = 0;
    s->pcr_phy_addr_reg = 0;
    s->pcr_phy_addr = 0;
    s->pcr_rd_data = 0;
    s->pcr_rd_ready = 0;
    s->pcr_wt_done = 0;
}

static void mt7628_eth_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    mt7628EthState *s = MT7628_ETH(obj);

    /* Memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &mt7628_eth_ops, s,
                          TYPE_MT7628_ETH, 96 * KiB);
    sysbus_init_mmio(sbd, &s->iomem);

    /* interrupt */
    sysbus_init_irq(sbd, &s->irq);
}

static void mt7628_eth_set_link(NetClientState *nc)
{
    mt7628EthState *s = qemu_get_nic_opaque(nc);

    mt7628_eth_mii_set_link(s, !nc->link_down);
}

static bool mt7628_eth_can_receive(NetClientState *nc)
{
    mt7628EthState *s = qemu_get_nic_opaque(nc);
    if (s->rx_dma_enable == 0) {
        DPRINTF("%s: rx dma disable\n", __func__);
        return 0;
    }
    if (s->rx_dma_busy == 1) {
        DPRINTF("%s: rx dma busy\n", __func__);
        return 0;
    }
    if (s->rx_ring_base == 0) {
        DPRINTF("%s: rx dma ring base not setup\n", __func__);
        return 0;
    }
    if (s->rx_ring_maxi == 0) {
        DPRINTF("%s: rx dma range not setup\n", __func__);
        return 0;
    }
    if (s->rx_ring_dmai >= s->rx_ring_maxi) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: rx ring dma index is out of range",
                     __func__);
        return 0;
    }
    if (s->rx_ring_cpui >= s->rx_ring_maxi) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: rx ring cpu index is out of range",
                      __func__);
        return 0;
    }
    if (s->rx_ring_cpui == s->rx_ring_dmai) {
        DPRINTF("%s: rx ring cpu idx same with dma idx\n", __func__);
        return 0;
    }

    struct mt7628_eth_ring rx_desc;
    mt7628_eth_dma_ring_read(s->rx_ring_base, s->rx_ring_dmai,
                             &rx_desc);
    if (rx_desc.addr == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: rx desc addr not setup", __func__);
        return 0;
    }
    if (test_bit(PKT_DMA_DONE, (void *)&rx_desc.info1)) {
        DPRINTF("%s: this pkt is used\n", __func__);
        return 0; /* used */
    }

    return 1;
}

static ssize_t mt7628_eth_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    mt7628EthState *s = qemu_get_nic_opaque(nc);
    if (!mt7628_eth_can_receive(nc)) {
        return -1;
    }
    if (size > MTK_QDMA_PAGE_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: packet is too large for recive\n",
                      __func__);
        return -1;
    }

    struct mt7628_eth_ring rx_desc;

    /* start recv */
    s->rx_dma_busy = 1;



    DPRINTF("%s: rx dma idx is : %d\n", __func__, s->rx_ring_dmai);
    DPRINTF("%s: rx cpu idx is : %d\n", __func__, s->rx_ring_cpui);

    /* recv */
    mt7628_eth_dma_ring_read(s->rx_ring_base, s->rx_ring_dmai,
                             &rx_desc);
    rx_desc.info1 |= size << PKT_LEN_SHIFT;
    rx_desc.info1 |= 1 << PKT_DMA_DONE;
    dma_memory_write(&address_space_memory, rx_desc.addr,
                    buf, size, MEMTXATTRS_UNSPECIFIED);
    mt7628_eth_dma_ring_write(s->rx_ring_base, s->rx_ring_dmai,
                             &rx_desc);
    s->rx_bytes += size;
    s->rx_packets += 1;
    DPRINTF("%s: rx bytes total %x\n", __func__, s->rx_bytes);
    DPRINTF("%s: rx packets total %x\n", __func__, s->rx_packets);

    /* interrupt */
    s->irq_stat |= (1 << INT_RX_DONE);
    mt7628_eth_update_irq(s);

    /* point to next */
    s->rx_ring_dmai = mt7628_eth_dma_ring_next(s->rx_ring_dmai,
                                               s->rx_ring_maxi);

    /* recv end */
    s->rx_dma_busy = 0;
    return size;
}

static NetClientInfo net_mt7628_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mt7628_eth_can_receive,
    .receive = mt7628_eth_receive,
    .link_status_changed = mt7628_eth_set_link,
};


static void mt7628_eth_realize(DeviceState *dev, Error **errp)
{
    mt7628EthState *s = MT7628_ETH(dev);

    s->nic = qemu_new_nic(&net_mt7628_eth_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}


static const VMStateDescription mt7628_eth_vmstate = {
    .name = "mt7628-eth",
    .version_id = 1,
    .minimum_version_id = 1,
};

static Property mt7628_eth_properties[] = {
    DEFINE_NIC_PROPERTIES(mt7628EthState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void mt7628_eth_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = mt7628_eth_realize;
    dc->reset = mt7628_eth_reset;
    dc->vmsd = &mt7628_eth_vmstate;
    device_class_set_props(dc, mt7628_eth_properties);
}

static const TypeInfo mt7628_eth_info = {
    .name = TYPE_MT7628_ETH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = mt7628_eth_init,
    .instance_size = sizeof(mt7628EthState),
    .class_init = mt7628_eth_class_init,
};

static void mt7628_eth_register(void)
{
    type_register_static(&mt7628_eth_info);
}

type_init(mt7628_eth_register)

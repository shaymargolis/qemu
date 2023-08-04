/*
 * Allwinner Sun8i Ethernet MAC emulation
 *
 * Copyright (C) 2019 Niek Linnenbank <nieklinnenbank@gmail.com>
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
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "trace.h"
#include "net/checksum.h"
#include "qemu/module.h"
#include "exec/cpu-common.h"
#include "sysemu/dma.h"
#include "rt3050-mac.h"

#include "net/eth.h"
#include "net/net.h"
#include "sysemu/dma.h"


// static inline MemTxResult dma_memory_read(AddressSpace *as, dma_addr_t addr,
//                                           void *buf, dma_addr_t len,
//                                           MemTxAttrs attrs)
// static inline MemTxResult dma_memory_write(AddressSpace *as, dma_addr_t addr,
//                                            const void *buf, dma_addr_t len,
//                                            MemTxAttrs attrs)

// MemoryRegion *address_space_mem = get_system_memory();

#define RT5350_PDMA_OFFSET (0x800)

#define RT5350_TX_BASE_PTR0 (RT5350_PDMA_OFFSET + 0x00)
#define RT5350_TX_MAX_CNT0  (RT5350_PDMA_OFFSET + 0x04)
#define RT5350_TX_CTX_IDX0  (RT5350_PDMA_OFFSET + 0x08)
#define RT5350_TX_DTX_IDX0  (RT5350_PDMA_OFFSET + 0x0C)
#define RT5350_TX_BASE_PTR1 (RT5350_PDMA_OFFSET + 0x10)
#define RT5350_TX_MAX_CNT1  (RT5350_PDMA_OFFSET + 0x14)
#define RT5350_TX_CTX_IDX1  (RT5350_PDMA_OFFSET + 0x18)
#define RT5350_TX_DTX_IDX1  (RT5350_PDMA_OFFSET + 0x1C)
#define RT5350_TX_BASE_PTR2 (RT5350_PDMA_OFFSET + 0x20)
#define RT5350_TX_MAX_CNT2  (RT5350_PDMA_OFFSET + 0x24)
#define RT5350_TX_CTX_IDX2  (RT5350_PDMA_OFFSET + 0x28)
#define RT5350_TX_DTX_IDX2  (RT5350_PDMA_OFFSET + 0x2C)
#define RT5350_TX_BASE_PTR3 (RT5350_PDMA_OFFSET + 0x30)
#define RT5350_TX_MAX_CNT3  (RT5350_PDMA_OFFSET + 0x34)
#define RT5350_TX_CTX_IDX3  (RT5350_PDMA_OFFSET + 0x38)
#define RT5350_TX_DTX_IDX3  (RT5350_PDMA_OFFSET + 0x3C)
#define RT5350_RX_BASE_PTR0 (RT5350_PDMA_OFFSET + 0x100)
#define RT5350_RX_MAX_CNT0  (RT5350_PDMA_OFFSET + 0x104)
#define RT5350_RX_CALC_IDX0 (RT5350_PDMA_OFFSET + 0x108)
#define RT5350_RX_DRX_IDX0  (RT5350_PDMA_OFFSET + 0x10C)
#define RT5350_RX_BASE_PTR1 (RT5350_PDMA_OFFSET + 0x110)
#define RT5350_RX_MAX_CNT1  (RT5350_PDMA_OFFSET + 0x114)
#define RT5350_RX_CALC_IDX1 (RT5350_PDMA_OFFSET + 0x118)
#define RT5350_RX_DRX_IDX1  (RT5350_PDMA_OFFSET + 0x11C)
#define RT5350_PDMA_GLO_CFG (RT5350_PDMA_OFFSET + 0x204)
#define RT5350_PDMA_RST_CFG (RT5350_PDMA_OFFSET + 0x208)
#define RT5350_DLY_INT_CFG  (RT5350_PDMA_OFFSET + 0x20c)
#define RT5350_FE_INT_STATUS    (RT5350_PDMA_OFFSET + 0x220)
#define RT5350_FE_INT_ENABLE    (RT5350_PDMA_OFFSET + 0x228)
#define RT5350_PDMA_SCH_CFG (RT5350_PDMA_OFFSET + 0x280)

static uint64_t ralink_mac_driver_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    RalinkMacState *s = opaque;
    u32 val;

    printf("[%s] Read from %08X\n", __func__, (u32)addr);

    val = 0;

    switch (addr) {
    case RT5350_TX_BASE_PTR0:
        val = s->nic_tx_ring;
        break;
    case RT5350_TX_MAX_CNT0:
        val = s->nic_tx_max;
        break;
    case RT5350_TX_CTX_IDX0:
        val = s->nic_tx_cpu_pointer;
        break;
    case RT5350_TX_DTX_IDX0:
        val = s->nic_tx_dma_pointer;
        break;
    case RT5350_RX_BASE_PTR0:
        val = s->nic_rx_ring;
        break;
    case RT5350_RX_MAX_CNT0:
        val = s->nic_rx_max;
        break;
    case RT5350_RX_CALC_IDX0:
        val = s->nic_rx_cpu_pointer;
        break;
    case RT5350_RX_DRX_IDX0:
        val = s->nic_rx_dma_pointer;
        break;
    }

    return val;
}

static void g_print_bytes(const char *title, u8 *buff, int len)
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

static void ralink_mac_packet_sent(RalinkMacState *s, AddressSpace *as)
{
    int i = 0;
    int res = MEMTX_OK;
    u32 packet_ring[4];
    u8 *packet = NULL;

    printf("Starting to send %d/%d size %d\n", s->nic_tx_dma_pointer, s->nic_tx_cpu_pointer, s->nic_tx_max);

    if (s->nic_tx_max == 0) {
        return;
    }

    for (i = s->nic_tx_dma_pointer; i != s->nic_tx_cpu_pointer; i = ((i + 1) % s->nic_tx_max)) {
        res = dma_memory_read(as, s->nic_tx_ring + (i * 0x10), packet_ring, 0x10, MEMTXATTRS_UNSPECIFIED);
        if (MEMTX_OK != res) {
            printf("ERROR!\n");
        }

        g_print_bytes("PacketRing", (u8 *)packet_ring, 0x10);

        int packet_len = (packet_ring[1] & 0xfffffff) >> 16;
        packet = malloc(packet_len);

        res = dma_memory_read(as, packet_ring[0], packet, packet_len, MEMTXATTRS_UNSPECIFIED);
        if (MEMTX_OK != res) {
            printf("ERROR! 2\n");
        }

        g_print_bytes("Packet", packet, packet_len);

        NetClientState *nc = qemu_get_queue(s->nic);
        qemu_send_packet(nc, packet, packet_len);
    }

    s->nic_tx_dma_pointer = i;
}

static void ralink_mac_driver_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    RalinkMacState *s = opaque;

    // CPUMIPSState *env;
    // env = opaque;
    u32 val = (u32)value;
    // MemoryRegion *address_space_mem = get_system_memory();

    printf("[%s] Write to %08X val %08X (%d)\n", __func__, (u32)addr, val, (u32)size);

    // static inline MemTxResult dma_memory_read(AddressSpace *as, dma_addr_t addr,
    //                                           void *buf, dma_addr_t len,
    //                                           MemTxAttrs attrs)

    switch (addr) {
    case RT5350_TX_BASE_PTR0:
        s->nic_tx_ring = val;
        break;
    case RT5350_TX_MAX_CNT0:
        s->nic_tx_max = val;
        break;
    case RT5350_TX_CTX_IDX0:
        s->nic_tx_cpu_pointer = val;

        // When the CPU moves, We will send the given packet...
        ralink_mac_packet_sent(s, &s->dma_as);
        break;
    case RT5350_TX_DTX_IDX0:
        s->nic_tx_dma_pointer = val;
        break;
    case RT5350_RX_BASE_PTR0:
        s->nic_rx_ring = val;
        break;
    case RT5350_RX_MAX_CNT0:
        s->nic_rx_max = val;
        break;
    case RT5350_RX_CALC_IDX0:
        s->nic_rx_cpu_pointer = val;
        break;
    case RT5350_RX_DRX_IDX0:
        s->nic_rx_dma_pointer = val;
        break;
    }
}

const MemoryRegionOps ralink_mac_driver_ops = {
    .read = ralink_mac_driver_read,
    .write = ralink_mac_driver_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 4,
    .impl.max_access_size = 4,
};

static bool ralink_mac_can_receive(NetClientState *nc)
{
    // ralink_macState *s = qemu_get_nic_opaque(nc);

    // return ralink_macx_rx_ready(&s->parent_obj, s->mac_reg) &&
    //     ralink_mac_has_rxbufs(s, 1) && !timer_pending(s->flush_queue_timer);

    return true;
}

static ssize_t ralink_mac_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    // Nothing
    return size;
}

static void ralink_mac_set_link_status(NetClientState *nc)
{
    // Nothing
}


static NetClientInfo net_ralink_mac_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ralink_mac_can_receive,
    .receive = ralink_mac_receive,
    .link_status_changed = ralink_mac_set_link_status,
};

static void ralink_mac_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    RalinkMacState *s = RALINK_MAC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &ralink_mac_driver_ops,
                           s, TYPE_RALINK_MAC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->nic_rx_ring = 0;
    s->nic_rx_dma_pointer = 0;
    s->nic_rx_cpu_pointer = 0;
    s->nic_rx_max = 0;

    s->nic_tx_ring = 0;
    s->nic_tx_dma_pointer = 0;
    s->nic_tx_cpu_pointer = 0;
    s->nic_tx_max = 0;
}

static void ralink_mac_set_link(NetClientState *nc)
{

}

static void ralink_mac_realize(DeviceState *dev, Error **errp)
{
    RalinkMacState *s = RALINK_MAC(dev);

    if (!s->dma_mr) {
        error_setg(errp, TYPE_RALINK_MAC " 'dma-memory' link not set");
        return;
    }

    address_space_init(&s->dma_as, s->dma_mr, "emac-dma");

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_ralink_mac_info, &s->conf,
                           object_get_typename(OBJECT(dev)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static Property ralink_mac_properties[] = {
    DEFINE_NIC_PROPERTIES(RalinkMacState, conf),
    DEFINE_PROP_UINT8("phy-addr", RalinkMacState, mii_phy_addr, 0),
    DEFINE_PROP_LINK("dma-memory", RalinkMacState, dma_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static int ralink_mac_post_load(void *opaque, int version_id)
{
    RalinkMacState *s = opaque;

    ralink_mac_set_link(qemu_get_queue(s->nic));

    return 0;
}

static const VMStateDescription vmstate_aw_emac = {
    .name = "allwinner-sun8i-emac",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ralink_mac_post_load,
    .fields = (VMStateField[]) {
        // VMSTATE_UINT8(mii_phy_addr, RalinkMacState),
        // VMSTATE_UINT32(mii_cmd, RalinkMacState),
        // VMSTATE_UINT32(mii_data, RalinkMacState),
        // VMSTATE_UINT32(mii_cr, RalinkMacState),
        // VMSTATE_UINT32(mii_st, RalinkMacState),
        // VMSTATE_UINT32(mii_adv, RalinkMacState),
        // VMSTATE_UINT32(basic_ctl0, RalinkMacState),
        // VMSTATE_UINT32(basic_ctl1, RalinkMacState),
        // VMSTATE_UINT32(int_en, RalinkMacState),
        // VMSTATE_UINT32(int_sta, RalinkMacState),
        // VMSTATE_UINT32(frm_flt, RalinkMacState),
        // VMSTATE_UINT32(rx_ctl0, RalinkMacState),
        // VMSTATE_UINT32(rx_ctl1, RalinkMacState),
        // VMSTATE_UINT32(rx_desc_head, RalinkMacState),
        // VMSTATE_UINT32(rx_desc_curr, RalinkMacState),
        // VMSTATE_UINT32(tx_ctl0, RalinkMacState),
        // VMSTATE_UINT32(tx_ctl1, RalinkMacState),
        // VMSTATE_UINT32(tx_desc_head, RalinkMacState),
        // VMSTATE_UINT32(tx_desc_curr, RalinkMacState),
        // VMSTATE_UINT32(tx_flowctl, RalinkMacState),
        VMSTATE_END_OF_LIST()
    }
};

static void ralink_mac_reset(DeviceState *dev)
{
}

static void ralink_mac_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ralink_mac_realize;
    dc->reset = ralink_mac_reset;
    dc->vmsd = &vmstate_aw_emac;
    device_class_set_props(dc, ralink_mac_properties);
}

static const TypeInfo ralink_mac_info = {
    .name           = TYPE_RALINK_MAC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(RalinkMacState),
    .instance_init  = ralink_mac_init,
    .class_init     = ralink_mac_class_init,
};

static void ralink_mac_register_types(void)
{
    type_register_static(&ralink_mac_info);
}

type_init(ralink_mac_register_types)

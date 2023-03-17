/*
 * Mediatek mt7628 ethernet emulation
 *
 * Copyright (C) 2023 Lu Hui <luhux76@gmail.com>
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

#ifndef HW_MT7628_ETH_H
#define HW_MT7628_ETH_H

#include "qom/object.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "hw/net/mii.h"

#define TYPE_MT7628_ETH    "mt7628-eth"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628EthState, MT7628_ETH)

#define MT7628_ETH_PKT_MAX_LEN (0xFFFF)
#define MTK_QDMA_PAGE_SIZE 2048

struct mt7628_eth_ring {
    uint32_t addr;
    uint32_t info1;
    uint32_t info2;
    uint32_t info3;
};

struct mt7628EthState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    NICState *nic;
    NICConf   conf;

    /* mii */
    uint16_t bmcr;
    uint16_t bmsr;
    uint16_t anar;
    uint16_t anlpar;

    /* interrupt */
    qemu_irq irq;
    uint32_t irq_stat;
    uint32_t irq_mask;

    /* dma status */
    bool rx_dma_enable;
    bool tx_dma_enable;
    bool rx_dma_busy;
    bool tx_dma_busy;

    /* rx dma ring */
    uint32_t rx_ring_base;
    uint32_t rx_ring_maxi;
    uint32_t rx_ring_cpui;
    uint32_t rx_ring_dmai;

    /* tx dma ring */
    uint32_t tx_ring_base;
    uint32_t tx_ring_maxi;
    uint32_t tx_ring_cpui;
    uint32_t tx_ring_dmai;
    uint8_t  tx_buffer[MT7628_ETH_PKT_MAX_LEN];

    /* transfer counter */
    uint32_t tx_bytes;
    uint32_t tx_packets;
    uint32_t rx_bytes;
    uint32_t rx_packets;

    /* phy control */
    uint16_t pcr_wt_data;
    bool     pcr_rd_phy_cmd;
    bool     pcr_wt_phy_cmd;
    uint8_t  pcr_phy_addr_reg;
    uint8_t  pcr_phy_addr;
    uint16_t pcr_rd_data;
    bool     pcr_rd_ready;
    bool     pcr_wt_done;
};

#endif /* HW_MT7628_ETH_H */

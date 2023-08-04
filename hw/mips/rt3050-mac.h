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

#ifndef HW_RT3050_MAC_H
#define HW_RT3050_MAC_H

#include "qom/object.h"
#include "net/net.h"
#include "hw/sysbus.h"

/**
 * Object model
 * @{
 */

#define TYPE_RALINK_MAC "ralink-mac"
OBJECT_DECLARE_SIMPLE_TYPE(RalinkMacState, RALINK_MAC)

typedef unsigned int u32;
typedef unsigned char u8;


/** @} */

/**
 * Allwinner Sun8i EMAC object instance state
 */
struct RalinkMacState {
    /*< private >*/
    SysBusDevice  parent_obj;

    /** Maps I/O registers in physical memory */
    MemoryRegion iomem;

    /** Interrupt output signal to notify CPU */
    qemu_irq     irq;

    NICState *nic;
	NICConf conf;

	u32 nic_rx_ring;
	u32 nic_rx_dma_pointer;
	u32 nic_rx_cpu_pointer;
	u32 nic_rx_max;

	u32 nic_tx_ring;
	u32 nic_tx_dma_pointer;
	u32 nic_tx_cpu_pointer;
	u32 nic_tx_max;

	AddressSpace dma_as;
	MemoryRegion *dma_mr;

    uint8_t      mii_phy_addr;  /**< PHY address */

};

#endif /* HW_RT3050_MAC_H */

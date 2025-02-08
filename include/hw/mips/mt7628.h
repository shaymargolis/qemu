/*
 * MT7628 System on Chip emulation
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

#ifndef HW_MIPS_MT7628_H
#define HW_MIPS_MT7628_H

#include "qom/object.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/mips/bios.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/misc/mt7628-sysctrl.h"
#include "hw/intc/mt7628-intc.h"
#include "hw/ssi/mt7628-spi.h"
#include "hw/net/mt7628-eth.h"

/**
 * MT7628 device list
 *
 * This enumeration is can be used refer to a particular device in the
 * MT7628 SoC. For example, the physical memory base address for
 * each device can be found in the mt7628State object in the memmap member
 * using the device enum value as index.
 *
 * @see mt7628State
 */
enum {
    MT7628_DEV_DDR,
    MT7628_DEV_SYSCTRL,
    MT7628_DEV_INTC,
    MT7628_DEV_UART0,
    MT7628_DEV_UART1,
    MT7628_DEV_UART2,
    MT7628_DEV_SPI,
    MT7628_DEV_EHCI,
    MT7628_DEV_OHCI,
    MT7628_DEV_ETH,
    MT7628_DEV_BOOTROM,
    MT7628_DEV_FLASH_DIRECT,
};


/* mt7628 cpu interrupt table */


enum {
    MT7628_CPU_IRQ_INTC = 2,
    MT7628_CPU_IRQ_ETH  = 5,
};

#define TYPE_MT7628 "mt7628"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628State, MT7628)

struct mt7628State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    bool found_flash;

    MIPSCPU *cpu;
    const hwaddr *memmap;
    mt7628SysCtrlState sysctrl;
    mt7628intcState intc;
    mt7628SpiState spi;
    EHCISysBusState ehci;
    OHCISysBusState ohci;
    mt7628EthState eth;
    MemoryRegion bootrom;
};

#endif /* HW_MIPS_MT7628_H */

/*
 * Mediatek mt7628 System Control emulation
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

#ifndef HW_MISC_MT7628_SYSCTRL_H
#define HW_MISC_MT7628_SYSCTRL_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_MT7628_SYSCTRL    "mt7628-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(mt7628SysCtrlState, MT7628_SYSCTRL)

struct mt7628SysCtrlState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    /* chip info */
    uint32_t chipname0; /* ascii chip name 0~3 */
    uint32_t chipname1; /* ascii chip name 4~7 */
    uint32_t efuse_cfg;
    uint8_t  chip_pkg_id;
    uint8_t  chip_ver_id;
    uint8_t  chip_eco_id;

    /* startup info */
    uint8_t chip_mode; /* 010 boot from XTAL, 000 boot from PLL */
    uint8_t dram_type; /* 0 DDR2, 1 DDR1 */
    uint8_t xtal_freq; /* 0 25Mhz, 1 40Mhz */

    /* reset control */
    bool reset_system;
};

#endif /* HW_MISC_MT7628_SYSCTRL_H */

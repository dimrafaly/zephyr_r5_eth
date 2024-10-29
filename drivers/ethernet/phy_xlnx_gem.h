/*
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 *
 * PHY management interface and related data
 *
 * Copyright (c) 2021, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_
#define _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_

#include <zephyr/kernel.h>
#include <zephyr/types.h>

/* Event codes used to indicate a particular state change to the driver */
#define PHY_XLNX_GEM_EVENT_LINK_SPEED_CHANGED		(1 << 0)
#define PHY_XLNX_GEM_EVENT_LINK_STATE_CHANGED		(1 << 1)
#define PHY_XLNX_GEM_EVENT_AUTONEG_COMPLETE		(1 << 2)

/* PHY register addresses & constants that are not vendor-specific */
#define PHY_IDENTIFIER_1_REGISTER			2
#define PHY_IDENTIFIER_2_REGISTER			3

/* PHY registers & constants -> Marvell Alaska specific */

/* Marvell PHY ID bits [3..0] = revision -> discard during ID check */
#define PHY_MRVL_PHY_ID_MODEL_MASK			0xFFFFFFF0
#define PHY_MRVL_PHY_ID_MODEL_88E1111			0x01410CC0
#define PHY_MRVL_PHY_ID_MODEL_88E151X			0x01410DD0

#define PHY_MRVL_BASE_REGISTERS_PAGE			0
#define PHY_MRVL_COPPER_CONTROL_REGISTER		0
#define PHY_MRVL_COPPER_STATUS_REGISTER			1
#define PHY_MRVL_COPPER_AUTONEG_ADV_REGISTER		4
#define PHY_MRVL_COPPER_LINK_PARTNER_ABILITY_REGISTER	5
#define PHY_MRVL_1000BASET_CONTROL_REGISTER		9
#define PHY_MRVL_COPPER_CONTROL_1_REGISTER		16
#define PHY_MRVL_COPPER_STATUS_1_REGISTER		17
#define PHY_MRVL_COPPER_INT_ENABLE_REGISTER		18
#define PHY_MRVL_COPPER_INT_STATUS_REGISTER		19
#define PHY_MRVL_COPPER_PAGE_SWITCH_REGISTER		22
#define PHY_MRVL_GENERAL_CONTROL_1_REGISTER		20
#define PHY_MRVL_GENERAL_CONTROL_1_PAGE			18

#define PHY_MRVL_GENERAL_CONTROL_1_RESET_BIT		(1 << 15)

#define PHY_MRVL_COPPER_CONTROL_RESET_BIT		(1 << 15)
#define PHY_MRVL_COPPER_CONTROL_AUTONEG_ENABLE_BIT	(1 << 12)

#define PHY_MRVL_ADV_1000BASET_FDX_BIT			(1 << 9)
#define PHY_MRVL_ADV_1000BASET_HDX_BIT			(1 << 8)
#define PHY_MRVL_ADV_100BASET_FDX_BIT			(1 << 8)
#define PHY_MRVL_ADV_100BASET_HDX_BIT			(1 << 7)
#define PHY_MRVL_ADV_10BASET_FDX_BIT			(1 << 6)
#define PHY_MRVL_ADV_10BASET_HDX_BIT			(1 << 5)
#define PHY_MRVL_ADV_SELECTOR_802_3			0x0001

#define PHY_MRVL_MDIX_CONFIG_MASK			0x0003
#define PHY_MRVL_MDIX_CONFIG_SHIFT			5
#define PHY_MRVL_MDIX_AUTO_CROSSOVER_ENABLE		0x0003
#define PHY_MRVL_MODE_CONFIG_MASK			0x0007
#define PHY_MRVL_MODE_CONFIG_SHIFT			0

#define PHY_MRVL_COPPER_SPEED_CHANGED_INT_BIT		(1 << 14)
#define PHY_MRVL_COPPER_DUPLEX_CHANGED_INT_BIT		(1 << 13)
#define PHY_MRVL_COPPER_AUTONEG_COMPLETED_INT_BIT	(1 << 11)
#define PHY_MRVL_COPPER_LINK_STATUS_CHANGED_INT_BIT	(1 << 10)
#define PHY_MRVL_COPPER_LINK_STATUS_BIT_SHIFT		5

#define PHY_MRVL_LINK_SPEED_SHIFT			14
#define PHY_MRVL_LINK_SPEED_MASK			0x3
#define PHY_MRVL_LINK_SPEED_10MBIT			0
#define PHY_MRVL_LINK_SPEED_100MBIT			1
#define PHY_MRVL_LINK_SPEED_1GBIT			2

/*TI TLK105 & DP83822*/

/* TI PHY ID bits [3..0] = revision -> discard during ID check */
#define PHY_TI_PHY_ID_MODEL_MASK			0xFFFFFFF0
#define PHY_TI_PHY_ID_MODEL_DP83822			0x2000A240
#define PHY_TI_PHY_ID_MODEL_TLK105			0x2000A210

#define PHY_TI_PHY_SPECIFIC_CONTROL_REGISTER		0x0010
#define PHY_TI_BASIC_MODE_CONTROL_REGISTER		0x0000
#define PHY_TI_BASIC_MODE_STATUS_REGISTER		0x0001
#define PHY_TI_AUTONEG_ADV_REGISTER			0x0004
#define PHY_TI_CONTROL_REGISTER_1			0x0009
#define PHY_TI_PHY_STATUS_REGISTER			0x0010
#define PHY_TI_MII_INTERRUPT_STATUS_REGISTER_1		0x0012
#define PHY_TI_LED_CONTROL_REGISTER			0x0018
#define PHY_TI_PHY_CONTROL_REGISTER			0x0019

#define PHY_TI_BASIC_MODE_CONTROL_RESET_BIT		(1 << 15)
#define PHY_TI_BASIC_MODE_CONTROL_AUTONEG_ENABLE_BIT	(1 << 12)

#define PHY_TI_BASIC_MODE_STATUS_LINK_STATUS_BIT	(1 << 2)

#define PHY_TI_LINK_STATUS_CHANGED_INT_BIT		(1 << 13)
#define PHY_TI_SPEED_CHANGED_INT_BIT			(1 << 12)
#define PHY_TI_DUPLEX_CHANGED_INT_BIT			(1 << 11)
#define PHY_TI_AUTONEG_COMPLETED_INT_BIT		(1 << 10)

#define PHY_TI_ADV_SELECTOR_802_3			0x0001
#define PHY_TI_ADV_100BASET_FDX_BIT			(1 << 8)
#define PHY_TI_ADV_100BASET_HDX_BIT			(1 << 7)
#define PHY_TI_ADV_10BASET_FDX_BIT			(1 << 6)
#define PHY_TI_ADV_10BASET_HDX_BIT			(1 << 5)

#define PHY_TI_CR1_ROBUST_AUTO_MDIX_BIT			(1 << 5)

#define PHY_TI_PHY_CONTROL_AUTO_MDIX_ENABLE_BIT		(1 << 15)
#define PHY_TI_PHY_CONTROL_FORCE_MDIX_BIT		(1 << 14)
#define PHY_TI_PHY_CONTROL_LED_CONFIG_LINK_ONLY_BIT	(1 << 5)

#define PHY_TI_LED_CONTROL_BLINK_RATE_SHIFT		9
#define PHY_TI_LED_CONTROL_BLINK_RATE_20HZ		0
#define PHY_TI_LED_CONTROL_BLINK_RATE_10HZ		1
#define PHY_TI_LED_CONTROL_BLINK_RATE_5HZ		2
#define PHY_TI_LED_CONTROL_BLINK_RATE_2HZ		3

#define PHY_TI_PHY_STATUS_LINK_BIT			(1 << 0)
#define PHY_TI_PHY_STATUS_SPEED_BIT			(1 << 1)

/* Micrel KSZ9031*/
// Micrel KSZ9031 PHY ID
#define PHY_MICREL_ID_MODEL_KSZ9031  0x00221620
#define PHY_MICREL_PHY_ID_MODEL_MASK 0x00fffff0

// PHY register addresses
#define PHY_MICREL_BASIC_CONTROL_REGISTER 0x00 // Basic Control Register
#define PHY_MICREL_BASIC_STATUS_REGISTER  0x01 // Basic Status Register

// Basic Control Register (0x00) bit masks
#define PHY_MICREL_BASIC_CONTROL_RESET_BIT           (1 << 15) // Reset bit
#define PHY_MICREL_BASIC_CONTROL_AUTONEG_ENABLE_BIT  (1 << 12) // Auto-negotiation enable
#define PHY_MICREL_BASIC_CONTROL_ISOLATE_BIT         (1 << 10) // Electrically isolate PHY
#define PHY_MICREL_BASIC_CONTROL_RESTART_AUTONEG_BIT (1 << 9)  // Restart auto-negotiation

// Basic Status Register (0x01) bit masks
#define PHY_MICREL_BASIC_STATUS_AUTONEG_COMPLETED_BIT (1 << 5) // Auto-negotiation complete
#define PHY_MICREL_BASIC_STATUS_LINK_UP               (1 << 2) // Link status bit

// KSZ9031-Specific Registers
#define PHY_MICREL_MMD_ACCESS_CONTROL        0x0D // Acces Control Register
#define PHY_MICREL_MMD_ACCESS_REGISTER_DATA  0x0E // Basic Status Register
#define PHY_MICREL_CONTROL_REGISTER          0x1F // Control register
#define PHY_MICREL_INTERRUPT_STATUS_REGISTER 0x1B // Interrupt Status Register

// PHY Status Register (0x1F) bit masks
#define PHY_MICREL_CONTROL_REGISTER_SPEED_1000MBPS (1 << 6) // 1000 Mbps speed status
#define PHY_MICREL_CONTROL_REGISTER_SPEED_100MBPS  (1 << 5) // 100 Mbps speed status
#define PHY_MICREL_CONTROL_REGISTER_SPEED_10MBPS   (1 << 4) // 10 Mbps speed status

// Interrupt Status Register (0x1B) bit masks
#define PHY_MICREL_LINK_DOWN_STATUS_INTERRUPT         (1 << 2) // Link status change interrupt
#define PHY_MICREL_LINK_UP_STATUS_INTERRUPT           (1 << 0) // Link status change interrupt
#define PHY_MICREL_LINK_PARTNER_ACKNOWLEDGE_INTERRUPT (1 << 3) // Link status change interrupt

/**
 * @brief Vendor-specific PHY management function pointer table struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_gem_api {
	void (*phy_reset_func)(const struct device *dev);
	void (*phy_configure_func)(const struct device *dev);
	uint16_t (*phy_poll_status_change_func)(const struct device *dev);
	uint8_t (*phy_poll_link_status_func)(const struct device *dev);
	enum eth_xlnx_link_speed (*phy_poll_link_speed_func)(const struct device *dev);
};

/**
 * @brief Supported PHY list entry struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_gem_supported_dev {
	uint32_t phy_id;
	uint32_t phy_id_mask;
	struct phy_xlnx_gem_api *api;
	const char *identifier;
};

/* PHY identification function -> generic, not vendor-specific */
int phy_xlnx_gem_detect(const struct device *dev);

#endif /* _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_ */

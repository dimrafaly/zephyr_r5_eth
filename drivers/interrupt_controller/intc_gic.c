/*
 * Copyright (c) 2018 Marvell
 * Copyright (c) 2018 Lexmark International, Inc.
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NOTE: This driver implements the GICv1 and GICv2 interfaces.
 */

#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/sys/barrier.h>

#if defined(CONFIG_GIC_V1)
#define DT_DRV_COMPAT arm_gic_v1
#elif defined(CONFIG_GIC_V2)
#define DT_DRV_COMPAT arm_gic_v2
#else
#error "Unknown GIC controller compatible for this configuration"
#endif

static const uint64_t cpu_mpid_list[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), DT_REG_ADDR, (,))
};

BUILD_ASSERT(ARRAY_SIZE(cpu_mpid_list) >= CONFIG_MP_MAX_NUM_CPUS,
		"The count of CPU Cores nodes in dts is less than CONFIG_MP_MAX_NUM_CPUS\n");

BUILD_ASSERT(ARRAY_SIZE(cpu_mpid_list) == 1,
		"More than one cpu initialized\n");

typedef struct {
    unsigned int irq;       // The IRQ number
    uint8_t cpu_mask;       // The CPU mask associated with this IRQ
} irq_config_t;

static const irq_config_t active_irqs[] = {
    {53, 0x02}, // UART0 RPU1
    {54, 0x02}, // UART1 RPU1
	{66, 0x02}, // ipi chanel2 RPU1
    {71, 0x02}, // TTC1 channel1 RPU1
    {122, 0x02},// PL-PS irq channel1 RPU1
    {53, 0x01}, // UART0 RPU0
    {54, 0x01}, // UART1 RPU0
    {65, 0x01}, // ipi channel1 RPU0
    {68, 0x01}, // TTC0  channel0 RPU0
    {89, 0x01}, // GEM0 RPU0
    {90, 0x01}, // GEM0 wakup RPU0
    {48, 0x01}, // GPIO
    {55, 0x01}, // CAN0
};

void arm_gic_irq_enable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ISENABLERn + int_grp * 4));
}

void arm_gic_irq_disable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ICENABLERn + int_grp * 4));
}

bool arm_gic_irq_is_enabled(unsigned int irq)
{
	int int_grp, int_off;
	unsigned int enabler;

	int_grp = irq / 32;
	int_off = irq % 32;

	enabler = sys_read32(GICD_ISENABLERn + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

bool arm_gic_irq_is_pending(unsigned int irq)
{
	int int_grp, int_off;
	unsigned int enabler;

	int_grp = irq / 32;
	int_off = irq % 32;

	enabler = sys_read32(GICD_ISPENDRn + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

void arm_gic_irq_set_pending(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ISPENDRn + int_grp * 4));
}

void arm_gic_irq_clear_pending(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ICPENDRn + int_grp * 4));
}

void arm_gic_irq_set_priority(
	unsigned int irq, unsigned int prio, uint32_t flags)
{
	int int_grp, int_off;
	uint32_t val;

	/* Set priority */
	sys_write8(prio & 0xff, GICD_IPRIORITYRn + irq);

	/* Set interrupt type */
	int_grp = (irq / 16) * 4;
	int_off = (irq % 16) * 2;

	/* GICD_ICFGR0 is read-only; SGIs are always edge-triggered */
	if (int_grp != 0) {
		val = sys_read32(GICD_ICFGRn + int_grp);
		val &= ~(GICD_ICFGR_MASK << int_off);
		if (flags & IRQ_TYPE_EDGE) {
			val |= (GICD_ICFGR_TYPE << int_off);
		}

		sys_write32(val, GICD_ICFGRn + int_grp);
	}
}

unsigned int arm_gic_get_active(void)
{
	unsigned int irq;

	/*
	 * "ARM Generic Interrupt Controller Architecture version 2.0" states that
	 * [4.4.5 End of Interrupt Register, GICC_EOIR)]:
	 * """
	 * For compatibility with possible extensions to the GIC architecture
	 * specification, ARM recommends that software preserves the entire register
	 * value read from the GICC_IAR when it acknowledges the interrupt, and uses
	 * that entire value for its corresponding write to the GICC_EOIR.
	 * """
	 * Because of that, we read the entire value here, to be later written back to GICC_EOIR
	 */
	irq = sys_read32(GICC_IAR);
	return irq;
}

void arm_gic_eoi(unsigned int irq)
{
	/*
	 * Ensure the write to peripheral registers are *complete* before the write
	 * to GIC_EOIR.
	 *
	 * Note: The completion guarantee depends on various factors of system design
	 * and the barrier is the best core can do by which execution of further
	 * instructions waits till the barrier is alive.
	 */
	barrier_dsync_fence_full();

	/* set to inactive */
	sys_write32(irq, GICC_EOIR);
}

void gic_raise_sgi(unsigned int sgi_id, uint64_t target_aff,
		uint16_t target_list)
{
	uint32_t sgi_val;

	ARG_UNUSED(target_aff);

	sgi_val = GICD_SGIR_TGTFILT_CPULIST |
		GICD_SGIR_CPULIST(target_list & GICD_SGIR_CPULIST_MASK) |
		sgi_id;

	barrier_dsync_fence_full();
	sys_write32(sgi_val, GICD_SGIR);
	barrier_isync_fence_full();
}

static void gic_dist_init(void)
{
    unsigned int gic_irqs, i;
    uint64_t my_mpid = read_mpidr() & 0xff; // current CPU
    uint32_t reg_val;

    /* Determine number of interrupts from GICD_TYPER */
    gic_irqs = sys_read32(GICD_TYPER) & 0x1f;
    gic_irqs = (gic_irqs + 1) * 32;
    if (gic_irqs > 1020U)
        gic_irqs = 1020U;

    /* Disable Distributor temporarily */
    sys_write32(0, GICD_CTLR);

    for (i = 0; i < ARRAY_SIZE(active_irqs); i++) {
        unsigned int irq = active_irqs[i].irq;
        uint8_t cpu_mask = active_irqs[i].cpu_mask & 0xff;

        /* Only SPIs */
        if (irq < GIC_SPI_INT_BASE)
            continue;

        /* Only interrupts targeting this CPU */
        if ((1ULL << my_mpid) != cpu_mask)
            continue;

        /* ITARGETSR: 1 byte per IRQ */
        uint32_t reg_base = GICD_ITARGETSRn + (irq & ~0x3);
        uint8_t byte_off = irq & 0x3;
        reg_val = sys_read32(reg_base);
        reg_val &= ~(0xffU << (byte_off * 8));
        reg_val |= ((uint32_t)cpu_mask << (byte_off * 8));
        sys_write32(reg_val, reg_base);

        /* Priority: default 0xA0 */
        sys_write8(0xA0, GICD_IPRIORITYRn + irq);

        /* Level-sensitive, active low */
        uint32_t cfg_reg = GICD_ICFGRn + ((irq / 16) * 4);
        reg_val = sys_read32(cfg_reg);
        reg_val &= ~(0x3U << ((irq % 16) * 2));
        sys_write32(reg_val, cfg_reg);

        /* Group 0 */
        uint32_t grp_reg = GICD_IGROUPRn + ((irq / 32) * 4);
        reg_val = sys_read32(grp_reg);
        reg_val &= ~(1U << (irq % 32));
        sys_write32(reg_val, grp_reg);

        /* Disable SPI and clear active */
        sys_write32(1U << (irq % 32), GICD_ICENABLERn + ((irq / 32) * 4));
#ifndef CONFIG_GIC_V1
        sys_write32(1U << (irq % 32), GICD_ICACTIVERn + ((irq / 32) * 4));
#endif
    }

    /* Re-enable Distributor */
    sys_write32(1, GICD_CTLR);
}

static void gic_cpu_init(void)
{
	int i;
	uint32_t val;

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
#ifndef CONFIG_GIC_V1
	sys_write32(0xffffffff, GICD_ICACTIVERn);
#endif
	sys_write32(0xffff0000, GICD_ICENABLERn);
	sys_write32(0x0000ffff, GICD_ISENABLERn);

	/*
	 * Set priority on PPI and SGI interrupts
	 */
	for (i = 0; i < 32; i += 4) {
		sys_write32(0xa0a0a0a0, GICD_IPRIORITYRn + i);
	}

	sys_write32(0xf0, GICC_PMR);

	/*
	 * Enable interrupts and signal them using the IRQ signal.
	 */
	val = sys_read32(GICC_CTLR);
#ifndef CONFIG_GIC_V1
	val &= ~GICC_CTLR_BYPASS_MASK;
#endif
	val |= GICC_CTLR_ENABLE_MASK;
	sys_write32(val, GICC_CTLR);
}

#define GIC_PARENT_IRQ 0
#define GIC_PARENT_IRQ_PRI 0
#define GIC_PARENT_IRQ_FLAGS 0

/**
 * @brief Initialize the GIC device driver
 */
int arm_gic_init(const struct device *dev)
{
	/* Init of Distributor interface registers */
	gic_dist_init();

	/* Init CPU interface registers */
	gic_cpu_init();

	return 0;
}

DEVICE_DT_INST_DEFINE(0, arm_gic_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);

#ifdef CONFIG_SMP
void arm_gic_secondary_init(void)
{
	/* Init CPU interface registers for each secondary core */
	gic_cpu_init();
}
#endif

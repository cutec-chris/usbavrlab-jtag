/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   Cortex-A8(tm) TRM, ARM DDI 0344H                                      *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "cortex_a8.h"
#include "register.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_opcodes.h"

static int cortex_a8_poll(struct target *target);
static int cortex_a8_debug_entry(struct target *target);
static int cortex_a8_restore_context(struct target *target, bool bpwp);
static int cortex_a8_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint, uint8_t matchmode);
static int cortex_a8_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
static int cortex_a8_dap_read_coreregister_u32(struct target *target,
		uint32_t *value, int regnum);
static int cortex_a8_dap_write_coreregister_u32(struct target *target,
		uint32_t value, int regnum);
/*
 * FIXME do topology discovery using the ROM; don't
 * assume this is an OMAP3.
 */
#define swjdp_memoryap 0
#define swjdp_debugap 1
#define OMAP3530_DEBUG_BASE 0x54011000

/*
 * Cortex-A8 Basic debug access, very low level assumes state is saved
 */
static int cortex_a8_init_debug_access(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	int retval;
	uint32_t dummy;

	LOG_DEBUG(" ");

	/* Unlocking the debug registers for modification */
	/* The debugport might be uninitialised so try twice */
	retval = mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	if (retval != ERROR_OK)
		mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	/* Clear Sticky Power Down status Bit in PRSR to enable access to
	   the registers in the Core Power Domain */
	retval = mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_PRSR, &dummy);
	/* Enabling of instruction execution in debug mode is done in debug_entry code */

	/* Resync breakpoint registers */

	/* Since this is likley called from init or reset, update targtet state information*/
	cortex_a8_poll(target);

	return retval;
}

/* To reduce needless round-trips, pass in a pointer to the current
 * DSCR value.  Initialize it to zero if you just need to know the
 * value on return from this function; or DSCR_INSTR_COMP if you
 * happen to know that no instruction is pending.
 */
static int cortex_a8_exec_opcode(struct target *target,
		uint32_t opcode, uint32_t *dscr_p)
{
	uint32_t dscr;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	dscr = dscr_p ? *dscr_p : 0;

	LOG_DEBUG("exec opcode 0x%08" PRIx32, opcode);

	/* Wait for InstrCompl bit to be set */
	while ((dscr & DSCR_INSTR_COMP) == 0)
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("Could not read DSCR register, opcode = 0x%08" PRIx32, opcode);
			return retval;
		}
	}

	mem_ap_write_u32(swjdp, armv7a->debug_base + CPUDBG_ITR, opcode);

	do
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
	}
	while ((dscr & DSCR_INSTR_COMP) == 0); /* Wait for InstrCompl bit to be set */

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

/**************************************************************************
Read core register with very few exec_opcode, fast but needs work_area.
This can cause problems with MMU active.
**************************************************************************/
static int cortex_a8_read_regs_through_mem(struct target *target, uint32_t address,
		uint32_t * regfile)
{
	int retval = ERROR_OK;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	cortex_a8_dap_read_coreregister_u32(target, regfile, 0);
	cortex_a8_dap_write_coreregister_u32(target, address, 0);
	cortex_a8_exec_opcode(target, ARMV4_5_STMIA(0, 0xFFFE, 0, 0), NULL);
	dap_ap_select(swjdp, swjdp_memoryap);
	mem_ap_read_buf_u32(swjdp, (uint8_t *)(&regfile[1]), 4*15, address);
	dap_ap_select(swjdp, swjdp_debugap);

	return retval;
}

static int cortex_a8_dap_read_coreregister_u32(struct target *target,
		uint32_t *value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t reg = regnum&0xFF;
	uint32_t dscr = 0;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	if (reg > 17)
		return retval;

	if (reg < 15)
	{
		/* Rn to DCCTX, "MCR p14, 0, Rn, c0, c5, 0"  0xEE00nE15 */
		cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, reg, 0, 5, 0),
				&dscr);
	}
	else if (reg == 15)
	{
		/* "MOV r0, r15"; then move r0 to DCCTX */
		cortex_a8_exec_opcode(target, 0xE1A0000F, &dscr);
		cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
				&dscr);
	}
	else
	{
		/* "MRS r0, CPSR" or "MRS r0, SPSR"
		 * then move r0 to DCCTX
		 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRS(0, reg & 1), &dscr);
		cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
				&dscr);
	}

	/* Wait for DTRRXfull then read DTRRTX */
	while ((dscr & DSCR_DTR_TX_FULL) == 0)
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	}

	retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRTX, value);
	LOG_DEBUG("read DCC 0x%08" PRIx32, *value);

	return retval;
}

static int cortex_a8_dap_write_coreregister_u32(struct target *target,
		uint32_t value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t Rd = regnum&0xFF;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	LOG_DEBUG("register %i, value 0x%08" PRIx32, regnum, value);

	/* Check that DCCRX is not full */
	retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (dscr & DSCR_DTR_RX_FULL)
	{
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX with MCR(p14, 0, Rd, c0, c5, 0), opcode  0xEE000E15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
	}

	if (Rd > 17)
		return retval;

	/* Write DTRRX ... sets DSCR.DTRRXfull but exec_opcode() won't care */
	LOG_DEBUG("write DCC 0x%08" PRIx32, value);
	retval = mem_ap_write_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRRX, value);

	if (Rd < 15)
	{
		/* DCCRX to Rn, "MCR p14, 0, Rn, c0, c5, 0", 0xEE00nE15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, Rd, 0, 5, 0),
				&dscr);
	}
	else if (Rd == 15)
	{
		/* DCCRX to R0, "MCR p14, 0, R0, c0, c5, 0", 0xEE000E15
		 * then "mov r15, r0"
		 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		cortex_a8_exec_opcode(target, 0xE1A0F000, &dscr);
	}
	else
	{
		/* DCCRX to R0, "MCR p14, 0, R0, c0, c5, 0", 0xEE000E15
		 * then "MSR CPSR_cxsf, r0" or "MSR SPSR_cxsf, r0" (all fields)
		 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		cortex_a8_exec_opcode(target, ARMV4_5_MSR_GP(0, 0xF, Rd & 1),
				&dscr);

		/* "Prefetch flush" after modifying execution status in CPSR */
		if (Rd == 16)
			cortex_a8_exec_opcode(target,
					ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
					&dscr);
	}

	return retval;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int cortex_a8_dap_write_memap_register_u32(struct target *target, uint32_t address, uint32_t value)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	retval = mem_ap_write_atomic_u32(swjdp, address, value);

	return retval;
}

/*
 * Cortex-A8 implementation of Debug Programmer's Model
 *
 * NOTE the invariant:  these routines return with DSCR_INSTR_COMP set,
 * so there's no need to poll for it before executing an instruction.
 *
 * NOTE that in several of these cases the "stall" mode might be useful.
 * It'd let us queue a few operations together... prepare/finish might
 * be the places to enable/disable that mode.
 */

static inline struct cortex_a8_common *dpm_to_a8(struct arm_dpm *dpm)
{
	return container_of(dpm, struct cortex_a8_common, armv7a_common.dpm);
}

static int cortex_a8_write_dcc(struct cortex_a8_common *a8, uint32_t data)
{
	LOG_DEBUG("write DCC 0x%08" PRIx32, data);
	return mem_ap_write_u32(&a8->armv7a_common.swjdp_info,
			a8->armv7a_common.debug_base + CPUDBG_DTRRX, data);
}

static int cortex_a8_read_dcc(struct cortex_a8_common *a8, uint32_t *data,
		uint32_t *dscr_p)
{
	struct swjdp_common *swjdp = &a8->armv7a_common.swjdp_info;
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_read_atomic_u32(swjdp,
				a8->armv7a_common.debug_base + CPUDBG_DSCR,
				&dscr);
	}

	retval = mem_ap_read_atomic_u32(swjdp,
			a8->armv7a_common.debug_base + CPUDBG_DTRTX, data);
	//LOG_DEBUG("read DCC 0x%08" PRIx32, *data);

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int cortex_a8_dpm_prepare(struct arm_dpm *dpm)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	struct swjdp_common *swjdp = &a8->armv7a_common.swjdp_info;
	uint32_t dscr;
	int retval;

	/* set up invariant:  INSTR_COMP is set after ever DPM operation */
	do {
		retval = mem_ap_read_atomic_u32(swjdp,
				a8->armv7a_common.debug_base + CPUDBG_DSCR,
				&dscr);
	} while ((dscr & DSCR_INSTR_COMP) == 0);

	/* this "should never happen" ... */
	if (dscr & DSCR_DTR_RX_FULL) {
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX */
		retval = cortex_a8_exec_opcode(
				a8->armv7a_common.armv4_5_common.target,
				ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
	}

	return retval;
}

static int cortex_a8_dpm_finish(struct arm_dpm *dpm)
{
	/* REVISIT what could be done here? */
	return ERROR_OK;
}

static int cortex_a8_instr_write_data_dcc(struct arm_dpm *dpm,
		uint32_t opcode, uint32_t data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	retval = cortex_a8_write_dcc(a8, data);

	return cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			opcode,
			&dscr);
}

static int cortex_a8_instr_write_data_r0(struct arm_dpm *dpm,
		uint32_t opcode, uint32_t data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	retval = cortex_a8_write_dcc(a8, data);

	/* DCCRX to R0, "MCR p14, 0, R0, c0, c5, 0", 0xEE000E15 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
			&dscr);

	/* then the opcode, taking data from R0 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			opcode,
			&dscr);

	return retval;
}

static int cortex_a8_instr_cpsr_sync(struct arm_dpm *dpm)
{
	struct target *target = dpm->arm->target;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* "Prefetch flush" after modifying execution status in CPSR */
	return cortex_a8_exec_opcode(target,
			ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
			&dscr);
}

static int cortex_a8_instr_read_data_dcc(struct arm_dpm *dpm,
		uint32_t opcode, uint32_t *data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* the opcode, writing data to DCC */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			opcode,
			&dscr);

	return cortex_a8_read_dcc(a8, data, &dscr);
}


static int cortex_a8_instr_read_data_r0(struct arm_dpm *dpm,
		uint32_t opcode, uint32_t *data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	/* the opcode, writing data to R0 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			opcode,
			&dscr);

	/* write R0 to DCC */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.armv4_5_common.target,
			ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
			&dscr);

	return cortex_a8_read_dcc(a8, data, &dscr);
}

static int cortex_a8_bpwp_enable(struct arm_dpm *dpm, unsigned index,
		uint32_t addr, uint32_t control)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t vr = a8->armv7a_common.debug_base;
	uint32_t cr = a8->armv7a_common.debug_base;
	int retval;

	switch (index) {
	case 0 ... 15:		/* breakpoints */
		vr += CPUDBG_BVR_BASE;
		cr += CPUDBG_BCR_BASE;
		break;
	case 16 ... 31:		/* watchpoints */
		vr += CPUDBG_WVR_BASE;
		cr += CPUDBG_WCR_BASE;
		index -= 16;
		break;
	default:
		return ERROR_FAIL;
	}
	vr += 4 * index;
	cr += 4 * index;

	LOG_DEBUG("A8: bpwp enable, vr %08x cr %08x",
			(unsigned) vr, (unsigned) cr);

	retval = cortex_a8_dap_write_memap_register_u32(dpm->arm->target,
			vr, addr);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_memap_register_u32(dpm->arm->target,
			cr, control);
	return retval;
}

static int cortex_a8_bpwp_disable(struct arm_dpm *dpm, unsigned index)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t cr;

	switch (index) {
	case 0 ... 15:
		cr = a8->armv7a_common.debug_base + CPUDBG_BCR_BASE;
		break;
	case 16 ... 31:
		cr = a8->armv7a_common.debug_base + CPUDBG_WCR_BASE;
		index -= 16;
		break;
	default:
		return ERROR_FAIL;
	}
	cr += 4 * index;

	LOG_DEBUG("A8: bpwp disable, cr %08x", (unsigned) cr);

	/* clear control register */
	return cortex_a8_dap_write_memap_register_u32(dpm->arm->target, cr, 0);
}

static int cortex_a8_dpm_setup(struct cortex_a8_common *a8, uint32_t didr)
{
	struct arm_dpm *dpm = &a8->armv7a_common.dpm;
	int retval;

	dpm->arm = &a8->armv7a_common.armv4_5_common;
	dpm->didr = didr;

	dpm->prepare = cortex_a8_dpm_prepare;
	dpm->finish = cortex_a8_dpm_finish;

	dpm->instr_write_data_dcc = cortex_a8_instr_write_data_dcc;
	dpm->instr_write_data_r0 = cortex_a8_instr_write_data_r0;
	dpm->instr_cpsr_sync = cortex_a8_instr_cpsr_sync;

	dpm->instr_read_data_dcc = cortex_a8_instr_read_data_dcc;
	dpm->instr_read_data_r0 = cortex_a8_instr_read_data_r0;

	dpm->bpwp_enable = cortex_a8_bpwp_enable;
	dpm->bpwp_disable = cortex_a8_bpwp_disable;

	retval = arm_dpm_setup(dpm);
	if (retval == ERROR_OK)
		retval = arm_dpm_initialize(dpm);

	return retval;
}


/*
 * Cortex-A8 Run control
 */

static int cortex_a8_poll(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	enum target_state prev_target_state = target->state;
	uint8_t saved_apsel = dap_ap_get_select(swjdp);

	dap_ap_select(swjdp, swjdp_debugap);
	retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
	{
		dap_ap_select(swjdp, saved_apsel);
		return retval;
	}
	cortex_a8->cpudbg_dscr = dscr;

	if ((dscr & 0x3) == 0x3)
	{
		if (prev_target_state != TARGET_HALTED)
		{
			/* We have a halting debug event */
			LOG_DEBUG("Target halted");
			target->state = TARGET_HALTED;
			if ((prev_target_state == TARGET_RUNNING)
					|| (prev_target_state == TARGET_RESET))
			{
				retval = cortex_a8_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;

				target_call_event_callbacks(target,
						TARGET_EVENT_HALTED);
			}
			if (prev_target_state == TARGET_DEBUG_RUNNING)
			{
				LOG_DEBUG(" ");

				retval = cortex_a8_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;

				target_call_event_callbacks(target,
						TARGET_EVENT_DEBUG_HALTED);
			}
		}
	}
	else if ((dscr & 0x3) == 0x2)
	{
		target->state = TARGET_RUNNING;
	}
	else
	{
		LOG_DEBUG("Unknown target state dscr = 0x%08" PRIx32, dscr);
		target->state = TARGET_UNKNOWN;
	}

	dap_ap_select(swjdp, saved_apsel);

	return retval;
}

static int cortex_a8_halt(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	uint8_t saved_apsel = dap_ap_get_select(swjdp);
	dap_ap_select(swjdp, swjdp_debugap);

	/*
	 * Tell the core to be halted by writing DRCR with 0x1
	 * and then wait for the core to be halted.
	 */
	retval = mem_ap_write_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DRCR, 0x1);

	/*
	 * enter halting debug mode
	 */
	mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_DSCR, &dscr);
	retval = mem_ap_write_atomic_u32(swjdp,
		armv7a->debug_base + CPUDBG_DSCR, dscr | DSCR_HALT_DBG_MODE);

	if (retval != ERROR_OK)
		goto out;

	do {
		mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	} while ((dscr & DSCR_CORE_HALTED) == 0);

	target->debug_reason = DBG_REASON_DBGRQ;

out:
	dap_ap_select(swjdp, saved_apsel);
	return retval;
}

static int cortex_a8_resume(struct target *target, int current,
		uint32_t address, int handle_breakpoints, int debug_execution)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

//	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc, dscr;

	uint8_t saved_apsel = dap_ap_get_select(swjdp);
	dap_ap_select(swjdp, swjdp_debugap);

	if (!debug_execution)
		target_free_all_working_areas(target);

#if 0
	if (debug_execution)
	{
		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of
		 * masking with C_MASKINTS,
		 * This is probably the same issue as Cortex-M3 Errata 377493:
		 * C_MASKINTS in parallel with disabled interrupts can cause
		 * local faults to not be taken. */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_PRIMASK].value, 0, 32, 1);
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].valid = 1;

		/* Make sure we are in Thumb mode */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32,
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32) | (1 << 24));
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_xPSR].valid = 1;
	}
#endif

	/* current = 1: continue on current pc, otherwise continue at <address> */
	resume_pc = buf_get_u32(
			armv4_5->core_cache->reg_list[15].value,
			0, 32);
	if (!current)
		resume_pc = address;

	/* Make sure that the Armv7 gdb thumb fixups does not
	 * kill the return address
	 */
	switch (armv4_5->core_state)
	{
	case ARM_STATE_ARM:
		resume_pc &= 0xFFFFFFFC;
		break;
	case ARM_STATE_THUMB:
	case ARM_STATE_THUMB_EE:
		/* When the return address is loaded into PC
		 * bit 0 must be 1 to stay in Thumb state
		 */
		resume_pc |= 0x1;
		break;
	case ARM_STATE_JAZELLE:
		LOG_ERROR("How do I resume into Jazelle state??");
		return ERROR_FAIL;
	}
	LOG_DEBUG("resume pc = 0x%08" PRIx32, resume_pc);
	buf_set_u32(armv4_5->core_cache->reg_list[15].value,
			0, 32, resume_pc);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;

	cortex_a8_restore_context(target, handle_breakpoints);

#if 0
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		/* Single step past breakpoint at current address */
		if ((breakpoint = breakpoint_find(target, resume_pc)))
		{
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

#endif
	/* Restart core and wait for it to be started
	 * NOTE: this clears DSCR_ITR_EN and other bits.
	 *
	 * REVISIT: for single stepping, we probably want to
	 * disable IRQs by default, with optional override...
	 */
	mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_DRCR, 0x2);

	do {
		mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	} while ((dscr & DSCR_CORE_RESTARTED) == 0);

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(armv4_5->core_cache);

	if (!debug_execution)
	{
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32, resume_pc);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32, resume_pc);
	}

	dap_ap_select(swjdp, saved_apsel);

	return ERROR_OK;
}

static int cortex_a8_debug_entry(struct target *target)
{
	int i;
	uint32_t regfile[16], cpsr, dscr;
	int retval = ERROR_OK;
	struct working_area *regfile_working_area = NULL;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	struct reg *reg;

	LOG_DEBUG("dscr = 0x%08" PRIx32, cortex_a8->cpudbg_dscr);

	/* REVISIT surely we should not re-read DSCR !! */
	mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);

	/* REVISIT see A8 TRM 12.11.4 steps 2..3 -- make sure that any
	 * imprecise data aborts get discarded by issuing a Data
	 * Synchronization Barrier:  ARMV4_5_MCR(15, 0, 0, 7, 10, 4).
	 */

	/* Enable the ITR execution once we are in debug mode */
	dscr |= DSCR_ITR_EN;
	retval = mem_ap_write_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, dscr);

	/* Examine debug reason */
	arm_dpm_report_dscr(&armv7a->dpm, cortex_a8->cpudbg_dscr);

	/* save address of instruction that triggered the watchpoint? */
	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		uint32_t wfar;

		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_WFAR,
				&wfar);
		arm_dpm_report_wfar(&armv7a->dpm, wfar);
	}

	/* REVISIT fast_reg_read is never set ... */

	/* Examine target state and mode */
	if (cortex_a8->fast_reg_read)
		target_alloc_working_area(target, 64, &regfile_working_area);

	/* First load register acessible through core debug port*/
	if (!regfile_working_area)
	{
		retval = arm_dpm_read_current_registers(&armv7a->dpm);
	}
	else
	{
		dap_ap_select(swjdp, swjdp_memoryap);
		cortex_a8_read_regs_through_mem(target,
				regfile_working_area->address, regfile);
		dap_ap_select(swjdp, swjdp_memoryap);
		target_free_working_area(target, regfile_working_area);

		/* read Current PSR */
		cortex_a8_dap_read_coreregister_u32(target, &cpsr, 16);
		dap_ap_select(swjdp, swjdp_debugap);
		LOG_DEBUG("cpsr: %8.8" PRIx32, cpsr);

		arm_set_cpsr(armv4_5, cpsr);

		/* update cache */
		for (i = 0; i <= ARM_PC; i++)
		{
			reg = arm_reg_current(armv4_5, i);

			buf_set_u32(reg->value, 0, 32, regfile[i]);
			reg->valid = 1;
			reg->dirty = 0;
		}

		/* Fixup PC Resume Address */
		if (cpsr & (1 << 5))
		{
			// T bit set for Thumb or ThumbEE state
			regfile[ARM_PC] -= 4;
		}
		else
		{
			// ARM state
			regfile[ARM_PC] -= 8;
		}

		reg = armv4_5->core_cache->reg_list + 15;
		buf_set_u32(reg->value, 0, 32, regfile[ARM_PC]);
		reg->dirty = reg->valid;
	}

#if 0
/* TODO, Move this */
	uint32_t cp15_control_register, cp15_cacr, cp15_nacr;
	cortex_a8_read_cp(target, &cp15_control_register, 15, 0, 1, 0, 0);
	LOG_DEBUG("cp15_control_register = 0x%08x", cp15_control_register);

	cortex_a8_read_cp(target, &cp15_cacr, 15, 0, 1, 0, 2);
	LOG_DEBUG("cp15 Coprocessor Access Control Register = 0x%08x", cp15_cacr);

	cortex_a8_read_cp(target, &cp15_nacr, 15, 0, 1, 1, 2);
	LOG_DEBUG("cp15 Nonsecure Access Control Register = 0x%08x", cp15_nacr);
#endif

	/* Are we in an exception handler */
//	armv4_5->exception_number = 0;
	if (armv7a->post_debug_entry)
		armv7a->post_debug_entry(target);

	return retval;
}

static void cortex_a8_post_debug_entry(struct target *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	int retval;

	/* MRC p15,0,<Rt>,c1,c0,0 ; Read CP15 System Control Register */
	retval = armv7a->armv4_5_common.mrc(target, 15,
			0, 0,	/* op1, op2 */
			1, 0,	/* CRn, CRm */
			&cortex_a8->cp15_control_reg);
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32, cortex_a8->cp15_control_reg);

	if (armv7a->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		uint32_t cache_type_reg;

		/* MRC p15,0,<Rt>,c0,c0,1 ; Read CP15 Cache Type Register */
		retval = armv7a->armv4_5_common.mrc(target, 15,
				0, 1,	/* op1, op2 */
				0, 0,	/* CRn, CRm */
				&cache_type_reg);
		LOG_DEBUG("cp15 cache type: %8.8x", (unsigned) cache_type_reg);

		/* FIXME the armv4_4 cache info DOES NOT APPLY to Cortex-A8 */
		armv4_5_identify_cache(cache_type_reg,
				&armv7a->armv4_5_mmu.armv4_5_cache);
	}

	armv7a->armv4_5_mmu.mmu_enabled =
			(cortex_a8->cp15_control_reg & 0x1U) ? 1 : 0;
	armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled =
			(cortex_a8->cp15_control_reg & 0x4U) ? 1 : 0;
	armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled =
			(cortex_a8->cp15_control_reg & 0x1000U) ? 1 : 0;


}

static int cortex_a8_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *armv4_5 = &armv7a->armv4_5_common;
	struct breakpoint *breakpoint = NULL;
	struct breakpoint stepbreakpoint;
	struct reg *r;

	int timeout = 100;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	r = armv4_5->core_cache->reg_list + 15;
	if (!current)
	{
		buf_set_u32(r->value, 0, 32, address);
	}
	else
	{
		address = buf_get_u32(r->value, 0, 32);
	}

	/* The front-end may request us not to handle breakpoints.
	 * But since Cortex-A8 uses breakpoint for single step,
	 * we MUST handle breakpoints.
	 */
	handle_breakpoints = 1;
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, address);
		if (breakpoint)
			cortex_a8_unset_breakpoint(target, breakpoint);
	}

	/* Setup single step breakpoint */
	stepbreakpoint.address = address;
	stepbreakpoint.length = (armv4_5->core_state == ARM_STATE_THUMB)
			? 2 : 4;
	stepbreakpoint.type = BKPT_HARD;
	stepbreakpoint.set = 0;

	/* Break on IVA mismatch */
	cortex_a8_set_breakpoint(target, &stepbreakpoint, 0x04);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	cortex_a8_resume(target, 1, address, 0, 0);

	while (target->state != TARGET_HALTED)
	{
		cortex_a8_poll(target);
		if (--timeout == 0)
		{
			LOG_WARNING("timeout waiting for target halt");
			break;
		}
	}

	cortex_a8_unset_breakpoint(target, &stepbreakpoint);
	if (timeout > 0)
		target->debug_reason = DBG_REASON_BREAKPOINT;

	if (breakpoint)
		cortex_a8_set_breakpoint(target, breakpoint, 0);

	if (target->state != TARGET_HALTED)
		LOG_DEBUG("target stepped");

	return ERROR_OK;
}

static int cortex_a8_restore_context(struct target *target, bool bpwp)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	LOG_DEBUG(" ");

	if (armv7a->pre_restore_context)
		armv7a->pre_restore_context(target);

	arm_dpm_write_dirty_registers(&armv7a->dpm, bpwp);

	if (armv7a->post_restore_context)
		armv7a->post_restore_context(target);

	return ERROR_OK;
}


/*
 * Cortex-A8 Breakpoint and watchpoint fuctions
 */

/* Setup hardware Breakpoint Register Pair */
static int cortex_a8_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval;
	int brp_i=0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp * brp_list = cortex_a8->brp_list;

	if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		while (brp_list[brp_i].used && (brp_i < cortex_a8->brp_num))
			brp_i++ ;
		if (brp_i >= cortex_a8->brp_num)
		{
			LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
			return ERROR_FAIL;
		}
		breakpoint->set = brp_i + 1;
		if (breakpoint->length == 2)
		{
			byte_addr_select = (3 << (breakpoint->address & 0x02));
		}
		control = ((matchmode & 0x7) << 20)
				| (byte_addr_select << 5)
				| (3 << 1) | 1;
		brp_list[brp_i].used = 1;
		brp_list[brp_i].value = (breakpoint->address & 0xFFFFFFFC);
		brp_list[brp_i].control = control;
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control,
				brp_list[brp_i].value);
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		uint8_t code[4];
		if (breakpoint->length == 2)
		{
			buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		}
		else
		{
			buf_set_u32(code, 0, 32, ARMV5_BKPT(0x11));
		}
		retval = target->type->read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target->type->write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->set = 0x11; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int cortex_a8_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp * brp_list = cortex_a8->brp_list;

	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		int brp_i = breakpoint->set - 1;
		if ((brp_i < 0) || (brp_i >= cortex_a8->brp_num))
		{
			LOG_DEBUG("Invalid BRP number in breakpoint");
			return ERROR_OK;
		}
		LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
		brp_list[brp_i].used = 0;
		brp_list[brp_i].value = 0;
		brp_list[brp_i].control = 0;
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			retval = target->type->write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					4, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
		else
		{
			retval = target->type->write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					2, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int cortex_a8_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a8->brp_num_available < 1))
	{
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a8->brp_num_available--;
	cortex_a8_set_breakpoint(target, breakpoint, 0x00); /* Exact match */

	return ERROR_OK;
}

static int cortex_a8_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

#if 0
/* It is perfectly possible to remove brakpoints while the taget is running */
	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
#endif

	if (breakpoint->set)
	{
		cortex_a8_unset_breakpoint(target, breakpoint);
		if (breakpoint->type == BKPT_HARD)
			cortex_a8->brp_num_available++ ;
	}


	return ERROR_OK;
}



/*
 * Cortex-A8 Reset fuctions
 */

static int cortex_a8_assert_reset(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	LOG_DEBUG(" ");

	/* FIXME when halt is requested, make it work somehow... */

	/* Issue some kind of warm reset. */
	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else if (jtag_get_reset_config() & RESET_HAS_SRST) {
		/* REVISIT handle "pulls" cases, if there's
		 * hardware that needs them to work.
		 */
		jtag_add_reset(0, 1);
	} else {
		LOG_ERROR("%s: how to reset?", target_name(target));
		return ERROR_FAIL;
	}

	/* registers are now invalid */
	register_cache_invalidate(armv7a->armv4_5_common.core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int cortex_a8_deassert_reset(struct target *target)
{
	int retval;

	LOG_DEBUG(" ");

	/* be certain SRST is off */
	jtag_add_reset(0, 0);

	retval = cortex_a8_poll(target);

	if (target->reset_halt) {
		if (target->state != TARGET_HALTED) {
			LOG_WARNING("%s: ran after reset and before halt ...",
					target_name(target));
			if ((retval = target_halt(target)) != ERROR_OK)
				return retval;
		}
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 Memory access
 *
 * This is same Cortex M3 but we must also use the correct
 * ap number for every access.
 */

static int cortex_a8_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	int retval = ERROR_INVALID_ARGUMENTS;

	/* cortex_a8 handles unaligned memory access */

// ???	dap_ap_select(swjdp, swjdp_memoryap);

	if (count && buffer) {
		switch (size) {
		case 4:
			retval = mem_ap_read_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_read_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_read_buf_u8(swjdp, buffer, count, address);
			break;
		}
	}

	return retval;
}

static int cortex_a8_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	int retval = ERROR_INVALID_ARGUMENTS;

// ???	dap_ap_select(swjdp, swjdp_memoryap);

	if (count && buffer) {
		switch (size) {
		case 4:
			retval = mem_ap_write_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_write_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_write_buf_u8(swjdp, buffer, count, address);
			break;
		}
	}

	/* REVISIT this op is generic ARMv7-A/R stuff */
	if (retval == ERROR_OK && target->state == TARGET_HALTED)
	{
		struct arm_dpm *dpm = armv7a->armv4_5_common.dpm;

		retval = dpm->prepare(dpm);
		if (retval != ERROR_OK)
			return retval;

		/* The Cache handling will NOT work with MMU active, the
		 * wrong addresses will be invalidated!
		 *
		 * For both ICache and DCache, walk all cache lines in the
		 * address range. Cortex-A8 has fixed 64 byte line length.
		 *
		 * REVISIT per ARMv7, these may trigger watchpoints ...
		 */

		/* invalidate I-Cache */
		if (armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled)
		{
			/* ICIMVAU - Invalidate Cache single entry
			 * with MVA to PoU
			 *	MCR p15, 0, r0, c7, c5, 1
			 */
			for (uint32_t cacheline = address;
					cacheline < address + size * count;
					cacheline += 64) {
				retval = dpm->instr_write_data_r0(dpm,
					ARMV4_5_MCR(15, 0, 0, 7, 5, 1),
					cacheline);
			}
		}

		/* invalidate D-Cache */
		if (armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled)
		{
			/* DCIMVAC - Invalidate data Cache line
			 * with MVA to PoC
			 *	MCR p15, 0, r0, c7, c6, 1
			 */
			for (uint32_t cacheline = address;
					cacheline < address + size * count;
					cacheline += 64) {
				retval = dpm->instr_write_data_r0(dpm,
					ARMV4_5_MCR(15, 0, 0, 7, 6, 1),
					cacheline);
			}
		}

		/* (void) */ dpm->finish(dpm);
	}

	return retval;
}

static int cortex_a8_bulk_write_memory(struct target *target, uint32_t address,
		uint32_t count, uint8_t *buffer)
{
	return cortex_a8_write_memory(target, address, 4, count, buffer);
}


static int cortex_a8_dcc_read(struct swjdp_common *swjdp, uint8_t *value, uint8_t *ctrl)
{
#if 0
	u16 dcrdr;

	mem_ap_read_buf_u16(swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	*ctrl = (uint8_t)dcrdr;
	*value = (uint8_t)(dcrdr >> 8);

	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0))
	{
		dcrdr = 0;
		mem_ap_write_buf_u16(swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	}
#endif
	return ERROR_OK;
}


static int cortex_a8_handle_target_request(void *priv)
{
	struct target *target = priv;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	if (!target_was_examined(target))
		return ERROR_OK;
	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING)
	{
		uint8_t data = 0;
		uint8_t ctrl = 0;

		cortex_a8_dcc_read(swjdp, &data, &ctrl);

		/* check if we have data */
		if (ctrl & (1 << 0))
		{
			uint32_t request;

			/* we assume target is quick enough */
			request = data;
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 8);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 16);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 24);
			target_request(target, request);
		}
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 target information and configuration
 */

static int cortex_a8_examine_first(struct target *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	int i;
	int retval = ERROR_OK;
	uint32_t didr, ctypr, ttypr, cpuid;

	/* stop assuming this is an OMAP! */
	LOG_DEBUG("TODO - autoconfigure");

	/* Here we shall insert a proper ROM Table scan */
	armv7a->debug_base = OMAP3530_DEBUG_BASE;

	/* We do one extra read to ensure DAP is configured,
	 * we call ahbap_debugport_init(swjdp) instead
	 */
	ahbap_debugport_init(swjdp);
	mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_CPUID, &cpuid);
	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_CPUID, &cpuid)) != ERROR_OK)
	{
		LOG_DEBUG("Examine %s failed", "CPUID");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_CTYPR, &ctypr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine %s failed", "CTYPR");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_TTYPR, &ttypr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine %s failed", "TTYPR");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DIDR, &didr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine %s failed", "DIDR");
		return retval;
	}

	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);
	LOG_DEBUG("ctypr = 0x%08" PRIx32, ctypr);
	LOG_DEBUG("ttypr = 0x%08" PRIx32, ttypr);
	LOG_DEBUG("didr = 0x%08" PRIx32, didr);

	armv7a->armv4_5_common.core_type = ARM_MODE_MON;
	cortex_a8_dpm_setup(cortex_a8, didr);

	/* Setup Breakpoint Register Pairs */
	cortex_a8->brp_num = ((didr >> 24) & 0x0F) + 1;
	cortex_a8->brp_num_context = ((didr >> 20) & 0x0F) + 1;
	cortex_a8->brp_num_available = cortex_a8->brp_num;
	cortex_a8->brp_list = calloc(cortex_a8->brp_num, sizeof(struct cortex_a8_brp));
//	cortex_a8->brb_enabled = ????;
	for (i = 0; i < cortex_a8->brp_num; i++)
	{
		cortex_a8->brp_list[i].used = 0;
		if (i < (cortex_a8->brp_num-cortex_a8->brp_num_context))
			cortex_a8->brp_list[i].type = BRP_NORMAL;
		else
			cortex_a8->brp_list[i].type = BRP_CONTEXT;
		cortex_a8->brp_list[i].value = 0;
		cortex_a8->brp_list[i].control = 0;
		cortex_a8->brp_list[i].BRPn = i;
	}

	LOG_DEBUG("Configured %i hw breakpoints", cortex_a8->brp_num);

	target_set_examined(target);
	return ERROR_OK;
}

static int cortex_a8_examine(struct target *target)
{
	int retval = ERROR_OK;

	/* don't re-probe hardware after each reset */
	if (!target_was_examined(target))
		retval = cortex_a8_examine_first(target);

	/* Configure core debug access */
	if (retval == ERROR_OK)
		retval = cortex_a8_init_debug_access(target);

	return retval;
}

/*
 *	Cortex-A8 target creation and initialization
 */

static int cortex_a8_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* examine_first() does a bunch of this */
	return ERROR_OK;
}

static int cortex_a8_init_arch_info(struct target *target,
		struct cortex_a8_common *cortex_a8, struct jtag_tap *tap)
{
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct arm *armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	/* Setup struct cortex_a8_common */
	cortex_a8->common_magic = CORTEX_A8_COMMON_MAGIC;
	armv4_5->arch_info = armv7a;

	/* prepare JTAG information for the new target */
	cortex_a8->jtag_info.tap = tap;
	cortex_a8->jtag_info.scann_size = 4;

	swjdp->dp_select_value = -1;
	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;
	swjdp->jtag_info = &cortex_a8->jtag_info;
	swjdp->memaccess_tck = 80;

	/* Number of bits for tar autoincrement, impl. dep. at least 10 */
	swjdp->tar_autoincr_block = (1 << 10);

	cortex_a8->fast_reg_read = 0;

	/* register arch-specific functions */
	armv7a->examine_debug_reason = NULL;

	armv7a->post_debug_entry = cortex_a8_post_debug_entry;

	armv7a->pre_restore_context = NULL;
	armv7a->post_restore_context = NULL;
	armv7a->armv4_5_mmu.armv4_5_cache.ctype = -1;
//	armv7a->armv4_5_mmu.get_ttb = armv7a_get_ttb;
	armv7a->armv4_5_mmu.read_memory = cortex_a8_read_memory;
	armv7a->armv4_5_mmu.write_memory = cortex_a8_write_memory;
//	armv7a->armv4_5_mmu.disable_mmu_caches = armv7a_disable_mmu_caches;
//	armv7a->armv4_5_mmu.enable_mmu_caches = armv7a_enable_mmu_caches;
	armv7a->armv4_5_mmu.has_tiny_pages = 1;
	armv7a->armv4_5_mmu.mmu_enabled = 0;


//	arm7_9->handle_target_request = cortex_a8_handle_target_request;

	/* REVISIT v7a setup should be in a v7a-specific routine */
	arm_init_arch_info(target, armv4_5);
	armv7a->common_magic = ARMV7_COMMON_MAGIC;

	target_register_timer_callback(cortex_a8_handle_target_request, 1, 1, target);

	return ERROR_OK;
}

static int cortex_a8_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_a8_common *cortex_a8 = calloc(1, sizeof(struct cortex_a8_common));

	cortex_a8_init_arch_info(target, cortex_a8, target->tap);

	return ERROR_OK;
}

COMMAND_HANDLER(cortex_a8_handle_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	return armv4_5_handle_cache_info_command(CMD_CTX,
			&armv7a->armv4_5_mmu.armv4_5_cache);
}


COMMAND_HANDLER(cortex_a8_handle_dbginit_command)
{
	struct target *target = get_current_target(CMD_CTX);

	cortex_a8_init_debug_access(target);

	return ERROR_OK;
}

static const struct command_registration cortex_a8_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = cortex_a8_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about target caches",
	},
	{
		.name = "dbginit",
		.handler = cortex_a8_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_a8_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = armv7a_command_handlers,
	},
	{
		.name = "cortex_a8",
		.mode = COMMAND_ANY,
		.help = "Cortex-A8 command group",
		.chain = cortex_a8_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexa8_target = {
	.name = "cortex_a8",

	.poll = cortex_a8_poll,
	.arch_state = armv7a_arch_state,

	.target_request_data = NULL,

	.halt = cortex_a8_halt,
	.resume = cortex_a8_resume,
	.step = cortex_a8_step,

	.assert_reset = cortex_a8_assert_reset,
	.deassert_reset = cortex_a8_deassert_reset,
	.soft_reset_halt = NULL,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = cortex_a8_read_memory,
	.write_memory = cortex_a8_write_memory,
	.bulk_write_memory = cortex_a8_bulk_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a8_add_breakpoint,
	.remove_breakpoint = cortex_a8_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = cortex_a8_command_handlers,
	.target_create = cortex_a8_target_create,
	.init_target = cortex_a8_init_target,
	.examine = cortex_a8_examine,
};

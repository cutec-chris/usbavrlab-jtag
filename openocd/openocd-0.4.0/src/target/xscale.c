/***************************************************************************
 *   Copyright (C) 2006, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Michael Schwingen                                  *
 *   michael@schwingen.org                                                 *
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
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "xscale.h"
#include "target_type.h"
#include "arm_jtag.h"
#include "arm_simulator.h"
#include "arm_disassembler.h"
#include <helper/time_support.h>
#include "register.h"
#include "image.h"
#include "arm_opcodes.h"
#include "armv4_5.h"


/*
 * Important XScale documents available as of October 2009 include:
 *
 *  Intel XScale® Core Developer’s Manual, January 2004
 *		Order Number: 273473-002
 *	This has a chapter detailing debug facilities, and punts some
 *	details to chip-specific microarchitecture documents.
 *
 *  Hot-Debug for Intel XScale® Core Debug White Paper, May 2005
 *		Document Number: 273539-005
 *	Less detailed than the developer's manual, but summarizes those
 *	missing details (for most XScales) and gives LOTS of notes about
 *	debugger/handler interaction issues.  Presents a simpler reset
 *	and load-handler sequence than the arch doc.  (Note, OpenOCD
 *	doesn't currently support "Hot-Debug" as defined there.)
 *
 * Chip-specific microarchitecture documents may also be useful.
 */


/* forward declarations */
static int xscale_resume(struct target *, int current,
	uint32_t address, int handle_breakpoints, int debug_execution);
static int xscale_debug_entry(struct target *);
static int xscale_restore_banked(struct target *);
static int xscale_get_reg(struct reg *reg);
static int xscale_set_reg(struct reg *reg, uint8_t *buf);
static int xscale_set_breakpoint(struct target *, struct breakpoint *);
static int xscale_set_watchpoint(struct target *, struct watchpoint *);
static int xscale_unset_breakpoint(struct target *, struct breakpoint *);
static int xscale_read_trace(struct target *);


/* This XScale "debug handler" is loaded into the processor's
 * mini-ICache, which is 2K of code writable only via JTAG.
 *
 * FIXME  the OpenOCD "bin2char" utility currently doesn't handle
 * binary files cleanly.  It's string oriented, and terminates them
 * with a NUL character.  Better would be to generate the constants
 * and let other code decide names, scoping, and other housekeeping.
 */
static /* unsigned const char xscale_debug_handler[] = ... */
#include "xscale_debug.h"

static char *const xscale_reg_list[] =
{
	"XSCALE_MAINID",		/* 0 */
	"XSCALE_CACHETYPE",
	"XSCALE_CTRL",
	"XSCALE_AUXCTRL",
	"XSCALE_TTB",
	"XSCALE_DAC",
	"XSCALE_FSR",
	"XSCALE_FAR",
	"XSCALE_PID",
	"XSCALE_CPACCESS",
	"XSCALE_IBCR0",			/* 10 */
	"XSCALE_IBCR1",
	"XSCALE_DBR0",
	"XSCALE_DBR1",
	"XSCALE_DBCON",
	"XSCALE_TBREG",
	"XSCALE_CHKPT0",
	"XSCALE_CHKPT1",
	"XSCALE_DCSR",
	"XSCALE_TX",
	"XSCALE_RX",			/* 20 */
	"XSCALE_TXRXCTRL",
};

static const struct xscale_reg xscale_reg_arch_info[] =
{
	{XSCALE_MAINID, NULL},
	{XSCALE_CACHETYPE, NULL},
	{XSCALE_CTRL, NULL},
	{XSCALE_AUXCTRL, NULL},
	{XSCALE_TTB, NULL},
	{XSCALE_DAC, NULL},
	{XSCALE_FSR, NULL},
	{XSCALE_FAR, NULL},
	{XSCALE_PID, NULL},
	{XSCALE_CPACCESS, NULL},
	{XSCALE_IBCR0, NULL},
	{XSCALE_IBCR1, NULL},
	{XSCALE_DBR0, NULL},
	{XSCALE_DBR1, NULL},
	{XSCALE_DBCON, NULL},
	{XSCALE_TBREG, NULL},
	{XSCALE_CHKPT0, NULL},
	{XSCALE_CHKPT1, NULL},
	{XSCALE_DCSR, NULL}, /* DCSR accessed via JTAG or SW */
	{-1, NULL}, /* TX accessed via JTAG */
	{-1, NULL}, /* RX accessed via JTAG */
	{-1, NULL}, /* TXRXCTRL implicit access via JTAG */
};

/* convenience wrapper to access XScale specific registers */
static int xscale_set_reg_u32(struct reg *reg, uint32_t value)
{
	uint8_t buf[4];

	buf_set_u32(buf, 0, 32, value);

	return xscale_set_reg(reg, buf);
}

static const char xscale_not[] = "target is not an XScale";

static int xscale_verify_pointer(struct command_context *cmd_ctx,
		struct xscale_common *xscale)
{
	if (xscale->common_magic != XSCALE_COMMON_MAGIC) {
		command_print(cmd_ctx, xscale_not);
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

static int xscale_jtag_set_instr(struct jtag_tap *tap, uint32_t new_instr)
{
	if (tap == NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		struct scan_field field;
		uint8_t scratch[4];

		memset(&field, 0, sizeof field);
		field.tap = tap;
		field.num_bits = tap->ir_length;
		field.out_value = scratch;
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);

		jtag_add_ir_scan(1, &field, jtag_get_end_state());
	}

	return ERROR_OK;
}

static int xscale_read_dcsr(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;
	struct scan_field fields[3];
	uint8_t field0 = 0x0;
	uint8_t field0_check_value = 0x2;
	uint8_t field0_check_mask = 0x7;
	uint8_t field2 = 0x0;
	uint8_t field2_check_value = 0x0;
	uint8_t field2_check_mask = 0x1;

	jtag_set_end_state(TAP_DRPAUSE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_SELDCSR << xscale->xscale_variant);

	buf_set_u32(&field0, 1, 1, xscale->hold_rst);
	buf_set_u32(&field0, 2, 1, xscale->external_debug_break);

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0;
	uint8_t tmp;
	fields[0].in_value = &tmp;

	fields[1].tap = target->tap;
	fields[1].num_bits = 32;
	fields[1].in_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;

	fields[2].tap = target->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	uint8_t tmp2;
	fields[2].in_value = &tmp2;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	jtag_check_value_mask(fields + 0, &field0_check_value, &field0_check_mask);
	jtag_check_value_mask(fields + 2, &field2_check_value, &field2_check_mask);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while reading DCSR");
		return retval;
	}

	xscale->reg_cache->reg_list[XSCALE_DCSR].dirty = 0;
	xscale->reg_cache->reg_list[XSCALE_DCSR].valid = 1;

	/* write the register with the value we just read
	 * on this second pass, only the first bit of field0 is guaranteed to be 0)
	 */
	field0_check_mask = 0x1;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;
	fields[1].in_value = NULL;

	jtag_set_end_state(TAP_IDLE);

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	/* DANGER!!! this must be here. It will make sure that the arguments
	 * to jtag_set_check_value() does not go out of scope! */
	return jtag_execute_queue();
}


static void xscale_getbuf(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)in) = buf_get_u32(in, 0, 32);
}

static int xscale_receive(struct target *target, uint32_t *buffer, int num_words)
{
	if (num_words == 0)
		return ERROR_INVALID_ARGUMENTS;

	struct xscale_common *xscale = target_to_xscale(target);
	int retval = ERROR_OK;
	tap_state_t path[3];
	struct scan_field fields[3];
	uint8_t *field0 = malloc(num_words * 1);
	uint8_t field0_check_value = 0x2;
	uint8_t field0_check_mask = 0x6;
	uint32_t *field1 = malloc(num_words * 4);
	uint8_t field2_check_value = 0x0;
	uint8_t field2_check_mask = 0x1;
	int words_done = 0;
	int words_scheduled = 0;
	int i;

	path[0] = TAP_DRSELECT;
	path[1] = TAP_DRCAPTURE;
	path[2] = TAP_DRSHIFT;

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 3;
	fields[0].check_value = &field0_check_value;
	fields[0].check_mask = &field0_check_mask;

	fields[1].tap = target->tap;
	fields[1].num_bits = 32;

	fields[2].tap = target->tap;
	fields[2].num_bits = 1;
	fields[2].check_value = &field2_check_value;
	fields[2].check_mask = &field2_check_mask;

	jtag_set_end_state(TAP_IDLE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_DBGTX << xscale->xscale_variant);
	jtag_add_runtest(1, jtag_get_end_state()); /* ensures that we're in the TAP_IDLE state as the above could be a no-op */

	/* repeat until all words have been collected */
	int attempts = 0;
	while (words_done < num_words)
	{
		/* schedule reads */
		words_scheduled = 0;
		for (i = words_done; i < num_words; i++)
		{
			fields[0].in_value = &field0[i];

			jtag_add_pathmove(3, path);

			fields[1].in_value = (uint8_t *)(field1 + i);

			jtag_add_dr_scan_check(3, fields, jtag_set_end_state(TAP_IDLE));

			jtag_add_callback(xscale_getbuf, (jtag_callback_data_t)(field1 + i));

			words_scheduled++;
		}

		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			LOG_ERROR("JTAG error while receiving data from debug handler");
			break;
		}

		/* examine results */
		for (i = words_done; i < num_words; i++)
		{
			if (!(field0[0] & 1))
			{
				/* move backwards if necessary */
				int j;
				for (j = i; j < num_words - 1; j++)
				{
					field0[j] = field0[j + 1];
					field1[j] = field1[j + 1];
				}
				words_scheduled--;
			}
		}
		if (words_scheduled == 0)
		{
			if (attempts++==1000)
			{
				LOG_ERROR("Failed to receiving data from debug handler after 1000 attempts");
				retval = ERROR_TARGET_TIMEOUT;
				break;
			}
		}

		words_done += words_scheduled;
	}

	for (i = 0; i < num_words; i++)
		*(buffer++) = buf_get_u32((uint8_t*)&field1[i], 0, 32);

	free(field1);

	return retval;
}

static int xscale_read_tx(struct target *target, int consume)
{
	struct xscale_common *xscale = target_to_xscale(target);
	tap_state_t path[3];
	tap_state_t noconsume_path[6];
	int retval;
	struct timeval timeout, now;
	struct scan_field fields[3];
	uint8_t field0_in = 0x0;
	uint8_t field0_check_value = 0x2;
	uint8_t field0_check_mask = 0x6;
	uint8_t field2_check_value = 0x0;
	uint8_t field2_check_mask = 0x1;

	jtag_set_end_state(TAP_IDLE);

	xscale_jtag_set_instr(target->tap,
		XSCALE_DBGTX << xscale->xscale_variant);

	path[0] = TAP_DRSELECT;
	path[1] = TAP_DRCAPTURE;
	path[2] = TAP_DRSHIFT;

	noconsume_path[0] = TAP_DRSELECT;
	noconsume_path[1] = TAP_DRCAPTURE;
	noconsume_path[2] = TAP_DREXIT1;
	noconsume_path[3] = TAP_DRPAUSE;
	noconsume_path[4] = TAP_DREXIT2;
	noconsume_path[5] = TAP_DRSHIFT;

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 3;
	fields[0].in_value = &field0_in;

	fields[1].tap = target->tap;
	fields[1].num_bits = 32;
	fields[1].in_value = xscale->reg_cache->reg_list[XSCALE_TX].value;

	fields[2].tap = target->tap;
	fields[2].num_bits = 1;
	uint8_t tmp;
	fields[2].in_value = &tmp;

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 1, 0);

	for (;;)
	{
		/* if we want to consume the register content (i.e. clear TX_READY),
		 * we have to go straight from Capture-DR to Shift-DR
		 * otherwise, we go from Capture-DR to Exit1-DR to Pause-DR
		*/
		if (consume)
			jtag_add_pathmove(3, path);
		else
		{
			jtag_add_pathmove(ARRAY_SIZE(noconsume_path), noconsume_path);
		}

		jtag_add_dr_scan(3, fields, jtag_set_end_state(TAP_IDLE));

		jtag_check_value_mask(fields + 0, &field0_check_value, &field0_check_mask);
		jtag_check_value_mask(fields + 2, &field2_check_value, &field2_check_mask);

		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			LOG_ERROR("JTAG error while reading TX");
			return ERROR_TARGET_TIMEOUT;
		}

		gettimeofday(&now, NULL);
		if ((now.tv_sec > timeout.tv_sec) || ((now.tv_sec == timeout.tv_sec)&& (now.tv_usec > timeout.tv_usec)))
		{
			LOG_ERROR("time out reading TX register");
			return ERROR_TARGET_TIMEOUT;
		}
		if (!((!(field0_in & 1)) && consume))
		{
			goto done;
		}
		if (debug_level >= 3)
		{
			LOG_DEBUG("waiting 100ms");
			alive_sleep(100); /* avoid flooding the logs */
		} else
		{
			keep_alive();
		}
	}
	done:

	if (!(field0_in & 1))
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	return ERROR_OK;
}

static int xscale_write_rx(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;
	struct timeval timeout, now;
	struct scan_field fields[3];
	uint8_t field0_out = 0x0;
	uint8_t field0_in = 0x0;
	uint8_t field0_check_value = 0x2;
	uint8_t field0_check_mask = 0x6;
	uint8_t field2 = 0x0;
	uint8_t field2_check_value = 0x0;
	uint8_t field2_check_mask = 0x1;

	jtag_set_end_state(TAP_IDLE);

	xscale_jtag_set_instr(target->tap,
		XSCALE_DBGRX << xscale->xscale_variant);

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0_out;
	fields[0].in_value = &field0_in;

	fields[1].tap = target->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_RX].value;

	fields[2].tap = target->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	uint8_t tmp;
	fields[2].in_value = &tmp;

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 1, 0);

	/* poll until rx_read is low */
	LOG_DEBUG("polling RX");
	for (;;)
	{
		jtag_add_dr_scan(3, fields, jtag_set_end_state(TAP_IDLE));

		jtag_check_value_mask(fields + 0, &field0_check_value, &field0_check_mask);
		jtag_check_value_mask(fields + 2, &field2_check_value, &field2_check_mask);

		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			LOG_ERROR("JTAG error while writing RX");
			return retval;
		}

		gettimeofday(&now, NULL);
		if ((now.tv_sec > timeout.tv_sec) || ((now.tv_sec == timeout.tv_sec)&& (now.tv_usec > timeout.tv_usec)))
		{
			LOG_ERROR("time out writing RX register");
			return ERROR_TARGET_TIMEOUT;
		}
		if (!(field0_in & 1))
			goto done;
		if (debug_level >= 3)
		{
			LOG_DEBUG("waiting 100ms");
			alive_sleep(100); /* avoid flooding the logs */
		} else
		{
			keep_alive();
		}
	}
	done:

	/* set rx_valid */
	field2 = 0x1;
	jtag_add_dr_scan(3, fields, jtag_set_end_state(TAP_IDLE));

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while writing RX");
		return retval;
	}

	return ERROR_OK;
}

/* send count elements of size byte to the debug handler */
static int xscale_send(struct target *target, uint8_t *buffer, int count, int size)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t t[3];
	int bits[3];
	int retval;
	int done_count = 0;

	jtag_set_end_state(TAP_IDLE);

	xscale_jtag_set_instr(target->tap,
		XSCALE_DBGRX << xscale->xscale_variant);

	bits[0]=3;
	t[0]=0;
	bits[1]=32;
	t[2]=1;
	bits[2]=1;
	int endianness = target->endianness;
	while (done_count++ < count)
	{
		switch (size)
		{
		case 4:
			if (endianness == TARGET_LITTLE_ENDIAN)
			{
				t[1]=le_to_h_u32(buffer);
			} else
			{
				t[1]=be_to_h_u32(buffer);
			}
			break;
		case 2:
			if (endianness == TARGET_LITTLE_ENDIAN)
			{
				t[1]=le_to_h_u16(buffer);
			} else
			{
				t[1]=be_to_h_u16(buffer);
			}
			break;
		case 1:
			t[1]=buffer[0];
			break;
		default:
			LOG_ERROR("BUG: size neither 4, 2 nor 1");
			return ERROR_INVALID_ARGUMENTS;
		}
		jtag_add_dr_out(target->tap,
				3,
				bits,
				t,
				jtag_set_end_state(TAP_IDLE));
		buffer += size;
	}

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while sending data to debug handler");
		return retval;
	}

	return ERROR_OK;
}

static int xscale_send_u32(struct target *target, uint32_t value)
{
	struct xscale_common *xscale = target_to_xscale(target);

	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_RX].value, 0, 32, value);
	return xscale_write_rx(target);
}

static int xscale_write_dcsr(struct target *target, int hold_rst, int ext_dbg_brk)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;
	struct scan_field fields[3];
	uint8_t field0 = 0x0;
	uint8_t field0_check_value = 0x2;
	uint8_t field0_check_mask = 0x7;
	uint8_t field2 = 0x0;
	uint8_t field2_check_value = 0x0;
	uint8_t field2_check_mask = 0x1;

	if (hold_rst != -1)
		xscale->hold_rst = hold_rst;

	if (ext_dbg_brk != -1)
		xscale->external_debug_break = ext_dbg_brk;

	jtag_set_end_state(TAP_IDLE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_SELDCSR << xscale->xscale_variant);

	buf_set_u32(&field0, 1, 1, xscale->hold_rst);
	buf_set_u32(&field0, 2, 1, xscale->external_debug_break);

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0;
	uint8_t tmp;
	fields[0].in_value = &tmp;

	fields[1].tap = target->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;

	fields[2].tap = target->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	uint8_t tmp2;
	fields[2].in_value = &tmp2;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	jtag_check_value_mask(fields + 0, &field0_check_value, &field0_check_mask);
	jtag_check_value_mask(fields + 2, &field2_check_value, &field2_check_mask);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while writing DCSR");
		return retval;
	}

	xscale->reg_cache->reg_list[XSCALE_DCSR].dirty = 0;
	xscale->reg_cache->reg_list[XSCALE_DCSR].valid = 1;

	return ERROR_OK;
}

/* parity of the number of bits 0 if even; 1 if odd. for 32 bit words */
static unsigned int parity (unsigned int v)
{
	// unsigned int ov = v;
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	// LOG_DEBUG("parity of 0x%x is %i", ov, (0x6996 >> v) & 1);
	return (0x6996 >> v) & 1;
}

static int xscale_load_ic(struct target *target, uint32_t va, uint32_t buffer[8])
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint8_t packet[4];
	uint8_t cmd;
	int word;
	struct scan_field fields[2];

	LOG_DEBUG("loading miniIC at 0x%8.8" PRIx32 "", va);

	/* LDIC into IR */
	jtag_set_end_state(TAP_IDLE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_LDIC << xscale->xscale_variant);

	/* CMD is b011 to load a cacheline into the Mini ICache.
	 * Loading into the main ICache is deprecated, and unused.
	 * It's followed by three zero bits, and 27 address bits.
	 */
	buf_set_u32(&cmd, 0, 6, 0x3);

	/* virtual address of desired cache line */
	buf_set_u32(packet, 0, 27, va >> 5);

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 6;
	fields[0].out_value = &cmd;

	fields[1].tap = target->tap;
	fields[1].num_bits = 27;
	fields[1].out_value = packet;

	jtag_add_dr_scan(2, fields, jtag_get_end_state());

	/* rest of packet is a cacheline: 8 instructions, with parity */
	fields[0].num_bits = 32;
	fields[0].out_value = packet;

	fields[1].num_bits = 1;
	fields[1].out_value = &cmd;

	for (word = 0; word < 8; word++)
	{
		buf_set_u32(packet, 0, 32, buffer[word]);

		uint32_t value;
		memcpy(&value, packet, sizeof(uint32_t));
		cmd = parity(value);

		jtag_add_dr_scan(2, fields, jtag_get_end_state());
	}

	return jtag_execute_queue();
}

static int xscale_invalidate_ic_line(struct target *target, uint32_t va)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint8_t packet[4];
	uint8_t cmd;
	struct scan_field fields[2];

	jtag_set_end_state(TAP_IDLE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_LDIC << xscale->xscale_variant);

	/* CMD for invalidate IC line b000, bits [6:4] b000 */
	buf_set_u32(&cmd, 0, 6, 0x0);

	/* virtual address of desired cache line */
	buf_set_u32(packet, 0, 27, va >> 5);

	memset(&fields, 0, sizeof fields);

	fields[0].tap = target->tap;
	fields[0].num_bits = 6;
	fields[0].out_value = &cmd;

	fields[1].tap = target->tap;
	fields[1].num_bits = 27;
	fields[1].out_value = packet;

	jtag_add_dr_scan(2, fields, jtag_get_end_state());

	return ERROR_OK;
}

static int xscale_update_vectors(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int i;
	int retval;

	uint32_t low_reset_branch, high_reset_branch;

	for (i = 1; i < 8; i++)
	{
		/* if there's a static vector specified for this exception, override */
		if (xscale->static_high_vectors_set & (1 << i))
		{
			xscale->high_vectors[i] = xscale->static_high_vectors[i];
		}
		else
		{
			retval = target_read_u32(target, 0xffff0000 + 4*i, &xscale->high_vectors[i]);
			if (retval == ERROR_TARGET_TIMEOUT)
				return retval;
			if (retval != ERROR_OK)
			{
				/* Some of these reads will fail as part of normal execution */
				xscale->high_vectors[i] = ARMV4_5_B(0xfffffe, 0);
			}
		}
	}

	for (i = 1; i < 8; i++)
	{
		if (xscale->static_low_vectors_set & (1 << i))
		{
			xscale->low_vectors[i] = xscale->static_low_vectors[i];
		}
		else
		{
			retval = target_read_u32(target, 0x0 + 4*i, &xscale->low_vectors[i]);
			if (retval == ERROR_TARGET_TIMEOUT)
				return retval;
			if (retval != ERROR_OK)
			{
				/* Some of these reads will fail as part of normal execution */
				xscale->low_vectors[i] = ARMV4_5_B(0xfffffe, 0);
			}
		}
	}

	/* calculate branches to debug handler */
	low_reset_branch = (xscale->handler_address + 0x20 - 0x0 - 0x8) >> 2;
	high_reset_branch = (xscale->handler_address + 0x20 - 0xffff0000 - 0x8) >> 2;

	xscale->low_vectors[0] = ARMV4_5_B((low_reset_branch & 0xffffff), 0);
	xscale->high_vectors[0] = ARMV4_5_B((high_reset_branch & 0xffffff), 0);

	/* invalidate and load exception vectors in mini i-cache */
	xscale_invalidate_ic_line(target, 0x0);
	xscale_invalidate_ic_line(target, 0xffff0000);

	xscale_load_ic(target, 0x0, xscale->low_vectors);
	xscale_load_ic(target, 0xffff0000, xscale->high_vectors);

	return ERROR_OK;
}

static int xscale_arch_state(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;

	static const char *state[] =
	{
		"disabled", "enabled"
	};

	static const char *arch_dbg_reason[] =
	{
		"", "\n(processor reset)", "\n(trace buffer full)"
	};

	if (armv4_5->common_magic != ARM_COMMON_MAGIC)
	{
		LOG_ERROR("BUG: called for a non-ARMv4/5 target");
		return ERROR_INVALID_ARGUMENTS;
	}

	arm_arch_state(target);
	LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s%s",
			 state[xscale->armv4_5_mmu.mmu_enabled],
			 state[xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
			 state[xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled],
			 arch_dbg_reason[xscale->arch_debug_reason]);

	return ERROR_OK;
}

static int xscale_poll(struct target *target)
{
	int retval = ERROR_OK;

	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
	{
		enum target_state previous_state = target->state;
		if ((retval = xscale_read_tx(target, 0)) == ERROR_OK)
		{

			/* there's data to read from the tx register, we entered debug state */
			target->state = TARGET_HALTED;

			/* process debug entry, fetching current mode regs */
			retval = xscale_debug_entry(target);
		}
		else if (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		{
			LOG_USER("error while polling TX register, reset CPU");
			/* here we "lie" so GDB won't get stuck and a reset can be perfomed */
			target->state = TARGET_HALTED;
		}

		/* debug_entry could have overwritten target state (i.e. immediate resume)
		 * don't signal event handlers in that case
		 */
		if (target->state != TARGET_HALTED)
			return ERROR_OK;

		/* if target was running, signal that we halted
		 * otherwise we reentered from debug execution */
		if (previous_state == TARGET_RUNNING)
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		else
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
	}

	return retval;
}

static int xscale_debug_entry(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	uint32_t pc;
	uint32_t buffer[10];
	int i;
	int retval;
	uint32_t moe;

	/* clear external dbg break (will be written on next DCSR read) */
	xscale->external_debug_break = 0;
	if ((retval = xscale_read_dcsr(target)) != ERROR_OK)
		return retval;

	/* get r0, pc, r1 to r7 and cpsr */
	if ((retval = xscale_receive(target, buffer, 10)) != ERROR_OK)
		return retval;

	/* move r0 from buffer to register cache */
	buf_set_u32(armv4_5->core_cache->reg_list[0].value, 0, 32, buffer[0]);
	armv4_5->core_cache->reg_list[0].dirty = 1;
	armv4_5->core_cache->reg_list[0].valid = 1;
	LOG_DEBUG("r0: 0x%8.8" PRIx32 "", buffer[0]);

	/* move pc from buffer to register cache */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, buffer[1]);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;
	LOG_DEBUG("pc: 0x%8.8" PRIx32 "", buffer[1]);

	/* move data from buffer to register cache */
	for (i = 1; i <= 7; i++)
	{
		buf_set_u32(armv4_5->core_cache->reg_list[i].value, 0, 32, buffer[1 + i]);
		armv4_5->core_cache->reg_list[i].dirty = 1;
		armv4_5->core_cache->reg_list[i].valid = 1;
		LOG_DEBUG("r%i: 0x%8.8" PRIx32 "", i, buffer[i + 1]);
	}

	arm_set_cpsr(armv4_5, buffer[9]);
	LOG_DEBUG("cpsr: 0x%8.8" PRIx32 "", buffer[9]);

	if (!is_arm_mode(armv4_5->core_mode))
	{
		target->state = TARGET_UNKNOWN;
		LOG_ERROR("cpsr contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}
	LOG_DEBUG("target entered debug state in %s mode",
			 arm_mode_name(armv4_5->core_mode));

	/* get banked registers, r8 to r14, and spsr if not in USR/SYS mode */
	if (armv4_5->spsr) {
		xscale_receive(target, buffer, 8);
		buf_set_u32(armv4_5->spsr->value, 0, 32, buffer[7]);
		armv4_5->spsr->dirty = false;
		armv4_5->spsr->valid = true;
	}
	else
	{
		/* r8 to r14, but no spsr */
		xscale_receive(target, buffer, 7);
	}

	/* move data from buffer to right banked register in cache */
	for (i = 8; i <= 14; i++)
	{
		struct reg *r = arm_reg_current(armv4_5, i);

		buf_set_u32(r->value, 0, 32, buffer[i - 8]);
		r->dirty = false;
		r->valid = true;
	}

	/* examine debug reason */
	xscale_read_dcsr(target);
	moe = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 2, 3);

	/* stored PC (for calculating fixup) */
	pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	switch (moe)
	{
		case 0x0: /* Processor reset */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_RESET;
			pc -= 4;
			break;
		case 0x1: /* Instruction breakpoint hit */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x2: /* Data breakpoint hit */
			target->debug_reason = DBG_REASON_WATCHPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x3: /* BKPT instruction executed */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x4: /* Ext. debug event */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x5: /* Vector trap occured */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x6: /* Trace buffer full break */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_TB_FULL;
			pc -= 4;
			break;
		case 0x7: /* Reserved (may flag Hot-Debug support) */
		default:
			LOG_ERROR("Method of Entry is 'Reserved'");
			exit(-1);
			break;
	}

	/* apply PC fixup */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, pc);

	/* on the first debug entry, identify cache type */
	if (xscale->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		uint32_t cache_type_reg;

		/* read cp15 cache type register */
		xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CACHETYPE]);
		cache_type_reg = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CACHETYPE].value, 0, 32);

		armv4_5_identify_cache(cache_type_reg, &xscale->armv4_5_mmu.armv4_5_cache);
	}

	/* examine MMU and Cache settings */
	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	xscale->cp15_control_reg = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);
	xscale->armv4_5_mmu.mmu_enabled = (xscale->cp15_control_reg & 0x1U) ? 1 : 0;
	xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (xscale->cp15_control_reg & 0x4U) ? 1 : 0;
	xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled = (xscale->cp15_control_reg & 0x1000U) ? 1 : 0;

	/* tracing enabled, read collected trace data */
	if (xscale->trace.buffer_enabled)
	{
		xscale_read_trace(target);
		xscale->trace.buffer_fill--;

		/* resume if we're still collecting trace data */
		if ((xscale->arch_debug_reason == XSCALE_DBG_REASON_TB_FULL)
			&& (xscale->trace.buffer_fill > 0))
		{
			xscale_resume(target, 1, 0x0, 1, 0);
		}
		else
		{
			xscale->trace.buffer_enabled = 0;
		}
	}

	return ERROR_OK;
}

static int xscale_halt(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);

	LOG_DEBUG("target->state: %s",
		  target_state_name(target));

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}
	else if (target->state == TARGET_UNKNOWN)
	{
		/* this must not happen for a xscale target */
		LOG_ERROR("target was in unknown state when halt was requested");
		return ERROR_TARGET_INVALID;
	}
	else if (target->state == TARGET_RESET)
	{
		LOG_DEBUG("target->state == TARGET_RESET");
	}
	else
	{
		/* assert external dbg break */
		xscale->external_debug_break = 1;
		xscale_read_dcsr(target);

		target->debug_reason = DBG_REASON_DBGRQ;
	}

	return ERROR_OK;
}

static int xscale_enable_single_step(struct target *target, uint32_t next_pc)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct reg *ibcr0 = &xscale->reg_cache->reg_list[XSCALE_IBCR0];
	int retval;

	if (xscale->ibcr0_used)
	{
		struct breakpoint *ibcr0_bp = breakpoint_find(target, buf_get_u32(ibcr0->value, 0, 32) & 0xfffffffe);

		if (ibcr0_bp)
		{
			xscale_unset_breakpoint(target, ibcr0_bp);
		}
		else
		{
			LOG_ERROR("BUG: xscale->ibcr0_used is set, but no breakpoint with that address found");
			exit(-1);
		}
	}

	if ((retval = xscale_set_reg_u32(ibcr0, next_pc | 0x1)) != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int xscale_disable_single_step(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct reg *ibcr0 = &xscale->reg_cache->reg_list[XSCALE_IBCR0];
	int retval;

	if ((retval = xscale_set_reg_u32(ibcr0, 0x0)) != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static void xscale_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	while (watchpoint)
	{
		if (watchpoint->set == 0)
			xscale_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static void xscale_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set == 0)
			xscale_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int xscale_resume(struct target *target, int current,
		uint32_t address, int handle_breakpoints, int debug_execution)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	struct breakpoint *breakpoint = target->breakpoints;
	uint32_t current_pc;
	int retval;
	int i;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution)
	{
		target_free_all_working_areas(target);
	}

	/* update vector tables */
	if ((retval = xscale_update_vectors(target)) != ERROR_OK)
		return retval;

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);

	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	/* if we're at the reset vector, we have to simulate the branch */
	if (current_pc == 0x0)
	{
		arm_simulate_step(target, NULL);
		current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			uint32_t next_pc;

			/* there's a breakpoint at the current PC, we have to step over it */
			LOG_DEBUG("unset breakpoint at 0x%8.8" PRIx32 "", breakpoint->address);
			xscale_unset_breakpoint(target, breakpoint);

			/* calculate PC of next instruction */
			if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
			{
				uint32_t current_opcode;
				target_read_u32(target, current_pc, &current_opcode);
				LOG_ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8" PRIx32 "", current_opcode);
			}

			LOG_DEBUG("enable single-step");
			xscale_enable_single_step(target, next_pc);

			/* restore banked registers */
			retval = xscale_restore_banked(target);

			/* send resume request (command 0x30 or 0x31)
			 * clean the trace buffer if it is to be enabled (0x62) */
			if (xscale->trace.buffer_enabled)
			{
				xscale_send_u32(target, 0x62);
				xscale_send_u32(target, 0x31);
			}
			else
				xscale_send_u32(target, 0x30);

			/* send CPSR */
			xscale_send_u32(target,
				buf_get_u32(armv4_5->cpsr->value, 0, 32));
			LOG_DEBUG("writing cpsr with value 0x%8.8" PRIx32,
				buf_get_u32(armv4_5->cpsr->value, 0, 32));

			for (i = 7; i >= 0; i--)
			{
				/* send register */
				xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
				LOG_DEBUG("writing r%i with value 0x%8.8" PRIx32 "", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
			}

			/* send PC */
			xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
			LOG_DEBUG("writing PC with value 0x%8.8" PRIx32 "", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

			/* wait for and process debug entry */
			xscale_debug_entry(target);

			LOG_DEBUG("disable single-step");
			xscale_disable_single_step(target);

			LOG_DEBUG("set breakpoint at 0x%8.8" PRIx32 "", breakpoint->address);
			xscale_set_breakpoint(target, breakpoint);
		}
	}

	/* enable any pending breakpoints and watchpoints */
	xscale_enable_breakpoints(target);
	xscale_enable_watchpoints(target);

	/* restore banked registers */
	retval = xscale_restore_banked(target);

	/* send resume request (command 0x30 or 0x31)
	 * clean the trace buffer if it is to be enabled (0x62) */
	if (xscale->trace.buffer_enabled)
	{
		xscale_send_u32(target, 0x62);
		xscale_send_u32(target, 0x31);
	}
	else
		xscale_send_u32(target, 0x30);

	/* send CPSR */
	xscale_send_u32(target, buf_get_u32(armv4_5->cpsr->value, 0, 32));
	LOG_DEBUG("writing cpsr with value 0x%8.8" PRIx32,
			buf_get_u32(armv4_5->cpsr->value, 0, 32));

	for (i = 7; i >= 0; i--)
	{
		/* send register */
		xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
		LOG_DEBUG("writing r%i with value 0x%8.8" PRIx32 "", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
	}

	/* send PC */
	xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	LOG_DEBUG("writing PC with value 0x%8.8" PRIx32 "", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

	target->debug_reason = DBG_REASON_NOTHALTED;

	if (!debug_execution)
	{
		/* registers are now invalid */
		register_cache_invalidate(armv4_5->core_cache);
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
	}

	LOG_DEBUG("target resumed");

	return ERROR_OK;
}

static int xscale_step_inner(struct target *target, int current,
		uint32_t address, int handle_breakpoints)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	uint32_t next_pc;
	int retval;
	int i;

	target->debug_reason = DBG_REASON_SINGLESTEP;

	/* calculate PC of next instruction */
	if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
	{
		uint32_t current_opcode, current_pc;
		current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

		target_read_u32(target, current_pc, &current_opcode);
		LOG_ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8" PRIx32 "", current_opcode);
		return retval;
	}

	LOG_DEBUG("enable single-step");
	if ((retval = xscale_enable_single_step(target, next_pc)) != ERROR_OK)
		return retval;

	/* restore banked registers */
	if ((retval = xscale_restore_banked(target)) != ERROR_OK)
		return retval;

	/* send resume request (command 0x30 or 0x31)
	 * clean the trace buffer if it is to be enabled (0x62) */
	if (xscale->trace.buffer_enabled)
	{
		if ((retval = xscale_send_u32(target, 0x62)) != ERROR_OK)
			return retval;
		if ((retval = xscale_send_u32(target, 0x31)) != ERROR_OK)
			return retval;
	}
	else
		if ((retval = xscale_send_u32(target, 0x30)) != ERROR_OK)
			return retval;

	/* send CPSR */
	retval = xscale_send_u32(target,
			buf_get_u32(armv4_5->cpsr->value, 0, 32));
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("writing cpsr with value 0x%8.8" PRIx32,
			buf_get_u32(armv4_5->cpsr->value, 0, 32));

	for (i = 7; i >= 0; i--)
	{
		/* send register */
		if ((retval = xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32))) != ERROR_OK)
			return retval;
		LOG_DEBUG("writing r%i with value 0x%8.8" PRIx32 "", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
	}

	/* send PC */
	if ((retval = xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))) != ERROR_OK)
		return retval;
	LOG_DEBUG("writing PC with value 0x%8.8" PRIx32, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* registers are now invalid */
	register_cache_invalidate(armv4_5->core_cache);

	/* wait for and process debug entry */
	if ((retval = xscale_debug_entry(target)) != ERROR_OK)
		return retval;

	LOG_DEBUG("disable single-step");
	if ((retval = xscale_disable_single_step(target)) != ERROR_OK)
		return retval;

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static int xscale_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints)
{
	struct arm *armv4_5 = target_to_arm(target);
	struct breakpoint *breakpoint = target->breakpoints;

	uint32_t current_pc;
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);

	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	/* if we're at the reset vector, we have to simulate the step */
	if (current_pc == 0x0)
	{
		if ((retval = arm_simulate_step(target, NULL)) != ERROR_OK)
			return retval;
		current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

		target->debug_reason = DBG_REASON_SINGLESTEP;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);

		return ERROR_OK;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			if ((retval = xscale_unset_breakpoint(target, breakpoint)) != ERROR_OK)
				return retval;
		}

	retval = xscale_step_inner(target, current, address, handle_breakpoints);

	if (breakpoint)
	{
		xscale_set_breakpoint(target, breakpoint);
	}

	LOG_DEBUG("target stepped");

	return ERROR_OK;

}

static int xscale_assert_reset(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);

	LOG_DEBUG("target->state: %s",
		  target_state_name(target));

	/* select DCSR instruction (set endstate to R-T-I to ensure we don't
	 * end up in T-L-R, which would reset JTAG
	 */
	jtag_set_end_state(TAP_IDLE);
	xscale_jtag_set_instr(target->tap,
		XSCALE_SELDCSR << xscale->xscale_variant);

	/* set Hold reset, Halt mode and Trap Reset */
	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
	xscale_write_dcsr(target, 1, 0);

	/* select BYPASS, because having DCSR selected caused problems on the PXA27x */
	xscale_jtag_set_instr(target->tap, ~0);
	jtag_execute_queue();

	/* assert reset */
	jtag_add_reset(0, 1);

	/* sleep 1ms, to be sure we fulfill any requirements */
	jtag_add_sleep(1000);
	jtag_execute_queue();

	target->state = TARGET_RESET;

    if (target->reset_halt)
    {
	int retval;
		if ((retval = target_halt(target)) != ERROR_OK)
			return retval;
    }

	return ERROR_OK;
}

static int xscale_deassert_reset(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct breakpoint *breakpoint = target->breakpoints;

	LOG_DEBUG("-");

	xscale->ibcr_available = 2;
	xscale->ibcr0_used = 0;
	xscale->ibcr1_used = 0;

	xscale->dbr_available = 2;
	xscale->dbr0_used = 0;
	xscale->dbr1_used = 0;

	/* mark all hardware breakpoints as unset */
	while (breakpoint)
	{
		if (breakpoint->type == BKPT_HARD)
		{
			breakpoint->set = 0;
		}
		breakpoint = breakpoint->next;
	}

	register_cache_invalidate(xscale->armv4_5_common.core_cache);

	/* FIXME mark hardware watchpoints got unset too.  Also,
	 * at least some of the XScale registers are invalid...
	 */

	/*
	 * REVISIT:  *assumes* we had a SRST+TRST reset so the mini-icache
	 * contents got invalidated.  Safer to force that, so writing new
	 * contents can't ever fail..
	 */
	{
		uint32_t address;
		unsigned buf_cnt;
		const uint8_t *buffer = xscale_debug_handler;
		int retval;

		/* release SRST */
		jtag_add_reset(0, 0);

		/* wait 300ms; 150 and 100ms were not enough */
		jtag_add_sleep(300*1000);

		jtag_add_runtest(2030, jtag_set_end_state(TAP_IDLE));
		jtag_execute_queue();

		/* set Hold reset, Halt mode and Trap Reset */
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
		xscale_write_dcsr(target, 1, 0);

		/* Load the debug handler into the mini-icache.  Since
		 * it's using halt mode (not monitor mode), it runs in
		 * "Special Debug State" for access to registers, memory,
		 * coprocessors, trace data, etc.
		 */
		address = xscale->handler_address;
		for (unsigned binary_size = sizeof xscale_debug_handler - 1;
				binary_size > 0;
				binary_size -= buf_cnt, buffer += buf_cnt)
		{
			uint32_t cache_line[8];
			unsigned i;

			buf_cnt = binary_size;
			if (buf_cnt > 32)
				buf_cnt = 32;

			for (i = 0; i < buf_cnt; i += 4)
			{
				/* convert LE buffer to host-endian uint32_t */
				cache_line[i / 4] = le_to_h_u32(&buffer[i]);
			}

			for (; i < 32; i += 4)
			{
				cache_line[i / 4] = 0xe1a08008;
			}

			/* only load addresses other than the reset vectors */
			if ((address % 0x400) != 0x0)
			{
				retval = xscale_load_ic(target, address,
						cache_line);
				if (retval != ERROR_OK)
					return retval;
			}

			address += buf_cnt;
		};

		retval = xscale_load_ic(target, 0x0,
					xscale->low_vectors);
		if (retval != ERROR_OK)
			return retval;
		retval = xscale_load_ic(target, 0xffff0000,
					xscale->high_vectors);
		if (retval != ERROR_OK)
			return retval;

		jtag_add_runtest(30, jtag_set_end_state(TAP_IDLE));

		jtag_add_sleep(100000);

		/* set Hold reset, Halt mode and Trap Reset */
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
		xscale_write_dcsr(target, 1, 0);

		/* clear Hold reset to let the target run (should enter debug handler) */
		xscale_write_dcsr(target, 0, 1);
		target->state = TARGET_RUNNING;

		if (!target->reset_halt)
		{
			jtag_add_sleep(10000);

			/* we should have entered debug now */
			xscale_debug_entry(target);
			target->state = TARGET_HALTED;

			/* resume the target */
			xscale_resume(target, 1, 0x0, 1, 0);
		}
	}

	return ERROR_OK;
}

static int xscale_read_core_reg(struct target *target, struct reg *r,
		int num, enum arm_mode mode)
{
	/** \todo add debug handler support for core register reads */
	LOG_ERROR("not implemented");
	return ERROR_OK;
}

static int xscale_write_core_reg(struct target *target, struct reg *r,
		int num, enum arm_mode mode, uint32_t value)
{
	/** \todo add debug handler support for core register writes */
	LOG_ERROR("not implemented");
	return ERROR_OK;
}

static int xscale_full_context(struct target *target)
{
	struct arm *armv4_5 = target_to_arm(target);

	uint32_t *buffer;

	int i, j;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	buffer = malloc(4 * 8);

	/* iterate through processor modes (FIQ, IRQ, SVC, ABT, UND and SYS)
	 * we can't enter User mode on an XScale (unpredictable),
	 * but User shares registers with SYS
	 */
	for (i = 1; i < 7; i++)
	{
		enum arm_mode mode = armv4_5_number_to_mode(i);
		bool valid = true;
		struct reg *r;

		if (mode == ARM_MODE_USR)
			continue;

		/* check if there are invalid registers in the current mode
		 */
		for (j = 0; valid && j <= 16; j++)
		{
			if (!ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, j).valid)
				valid = false;
		}
		if (valid)
			continue;

		/* request banked registers */
		xscale_send_u32(target, 0x0);

		/* send CPSR for desired bank mode */
		xscale_send_u32(target, mode | 0xc0 /* I/F bits */);

		/* get banked registers:  r8 to r14; and SPSR
		 * except in USR/SYS mode
		 */
		if (mode != ARM_MODE_SYS) {
			/* SPSR */
			r = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, 16);

			xscale_receive(target, buffer, 8);

			buf_set_u32(r->value, 0, 32, buffer[7]);
			r->dirty = false;
			r->valid = true;
		} else {
			xscale_receive(target, buffer, 7);
		}

		/* move data from buffer to register cache */
		for (j = 8; j <= 14; j++)
		{
			r = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, j);

			buf_set_u32(r->value, 0, 32, buffer[j - 8]);
			r->dirty = false;
			r->valid = true;
		}
	}

	free(buffer);

	return ERROR_OK;
}

static int xscale_restore_banked(struct target *target)
{
	struct arm *armv4_5 = target_to_arm(target);

	int i, j;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* iterate through processor modes (FIQ, IRQ, SVC, ABT, UND and SYS)
	 * and check if any banked registers need to be written.  Ignore
	 * USR mode (number 0) in favor of SYS; we can't enter User mode on
	 * an XScale (unpredictable), but they share all registers.
	 */
	for (i = 1; i < 7; i++)
	{
		enum arm_mode mode = armv4_5_number_to_mode(i);
		struct reg *r;

		if (mode == ARM_MODE_USR)
			continue;

		/* check if there are dirty registers in this mode */
		for (j = 8; j <= 14; j++)
		{
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, j).dirty)
				goto dirty;
		}

		/* if not USR/SYS, check if the SPSR needs to be written */
		if (mode != ARM_MODE_SYS)
		{
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, 16).dirty)
				goto dirty;
		}

		/* there's nothing to flush for this mode */
		continue;

dirty:
		/* command 0x1:  "send banked registers" */
		xscale_send_u32(target, 0x1);

		/* send CPSR for desired mode */
		xscale_send_u32(target, mode | 0xc0 /* I/F bits */);

		/* send r8 to r14/lr ... only FIQ needs more than r13..r14,
		 * but this protocol doesn't understand that nuance.
		 */
		for (j = 8; j <= 14; j++) {
			r = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, j);
			xscale_send_u32(target, buf_get_u32(r->value, 0, 32));
			r->dirty = false;
		}

		/* send spsr if not in USR/SYS mode */
		if (mode != ARM_MODE_SYS) {
			r = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache,
					mode, 16);
			xscale_send_u32(target, buf_get_u32(r->value, 0, 32));
			r->dirty = false;
		}
	}

	return ERROR_OK;
}

static int xscale_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t *buf32;
	uint32_t i;
	int retval;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* send memory read request (command 0x1n, n: access size) */
	if ((retval = xscale_send_u32(target, 0x10 | size)) != ERROR_OK)
		return retval;

	/* send base address for read request */
	if ((retval = xscale_send_u32(target, address)) != ERROR_OK)
		return retval;

	/* send number of requested data words */
	if ((retval = xscale_send_u32(target, count)) != ERROR_OK)
		return retval;

	/* receive data from target (count times 32-bit words in host endianness) */
	buf32 = malloc(4 * count);
	if ((retval = xscale_receive(target, buf32, count)) != ERROR_OK)
		return retval;

	/* extract data from host-endian buffer into byte stream */
	for (i = 0; i < count; i++)
	{
		switch (size)
		{
			case 4:
				target_buffer_set_u32(target, buffer, buf32[i]);
				buffer += 4;
				break;
			case 2:
				target_buffer_set_u16(target, buffer, buf32[i] & 0xffff);
				buffer += 2;
				break;
			case 1:
				*buffer++ = buf32[i] & 0xff;
				break;
			default:
				LOG_ERROR("invalid read size");
				return ERROR_INVALID_ARGUMENTS;
		}
	}

	free(buf32);

	/* examine DCSR, to see if Sticky Abort (SA) got set */
	if ((retval = xscale_read_dcsr(target)) != ERROR_OK)
		return retval;
	if (buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 5, 1) == 1)
	{
		/* clear SA bit */
		if ((retval = xscale_send_u32(target, 0x60)) != ERROR_OK)
			return retval;

		return ERROR_TARGET_DATA_ABORT;
	}

	return ERROR_OK;
}

static int xscale_read_phys_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct xscale_common *xscale = target_to_xscale(target);

	/* with MMU inactive, there are only physical addresses */
	if (!xscale->armv4_5_mmu.mmu_enabled)
		return xscale_read_memory(target, address, size, count, buffer);

	/** \todo: provide a non-stub implementation of this routine. */
	LOG_ERROR("%s: %s is not implemented.  Disable MMU?",
			target_name(target), __func__);
	return ERROR_FAIL;
}

static int xscale_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* send memory write request (command 0x2n, n: access size) */
	if ((retval = xscale_send_u32(target, 0x20 | size)) != ERROR_OK)
		return retval;

	/* send base address for read request */
	if ((retval = xscale_send_u32(target, address)) != ERROR_OK)
		return retval;

	/* send number of requested data words to be written*/
	if ((retval = xscale_send_u32(target, count)) != ERROR_OK)
		return retval;

	/* extract data from host-endian buffer into byte stream */
#if 0
	for (i = 0; i < count; i++)
	{
		switch (size)
		{
			case 4:
				value = target_buffer_get_u32(target, buffer);
				xscale_send_u32(target, value);
				buffer += 4;
				break;
			case 2:
				value = target_buffer_get_u16(target, buffer);
				xscale_send_u32(target, value);
				buffer += 2;
				break;
			case 1:
				value = *buffer;
				xscale_send_u32(target, value);
				buffer += 1;
				break;
			default:
				LOG_ERROR("should never get here");
				exit(-1);
		}
	}
#endif
	if ((retval = xscale_send(target, buffer, count, size)) != ERROR_OK)
		return retval;

	/* examine DCSR, to see if Sticky Abort (SA) got set */
	if ((retval = xscale_read_dcsr(target)) != ERROR_OK)
		return retval;
	if (buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 5, 1) == 1)
	{
		/* clear SA bit */
		if ((retval = xscale_send_u32(target, 0x60)) != ERROR_OK)
			return retval;

		return ERROR_TARGET_DATA_ABORT;
	}

	return ERROR_OK;
}

static int xscale_write_phys_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct xscale_common *xscale = target_to_xscale(target);

	/* with MMU inactive, there are only physical addresses */
	if (!xscale->armv4_5_mmu.mmu_enabled)
		return xscale_read_memory(target, address, size, count, buffer);

	/** \todo: provide a non-stub implementation of this routine. */
	LOG_ERROR("%s: %s is not implemented.  Disable MMU?",
			target_name(target), __func__);
	return ERROR_FAIL;
}

static int xscale_bulk_write_memory(struct target *target, uint32_t address,
		uint32_t count, uint8_t *buffer)
{
	return xscale_write_memory(target, address, 4, count, buffer);
}

static uint32_t xscale_get_ttb(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t ttb;

	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_TTB]);
	ttb = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_TTB].value, 0, 32);

	return ttb;
}

static void xscale_disable_mmu_caches(struct target *target, int mmu,
		int d_u_cache, int i_cache)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t cp15_control;

	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	cp15_control = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);

	if (mmu)
		cp15_control &= ~0x1U;

	if (d_u_cache)
	{
		/* clean DCache */
		xscale_send_u32(target, 0x50);
		xscale_send_u32(target, xscale->cache_clean_address);

		/* invalidate DCache */
		xscale_send_u32(target, 0x51);

		cp15_control &= ~0x4U;
	}

	if (i_cache)
	{
		/* invalidate ICache */
		xscale_send_u32(target, 0x52);
		cp15_control &= ~0x1000U;
	}

	/* write new cp15 control register */
	xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_CTRL], cp15_control);

	/* execute cpwait to ensure outstanding operations complete */
	xscale_send_u32(target, 0x53);
}

static void xscale_enable_mmu_caches(struct target *target, int mmu,
		int d_u_cache, int i_cache)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t cp15_control;

	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	cp15_control = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);

	if (mmu)
		cp15_control |= 0x1U;

	if (d_u_cache)
		cp15_control |= 0x4U;

	if (i_cache)
		cp15_control |= 0x1000U;

	/* write new cp15 control register */
	xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_CTRL], cp15_control);

	/* execute cpwait to ensure outstanding operations complete */
	xscale_send_u32(target, 0x53);
}

static int xscale_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	int retval;
	struct xscale_common *xscale = target_to_xscale(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		uint32_t value = breakpoint->address | 1;
		if (!xscale->ibcr0_used)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR0], value);
			xscale->ibcr0_used = 1;
			breakpoint->set = 1;	/* breakpoint set on first breakpoint register */
		}
		else if (!xscale->ibcr1_used)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR1], value);
			xscale->ibcr1_used = 1;
			breakpoint->set = 2;	/* breakpoint set on second breakpoint register */
		}
		else
		{
			LOG_ERROR("BUG: no hardware comparator available");
			return ERROR_OK;
		}
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		if (breakpoint->length == 4)
		{
			/* keep the original instruction in target endianness */
			if ((retval = target_read_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			/* write the original instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			if ((retval = target_write_u32(target, breakpoint->address, xscale->arm_bkpt)) != ERROR_OK)
			{
				return retval;
			}
		}
		else
		{
			/* keep the original instruction in target endianness */
			if ((retval = target_read_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			/* write the original instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			if ((retval = target_write_u32(target, breakpoint->address, xscale->thumb_bkpt)) != ERROR_OK)
			{
				return retval;
			}
		}
		breakpoint->set = 1;
	}

	return ERROR_OK;
}

static int xscale_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);

	if ((breakpoint->type == BKPT_HARD) && (xscale->ibcr_available < 1))
	{
		LOG_INFO("no breakpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->length != 2) && (breakpoint->length != 4))
	{
		LOG_INFO("only breakpoints of two (Thumb) or four (ARM) bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		xscale->ibcr_available--;
	}

	return ERROR_OK;
}

static int xscale_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	int retval;
	struct xscale_common *xscale = target_to_xscale(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		if (breakpoint->set == 1)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR0], 0x0);
			xscale->ibcr0_used = 0;
		}
		else if (breakpoint->set == 2)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR1], 0x0);
			xscale->ibcr1_used = 0;
		}
		breakpoint->set = 0;
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			if ((retval = target_write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
		}
		else
		{
			if ((retval = target_write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
		}
		breakpoint->set = 0;
	}

	return ERROR_OK;
}

static int xscale_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
	{
		xscale_unset_breakpoint(target, breakpoint);
	}

	if (breakpoint->type == BKPT_HARD)
		xscale->ibcr_available++;

	return ERROR_OK;
}

static int xscale_set_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);
	uint8_t enable = 0;
	struct reg *dbcon = &xscale->reg_cache->reg_list[XSCALE_DBCON];
	uint32_t dbcon_value = buf_get_u32(dbcon->value, 0, 32);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	xscale_get_reg(dbcon);

	switch (watchpoint->rw)
	{
		case WPT_READ:
			enable = 0x3;
			break;
		case WPT_ACCESS:
			enable = 0x2;
			break;
		case WPT_WRITE:
			enable = 0x1;
			break;
		default:
			LOG_ERROR("BUG: watchpoint->rw neither read, write nor access");
	}

	if (!xscale->dbr0_used)
	{
		xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_DBR0], watchpoint->address);
		dbcon_value |= enable;
		xscale_set_reg_u32(dbcon, dbcon_value);
		watchpoint->set = 1;
		xscale->dbr0_used = 1;
	}
	else if (!xscale->dbr1_used)
	{
		xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_DBR1], watchpoint->address);
		dbcon_value |= enable << 2;
		xscale_set_reg_u32(dbcon, dbcon_value);
		watchpoint->set = 2;
		xscale->dbr1_used = 1;
	}
	else
	{
		LOG_ERROR("BUG: no hardware comparator available");
		return ERROR_OK;
	}

	return ERROR_OK;
}

static int xscale_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);

	if (xscale->dbr_available < 1)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((watchpoint->length != 1) && (watchpoint->length != 2) && (watchpoint->length != 4))
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	xscale->dbr_available--;

	return ERROR_OK;
}

static int xscale_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct reg *dbcon = &xscale->reg_cache->reg_list[XSCALE_DBCON];
	uint32_t dbcon_value = buf_get_u32(dbcon->value, 0, 32);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!watchpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (watchpoint->set == 1)
	{
		dbcon_value &= ~0x3;
		xscale_set_reg_u32(dbcon, dbcon_value);
		xscale->dbr0_used = 0;
	}
	else if (watchpoint->set == 2)
	{
		dbcon_value &= ~0xc;
		xscale_set_reg_u32(dbcon, dbcon_value);
		xscale->dbr1_used = 0;
	}
	watchpoint->set = 0;

	return ERROR_OK;
}

static int xscale_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct xscale_common *xscale = target_to_xscale(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
	{
		xscale_unset_watchpoint(target, watchpoint);
	}

	xscale->dbr_available++;

	return ERROR_OK;
}

static int xscale_get_reg(struct reg *reg)
{
	struct xscale_reg *arch_info = reg->arch_info;
	struct target *target = arch_info->target;
	struct xscale_common *xscale = target_to_xscale(target);

	/* DCSR, TX and RX are accessible via JTAG */
	if (strcmp(reg->name, "XSCALE_DCSR") == 0)
	{
		return xscale_read_dcsr(arch_info->target);
	}
	else if (strcmp(reg->name, "XSCALE_TX") == 0)
	{
		/* 1 = consume register content */
		return xscale_read_tx(arch_info->target, 1);
	}
	else if (strcmp(reg->name, "XSCALE_RX") == 0)
	{
		/* can't read from RX register (host -> debug handler) */
		return ERROR_OK;
	}
	else if (strcmp(reg->name, "XSCALE_TXRXCTRL") == 0)
	{
		/* can't (explicitly) read from TXRXCTRL register */
		return ERROR_OK;
	}
	else /* Other DBG registers have to be transfered by the debug handler */
	{
		/* send CP read request (command 0x40) */
		xscale_send_u32(target, 0x40);

		/* send CP register number */
		xscale_send_u32(target, arch_info->dbg_handler_number);

		/* read register value */
		xscale_read_tx(target, 1);
		buf_cpy(xscale->reg_cache->reg_list[XSCALE_TX].value, reg->value, 32);

		reg->dirty = 0;
		reg->valid = 1;
	}

	return ERROR_OK;
}

static int xscale_set_reg(struct reg *reg, uint8_t* buf)
{
	struct xscale_reg *arch_info = reg->arch_info;
	struct target *target = arch_info->target;
	struct xscale_common *xscale = target_to_xscale(target);
	uint32_t value = buf_get_u32(buf, 0, 32);

	/* DCSR, TX and RX are accessible via JTAG */
	if (strcmp(reg->name, "XSCALE_DCSR") == 0)
	{
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 0, 32, value);
		return xscale_write_dcsr(arch_info->target, -1, -1);
	}
	else if (strcmp(reg->name, "XSCALE_RX") == 0)
	{
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_RX].value, 0, 32, value);
		return xscale_write_rx(arch_info->target);
	}
	else if (strcmp(reg->name, "XSCALE_TX") == 0)
	{
		/* can't write to TX register (debug-handler -> host) */
		return ERROR_OK;
	}
	else if (strcmp(reg->name, "XSCALE_TXRXCTRL") == 0)
	{
		/* can't (explicitly) write to TXRXCTRL register */
		return ERROR_OK;
	}
	else /* Other DBG registers have to be transfered by the debug handler */
	{
		/* send CP write request (command 0x41) */
		xscale_send_u32(target, 0x41);

		/* send CP register number */
		xscale_send_u32(target, arch_info->dbg_handler_number);

		/* send CP register value */
		xscale_send_u32(target, value);
		buf_set_u32(reg->value, 0, 32, value);
	}

	return ERROR_OK;
}

static int xscale_write_dcsr_sw(struct target *target, uint32_t value)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct reg *dcsr = &xscale->reg_cache->reg_list[XSCALE_DCSR];
	struct xscale_reg *dcsr_arch_info = dcsr->arch_info;

	/* send CP write request (command 0x41) */
	xscale_send_u32(target, 0x41);

	/* send CP register number */
	xscale_send_u32(target, dcsr_arch_info->dbg_handler_number);

	/* send CP register value */
	xscale_send_u32(target, value);
	buf_set_u32(dcsr->value, 0, 32, value);

	return ERROR_OK;
}

static int xscale_read_trace(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	struct xscale_trace_data **trace_data_p;

	/* 258 words from debug handler
	 * 256 trace buffer entries
	 * 2 checkpoint addresses
	 */
	uint32_t trace_buffer[258];
	int is_address[256];
	int i, j;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target must be stopped to read trace data");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* send read trace buffer command (command 0x61) */
	xscale_send_u32(target, 0x61);

	/* receive trace buffer content */
	xscale_receive(target, trace_buffer, 258);

	/* parse buffer backwards to identify address entries */
	for (i = 255; i >= 0; i--)
	{
		is_address[i] = 0;
		if (((trace_buffer[i] & 0xf0) == 0x90) ||
			((trace_buffer[i] & 0xf0) == 0xd0))
		{
			if (i >= 3)
				is_address[--i] = 1;
			if (i >= 2)
				is_address[--i] = 1;
			if (i >= 1)
				is_address[--i] = 1;
			if (i >= 0)
				is_address[--i] = 1;
		}
	}


	/* search first non-zero entry */
	for (j = 0; (j < 256) && (trace_buffer[j] == 0) && (!is_address[j]); j++)
		;

	if (j == 256)
	{
		LOG_DEBUG("no trace data collected");
		return ERROR_XSCALE_NO_TRACE_DATA;
	}

	for (trace_data_p = &xscale->trace.data; *trace_data_p; trace_data_p = &(*trace_data_p)->next)
		;

	*trace_data_p = malloc(sizeof(struct xscale_trace_data));
	(*trace_data_p)->next = NULL;
	(*trace_data_p)->chkpt0 = trace_buffer[256];
	(*trace_data_p)->chkpt1 = trace_buffer[257];
	(*trace_data_p)->last_instruction = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	(*trace_data_p)->entries = malloc(sizeof(struct xscale_trace_entry) * (256 - j));
	(*trace_data_p)->depth = 256 - j;

	for (i = j; i < 256; i++)
	{
		(*trace_data_p)->entries[i - j].data = trace_buffer[i];
		if (is_address[i])
			(*trace_data_p)->entries[i - j].type = XSCALE_TRACE_ADDRESS;
		else
			(*trace_data_p)->entries[i - j].type = XSCALE_TRACE_MESSAGE;
	}

	return ERROR_OK;
}

static int xscale_read_instruction(struct target *target,
		struct arm_instruction *instruction)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int i;
	int section = -1;
	size_t size_read;
	uint32_t opcode;
	int retval;

	if (!xscale->trace.image)
		return ERROR_TRACE_IMAGE_UNAVAILABLE;

	/* search for the section the current instruction belongs to */
	for (i = 0; i < xscale->trace.image->num_sections; i++)
	{
		if ((xscale->trace.image->sections[i].base_address <= xscale->trace.current_pc) &&
			(xscale->trace.image->sections[i].base_address + xscale->trace.image->sections[i].size > xscale->trace.current_pc))
		{
			section = i;
			break;
		}
	}

	if (section == -1)
	{
		/* current instruction couldn't be found in the image */
		return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
	}

	if (xscale->trace.core_state == ARM_STATE_ARM)
	{
		uint8_t buf[4];
		if ((retval = image_read_section(xscale->trace.image, section,
			xscale->trace.current_pc - xscale->trace.image->sections[section].base_address,
			4, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u32(target, buf);
		arm_evaluate_opcode(opcode, xscale->trace.current_pc, instruction);
	}
	else if (xscale->trace.core_state == ARM_STATE_THUMB)
	{
		uint8_t buf[2];
		if ((retval = image_read_section(xscale->trace.image, section,
			xscale->trace.current_pc - xscale->trace.image->sections[section].base_address,
			2, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u16(target, buf);
		thumb_evaluate_opcode(opcode, xscale->trace.current_pc, instruction);
	}
	else
	{
		LOG_ERROR("BUG: unknown core state encountered");
		exit(-1);
	}

	return ERROR_OK;
}

static int xscale_branch_address(struct xscale_trace_data *trace_data,
		int i, uint32_t *target)
{
	/* if there are less than four entries prior to the indirect branch message
	 * we can't extract the address */
	if (i < 4)
	{
		return -1;
	}

	*target = (trace_data->entries[i-1].data) | (trace_data->entries[i-2].data << 8) |
				(trace_data->entries[i-3].data << 16) | (trace_data->entries[i-4].data << 24);

	return 0;
}

static int xscale_analyze_trace(struct target *target, struct command_context *cmd_ctx)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int next_pc_ok = 0;
	uint32_t next_pc = 0x0;
	struct xscale_trace_data *trace_data = xscale->trace.data;
	int retval;

	while (trace_data)
	{
		int i, chkpt;
		int rollover;
		int branch;
		int exception;
		xscale->trace.core_state = ARM_STATE_ARM;

		chkpt = 0;
		rollover = 0;

		for (i = 0; i < trace_data->depth; i++)
		{
			next_pc_ok = 0;
			branch = 0;
			exception = 0;

			if (trace_data->entries[i].type == XSCALE_TRACE_ADDRESS)
				continue;

			switch ((trace_data->entries[i].data & 0xf0) >> 4)
			{
				case 0:		/* Exceptions */
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
					exception = (trace_data->entries[i].data & 0x70) >> 4;
					next_pc_ok = 1;
					next_pc = (trace_data->entries[i].data & 0xf0) >> 2;
					command_print(cmd_ctx, "--- exception %i ---", (trace_data->entries[i].data & 0xf0) >> 4);
					break;
				case 8:		/* Direct Branch */
					branch = 1;
					break;
				case 9:		/* Indirect Branch */
					branch = 1;
					if (xscale_branch_address(trace_data, i, &next_pc) == 0)
					{
						next_pc_ok = 1;
					}
					break;
				case 13:	/* Checkpointed Indirect Branch */
					if (xscale_branch_address(trace_data, i, &next_pc) == 0)
					{
						next_pc_ok = 1;
						if (((chkpt == 0) && (next_pc != trace_data->chkpt0))
							|| ((chkpt == 1) && (next_pc != trace_data->chkpt1)))
							LOG_WARNING("checkpointed indirect branch target address doesn't match checkpoint");
					}
					/* explicit fall-through */
				case 12:	/* Checkpointed Direct Branch */
					branch = 1;
					if (chkpt == 0)
					{
						next_pc_ok = 1;
						next_pc = trace_data->chkpt0;
						chkpt++;
					}
					else if (chkpt == 1)
					{
						next_pc_ok = 1;
						next_pc = trace_data->chkpt0;
						chkpt++;
					}
					else
					{
						LOG_WARNING("more than two checkpointed branches encountered");
					}
					break;
				case 15:	/* Roll-over */
					rollover++;
					continue;
				default:	/* Reserved */
					command_print(cmd_ctx, "--- reserved trace message ---");
					LOG_ERROR("BUG: trace message %i is reserved", (trace_data->entries[i].data & 0xf0) >> 4);
					return ERROR_OK;
			}

			if (xscale->trace.pc_ok)
			{
				int executed = (trace_data->entries[i].data & 0xf) + rollover * 16;
				struct arm_instruction instruction;

				if ((exception == 6) || (exception == 7))
				{
					/* IRQ or FIQ exception, no instruction executed */
					executed -= 1;
				}

				while (executed-- >= 0)
				{
					if ((retval = xscale_read_instruction(target, &instruction)) != ERROR_OK)
					{
						/* can't continue tracing with no image available */
						if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
						{
							return retval;
						}
						else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
						{
							/* TODO: handle incomplete images */
						}
					}

					/* a precise abort on a load to the PC is included in the incremental
					 * word count, other instructions causing data aborts are not included
					 */
					if ((executed == 0) && (exception == 4)
						&& ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDM)))
					{
						if ((instruction.type == ARM_LDM)
							&& ((instruction.info.load_store_multiple.register_list & 0x8000) == 0))
						{
							executed--;
						}
						else if (((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDRSH))
							&& (instruction.info.load_store.Rd != 15))
						{
							executed--;
						}
					}

					/* only the last instruction executed
					 * (the one that caused the control flow change)
					 * could be a taken branch
					 */
					if (((executed == -1) && (branch == 1)) &&
						(((instruction.type == ARM_B) ||
							(instruction.type == ARM_BL) ||
							(instruction.type == ARM_BLX)) &&
							(instruction.info.b_bl_bx_blx.target_address != 0xffffffff)))
					{
						xscale->trace.current_pc = instruction.info.b_bl_bx_blx.target_address;
					}
					else
					{
						xscale->trace.current_pc += (xscale->trace.core_state == ARM_STATE_ARM) ? 4 : 2;
					}
					command_print(cmd_ctx, "%s", instruction.text);
				}

				rollover = 0;
			}

			if (next_pc_ok)
			{
				xscale->trace.current_pc = next_pc;
				xscale->trace.pc_ok = 1;
			}
		}

		for (; xscale->trace.current_pc < trace_data->last_instruction; xscale->trace.current_pc += (xscale->trace.core_state == ARM_STATE_ARM) ? 4 : 2)
		{
			struct arm_instruction instruction;
			if ((retval = xscale_read_instruction(target, &instruction)) != ERROR_OK)
			{
				/* can't continue tracing with no image available */
				if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
				{
					return retval;
				}
				else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
				{
					/* TODO: handle incomplete images */
				}
			}
			command_print(cmd_ctx, "%s", instruction.text);
		}

		trace_data = trace_data->next;
	}

	return ERROR_OK;
}

static const struct reg_arch_type xscale_reg_type = {
	.get = xscale_get_reg,
	.set = xscale_set_reg,
};

static void xscale_build_reg_cache(struct target *target)
{
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct xscale_reg *arch_info = malloc(sizeof(xscale_reg_arch_info));
	int i;
	int num_regs = ARRAY_SIZE(xscale_reg_arch_info);

	(*cache_p) = arm_build_reg_cache(target, armv4_5);

	(*cache_p)->next = malloc(sizeof(struct reg_cache));
	cache_p = &(*cache_p)->next;

	/* fill in values for the xscale reg cache */
	(*cache_p)->name = "XScale registers";
	(*cache_p)->next = NULL;
	(*cache_p)->reg_list = malloc(num_regs * sizeof(struct reg));
	(*cache_p)->num_regs = num_regs;

	for (i = 0; i < num_regs; i++)
	{
		(*cache_p)->reg_list[i].name = xscale_reg_list[i];
		(*cache_p)->reg_list[i].value = calloc(4, 1);
		(*cache_p)->reg_list[i].dirty = 0;
		(*cache_p)->reg_list[i].valid = 0;
		(*cache_p)->reg_list[i].size = 32;
		(*cache_p)->reg_list[i].arch_info = &arch_info[i];
		(*cache_p)->reg_list[i].type = &xscale_reg_type;
		arch_info[i] = xscale_reg_arch_info[i];
		arch_info[i].target = target;
	}

	xscale->reg_cache = (*cache_p);
}

static int xscale_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	xscale_build_reg_cache(target);
	return ERROR_OK;
}

static int xscale_init_arch_info(struct target *target,
		struct xscale_common *xscale, struct jtag_tap *tap, const char *variant)
{
	struct arm *armv4_5;
	uint32_t high_reset_branch, low_reset_branch;
	int i;

	armv4_5 = &xscale->armv4_5_common;

	/* store architecture specfic data */
	xscale->common_magic = XSCALE_COMMON_MAGIC;

	/* we don't really *need* a variant param ... */
	if (variant) {
		int ir_length = 0;

		if (strcmp(variant, "pxa250") == 0
				|| strcmp(variant, "pxa255") == 0
				|| strcmp(variant, "pxa26x") == 0)
			ir_length = 5;
		else if (strcmp(variant, "pxa27x") == 0
				|| strcmp(variant, "ixp42x") == 0
				|| strcmp(variant, "ixp45x") == 0
				|| strcmp(variant, "ixp46x") == 0)
			ir_length = 7;
		else if (strcmp(variant, "pxa3xx") == 0)
			ir_length = 11;
		else
			LOG_WARNING("%s: unrecognized variant %s",
				tap->dotted_name, variant);

		if (ir_length && ir_length != tap->ir_length) {
			LOG_WARNING("%s: IR length for %s is %d; fixing",
				tap->dotted_name, variant, ir_length);
			tap->ir_length = ir_length;
		}
	}

	/* PXA3xx shifts the JTAG instructions */
	if (tap->ir_length == 11)
		xscale->xscale_variant = XSCALE_PXA3XX;
	else
		xscale->xscale_variant = XSCALE_IXP4XX_PXA2XX;

	/* the debug handler isn't installed (and thus not running) at this time */
	xscale->handler_address = 0xfe000800;

	/* clear the vectors we keep locally for reference */
	memset(xscale->low_vectors, 0, sizeof(xscale->low_vectors));
	memset(xscale->high_vectors, 0, sizeof(xscale->high_vectors));

	/* no user-specified vectors have been configured yet */
	xscale->static_low_vectors_set = 0x0;
	xscale->static_high_vectors_set = 0x0;

	/* calculate branches to debug handler */
	low_reset_branch = (xscale->handler_address + 0x20 - 0x0 - 0x8) >> 2;
	high_reset_branch = (xscale->handler_address + 0x20 - 0xffff0000 - 0x8) >> 2;

	xscale->low_vectors[0] = ARMV4_5_B((low_reset_branch & 0xffffff), 0);
	xscale->high_vectors[0] = ARMV4_5_B((high_reset_branch & 0xffffff), 0);

	for (i = 1; i <= 7; i++)
	{
		xscale->low_vectors[i] = ARMV4_5_B(0xfffffe, 0);
		xscale->high_vectors[i] = ARMV4_5_B(0xfffffe, 0);
	}

	/* 64kB aligned region used for DCache cleaning */
	xscale->cache_clean_address = 0xfffe0000;

	xscale->hold_rst = 0;
	xscale->external_debug_break = 0;

	xscale->ibcr_available = 2;
	xscale->ibcr0_used = 0;
	xscale->ibcr1_used = 0;

	xscale->dbr_available = 2;
	xscale->dbr0_used = 0;
	xscale->dbr1_used = 0;

	LOG_INFO("%s: hardware has 2 breakpoints and 2 watchpoints",
			target_name(target));

	xscale->arm_bkpt = ARMV5_BKPT(0x0);
	xscale->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;

	xscale->vector_catch = 0x1;

	xscale->trace.capture_status = TRACE_IDLE;
	xscale->trace.data = NULL;
	xscale->trace.image = NULL;
	xscale->trace.buffer_enabled = 0;
	xscale->trace.buffer_fill = 0;

	/* prepare ARMv4/5 specific information */
	armv4_5->arch_info = xscale;
	armv4_5->read_core_reg = xscale_read_core_reg;
	armv4_5->write_core_reg = xscale_write_core_reg;
	armv4_5->full_context = xscale_full_context;

	arm_init_arch_info(target, armv4_5);

	xscale->armv4_5_mmu.armv4_5_cache.ctype = -1;
	xscale->armv4_5_mmu.get_ttb = xscale_get_ttb;
	xscale->armv4_5_mmu.read_memory = xscale_read_memory;
	xscale->armv4_5_mmu.write_memory = xscale_write_memory;
	xscale->armv4_5_mmu.disable_mmu_caches = xscale_disable_mmu_caches;
	xscale->armv4_5_mmu.enable_mmu_caches = xscale_enable_mmu_caches;
	xscale->armv4_5_mmu.has_tiny_pages = 1;
	xscale->armv4_5_mmu.mmu_enabled = 0;

	return ERROR_OK;
}

static int xscale_target_create(struct target *target, Jim_Interp *interp)
{
	struct xscale_common *xscale;

	if (sizeof xscale_debug_handler - 1 > 0x800) {
		LOG_ERROR("debug_handler.bin: larger than 2kb");
		return ERROR_FAIL;
	}

	xscale = calloc(1, sizeof(*xscale));
	if (!xscale)
		return ERROR_FAIL;

	return xscale_init_arch_info(target, xscale, target->tap,
			target->variant);
}

COMMAND_HANDLER(xscale_handle_debug_handler_command)
{
	struct target *target = NULL;
	struct xscale_common *xscale;
	int retval;
	uint32_t handler_address;

	if (CMD_ARGC < 2)
	{
		LOG_ERROR("'xscale debug_handler <target#> <address>' command takes two required operands");
		return ERROR_OK;
	}

	if ((target = get_target(CMD_ARGV[0])) == NULL)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	xscale = target_to_xscale(target);
	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], handler_address);

	if (((handler_address >= 0x800) && (handler_address <= 0x1fef800)) ||
		((handler_address >= 0xfe000800) && (handler_address <= 0xfffff800)))
	{
		xscale->handler_address = handler_address;
	}
	else
	{
		LOG_ERROR("xscale debug_handler <address> must be between 0x800 and 0x1fef800 or between 0xfe000800 and 0xfffff800");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_cache_clean_address_command)
{
	struct target *target = NULL;
	struct xscale_common *xscale;
	int retval;
	uint32_t cache_clean_address;

	if (CMD_ARGC < 2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target = get_target(CMD_ARGV[0]);
	if (target == NULL)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[0]);
		return ERROR_FAIL;
	}
	xscale = target_to_xscale(target);
	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cache_clean_address);

	if (cache_clean_address & 0xffff)
	{
		LOG_ERROR("xscale cache_clean_address <address> must be 64kb aligned");
	}
	else
	{
		xscale->cache_clean_address = cache_clean_address;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	return armv4_5_handle_cache_info_command(CMD_CTX, &xscale->armv4_5_mmu.armv4_5_cache);
}

static int xscale_virt2phys(struct target *target,
		uint32_t virtual, uint32_t *physical)
{
	struct xscale_common *xscale = target_to_xscale(target);
	int type;
	uint32_t cb;
	int domain;
	uint32_t ap;

	if (xscale->common_magic != XSCALE_COMMON_MAGIC) {
		LOG_ERROR(xscale_not);
		return ERROR_TARGET_INVALID;
	}

	uint32_t ret = armv4_5_mmu_translate_va(target, &xscale->armv4_5_mmu, virtual, &type, &cb, &domain, &ap);
	if (type == -1)
	{
		return ret;
	}
	*physical = ret;
	return ERROR_OK;
}

static int xscale_mmu(struct target *target, int *enabled)
{
	struct xscale_common *xscale = target_to_xscale(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_INVALID;
	}
	*enabled = xscale->armv4_5_mmu.mmu_enabled;
	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_mmu_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	if (CMD_ARGC >= 1)
	{
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		if (enable)
			xscale_enable_mmu_caches(target, 1, 0, 0);
		else
			xscale_disable_mmu_caches(target, 1, 0, 0);
		xscale->armv4_5_mmu.mmu_enabled = enable;
	}

	command_print(CMD_CTX, "mmu %s", (xscale->armv4_5_mmu.mmu_enabled) ? "enabled" : "disabled");

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_idcache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);

	int retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	bool icache;
	COMMAND_PARSE_BOOL(CMD_NAME, icache, "icache", "dcache");

	if (CMD_ARGC >= 1)
	{
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		if (enable)
			xscale_enable_mmu_caches(target, 1, 0, 0);
		else
			xscale_disable_mmu_caches(target, 1, 0, 0);
		if (icache)
			xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled = enable;
		else
			xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = enable;
	}

	bool enabled = icache ?
					xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled :
					xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled;
	const char *msg = enabled ? "enabled" : "disabled";
	command_print(CMD_CTX, "%s %s", CMD_NAME, msg);

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_vector_catch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "usage: xscale vector_catch [mask]");
	}
	else
	{
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], xscale->vector_catch);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 8, xscale->vector_catch);
		xscale_write_dcsr(target, -1, -1);
	}

	command_print(CMD_CTX, "vector catch mask: 0x%2.2x", xscale->vector_catch);

	return ERROR_OK;
}


COMMAND_HANDLER(xscale_handle_vector_table_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int err = 0;
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC == 0) /* print current settings */
	{
		int idx;

		command_print(CMD_CTX, "active user-set static vectors:");
		for (idx = 1; idx < 8; idx++)
			if (xscale->static_low_vectors_set & (1 << idx))
				command_print(CMD_CTX, "low  %d: 0x%" PRIx32, idx, xscale->static_low_vectors[idx]);
		for (idx = 1; idx < 8; idx++)
			if (xscale->static_high_vectors_set & (1 << idx))
				command_print(CMD_CTX, "high %d: 0x%" PRIx32, idx, xscale->static_high_vectors[idx]);
		return ERROR_OK;
	}

	if (CMD_ARGC != 3)
		err = 1;
	else
	{
		int idx;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], idx);
		uint32_t vec;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], vec);

		if (idx < 1 || idx >= 8)
			err = 1;

		if (!err && strcmp(CMD_ARGV[0], "low") == 0)
		{
			xscale->static_low_vectors_set |= (1<<idx);
			xscale->static_low_vectors[idx] = vec;
		}
		else if (!err && (strcmp(CMD_ARGV[0], "high") == 0))
		{
			xscale->static_high_vectors_set |= (1<<idx);
			xscale->static_high_vectors[idx] = vec;
		}
		else
			err = 1;
	}

	if (err)
		command_print(CMD_CTX, "usage: xscale vector_table <high|low> <index> <code>");

	return ERROR_OK;
}


COMMAND_HANDLER(xscale_handle_trace_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	struct arm *armv4_5 = &xscale->armv4_5_common;
	uint32_t dcsr_value;
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	if ((CMD_ARGC >= 1) && (strcmp("enable", CMD_ARGV[0]) == 0))
	{
		struct xscale_trace_data *td, *next_td;
		xscale->trace.buffer_enabled = 1;

		/* free old trace data */
		td = xscale->trace.data;
		while (td)
		{
			next_td = td->next;

			if (td->entries)
				free(td->entries);
			free(td);
			td = next_td;
		}
		xscale->trace.data = NULL;
	}
	else if ((CMD_ARGC >= 1) && (strcmp("disable", CMD_ARGV[0]) == 0))
	{
		xscale->trace.buffer_enabled = 0;
	}

	if ((CMD_ARGC >= 2) && (strcmp("fill", CMD_ARGV[1]) == 0))
	{
		uint32_t fill = 1;
		if (CMD_ARGC >= 3)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], fill);
		xscale->trace.buffer_fill = fill;
	}
	else if ((CMD_ARGC >= 2) && (strcmp("wrap", CMD_ARGV[1]) == 0))
	{
		xscale->trace.buffer_fill = -1;
	}

	if (xscale->trace.buffer_enabled)
	{
		/* if we enable the trace buffer in fill-once
		 * mode we know the address of the first instruction */
		xscale->trace.pc_ok = 1;
		xscale->trace.current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	}
	else
	{
		/* otherwise the address is unknown, and we have no known good PC */
		xscale->trace.pc_ok = 0;
	}

	command_print(CMD_CTX, "trace buffer %s (%s)",
		(xscale->trace.buffer_enabled) ? "enabled" : "disabled",
		(xscale->trace.buffer_fill > 0) ? "fill" : "wrap");

	dcsr_value = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 0, 32);
	if (xscale->trace.buffer_fill >= 0)
		xscale_write_dcsr_sw(target, (dcsr_value & 0xfffffffc) | 2);
	else
		xscale_write_dcsr_sw(target, dcsr_value & 0xfffffffc);

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_trace_image_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "usage: xscale trace_image <file> [base address] [type]");
		return ERROR_OK;
	}

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (xscale->trace.image)
	{
		image_close(xscale->trace.image);
		free(xscale->trace.image);
		command_print(CMD_CTX, "previously loaded image found and closed");
	}

	xscale->trace.image = malloc(sizeof(struct image));
	xscale->trace.image->base_address_set = 0;
	xscale->trace.image->start_address_set = 0;

	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (CMD_ARGC >= 2)
	{
		xscale->trace.image->base_address_set = 1;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], xscale->trace.image->base_address);
	}
	else
	{
		xscale->trace.image->base_address_set = 0;
	}

	if (image_open(xscale->trace.image, CMD_ARGV[0], (CMD_ARGC >= 3) ? CMD_ARGV[2] : NULL) != ERROR_OK)
	{
		free(xscale->trace.image);
		xscale->trace.image = NULL;
		return ERROR_OK;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_dump_trace_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	struct xscale_trace_data *trace_data;
	struct fileio file;
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "usage: xscale dump_trace <file>");
		return ERROR_OK;
	}

	trace_data = xscale->trace.data;

	if (!trace_data)
	{
		command_print(CMD_CTX, "no trace data collected");
		return ERROR_OK;
	}

	if (fileio_open(&file, CMD_ARGV[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	while (trace_data)
	{
		int i;

		fileio_write_u32(&file, trace_data->chkpt0);
		fileio_write_u32(&file, trace_data->chkpt1);
		fileio_write_u32(&file, trace_data->last_instruction);
		fileio_write_u32(&file, trace_data->depth);

		for (i = 0; i < trace_data->depth; i++)
			fileio_write_u32(&file, trace_data->entries[i].data | ((trace_data->entries[i].type & 0xffff) << 16));

		trace_data = trace_data->next;
	}

	fileio_close(&file);

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_analyze_trace_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	xscale_analyze_trace(target, CMD_CTX);

	return ERROR_OK;
}

COMMAND_HANDLER(xscale_handle_cp15)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xscale_common *xscale = target_to_xscale(target);
	int retval;

	retval = xscale_verify_pointer(CMD_CTX, xscale);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}
	uint32_t reg_no = 0;
	struct reg *reg = NULL;
	if (CMD_ARGC > 0)
	{
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg_no);
		/*translate from xscale cp15 register no to openocd register*/
		switch (reg_no)
		{
		case 0:
			reg_no = XSCALE_MAINID;
			break;
		case 1:
			reg_no = XSCALE_CTRL;
			break;
		case 2:
			reg_no = XSCALE_TTB;
			break;
		case 3:
			reg_no = XSCALE_DAC;
			break;
		case 5:
			reg_no = XSCALE_FSR;
			break;
		case 6:
			reg_no = XSCALE_FAR;
			break;
		case 13:
			reg_no = XSCALE_PID;
			break;
		case 15:
			reg_no = XSCALE_CPACCESS;
			break;
		default:
			command_print(CMD_CTX, "invalid register number");
			return ERROR_INVALID_ARGUMENTS;
		}
		reg = &xscale->reg_cache->reg_list[reg_no];

	}
	if (CMD_ARGC == 1)
	{
		uint32_t value;

		/* read cp15 control register */
		xscale_get_reg(reg);
		value = buf_get_u32(reg->value, 0, 32);
		command_print(CMD_CTX, "%s (/%i): 0x%" PRIx32 "", reg->name, (int)(reg->size), value);
	}
	else if (CMD_ARGC == 2)
	{
		uint32_t value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

		/* send CP write request (command 0x41) */
		xscale_send_u32(target, 0x41);

		/* send CP register number */
		xscale_send_u32(target, reg_no);

		/* send CP register value */
		xscale_send_u32(target, value);

		/* execute cpwait to ensure outstanding operations complete */
		xscale_send_u32(target, 0x53);
	}
	else
	{
		command_print(CMD_CTX, "usage: cp15 [register]<, [value]>");
	}

	return ERROR_OK;
}

static const struct command_registration xscale_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = xscale_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about CPU caches",
	},
	{
		.name = "mmu",
		.handler = xscale_handle_mmu_command,
		.mode = COMMAND_EXEC,
		.help = "enable or disable the MMU",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "icache",
		.handler = xscale_handle_idcache_command,
		.mode = COMMAND_EXEC,
		.help = "display ICache state, optionally enabling or "
			"disabling it",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "dcache",
		.handler = xscale_handle_idcache_command,
		.mode = COMMAND_EXEC,
		.help = "display DCache state, optionally enabling or "
			"disabling it",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "vector_catch",
		.handler = xscale_handle_vector_catch_command,
		.mode = COMMAND_EXEC,
		.help = "set or display 8-bit mask of vectors "
			"that should trigger debug entry",
		.usage = "[mask]",
	},
	{
		.name = "vector_table",
		.handler = xscale_handle_vector_table_command,
		.mode = COMMAND_EXEC,
		.help = "set vector table entry in mini-ICache, "
			"or display current tables",
		.usage = "[('high'|'low') index code]",
	},
	{
		.name = "trace_buffer",
		.handler = xscale_handle_trace_buffer_command,
		.mode = COMMAND_EXEC,
		.help = "display trace buffer status, enable or disable "
			"tracing, and optionally reconfigure trace mode",
		.usage = "['enable'|'disable' ['fill' number|'wrap']]",
	},
	{
		.name = "dump_trace",
		.handler = xscale_handle_dump_trace_command,
		.mode = COMMAND_EXEC,
		.help = "dump content of trace buffer to file",
		.usage = "filename",
	},
	{
		.name = "analyze_trace",
		.handler = xscale_handle_analyze_trace_buffer_command,
		.mode = COMMAND_EXEC,
		.help = "analyze content of trace buffer",
		.usage = "",
	},
	{
		.name = "trace_image",
		.handler = xscale_handle_trace_image_command,
		.mode = COMMAND_EXEC,
		.help = "load image from file to address (default 0)",
		.usage = "filename [offset [filetype]]",
	},
	{
		.name = "cp15",
		.handler = xscale_handle_cp15,
		.mode = COMMAND_EXEC,
		.help = "Read or write coprocessor 15 register.",
		.usage = "register [value]",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration xscale_any_command_handlers[] = {
	{
		.name = "debug_handler",
		.handler = xscale_handle_debug_handler_command,
		.mode = COMMAND_ANY,
		.help = "Change address used for debug handler.",
		.usage = "target address",
	},
	{
		.name = "cache_clean_address",
		.handler = xscale_handle_cache_clean_address_command,
		.mode = COMMAND_ANY,
		.help = "Change address used for cleaning data cache.",
		.usage = "address",
	},
	{
		.chain = xscale_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration xscale_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.name = "xscale",
		.mode = COMMAND_ANY,
		.help = "xscale command group",
		.chain = xscale_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type xscale_target =
{
	.name = "xscale",

	.poll = xscale_poll,
	.arch_state = xscale_arch_state,

	.target_request_data = NULL,

	.halt = xscale_halt,
	.resume = xscale_resume,
	.step = xscale_step,

	.assert_reset = xscale_assert_reset,
	.deassert_reset = xscale_deassert_reset,
	.soft_reset_halt = NULL,

	/* REVISIT on some cores, allow exporting iwmmxt registers ... */
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = xscale_read_memory,
	.read_phys_memory = xscale_read_phys_memory,
	.write_memory = xscale_write_memory,
	.write_phys_memory = xscale_write_phys_memory,
	.bulk_write_memory = xscale_bulk_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = xscale_add_breakpoint,
	.remove_breakpoint = xscale_remove_breakpoint,
	.add_watchpoint = xscale_add_watchpoint,
	.remove_watchpoint = xscale_remove_watchpoint,

	.commands = xscale_command_handlers,
	.target_create = xscale_target_create,
	.init_target = xscale_init_target,

	.virt2phys = xscale_virt2phys,
	.mmu = xscale_mmu
};

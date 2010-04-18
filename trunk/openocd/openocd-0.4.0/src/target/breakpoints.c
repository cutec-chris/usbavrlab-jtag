/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "target.h"
#include <helper/log.h>
#include "breakpoints.h"


static char *breakpoint_type_strings[] =
{
	"hardware",
	"software"
};

static char *watchpoint_rw_strings[] =
{
	"read",
	"write",
	"access"
};

// monotonic counter/id-number for breakpoints and watch points
static int bpwp_unique_id;

int breakpoint_add(struct target *target, uint32_t address, uint32_t length, enum breakpoint_type type)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	char *reason;
	int retval;
	int n;

	n = 0;
	while (breakpoint)
	{
		n++;
		if (breakpoint->address == address) {
			/* FIXME don't assume "same address" means "same
			 * breakpoint" ... check all the parameters before
			 * succeeding.
			 */
			LOG_DEBUG("Duplicate Breakpoint address: 0x%08" PRIx32 " (BP %d)",
				  address, breakpoint->unique_id );
			return ERROR_OK;
		}
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	(*breakpoint_p) = malloc(sizeof(struct breakpoint));
	(*breakpoint_p)->address = address;
	(*breakpoint_p)->length = length;
	(*breakpoint_p)->type = type;
	(*breakpoint_p)->set = 0;
	(*breakpoint_p)->orig_instr = malloc(length);
	(*breakpoint_p)->next = NULL;
	(*breakpoint_p)->unique_id = bpwp_unique_id++;

	retval = target_add_breakpoint(target, *breakpoint_p);
	switch (retval) {
	case ERROR_OK:
		break;
	case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
		reason = "resource not available";
		goto fail;
	case ERROR_TARGET_NOT_HALTED:
		reason = "target running";
		goto fail;
	default:
		reason = "unknown reason";
fail:
		LOG_ERROR("can't add breakpoint: %s", reason);
		free((*breakpoint_p)->orig_instr);
		free(*breakpoint_p);
		*breakpoint_p = NULL;
		return retval;
	}

	LOG_DEBUG("added %s breakpoint at 0x%8.8" PRIx32 " of length 0x%8.8x, (BPID: %d)",
			  breakpoint_type_strings[(*breakpoint_p)->type],
			  (*breakpoint_p)->address, (*breakpoint_p)->length,
			  (*breakpoint_p)->unique_id  );

	return ERROR_OK;
}

/* free up a breakpoint */
static void breakpoint_free(struct target *target, struct breakpoint *breakpoint_remove)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	int retval;

	while (breakpoint)
	{
		if (breakpoint == breakpoint_remove)
			break;
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	if (breakpoint == NULL)
		return;

	retval = target_remove_breakpoint(target, breakpoint);

	LOG_DEBUG("free BPID: %d --> %d", breakpoint->unique_id, retval);
	(*breakpoint_p) = breakpoint->next;
	free(breakpoint->orig_instr);
	free(breakpoint);
}

void breakpoint_remove(struct target *target, uint32_t address)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;

	while (breakpoint)
	{
		if (breakpoint->address == address)
			break;
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	if (breakpoint)
	{
		breakpoint_free(target, breakpoint);
	}
	else
	{
		LOG_ERROR("no breakpoint at address 0x%8.8" PRIx32 " found", address);
	}
}

void breakpoint_clear_target(struct target *target)
{
	struct breakpoint *breakpoint;

	LOG_DEBUG("Delete all breakpoints for target: %s",
			target_name(target));
	while ((breakpoint = target->breakpoints) != NULL)
	{
		breakpoint_free(target, breakpoint);
	}
}

struct breakpoint* breakpoint_find(struct target *target, uint32_t address)
{
	struct breakpoint *breakpoint = target->breakpoints;

	while (breakpoint)
	{
		if (breakpoint->address == address)
			return breakpoint;
		breakpoint = breakpoint->next;
	}

	return NULL;
}

int watchpoint_add(struct target *target, uint32_t address, uint32_t length,
		enum watchpoint_rw rw, uint32_t value, uint32_t mask)
{
	struct watchpoint *watchpoint = target->watchpoints;
	struct watchpoint **watchpoint_p = &target->watchpoints;
	int retval;
	char *reason;

	while (watchpoint)
	{
		if (watchpoint->address == address) {
			if (watchpoint->length != length
					|| watchpoint->value != value
					|| watchpoint->mask != mask
					|| watchpoint->rw != rw) {
				LOG_ERROR("address 0x%8.8" PRIx32
						"already has watchpoint %d",
						address, watchpoint->unique_id);
				return ERROR_FAIL;
			}

			/* ignore duplicate watchpoint */
			return ERROR_OK;
		}
		watchpoint_p = &watchpoint->next;
		watchpoint = watchpoint->next;
	}

	(*watchpoint_p) = calloc(1, sizeof(struct watchpoint));
	(*watchpoint_p)->address = address;
	(*watchpoint_p)->length = length;
	(*watchpoint_p)->value = value;
	(*watchpoint_p)->mask = mask;
	(*watchpoint_p)->rw = rw;
	(*watchpoint_p)->unique_id = bpwp_unique_id++;

	retval = target_add_watchpoint(target, *watchpoint_p);
	switch (retval) {
	case ERROR_OK:
		break;
	case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
		reason = "resource not available";
		goto bye;
	case ERROR_TARGET_NOT_HALTED:
		reason = "target running";
		goto bye;
	default:
		reason = "unrecognized error";
bye:
		LOG_ERROR("can't add %s watchpoint at 0x%8.8" PRIx32 ", %s",
			 watchpoint_rw_strings[(*watchpoint_p)->rw],
			 address, reason);
		free (*watchpoint_p);
		*watchpoint_p = NULL;
		return retval;
	}

	LOG_DEBUG("added %s watchpoint at 0x%8.8" PRIx32
			" of length 0x%8.8" PRIx32 " (WPID: %d)",
			watchpoint_rw_strings[(*watchpoint_p)->rw],
			(*watchpoint_p)->address,
			(*watchpoint_p)->length,
			(*watchpoint_p)->unique_id );

	return ERROR_OK;
}

static void watchpoint_free(struct target *target, struct watchpoint *watchpoint_remove)
{
	struct watchpoint *watchpoint = target->watchpoints;
	struct watchpoint **watchpoint_p = &target->watchpoints;
	int retval;

	while (watchpoint)
	{
		if (watchpoint == watchpoint_remove)
			break;
		watchpoint_p = &watchpoint->next;
		watchpoint = watchpoint->next;
	}

	if (watchpoint == NULL)
		return;
	retval = target_remove_watchpoint(target, watchpoint);
	LOG_DEBUG("free WPID: %d --> %d", watchpoint->unique_id, retval);
	(*watchpoint_p) = watchpoint->next;
	free(watchpoint);
}

void watchpoint_remove(struct target *target, uint32_t address)
{
	struct watchpoint *watchpoint = target->watchpoints;
	struct watchpoint **watchpoint_p = &target->watchpoints;

	while (watchpoint)
	{
		if (watchpoint->address == address)
			break;
		watchpoint_p = &watchpoint->next;
		watchpoint = watchpoint->next;
	}

	if (watchpoint)
	{
		watchpoint_free(target, watchpoint);
	}
	else
	{
		LOG_ERROR("no watchpoint at address 0x%8.8" PRIx32 " found", address);
	}
}

void watchpoint_clear_target(struct target *target)
{
	struct watchpoint *watchpoint;

	LOG_DEBUG("Delete all watchpoints for target: %s",
			target_name(target));
	while ((watchpoint = target->watchpoints) != NULL)
	{
		watchpoint_free(target, watchpoint);
	}
}

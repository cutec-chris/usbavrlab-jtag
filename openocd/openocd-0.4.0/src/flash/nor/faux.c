/***************************************************************************
 *   Copyright (C) 2009 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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

#include "imp.h"
#include <target/image.h>
#include "hello.h"


struct faux_flash_bank
{
	struct target *target;
	uint8_t *memory;
	uint32_t start_address;
};

static const int sectorSize = 0x10000;


/* flash bank faux <base> <size> <chip_width> <bus_width> <target#> <driverPath>
 */
FLASH_BANK_COMMAND_HANDLER(faux_flash_bank_command)
{
	struct faux_flash_bank *info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank faux configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	info = malloc(sizeof(struct faux_flash_bank));
	if (info == NULL)
	{
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	info->memory = malloc(bank->size);
	if (info == NULL)
	{
		free(info);
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = info;

	/* Use 0x10000 as a fixed sector size. */
	int i = 0;
	uint32_t offset = 0;
	bank->num_sectors = bank->size/sectorSize;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = sectorSize;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	info->target = get_target(CMD_ARGV[5]);
	if (info->target == NULL)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[5]);
		free(info->memory);
		free(info);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int faux_erase(struct flash_bank *bank, int first, int last)
{
	struct faux_flash_bank *info = bank->driver_priv;
	memset(info->memory + first*sectorSize, 0xff, sectorSize*(last-first + 1));
	return ERROR_OK;
}

static int faux_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_USER("set protection sector %d to %d to %s", first, last, set?"on":"off");
	return ERROR_OK;
}

static int faux_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct faux_flash_bank *info = bank->driver_priv;
	memcpy(info->memory + offset, buffer, count);
	return ERROR_OK;
}

static int faux_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int faux_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "faux flash driver");
	return ERROR_OK;
}

static int faux_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

static const struct command_registration faux_command_handlers[] = {
	{
		.name = "faux",
		.mode = COMMAND_ANY,
		.help = "faux flash command group",
		.chain = hello_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver faux_flash = {
	.name = "faux",
	.commands = faux_command_handlers,
	.flash_bank_command = faux_flash_bank_command,
	.erase = faux_erase,
	.protect = faux_protect,
	.write = faux_write,
	.probe = faux_probe,
	.auto_probe = faux_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = faux_protect_check,
	.info = faux_info
};

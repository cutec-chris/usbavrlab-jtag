/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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
#include <helper/time_support.h>
#include <target/image.h>

/**
 * @file
 * Implements Tcl commands used to access NOR flash facilities.
 */

COMMAND_HELPER(flash_command_get_bank, unsigned name_index,
		struct flash_bank **bank)
{
	const char *name = CMD_ARGV[name_index];
	*bank = get_flash_bank_by_name(name);
	if (*bank)
		return ERROR_OK;

	unsigned bank_num;
	COMMAND_PARSE_NUMBER(uint, name, bank_num);

	*bank = get_flash_bank_by_num(bank_num);
	if (!*bank)
	{
		command_print(CMD_CTX, "flash bank '%s' not found", name);
		return ERROR_INVALID_ARGUMENTS;
	}
	return ERROR_OK;
}


COMMAND_HANDLER(handle_flash_info_command)
{
	struct flash_bank *p;
	uint32_t i = 0;
	int j = 0;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned bank_nr;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], bank_nr);

	for (p = flash_bank_list(); p; p = p->next, i++)
	{
		if (i != bank_nr)
			continue;

		char buf[1024];

		/* attempt auto probe */
		if ((retval = p->driver->auto_probe(p)) != ERROR_OK)
			return retval;

		command_print(CMD_CTX,
			      "#%" PRIi32 " : %s at 0x%8.8" PRIx32 ", size 0x%8.8" PRIx32 ", buswidth %i, chipwidth %i",
			      i,
			      p->driver->name,
			      p->base,
			      p->size,
			      p->bus_width,
			      p->chip_width);
		for (j = 0; j < p->num_sectors; j++)
		{
			char *protect_state;

			if (p->sectors[j].is_protected == 0)
				protect_state = "not protected";
			else if (p->sectors[j].is_protected == 1)
				protect_state = "protected";
			else
				protect_state = "protection state unknown";

			command_print(CMD_CTX,
				      "\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
				      j,
				      p->sectors[j].offset,
				      p->sectors[j].size,
				      p->sectors[j].size >> 10,
				      protect_state);
		}

		*buf = '\0'; /* initialize buffer, otherwise it migh contain garbage if driver function fails */
		retval = p->driver->info(p, buf, sizeof(buf));
		command_print(CMD_CTX, "%s", buf);
		if (retval != ERROR_OK)
			LOG_ERROR("error retrieving flash info (%d)", retval);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_probe_command)
{
	int retval;

	if (CMD_ARGC != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned bank_nr;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], bank_nr);
	struct flash_bank *p = get_flash_bank_by_num_noprobe(bank_nr);
	if (p)
	{
		if ((retval = p->driver->probe(p)) == ERROR_OK)
		{
			command_print(CMD_CTX, "flash '%s' found at 0x%8.8" PRIx32, p->driver->name, p->base);
		}
		else if (retval == ERROR_FLASH_BANK_INVALID)
		{
			command_print(CMD_CTX, "probing failed for flash bank '#%s' at 0x%8.8" PRIx32,
						  CMD_ARGV[0], p->base);
		}
		else
		{
			command_print(CMD_CTX, "unknown error when probing flash bank '#%s' at 0x%8.8" PRIx32,
						  CMD_ARGV[0], p->base);
		}
	}
	else
	{
		command_print(CMD_CTX, "flash bank '#%s' is out of bounds", CMD_ARGV[0]);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_erase_check_command)
{
	if (CMD_ARGC != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	int j;
	if ((retval = p->driver->erase_check(p)) == ERROR_OK)
	{
		command_print(CMD_CTX, "successfully checked erase state");
	}
	else
	{
		command_print(CMD_CTX, "unknown error when checking erase state of flash bank #%s at 0x%8.8" PRIx32,
			CMD_ARGV[0], p->base);
	}

	for (j = 0; j < p->num_sectors; j++)
	{
		char *erase_state;

		if (p->sectors[j].is_erased == 0)
			erase_state = "not erased";
		else if (p->sectors[j].is_erased == 1)
			erase_state = "erased";
		else
			erase_state = "erase state unknown";

		command_print(CMD_CTX,
			      "\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
			      j,
			      p->sectors[j].offset,
			      p->sectors[j].size,
			      p->sectors[j].size >> 10,
			      erase_state);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_erase_address_command)
{
	struct flash_bank *p;
	int retval;
	int address;
	int length;
	bool do_pad = false;
	struct target *target = get_current_target(CMD_CTX);

	switch (CMD_ARGC) {
	case 3:
		/* Optionally pad out the address range to block/sector
		 * boundaries.  We can't know if there's data in that part
		 * of the flash; only do padding if we're told to.
		 */
		if (strcmp("pad", CMD_ARGV[0]) != 0)
			return ERROR_COMMAND_SYNTAX_ERROR;
		do_pad = true;
		CMD_ARGC--;
		CMD_ARGV++;
		/* FALL THROUGH */
	case 2:
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], address);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], length);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (length <= 0)
	{
		command_print(CMD_CTX, "Length must be >0");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	p = get_flash_bank_by_addr(target, address);
	if (p == NULL)
	{
		return ERROR_FAIL;
	}

	/* We can't know if we did a resume + halt, in which case we no longer know the erased state */
	flash_set_dirty();

	struct duration bench;
	duration_start(&bench);

	retval = flash_erase_address_range(target, do_pad, address, length);

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK))
	{
		command_print(CMD_CTX, "erased address 0x%8.8x (length %i)"
				" in %fs (%0.3f kb/s)", address, length,
				duration_elapsed(&bench), duration_kbps(&bench, length));
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_protect_check_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	if ((retval = p->driver->protect_check(p)) == ERROR_OK)
	{
		command_print(CMD_CTX, "successfully checked protect state");
	}
	else if (retval == ERROR_FLASH_OPERATION_FAILED)
	{
		command_print(CMD_CTX, "checking protection state failed (possibly unsupported) by flash #%s at 0x%8.8" PRIx32, CMD_ARGV[0], p->base);
	}
	else
	{
		command_print(CMD_CTX, "unknown error when checking protection state of flash bank '#%s' at 0x%8.8" PRIx32, CMD_ARGV[0], p->base);
	}

	return ERROR_OK;
}

static int flash_check_sector_parameters(struct command_context *cmd_ctx,
		uint32_t first, uint32_t last, uint32_t num_sectors)
{
	if (!(first <= last)) {
		command_print(cmd_ctx, "ERROR: "
				"first sector must be <= last sector");
		return ERROR_FAIL;
	}

	if (!(last <= (num_sectors - 1))) {
		command_print(cmd_ctx, "ERROR: last sector must be <= %d",
				(int) num_sectors - 1);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_erase_command)
{
	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t bank_nr;
	uint32_t first;
	uint32_t last;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bank_nr);
	struct flash_bank *p = get_flash_bank_by_num(bank_nr);
	if (!p)
		return ERROR_OK;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	if (strcmp(CMD_ARGV[2], "last") == 0)
		last = p->num_sectors - 1;
	else
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	int retval;
	if ((retval = flash_check_sector_parameters(CMD_CTX,
			first, last, p->num_sectors)) != ERROR_OK)
		return retval;

	struct duration bench;
	duration_start(&bench);

	retval = flash_driver_erase(p, first, last);

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK))
	{
		command_print(CMD_CTX, "erased sectors %" PRIu32 " "
				"through %" PRIu32" on flash bank %" PRIu32 " "
				"in %fs", first, last, bank_nr, duration_elapsed(&bench));
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_protect_command)
{
	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t bank_nr;
	uint32_t first;
	uint32_t last;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bank_nr);
	struct flash_bank *p = get_flash_bank_by_num(bank_nr);
	if (!p)
		return ERROR_OK;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	if (strcmp(CMD_ARGV[2], "last") == 0)
		last = p->num_sectors - 1;
	else
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	bool set;
	COMMAND_PARSE_ON_OFF(CMD_ARGV[3], set);

	int retval;
	if ((retval = flash_check_sector_parameters(CMD_CTX,
			first, last, p->num_sectors)) != ERROR_OK)
		return retval;

	retval = flash_driver_protect(p, set, first, last);
	if (retval == ERROR_OK) {
		command_print(CMD_CTX, "%s protection for sectors %i "
				"through %i on flash bank %i",
			(set) ? "set" : "cleared", (int) first,
			(int) last, (int) bank_nr);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_write_image_command)
{
	struct target *target = get_current_target(CMD_CTX);

	struct image image;
	uint32_t written;

	int retval;

	if (CMD_ARGC < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* flash auto-erase is disabled by default*/
	int auto_erase = 0;
	bool auto_unlock = false;

	for (;;)
	{
		if (strcmp(CMD_ARGV[0], "erase") == 0)
		{
			auto_erase = 1;
			CMD_ARGV++;
			CMD_ARGC--;
			command_print(CMD_CTX, "auto erase enabled");
		} else if (strcmp(CMD_ARGV[0], "unlock") == 0)
		{
			auto_unlock = true;
			CMD_ARGV++;
			CMD_ARGC--;
			command_print(CMD_CTX, "auto unlock enabled");
		} else
		{
			break;
		}
	}

	if (CMD_ARGC < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!target)
	{
		LOG_ERROR("no target selected");
		return ERROR_FAIL;
	}

	struct duration bench;
	duration_start(&bench);

	if (CMD_ARGC >= 2)
	{
		image.base_address_set = 1;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], image.base_address);
	}
	else
	{
		image.base_address_set = 0;
		image.base_address = 0x0;
	}

	image.start_address_set = 0;

	retval = image_open(&image, CMD_ARGV[0], (CMD_ARGC == 3) ? CMD_ARGV[2] : NULL);
	if (retval != ERROR_OK)
	{
		return retval;
	}

	retval = flash_write_unlock(target, &image, &written, auto_erase, auto_unlock);
	if (retval != ERROR_OK)
	{
		image_close(&image);
		return retval;
	}

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK))
	{
		command_print(CMD_CTX, "wrote %" PRIu32 " bytes from file %s "
				"in %fs (%0.3f kb/s)", written, CMD_ARGV[0],
				duration_elapsed(&bench), duration_kbps(&bench, written));
	}

	image_close(&image);

	return retval;
}

COMMAND_HANDLER(handle_flash_fill_command)
{
	int err = ERROR_OK;
	uint32_t address;
	uint32_t pattern;
	uint32_t count;
	uint32_t wrote = 0;
	uint32_t cur_size = 0;
	uint32_t chunk_count;
	struct target *target = get_current_target(CMD_CTX);
	uint32_t i;
	uint32_t wordsize;
	int retval = ERROR_OK;

	static size_t const chunksize = 1024;
	uint8_t *chunk = malloc(chunksize);
	if (chunk == NULL)
		return ERROR_FAIL;

	uint8_t *readback = malloc(chunksize);
	if (readback == NULL)
	{
		free(chunk);
		return ERROR_FAIL;
	}


	if (CMD_ARGC != 3)
	{
		retval = ERROR_COMMAND_SYNTAX_ERROR;
		goto done;
	}


	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], pattern);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);

	if (count == 0)
		goto done;

	switch (CMD_NAME[4])
	{
	case 'w':
		wordsize = 4;
		break;
	case 'h':
		wordsize = 2;
		break;
	case 'b':
		wordsize = 1;
		break;
	default:
		retval = ERROR_COMMAND_SYNTAX_ERROR;
		goto done;
	}

	chunk_count = MIN(count, (chunksize / wordsize));
	switch (wordsize)
	{
	case 4:
		for (i = 0; i < chunk_count; i++)
		{
			target_buffer_set_u32(target, chunk + i * wordsize, pattern);
		}
		break;
	case 2:
		for (i = 0; i < chunk_count; i++)
		{
			target_buffer_set_u16(target, chunk + i * wordsize, pattern);
		}
		break;
	case 1:
		memset(chunk, pattern, chunk_count);
		break;
	default:
		LOG_ERROR("BUG: can't happen");
		exit(-1);
	}

	struct duration bench;
	duration_start(&bench);

	for (wrote = 0; wrote < (count*wordsize); wrote += cur_size)
	{
		struct flash_bank *bank;

		bank = get_flash_bank_by_addr(target, address);
		if (bank == NULL)
		{
			retval = ERROR_FAIL;
			goto done;
		}

		cur_size = MIN((count * wordsize - wrote), chunksize);
		err = flash_driver_write(bank, chunk, address - bank->base + wrote, cur_size);
		if (err != ERROR_OK)
		{
			retval = err;
			goto done;
		}

		err = target_read_buffer(target, address + wrote, cur_size, readback);
		if (err != ERROR_OK)
		{
			retval = err;
			goto done;
		}

		unsigned i;
		for (i = 0; i < cur_size; i++)
		{
			if (readback[i]!=chunk[i])
			{
				LOG_ERROR("Verfication error address 0x%08" PRIx32 ", read back 0x%02x, expected 0x%02x",
						  address + wrote + i, readback[i], chunk[i]);
				retval = ERROR_FAIL;
				goto done;
			}
		}
	}

	if (duration_measure(&bench) == ERROR_OK)
	{
		command_print(CMD_CTX, "wrote %" PRIu32 " bytes to 0x%8.8" PRIx32
				" in %fs (%0.3f kb/s)", wrote, address,
				duration_elapsed(&bench), duration_kbps(&bench, wrote));
	}

done:
	free(readback);
	free(chunk);

	return retval;
}

COMMAND_HANDLER(handle_flash_write_bank_command)
{
	uint32_t offset;
	uint8_t *buffer;
	struct fileio fileio;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct duration bench;
	duration_start(&bench);

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], offset);

	if (fileio_open(&fileio, CMD_ARGV[1], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	buffer = malloc(fileio.size);
	size_t buf_cnt;
	if (fileio_read(&fileio, fileio.size, buffer, &buf_cnt) != ERROR_OK)
	{
		free(buffer);
		fileio_close(&fileio);
		return ERROR_OK;
	}

	retval = flash_driver_write(p, buffer, offset, buf_cnt);

	free(buffer);
	buffer = NULL;

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK))
	{
		command_print(CMD_CTX, "wrote %ld bytes from file %s to flash bank %u"
				" at offset 0x%8.8" PRIx32 " in %fs (%0.3f kb/s)",
				(long)fileio.size, CMD_ARGV[1], p->bank_number, offset,
				duration_elapsed(&bench), duration_kbps(&bench, fileio.size));
	}

	fileio_close(&fileio);

	return retval;
}

void flash_set_dirty(void)
{
	struct flash_bank *c;
	int i;

	/* set all flash to require erasing */
	for (c = flash_bank_list(); c; c = c->next)
	{
		for (i = 0; i < c->num_sectors; i++)
		{
			c->sectors[i].is_erased = 0;
		}
	}
}

static const struct command_registration flash_exec_command_handlers[] = {
	{
		.name = "probe",
		.handler = handle_flash_probe_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Identify a flash bank.",
	},
	{
		.name = "info",
		.handler = handle_flash_info_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Print information about a flash bank.",
	},
	{
		.name = "erase_check",
		.handler = handle_flash_erase_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Check erase state of all blocks in a "
			"flash bank.",
	},
	{
		.name = "protect_check",
		.handler = handle_flash_protect_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Check protection state of all blocks in a "
			"flash bank.",
	},
	{
		.name = "erase_sector",
		.handler = handle_flash_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id first_sector_num last_sector_num",
		.help = "Erase a range of sectors in a flash bank.",
	},
	{
		.name = "erase_address",
		.handler = handle_flash_erase_address_command,
		.mode = COMMAND_EXEC,
		.usage = "['pad'] address length",
		.help = "Erase flash sectors starting at address and "
			"continuing for length bytes.  If 'pad' is specified, "
			"data outside that range may also be erased: the start "
			"address may be decreased, and length increased, so "
			"that all of the first and last sectors are erased.",
	},
	{
		.name = "fillw",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n words with 32-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "fillh",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n halfwords with 16-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "fillb",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n bytes with 8-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "write_bank",
		.handler = handle_flash_write_bank_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename offset",
		.help = "Write binary data from file to flash bank, "
			"starting at specified byte offset from the "
			"beginning of the bank.",
	},
	{
		.name = "write_image",
		.handler = handle_flash_write_image_command,
		.mode = COMMAND_EXEC,
		.usage = "[erase] [unlock] filename [offset [file_type]]",
		.help = "Write an image to flash.  Optionally first unprotect "
			"and/or erase the region to be used.  Allow optional "
			"offset from beginning of bank (defaults to zero)",
	},
	{
		.name = "protect",
		.handler = handle_flash_protect_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id first_sector [last_sector|'last'] "
			"('on'|'off')",
		.help = "Turn protection on or off for a range of sectors "
			"in a given flash bank.",
	},
	COMMAND_REGISTRATION_DONE
};

int flash_init_drivers(struct command_context *cmd_ctx)
{
	if (!flash_bank_list())
		return ERROR_OK;

	struct command *parent = command_find_in_context(cmd_ctx, "flash");
	return register_commands(cmd_ctx, parent, flash_exec_command_handlers);
}


COMMAND_HANDLER(handle_flash_bank_command)
{
	if (CMD_ARGC < 7)
	{
		LOG_ERROR("usage: flash bank <name> <driver> "
				"<base> <size> <chip_width> <bus_width>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	// save bank name and advance arguments for compatibility
	const char *bank_name = *CMD_ARGV++;
	CMD_ARGC--;

	struct target *target;
	if ((target = get_target(CMD_ARGV[5])) == NULL)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[5]);
		return ERROR_FAIL;
	}

	const char *driver_name = CMD_ARGV[0];
	struct flash_driver *driver = flash_driver_find_by_name(driver_name);
	if (NULL == driver)
	{
		/* no matching flash driver found */
		LOG_ERROR("flash driver '%s' not found", driver_name);
		return ERROR_FAIL;
	}

	/* register flash specific commands */
	if (NULL != driver->commands)
	{
		int retval = register_commands(CMD_CTX, NULL,
				driver->commands);
		if (ERROR_OK != retval)
		{
			LOG_ERROR("couldn't register '%s' commands",
					driver_name);
			return ERROR_FAIL;
		}
	}

	struct flash_bank *c = malloc(sizeof(*c));
	c->name = strdup(bank_name);
	c->target = target;
	c->driver = driver;
	c->driver_priv = NULL;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], c->base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], c->size);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], c->chip_width);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[4], c->bus_width);
	c->num_sectors = 0;
	c->sectors = NULL;
	c->next = NULL;

	int retval;
	retval = CALL_COMMAND_HANDLER(driver->flash_bank_command, c);
	if (ERROR_OK != retval)
	{
		LOG_ERROR("'%s' driver rejected flash bank at 0x%8.8" PRIx32,
				driver_name, c->base);
		free(c);
		return retval;
	}

	flash_bank_add(c);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_banks_command)
{
	if (CMD_ARGC != 0)
		return ERROR_INVALID_ARGUMENTS;

	unsigned n = 0;
	for (struct flash_bank *p = flash_bank_list(); p; p = p->next, n++)
	{
		LOG_USER("#%u: %s at 0x%8.8" PRIx32 ", size 0x%8.8" PRIx32 ", "
			"buswidth %u, chipwidth %u", n,
			p->driver->name, p->base, p->size,
			p->bus_width, p->chip_width);
	}
	return ERROR_OK;
}

static int jim_flash_list(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 1)
	{
		Jim_WrongNumArgs(interp, 1, argv,
				"no arguments to 'flash list' command");
		return JIM_ERR;
	}

	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);

	for (struct flash_bank *p = flash_bank_list(); p; p = p->next)
	{
		Jim_Obj *elem = Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "name", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, p->driver->name, -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "base", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->base));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "size", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->size));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "bus_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->bus_width));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "chip_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->chip_width));

		Jim_ListAppendElement(interp, list, elem);
	}

	Jim_SetResult(interp, list);

	return JIM_OK;
}


COMMAND_HANDLER(handle_flash_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool flash_initialized = false;
	if (flash_initialized)
	{
		LOG_INFO("'flash init' has already been called");
		return ERROR_OK;
	}
	flash_initialized = true;

	LOG_DEBUG("Initializing flash devices...");
	return flash_init_drivers(CMD_CTX);
}

static const struct command_registration flash_config_command_handlers[] = {
	{
		.name = "bank",
		.handler = handle_flash_bank_command,
		.mode = COMMAND_CONFIG,
		.usage = "bank_id driver_name base_address size_bytes "
			"chip_width_bytes bus_width_bytes target "
			"[driver_options ...]",
		.help = "Define a new bank with the given name, "
			"using the specified NOR flash driver.",
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = handle_flash_init_command,
		.help = "Initialize flash devices.",
	},
	{
		.name = "banks",
		.mode = COMMAND_ANY,
		.handler = handle_flash_banks_command,
		.help = "Display table with information about flash banks.",
	},
	{
		.name = "list",
		.mode = COMMAND_ANY,
		.jim_handler = jim_flash_list,
		.help = "Returns a list of details about the flash banks.",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration flash_command_handlers[] = {
	{
		.name = "flash",
		.mode = COMMAND_ANY,
		.help = "NOR flash command group",
		.chain = flash_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int flash_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, flash_command_handlers);
}

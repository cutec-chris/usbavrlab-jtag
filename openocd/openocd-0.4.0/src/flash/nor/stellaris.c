/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

/***************************************************************************
* STELLARIS is tested on LM3S811, LM3S6965
***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "stellaris.h"
#include <target/algorithm.h>
#include <target/armv7m.h>


#define DID0_VER(did0) ((did0 >> 28)&0x07)

static void stellaris_read_clock_info(struct flash_bank *bank);
static int stellaris_mass_erase(struct flash_bank *bank);

static struct {
	uint32_t partno;
	char *partname;
}	StellarisParts[] =
{
	{0x0001,"LM3S101"},
	{0x0002,"LM3S102"},
	{0x0019,"LM3S300"},
	{0x0011,"LM3S301"},
	{0x001A,"LM3S308"},
	{0x0012,"LM3S310"},
	{0x0013,"LM3S315"},
	{0x0014,"LM3S316"},
	{0x0017,"LM3S317"},
	{0x0015,"LM3S328"},
	{0x002A,"LM3S600"},
	{0x0021,"LM3S601"},
	{0x002B,"LM3S608"},
	{0x0022,"LM3S610"},
	{0x0023,"LM3S611"},
	{0x0024,"LM3S612"},
	{0x0025,"LM3S613"},
	{0x0026,"LM3S615"},
	{0x0028,"LM3S617"},
	{0x0029,"LM3S618"},
	{0x0027,"LM3S628"},
	{0x0038,"LM3S800"},
	{0x0031,"LM3S801"},
	{0x0039,"LM3S808"},
	{0x0032,"LM3S811"},
	{0x0033,"LM3S812"},
	{0x0034,"LM3S815"},
	{0x0036,"LM3S817"},
	{0x0037,"LM3S818"},
	{0x0035,"LM3S828"},
	{0x10BF,"LM3S1110"},
	{0x10C3,"LM3S1133"},
	{0x10C5,"LM3S1138"},
	{0x10C1,"LM3S1150"},
	{0x10C4,"LM3S1162"},
	{0x10C2,"LM3S1165"},
	{0x10C6,"LM3S1332"},
	{0x10BC,"LM3S1435"},
	{0x10BA,"LM3S1439"},
	{0x10BB,"LM3S1512"},
	{0x10C7,"LM3S1538"},
	{0x10DB,"LM3S1601"},
	{0x1006,"LM3S1607"},
	{0x10DA,"LM3S1608"},
	{0x10C0,"LM3S1620"},
	{0x1003,"LM3S1625"},
	{0x1004,"LM3S1626"},
	{0x1005,"LM3S1627"},
	{0x10B3,"LM3S1635"},
	{0x10BD,"LM3S1637"},
	{0x10B9,"LM3S1751"},
	{0x1010,"LM3S1776"},
	{0x1016,"LM3S1811"},
	{0x103D,"LM3S1816"},
	{0x10B4,"LM3S1850"},
	{0x10DD,"LM3S1911"},
	{0x10DC,"LM3S1918"},
	{0x10B7,"LM3S1937"},
	{0x10BE,"LM3S1958"},
	{0x10B5,"LM3S1960"},
	{0x10B8,"LM3S1968"},
	{0x100F,"LM3S1J11"},
	{0x103C,"LM3S1J16"},
	{0x100E,"LM3S1N11"},
	{0x103B,"LM3S1N16"},
	{0x1030,"LM3S1W16"},
	{0x102F,"LM3S1Z16"},
	{0x1051,"LM3S2110"},
	{0x1084,"LM3S2139"},
	{0x1039,"LM3S2276"},
	{0x10A2,"LM3S2410"},
	{0x1059,"LM3S2412"},
	{0x1056,"LM3S2432"},
	{0x105A,"LM3S2533"},
	{0x10E1,"LM3S2601"},
	{0x10E0,"LM3S2608"},
	{0x1033,"LM3S2616"},
	{0x1057,"LM3S2620"},
	{0x1085,"LM3S2637"},
	{0x1053,"LM3S2651"},
	{0x1080,"LM3S2671"},
	{0x1050,"LM3S2678"},
	{0x10A4,"LM3S2730"},
	{0x1052,"LM3S2739"},
	{0x103A,"LM3S2776"},
	{0x106D,"LM3S2793"},
	{0x10E3,"LM3S2911"},
	{0x10E2,"LM3S2918"},
	{0x1054,"LM3S2939"},
	{0x108F,"LM3S2948"},
	{0x1058,"LM3S2950"},
	{0x1055,"LM3S2965"},
	{0x106C,"LM3S2B93"},
	{0x1043,"LM3S3651"},
	{0x1044,"LM3S3739"},
	{0x1049,"LM3S3748"},
	{0x1045,"LM3S3749"},
	{0x1042,"LM3S3826"},
	{0x1041,"LM3S3J26"},
	{0x1040,"LM3S3N26"},
	{0x103F,"LM3S3W26"},
	{0x103E,"LM3S3Z26"},
	{0x1081,"LM3S5632"},
	{0x100C,"LM3S5651"},
	{0x108A,"LM3S5652"},
	{0x104D,"LM3S5656"},
	{0x1091,"LM3S5662"},
	{0x1096,"LM3S5732"},
	{0x1097,"LM3S5737"},
	{0x10A0,"LM3S5739"},
	{0x1099,"LM3S5747"},
	{0x10A7,"LM3S5749"},
	{0x109A,"LM3S5752"},
	{0x109C,"LM3S5762"},
	{0x1069,"LM3S5791"},
	{0x100B,"LM3S5951"},
	{0x104E,"LM3S5956"},
	{0x1068,"LM3S5B91"},
	{0x1009,"LM3S5K31"},
	{0x104A,"LM3S5K36"},
	{0x100A,"LM3S5P31"},
	{0x1048,"LM3S5P36"},
	{0x100D,"LM3S5P51"},
	{0x104C,"LM3S5P56"},
	{0x1007,"LM3S5R31"},
	{0x104B,"LM3S5R36"},
	{0x1047,"LM3S5T36"},
	{0x1046,"LM3S5Y36"},
	{0x10A1,"LM3S6100"},
	{0x1074,"LM3S6110"},
	{0x10A5,"LM3S6420"},
	{0x1082,"LM3S6422"},
	{0x1075,"LM3S6432"},
	{0x1076,"LM3S6537"},
	{0x1071,"LM3S6610"},
	{0x10E7,"LM3S6611"},
	{0x10E6,"LM3S6618"},
	{0x1083,"LM3S6633"},
	{0x108B,"LM3S6637"},
	{0x10A3,"LM3S6730"},
	{0x1077,"LM3S6753"},
	{0x10E9,"LM3S6911"},
	{0x10E8,"LM3S6918"},
	{0x1089,"LM3S6938"},
	{0x1072,"LM3S6950"},
	{0x1078,"LM3S6952"},
	{0x1073,"LM3S6965"},
	{0x1064,"LM3S8530"},
	{0x108E,"LM3S8538"},
	{0x1061,"LM3S8630"},
	{0x1063,"LM3S8730"},
	{0x108D,"LM3S8733"},
	{0x1086,"LM3S8738"},
	{0x1065,"LM3S8930"},
	{0x108C,"LM3S8933"},
	{0x1088,"LM3S8938"},
	{0x10A6,"LM3S8962"},
	{0x1062,"LM3S8970"},
	{0x10D7,"LM3S8971"},
	{0x1067,"LM3S9790"},
	{0x106B,"LM3S9792"},
	{0x1020,"LM3S9997"},
	{0x1066,"LM3S9B90"},
	{0x106A,"LM3S9B92"},
	{0x106E,"LM3S9B95"},
	{0x106F,"LM3S9B96"},
	{0x1018,"LM3S9L97"},
	{0,"Unknown part"}
};

static char * StellarisClassname[5] =
{
	"Sandstorm",
	"Fury",
	"Unknown",
	"DustDevil",
	"Tempest"
};

/***************************************************************************
*	openocd command interface                                              *
***************************************************************************/

/* flash_bank stellaris <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stellaris_flash_bank_command)
{
	struct stellaris_flash_bank *stellaris_info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank stellaris configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	stellaris_info = calloc(sizeof(struct stellaris_flash_bank), 1);
	bank->base = 0x0;
	bank->driver_priv = stellaris_info;

	stellaris_info->target_name = "Unknown target";

	/* part wasn't probed for info yet */
	stellaris_info->did1 = 0;

	/* TODO Specify the main crystal speed in kHz using an optional
	 * argument; ditto, the speed of an external oscillator used
	 * instead of a crystal.  Avoid programming flash using IOSC.
	 */
	return ERROR_OK;
}

static int stellaris_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed, device_class;
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;

	if (stellaris_info->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* Read main and master clock freqency register */
	stellaris_read_clock_info(bank);

	if (DID0_VER(stellaris_info->did0) > 0)
	{
		device_class = (stellaris_info->did0 >> 16) & 0xFF;
	}
	else
	{
		device_class = 0;
	}
	printed = snprintf(buf,
			   buf_size,
			   "\nTI/LMI Stellaris information: Chip is "
			   "class %i (%s) %s rev %c%i\n",
			   device_class,
			   StellarisClassname[device_class],
			   stellaris_info->target_name,
			   (int)('A' + ((stellaris_info->did0 >> 8) & 0xFF)),
			   (int)((stellaris_info->did0) & 0xFF));
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			   buf_size,
			   "did1: 0x%8.8" PRIx32 ", arch: 0x%4.4" PRIx32
			   ", eproc: %s, ramsize: %ik, flashsize: %ik\n",
			   stellaris_info->did1,
			   stellaris_info->did1,
			   "ARMv7M",
			   (int)((1 + ((stellaris_info->dc0 >> 16) & 0xFFFF))/4),
			   (int)((1 + (stellaris_info->dc0 & 0xFFFF))*2));
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			   buf_size,
			   "master clock: %ikHz%s, "
			   "rcc is 0x%" PRIx32 ", rcc2 is 0x%" PRIx32 "\n",
			   (int)(stellaris_info->mck_freq / 1000),
			   stellaris_info->mck_desc,
			   stellaris_info->rcc,
			   stellaris_info->rcc2);
	buf += printed;
	buf_size -= printed;

	if (stellaris_info->num_lockbits > 0)
	{
		printed = snprintf(buf,
				buf_size,
				"pagesize: %" PRIi32 ", pages: %d, "
				"lockbits: %i, pages per lockbit: %i\n",
				stellaris_info->pagesize,
				(unsigned) stellaris_info->num_pages,
				stellaris_info->num_lockbits,
				(unsigned) stellaris_info->pages_in_lockregion);
		buf += printed;
		buf_size -= printed;
	}
	return ERROR_OK;
}

/***************************************************************************
*	chip identification and status                                         *
***************************************************************************/

/* Set the flash timimg register to match current clocking */
static void stellaris_set_flash_timing(struct flash_bank *bank)
{
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t usecrl = (stellaris_info->mck_freq/1000000ul-1);

	LOG_DEBUG("usecrl = %i",(int)(usecrl));
	target_write_u32(target, SCB_BASE | USECRL, usecrl);
}

static const unsigned rcc_xtal[32] = {
	[0x00] = 1000000,		/* no pll */
	[0x01] = 1843200,		/* no pll */
	[0x02] = 2000000,		/* no pll */
	[0x03] = 2457600,		/* no pll */

	[0x04] = 3579545,
	[0x05] = 3686400,
	[0x06] = 4000000,		/* usb */
	[0x07] = 4096000,

	[0x08] = 4915200,
	[0x09] = 5000000,		/* usb */
	[0x0a] = 5120000,
	[0x0b] = 6000000,		/* (reset) usb */

	[0x0c] = 6144000,
	[0x0d] = 7372800,
	[0x0e] = 8000000,		/* usb */
	[0x0f] = 8192000,

	/* parts before DustDevil use just 4 bits for xtal spec */

	[0x10] = 10000000,		/* usb */
	[0x11] = 12000000,		/* usb */
	[0x12] = 12288000,
	[0x13] = 13560000,

	[0x14] = 14318180,
	[0x15] = 16000000,		/* usb */
	[0x16] = 16384000,
};

/** Read clock configuration and set stellaris_info->usec_clocks. */
static void stellaris_read_clock_info(struct flash_bank *bank)
{
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t rcc, rcc2, pllcfg, sysdiv, usesysdiv, bypass, oscsrc;
	unsigned xtal;
	unsigned long mainfreq;

	target_read_u32(target, SCB_BASE | RCC, &rcc);
	LOG_DEBUG("Stellaris RCC %" PRIx32 "", rcc);

	target_read_u32(target, SCB_BASE | RCC2, &rcc2);
	LOG_DEBUG("Stellaris RCC2 %" PRIx32 "", rcc);

	target_read_u32(target, SCB_BASE | PLLCFG, &pllcfg);
	LOG_DEBUG("Stellaris PLLCFG %" PRIx32 "", pllcfg);

	stellaris_info->rcc = rcc;
	stellaris_info->rcc = rcc2;

	sysdiv = (rcc >> 23) & 0xF;
	usesysdiv = (rcc >> 22) & 0x1;
	bypass = (rcc >> 11) & 0x1;
	oscsrc = (rcc >> 4) & 0x3;
	xtal = (rcc >> 6) & stellaris_info->xtal_mask;

	/* NOTE: post-Sandstorm parts have RCC2 which may override
	 * parts of RCC ... with more sysdiv options, option for
	 * 32768 Hz mainfreq, PLL controls.  On Sandstorm it reads
	 * as zero, so the "use RCC2" flag is always clear.
	 */
	if (rcc2 & (1 << 31)) {
		sysdiv = (rcc2 >> 23) & 0x3F;
		bypass = (rcc2 >> 11) & 0x1;
		oscsrc = (rcc2 >> 4) & 0x7;

		/* FIXME Tempest parts have an additional lsb for
		 * fractional sysdiv (200 MHz / 2.5 == 80 MHz)
		 */
	}

	stellaris_info->mck_desc = "";

	switch (oscsrc)
	{
		case 0:				/* MOSC */
			mainfreq = rcc_xtal[xtal];
			break;
		case 1:				/* IOSC */
			mainfreq = stellaris_info->iosc_freq;
			stellaris_info->mck_desc = stellaris_info->iosc_desc;
			break;
		case 2:				/* IOSC/4 */
			mainfreq = stellaris_info->iosc_freq / 4;
			stellaris_info->mck_desc = stellaris_info->iosc_desc;
			break;
		case 3:				/* lowspeed */
			/* Sandstorm doesn't have this 30K +/- 30% osc */
			mainfreq = 30000;
			stellaris_info->mck_desc = " (±30%)";
			break;
		case 8:				/* hibernation osc */
			/* not all parts support hibernation */
			mainfreq = 32768;
			break;

		default: /* NOTREACHED */
			mainfreq = 0;
			break;
	}

	/* PLL is used if it's not bypassed; its output is 200 MHz
	 * even when it runs at 400 MHz (adds divide-by-two stage).
	 */
	if (!bypass)
		mainfreq = 200000000;

	if (usesysdiv)
		stellaris_info->mck_freq = mainfreq/(1 + sysdiv);
	else
		stellaris_info->mck_freq = mainfreq;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
static int stellaris_read_part_info(struct flash_bank *bank)
{
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t did0, did1, ver, fam;
	int i;

	/* Read and parse chip identification register */
	target_read_u32(target, SCB_BASE | DID0, &did0);
	target_read_u32(target, SCB_BASE | DID1, &did1);
	target_read_u32(target, SCB_BASE | DC0, &stellaris_info->dc0);
	target_read_u32(target, SCB_BASE | DC1, &stellaris_info->dc1);
	LOG_DEBUG("did0 0x%" PRIx32 ", did1 0x%" PRIx32 ", dc0 0x%" PRIx32 ", dc1 0x%" PRIx32 "",
		  did0, did1, stellaris_info->dc0, stellaris_info->dc1);

	ver = did0 >> 28;
	if ((ver != 0) && (ver != 1))
	{
		LOG_WARNING("Unknown did0 version, cannot identify target");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (did1 == 0)
	{
		LOG_WARNING("Cannot identify target as a Stellaris");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	ver = did1 >> 28;
	fam = (did1 >> 24) & 0xF;
	if (((ver != 0) && (ver != 1)) || (fam != 0))
	{
		LOG_WARNING("Unknown did1 version/family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* For Sandstorm, Fury, DustDevil:  current data sheets say IOSC
	 * is 12 MHz, but some older parts have 15 MHz.  A few data sheets
	 * even give _both_ numbers!  We'll use current numbers; IOSC is
	 * always approximate.
	 *
	 * For Tempest:  IOSC is calibrated, 16 MHz
	 */
	stellaris_info->iosc_freq = 12000000;
	stellaris_info->iosc_desc = " (±30%)";
	stellaris_info->xtal_mask = 0x0f;

	switch ((did0 >> 28) & 0x7) {
	case 0:				/* Sandstorm */
		/*
		 * Current (2009-August) parts seem to be rev C2 and use 12 MHz.
		 * Parts before rev C0 used 15 MHz; some C0 parts use 15 MHz
		 * (LM3S618), but some other C0 parts are 12 MHz (LM3S811).
		 */
		if (((did0 >> 8) & 0xff) < 2) {
			stellaris_info->iosc_freq = 15000000;
			stellaris_info->iosc_desc = " (±50%)";
		}
		break;
	case 1:
		switch ((did0 >> 16) & 0xff) {
		case 1:			/* Fury */
			break;
		case 4:			/* Tempest */
			stellaris_info->iosc_freq = 16000000;	/* +/- 1% */
			stellaris_info->iosc_desc = " (±1%)";
			/* FALL THROUGH */
		case 3:			/* DustDevil */
			stellaris_info->xtal_mask = 0x1f;
			break;
		default:
			LOG_WARNING("Unknown did0 class");
		}
		break;
	default:
		LOG_WARNING("Unknown did0 version");
		break;
	}

	for (i = 0; StellarisParts[i].partno; i++)
	{
		if (StellarisParts[i].partno == ((did1 >> 16) & 0xFFFF))
			break;
	}

	stellaris_info->target_name = StellarisParts[i].partname;

	stellaris_info->did0 = did0;
	stellaris_info->did1 = did1;

	stellaris_info->num_lockbits = 1 + (stellaris_info->dc0 & 0xFFFF);
	stellaris_info->num_pages = 2 *(1 + (stellaris_info->dc0 & 0xFFFF));
	stellaris_info->pagesize = 1024;
	stellaris_info->pages_in_lockregion = 2;

	/* REVISIT for at least Tempest parts, read NVMSTAT.FWB too.
	 * That exposes a 32-word Flash Write Buffer ... enabling
	 * writes of more than one word at a time.
	 */

	return ERROR_OK;
}

/***************************************************************************
*	flash operations                                                       *
***************************************************************************/

static int stellaris_protect_check(struct flash_bank *bank)
{
	struct stellaris_flash_bank *stellaris = bank->driver_priv;
	int status = ERROR_OK;
	unsigned i;
	unsigned page;

	if (stellaris->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	for (i = 0; i < (unsigned) bank->num_sectors; i++)
		bank->sectors[i].is_protected = -1;

	/* Read each Flash Memory Protection Program Enable (FMPPE) register
	 * to report any pages that we can't write.  Ignore the Read Enable
	 * register (FMPRE).
	 */
	for (i = 0, page = 0;
			i < DIV_ROUND_UP(stellaris->num_lockbits, 32u);
			i++) {
		uint32_t lockbits;

		status = target_read_u32(bank->target,
				SCB_BASE + (i ? (FMPPE0 + 4 * i) : FMPPE),
				&lockbits);
		LOG_DEBUG("FMPPE%d = %#8.8x (status %d)", i,
				(unsigned) lockbits, status);
		if (status != ERROR_OK)
			goto done;

		for (unsigned j = 0; j < 32; j++) {
			unsigned k;

			for (k = 0; k < stellaris->pages_in_lockregion; k++) {
				if (page >= (unsigned) bank->num_sectors)
					goto done;
				bank->sectors[page++].is_protected =
						!(lockbits & (1 << j));
			}
		}
	}

done:
	return status;
}

static int stellaris_erase(struct flash_bank *bank, int first, int last)
{
	int banknr;
	uint32_t flash_fmc, flash_cris;
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stellaris_info->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((first < 0) || (last < first) || (last >= (int)stellaris_info->num_pages))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if ((first == 0) && (last == ((int)stellaris_info->num_pages-1)))
	{
		return stellaris_mass_erase(bank);
	}

	/* Refresh flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_timing(bank);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

	/* REVISIT this clobbers state set by any halted firmware ...
	 * it might want to process those IRQs.
	 */

	for (banknr = first; banknr <= last; banknr++)
	{
		/* Address is first word in page */
		target_write_u32(target, FLASH_FMA, banknr * stellaris_info->pagesize);
		/* Write erase command */
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_ERASE);
		/* Wait until erase complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		}
		while (flash_fmc & FMC_ERASE);

		/* Check acess violations */
		target_read_u32(target, FLASH_CRIS, &flash_cris);
		if (flash_cris & (AMASK))
		{
			LOG_WARNING("Error erasing flash page %i,  flash_cris 0x%" PRIx32 "", banknr, flash_cris);
			target_write_u32(target, FLASH_CRIS, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[banknr].is_erased = 1;
	}

	return ERROR_OK;
}

static int stellaris_protect(struct flash_bank *bank, int set, int first, int last)
{
	uint32_t fmppe, flash_fmc, flash_cris;
	int lockregion;

	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!set)
	{
		LOG_ERROR("Can't unprotect write-protected pages.");
		/* except by the "recover locked device" procedure ... */
		return ERROR_INVALID_ARGUMENTS;
	}

	if (stellaris_info->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* lockregions are 2 pages ... must protect [even..odd] */
	if ((first < 0) || (first & 1)
			|| (last < first) || !(last & 1)
			|| (last >= 2 * stellaris_info->num_lockbits))
	{
		LOG_ERROR("Can't protect unaligned or out-of-range sectors.");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Refresh flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_timing(bank);

	/* convert from pages to lockregions */
	first /= 2;
	last /= 2;

	/* FIXME this assumes single FMPPE, for a max of 64K of flash!!
	 * Current parts can be much bigger.
	 */
	if (last >= 32) {
		LOG_ERROR("No support yet for protection > 64K");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	target_read_u32(target, SCB_BASE | FMPPE, &fmppe);

	for (lockregion = first; lockregion <= last; lockregion++)
		fmppe &= ~(1 << lockregion);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

	/* REVISIT this clobbers state set by any halted firmware ...
	 * it might want to process those IRQs.
	 */

	LOG_DEBUG("fmppe 0x%" PRIx32 "",fmppe);
	target_write_u32(target, SCB_BASE | FMPPE, fmppe);

	/* Commit FMPPE */
	target_write_u32(target, FLASH_FMA, 1);

	/* Write commit command */
	/* REVISIT safety check, since this cannot be undone
	 * except by the "Recover a locked device" procedure.
	 * REVISIT DustDevil-A0 parts have an erratum making FMPPE commits
	 * inadvisable ... it makes future mass erase operations fail.
	 */
	LOG_WARNING("Flash protection cannot be removed once commited, commit is NOT executed !");
	/* target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_COMT); */

	/* Wait until erase complete */
	do
	{
		target_read_u32(target, FLASH_FMC, &flash_fmc);
	}
	while (flash_fmc & FMC_COMT);

	/* Check acess violations */
	target_read_u32(target, FLASH_CRIS, &flash_cris);
	if (flash_cris & (AMASK))
	{
		LOG_WARNING("Error setting flash page protection,  flash_cris 0x%" PRIx32 "", flash_cris);
		target_write_u32(target, FLASH_CRIS, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static const uint8_t stellaris_write_code[] =
{
/*
	Call with :
	r0 = buffer address
	r1 = destination address
	r2 = bytecount (in) - endaddr (work)

	Used registers:
	r3 = pFLASH_CTRL_BASE
	r4 = FLASHWRITECMD
	r5 = #1
	r6 = bytes written
	r7 = temp reg
*/
	0x07,0x4B,			/* ldr r3,pFLASH_CTRL_BASE */
	0x08,0x4C,			/* ldr r4,FLASHWRITECMD */
	0x01,0x25,			/* movs r5, 1 */
	0x00,0x26,			/* movs r6, #0 */
/* mainloop: */
	0x19,0x60,			/* str	r1, [r3, #0] */
	0x87,0x59,			/* ldr	r7, [r0, r6] */
	0x5F,0x60,			/* str	r7, [r3, #4] */
	0x9C,0x60,			/* str	r4, [r3, #8] */
/* waitloop: */
	0x9F,0x68,			/* ldr	r7, [r3, #8] */
	0x2F,0x42,			/* tst	r7, r5 */
	0xFC,0xD1,			/* bne	waitloop */
	0x04,0x31,			/* adds	r1, r1, #4 */
	0x04,0x36,			/* adds	r6, r6, #4 */
	0x96,0x42,			/* cmp	r6, r2 */
	0xF4,0xD1,			/* bne	mainloop */
						/* exit: */
	0xFE,0xE7,			/* b exit */
/* pFLASH_CTRL_BASE: */
	0x00,0xD0,0x0F,0x40,	/* .word	0x400FD000 */
/* FLASHWRITECMD: */
	0x01,0x00,0x42,0xA4	/* .word	0xA4420001 */
};

static int stellaris_write_block(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t wcount)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 8192;
	struct working_area *source;
	struct working_area *write_algorithm;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* power of two, and multiple of word size */
	static const unsigned buf_min = 128;

	/* for small buffers it's faster not to download an algorithm */
	if (wcount * 4 < buf_min)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	LOG_DEBUG("(bank=%p buffer=%p offset=%08" PRIx32 " wcount=%08" PRIx32 "",
			bank, buffer, offset, wcount);

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stellaris_write_code), &write_algorithm) != ERROR_OK)
	{
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* plus a buffer big enough for this data */
	if (wcount * 4 < buffer_size)
		buffer_size = wcount * 4;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= buf_min)
		{
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("retry target_alloc_working_area(%s, size=%u)",
				target_name(target), (unsigned) buffer_size);
	};

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stellaris_write_code),
			(uint8_t *) stellaris_write_code);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);

	while (wcount > 0)
	{
		uint32_t thisrun_count = (wcount > (buffer_size / 4)) ? (buffer_size / 4) : wcount;

		target_write_buffer(target, source->address, thisrun_count * 4, buffer);

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, 4*thisrun_count);
		LOG_DEBUG("Algorithm flash write %u words to 0x%" PRIx32
				", %u remaining",
				(unsigned) thisrun_count, address,
				(unsigned) (wcount - thisrun_count));
		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address,
				write_algorithm->address +
					sizeof(stellaris_write_code) - 10,
				10000, &armv7m_info);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("error %d executing stellaris "
					"flash write algorithm",
					retval);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		wcount -= thisrun_count;
	}

	/* REVISIT we could speed up writing multi-section images by
	 * not freeing the initialized write_algorithm this way.
	 */

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

static int stellaris_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t address = offset;
	uint32_t flash_cris, flash_fmc;
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("(bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
			bank, buffer, offset, count);

	if (stellaris_info->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (offset & 0x3)
	{
		LOG_WARNING("offset size must be word aligned");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	/* Refresh flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_timing(bank);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

	/* REVISIT this clobbers state set by any halted firmware ...
	 * it might want to process those IRQs.
	 */

	/* multiple words to be programmed? */
	if (words_remaining > 0)
	{
		/* try using a block write */
		retval = stellaris_write_block(bank, buffer, offset,
				words_remaining);
		if (retval != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				LOG_DEBUG("writing flash word-at-a-time");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				/* if an error occured, we examine the reason, and quit */
				target_read_u32(target, FLASH_CRIS, &flash_cris);

				LOG_ERROR("flash writing failed with CRIS: 0x%" PRIx32 "", flash_cris);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0)
	{
		if (!(address & 0xff))
			LOG_DEBUG("0x%" PRIx32 "", address);

		/* Program one word */
		target_write_u32(target, FLASH_FMA, address);
		target_write_buffer(target, FLASH_FMD, 4, buffer);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_WRITE);
		/* LOG_DEBUG("0x%x 0x%x 0x%x",address,buf_get_u32(buffer, 0, 32),FMC_WRKEY | FMC_WRITE); */
		/* Wait until write complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		} while (flash_fmc & FMC_WRITE);

		buffer += 4;
		address += 4;
		words_remaining--;
	}

	if (bytes_remaining)
	{
		uint8_t last_word[4] = {0xff, 0xff, 0xff, 0xff};
		int i = 0;

		while (bytes_remaining > 0)
		{
			last_word[i++] = *(buffer + bytes_written);
			bytes_remaining--;
			bytes_written++;
		}

		if (!(address & 0xff))
			LOG_DEBUG("0x%" PRIx32 "", address);

		/* Program one word */
		target_write_u32(target, FLASH_FMA, address);
		target_write_buffer(target, FLASH_FMD, 4, last_word);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_WRITE);
		/* LOG_DEBUG("0x%x 0x%x 0x%x",address,buf_get_u32(buffer, 0, 32),FMC_WRKEY | FMC_WRITE); */
		/* Wait until write complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		} while (flash_fmc & FMC_WRITE);
	}

	/* Check access violations */
	target_read_u32(target, FLASH_CRIS, &flash_cris);
	if (flash_cris & (AMASK))
	{
		LOG_DEBUG("flash_cris 0x%" PRIx32 "", flash_cris);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int stellaris_probe(struct flash_bank *bank)
{
	struct stellaris_flash_bank *stellaris_info = bank->driver_priv;
	int retval;

	/* If this is a stellaris chip, it has flash; probe() is just
	 * to figure out how much is present.  Only do it once.
	 */
	if (stellaris_info->did1 != 0)
		return ERROR_OK;

	/* stellaris_read_part_info() already handled error checking and
	 * reporting.  Note that it doesn't write, so we don't care about
	 * whether the target is halted or not.
	 */
	retval = stellaris_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	/* provide this for the benefit of the NOR flash framework */
	bank->size = 1024 * stellaris_info->num_pages;
	bank->num_sectors = stellaris_info->num_pages;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	for (int i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = i * stellaris_info->pagesize;
		bank->sectors[i].size = stellaris_info->pagesize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	return retval;
}

static int stellaris_mass_erase(struct flash_bank *bank)
{
	struct target *target = NULL;
	struct stellaris_flash_bank *stellaris_info = NULL;
	uint32_t flash_fmc;

	stellaris_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stellaris_info->did1 == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* Refresh flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_timing(bank);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

	/* REVISIT this clobbers state set by any halted firmware ...
	 * it might want to process those IRQs.
	 */

	target_write_u32(target, FLASH_FMA, 0);
	target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_MERASE);
	/* Wait until erase complete */
	do
	{
		target_read_u32(target, FLASH_FMC, &flash_fmc);
	}
	while (flash_fmc & FMC_MERASE);

	/* if device has > 128k, then second erase cycle is needed
	 * this is only valid for older devices, but will not hurt */
	if (stellaris_info->num_pages * stellaris_info->pagesize > 0x20000)
	{
		target_write_u32(target, FLASH_FMA, 0x20000);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_MERASE);
		/* Wait until erase complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		}
		while (flash_fmc & FMC_MERASE);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stellaris_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "stellaris mass_erase <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (stellaris_mass_erase(bank) == ERROR_OK)
	{
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
		{
			bank->sectors[i].is_erased = 1;
		}

		command_print(CMD_CTX, "stellaris mass erase complete");
	}
	else
	{
		command_print(CMD_CTX, "stellaris mass erase failed");
	}

	return ERROR_OK;
}

static const struct command_registration stellaris_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = stellaris_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase entire device",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration stellaris_command_handlers[] = {
	{
		.name = "stellaris",
		.mode = COMMAND_EXEC,
		.help = "Stellaris flash command group",
		.chain = stellaris_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stellaris_flash = {
	.name = "stellaris",
	.commands = stellaris_command_handlers,
	.flash_bank_command = stellaris_flash_bank_command,
	.erase = stellaris_erase,
	.protect = stellaris_protect,
	.write = stellaris_write,
	.probe = stellaris_probe,
	.auto_probe = stellaris_probe,
	.erase_check = default_flash_mem_blank_check,
	.protect_check = stellaris_protect_check,
	.info = stellaris_info,
};

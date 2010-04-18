/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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
#ifndef MIPS32_PRACC_H
#define MIPS32_PRACC_H

#include <target/mips32.h>
#include <target/mips_ejtag.h>

#define MIPS32_PRACC_FASTDATA_AREA		0xFF200000
#define MIPS32_PRACC_FASTDATA_SIZE		16
#define MIPS32_PRACC_TEXT				0xFF200200
#define MIPS32_PRACC_STACK				0xFF204000
#define MIPS32_PRACC_PARAM_IN			0xFF201000
#define MIPS32_PRACC_PARAM_IN_SIZE		0x1000
#define MIPS32_PRACC_PARAM_OUT			(MIPS32_PRACC_PARAM_IN + MIPS32_PRACC_PARAM_IN_SIZE)
#define MIPS32_PRACC_PARAM_OUT_SIZE		0x1000

#define MIPS32_FASTDATA_HANDLER_SIZE	0x80
#define UPPER16(uint32_t) 				(uint32_t >> 16)
#define LOWER16(uint32_t) 				(uint32_t & 0xFFFF)
#define NEG16(v) 						(((~(v)) + 1) & 0xFFFF)
/*#define NEG18(v) (((~(v)) + 1) & 0x3FFFF)*/

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, void *buf);
int mips32_pracc_write_mem(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, void *buf);
int mips32_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info, struct working_area *source,
		int write, uint32_t addr, int count, uint32_t *buf);

int mips32_pracc_read_mem8(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint8_t *buf);
int mips32_pracc_read_mem16(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint16_t *buf);
int mips32_pracc_read_mem32(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint32_t *buf);
int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info,
		uint32_t addr, uint32_t *buf);

int mips32_pracc_write_mem8(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint8_t *buf);
int mips32_pracc_write_mem16(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint16_t *buf);
int mips32_pracc_write_mem32(struct mips_ejtag *ejtag_info,
		uint32_t addr, int count, uint32_t *buf);
int mips32_pracc_write_u32(struct mips_ejtag *ejtag_info,
		uint32_t addr, uint32_t *buf);

int mips32_pracc_read_regs(struct mips_ejtag *ejtag_info, uint32_t *regs);
int mips32_pracc_write_regs(struct mips_ejtag *ejtag_info, uint32_t *regs);

int mips32_pracc_exec(struct mips_ejtag *ejtag_info, int code_len, const uint32_t *code,
		int num_param_in, uint32_t *param_in,
		int num_param_out, uint32_t *param_out, int cycle);

#endif

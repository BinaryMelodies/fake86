/*
  Fake86: A portable, open-source 8086 PC emulator.
  Copyright (C)2010-2013 Mike Chambers

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/* cpu.c: functions to emulate the 8086/V20 CPU in software. the heart of Fake86. */

#include "config.h"
#ifndef CPU_INSTRUCTION_FLOW_CACHE

#include <stdint.h>
#include <stdio.h>
#include "cpu.h"
#include "i8259.h"
#include "i8253.h"
#include "modregrm.h"

extern struct i8253_s i8253;

extern struct structpic i8259;
uint64_t curtimer, lasttimer, timerfreq;

uint8_t byteregtable[8] = { regal, regcl, regdl, regbl, regah, regch, regdh, regbh };

static const uint8_t parity[0x100] = {
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

uint8_t	RAM[0x100000], readonly[0x100000];
uint8_t	opcode, segoverride, reptype, bootdrive = 0, hdcount = 0, hltstate = 0;
uint16_t segregs[4], savecs, saveip, ip, useseg, oldsp;
uint8_t	tempcf, oldcf, cf, pf, af, zf, sf, tf, ifl, df, of, mode, reg, rm;
#ifdef CPU_EMULATE_8080
uint8_t md, mdenable;
#endif
uint16_t oper1, oper2, res16, disp16, temp16, dummy, stacksize, frametemp;
uint8_t	oper1b, oper2b, res8, disp8, temp8, nestlev, addrbyte;
uint32_t temp1, temp2, temp3, temp4, temp5, temp32, tempaddr32, ea;
#ifdef CPU_SEGMENT_WRAPAROUND
int8_t eawraparound; /* 8086 segments wrap around, this makes sure the wrap around is respected */
#endif
int32_t	result;
uint64_t totalexec;

extern uint16_t	VGA_SC[0x100], VGA_CRTC[0x100], VGA_ATTR[0x100], VGA_GC[0x100];
extern uint8_t updatedscreen;
union _bytewordregs_ regs;

uint8_t	portram[0x10000];
uint8_t	running = 0, debugmode, showcsip, mouseemu, didbootstrap = 0;
uint8_t	ethif;

extern uint8_t	vidmode;
extern uint8_t verbose;

extern void vidinterrupt();

extern uint8_t readVGA (uint32_t addr32);

void intcall86 (uint8_t intnum);

extern void	writeVGA (uint32_t addr32, uint8_t value);
extern void	portout (uint16_t portnum, uint8_t value);
extern void	portout16 (uint16_t portnum, uint16_t value);
extern uint8_t	portin (uint16_t portnum);
extern uint16_t portin16 (uint16_t portnum);

void write86 (uint32_t addr32, uint8_t value) {
	tempaddr32 = addr32 & 0xFFFFF;
#ifdef CPU_ADDR_MODE_CACHE
	if (!readonly[tempaddr32]) addrcachevalid[tempaddr32] = 0;
#endif
	if (readonly[tempaddr32] || (tempaddr32 >= 0xC0000) ) {
			return;
		}

	if ( (tempaddr32 >= 0xA0000) && (tempaddr32 <= 0xBFFFF) ) {
			if ( (vidmode != 0x13) && (vidmode != 0x12) && (vidmode != 0xD) && (vidmode != 0x10) ) {
					RAM[tempaddr32] = value;
					updatedscreen = 1;
				}
			else if ( ( (VGA_SC[4] & 6) == 0) && (vidmode != 0xD) && (vidmode != 0x10) && (vidmode != 0x12) ) {
					RAM[tempaddr32] = value;
					updatedscreen = 1;
				}
			else {
					writeVGA (tempaddr32 - 0xA0000, value);
				}

			updatedscreen = 1;
		}
	else {
			RAM[tempaddr32] = value;
		}
}

void writew86 (uint32_t addr32, uint16_t value) {
	/* warning: does not simulate 8086 wrap around */
	write86 (addr32, (uint8_t) value);
	write86 (addr32 + 1, (uint8_t) (value >> 8) );
}

uint8_t read86 (uint32_t addr32) {
	addr32 &= 0xFFFFF;
	if ( (addr32 >= 0xA0000) && (addr32 <= 0xBFFFF) ) {
			if ( (vidmode == 0xD) || (vidmode == 0xE) || (vidmode == 0x10) || (vidmode == 0x12) ) return (readVGA (addr32 - 0xA0000) );
			if ( (vidmode != 0x13) && (vidmode != 0x12) && (vidmode != 0xD) ) return (RAM[addr32]);
			if ( (VGA_SC[4] & 6) == 0)
				return (RAM[addr32]);
			else
				return (readVGA (addr32 - 0xA0000) );
		}

	if (!didbootstrap) {
			RAM[0x410] = 0x41; //ugly hack to make BIOS always believe we have an EGA/VGA card installed
			RAM[0x475] = hdcount; //the BIOS doesn't have any concept of hard drives, so here's another hack
		}

	return (RAM[addr32]);
}

uint16_t readw86 (uint32_t addr32) {
	return ( (uint16_t) read86 (addr32) | (uint16_t) (read86 (addr32 + 1) << 8) );
}

void flag_szp8 (uint8_t value) {
	if (!value) {
			zf = 1;
		}
	else {
			zf = 0;	/* set or clear zero flag */
		}

	if (value & 0x80) {
			sf = 1;
		}
	else {
			sf = 0;	/* set or clear sign flag */
		}

	pf = parity[value]; /* retrieve parity state from lookup table */
}

void flag_szp16 (uint16_t value) {
	if (!value) {
			zf = 1;
		}
	else {
			zf = 0;	/* set or clear zero flag */
		}

	if (value & 0x8000) {
			sf = 1;
		}
	else {
			sf = 0;	/* set or clear sign flag */
		}

	pf = parity[value & 255];	/* retrieve parity state from lookup table */
}

void flag_log8 (uint8_t value) {
	flag_szp8 (value);
	cf = 0;
	of = 0; /* bitwise logic ops always clear carry and overflow */
}

void flag_log16 (uint16_t value) {
	flag_szp16 (value);
	cf = 0;
	of = 0; /* bitwise logic ops always clear carry and overflow */
}

void flag_adc8 (uint8_t v1, uint8_t v2, uint8_t v3) {

	/* v1 = destination operand, v2 = source operand, v3 = carry flag */
	uint16_t	dst;

	dst = (uint16_t) v1 + (uint16_t) v2 + (uint16_t) v3;
	flag_szp8 ( (uint8_t) dst);
	if ( ( (dst ^ v1) & (dst ^ v2) & 0x80) == 0x80) {
			of = 1;
		}
	else {
			of = 0; /* set or clear overflow flag */
		}

	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0; /* set or clear carry flag */
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0; /* set or clear auxilliary flag */
		}
}

void flag_adc16 (uint16_t v1, uint16_t v2, uint16_t v3) {

	uint32_t	dst;

	dst = (uint32_t) v1 + (uint32_t) v2 + (uint32_t) v3;
	flag_szp16 ( (uint16_t) dst);
	if ( ( ( (dst ^ v1) & (dst ^ v2) ) & 0x8000) == 0x8000) {
			of = 1;
		}
	else {
			of = 0;
		}

	if (dst & 0xFFFF0000) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_add8 (uint8_t v1, uint8_t v2) {
	/* v1 = destination operand, v2 = source operand */
	uint16_t	dst;

	dst = (uint16_t) v1 + (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (dst ^ v1) & (dst ^ v2) & 0x80) == 0x80) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_add16 (uint16_t v1, uint16_t v2) {
	/* v1 = destination operand, v2 = source operand */
	uint32_t	dst;

	dst = (uint32_t) v1 + (uint32_t) v2;
	flag_szp16 ( (uint16_t) dst);
	if (dst & 0xFFFF0000) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (dst ^ v1) & (dst ^ v2) & 0x8000) == 0x8000) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_sbb8 (uint8_t v1, uint8_t v2, uint8_t v3) {

	/* v1 = destination operand, v2 = source operand, v3 = carry flag */
	uint16_t	dst;

	v2 += v3;
	dst = (uint16_t) v1 - (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( (dst ^ v1) & (v1 ^ v2) & 0x80) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( (v1 ^ v2 ^ dst) & 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_sbb16 (uint16_t v1, uint16_t v2, uint16_t v3) {

	/* v1 = destination operand, v2 = source operand, v3 = carry flag */
	uint32_t	dst;

	v2 += v3;
	dst = (uint32_t) v1 - (uint32_t) v2;
	flag_szp16 ( (uint16_t) dst);
	if (dst & 0xFFFF0000) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( (dst ^ v1) & (v1 ^ v2) & 0x8000) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( (v1 ^ v2 ^ dst) & 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_sub8 (uint8_t v1, uint8_t v2) {

	/* v1 = destination operand, v2 = source operand */
	uint16_t	dst;

	dst = (uint16_t) v1 - (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( (dst ^ v1) & (v1 ^ v2) & 0x80) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( (v1 ^ v2 ^ dst) & 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_sub16 (uint16_t v1, uint16_t v2) {

	/* v1 = destination operand, v2 = source operand */
	uint32_t	dst;

	dst = (uint32_t) v1 - (uint32_t) v2;
	flag_szp16 ( (uint16_t) dst);
	if (dst & 0xFFFF0000) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( (dst ^ v1) & (v1 ^ v2) & 0x8000) {
			of = 1;
		}
	else {
			of = 0;
		}

	if ( (v1 ^ v2 ^ dst) & 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void op_adc8() {
	res8 = oper1b + oper2b + cf;
	flag_adc8 (oper1b, oper2b, cf);
}

void op_adc16() {
	res16 = oper1 + oper2 + cf;
	flag_adc16 (oper1, oper2, cf);
}

void op_add8() {
	res8 = oper1b + oper2b;
	flag_add8 (oper1b, oper2b);
}

void op_add16() {
	res16 = oper1 + oper2;
	flag_add16 (oper1, oper2);
}

void op_and8() {
	res8 = oper1b & oper2b;
	flag_log8 (res8);
}

void op_and16() {
	res16 = oper1 & oper2;
	flag_log16 (res16);
}

void op_or8() {
	res8 = oper1b | oper2b;
	flag_log8 (res8);
}

void op_or16() {
	res16 = oper1 | oper2;
	flag_log16 (res16);
}

void op_xor8() {
	res8 = oper1b ^ oper2b;
	flag_log8 (res8);
}

void op_xor16() {
	res16 = oper1 ^ oper2;
	flag_log16 (res16);
}

void op_sub8() {
	res8 = oper1b - oper2b;
	flag_sub8 (oper1b, oper2b);
}

void op_sub16() {
	res16 = oper1 - oper2;
	flag_sub16 (oper1, oper2);
}

void op_sbb8() {
	res8 = oper1b - (oper2b + cf);
	flag_sbb8 (oper1b, oper2b, cf);
}

void op_sbb16() {
	res16 = oper1 - (oper2 + cf);
	flag_sbb16 (oper1, oper2, cf);
}

#ifdef CPU_EMULATE_8080
void flag_80log8 (uint8_t value) {
	flag_szp8 (value);
	cf = 0;
	af = 0;
}

#if 0
void flag_80and8 (uint8_t value) {
	flag_szp8 (value);
	cf = 0;
	af = 1; /* emulates 8085 behavior */
}
#endif

void flag_80adc8 (uint8_t v1, uint8_t v2, uint8_t v3) {

	/* v1 = destination operand, v2 = source operand, v3 = carry flag */
	uint16_t	dst;

	dst = (uint16_t) v1 + (uint16_t) v2 + (uint16_t) v3;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0; /* set or clear carry flag */
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0; /* set or clear auxilliary flag */
		}
}

void flag_80add8 (uint8_t v1, uint8_t v2) {
	/* v1 = destination operand, v2 = source operand */
	uint16_t	dst;

	dst = (uint16_t) v1 + (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_80add16 (uint16_t v1, uint16_t v2) {
	/* v1 = destination operand, v2 = source operand */
	uint32_t	dst;

	dst = (uint32_t) v1 + (uint32_t) v2;
	if (dst & 0xFFFF0000) {
			cf = 1;
		}
	else {
			cf = 0;
		}
}

void flag_80sub8 (uint8_t v1, uint8_t v2) {

	/* v1 = destination operand, v2 = source operand */
	uint16_t	dst;

	dst = (uint16_t) v1 - (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void flag_80sbb8 (uint8_t v1, uint8_t v2, uint8_t v3) {

	/* v1 = destination operand, v2 = source operand, v3 = carry flag */
	uint16_t	dst;

	v2 += v3;
	dst = (uint16_t) v1 - (uint16_t) v2;
	flag_szp8 ( (uint8_t) dst);
	if (dst & 0xFF00) {
			cf = 1;
		}
	else {
			cf = 0;
		}

	if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
			af = 1;
		}
	else {
			af = 0;
		}
}

void op_80adc8() {
	res8 = oper1b + oper2b + cf;
	flag_80adc8 (oper1b, oper2b, cf);
}

void op_80add8() {
	res8 = oper1b + oper2b;
	flag_80add8 (oper1b, oper2b);
}

void op_80add16() {
	res16 = oper1 + oper2;
	flag_80add16 (oper1, oper2);
}

void op_80and8() {
	res8 = oper1b & oper2b;
	flag_80log8 (res8);
	af = ((oper1b | oper2b) >> 3) & 1; /* Intel 8080 behavior */
}

void op_80or8() {
	res8 = oper1b | oper2b;
	flag_80log8 (res8);
}

void op_80xor8() {
	res8 = oper1b ^ oper2b;
	flag_80log8 (res8);
}

void op_80sub8() {
	res8 = oper1b - oper2b;
	flag_80sub8 (oper1b, oper2b);
}

void op_80sbb8() {
	res8 = oper1b - (oper2b + cf);
	flag_80sbb8 (oper1b, oper2b, cf);
}
#endif /* CPU_EMULATE_8080 */

void getea (uint8_t rmval) {
	uint32_t	tempea;

	tempea = 0;
	switch (mode) {
			case 0:
				switch (rmval) {
						case 0:
							tempea = regs.wordregs[regbx] + regs.wordregs[regsi];
							break;
						case 1:
							tempea = regs.wordregs[regbx] + regs.wordregs[regdi];
							break;
						case 2:
							tempea = regs.wordregs[regbp] + regs.wordregs[regsi];
							break;
						case 3:
							tempea = regs.wordregs[regbp] + regs.wordregs[regdi];
							break;
						case 4:
							tempea = regs.wordregs[regsi];
							break;
						case 5:
							tempea = regs.wordregs[regdi];
							break;
						case 6:
							tempea = disp16;
							break;
						case 7:
							tempea = regs.wordregs[regbx];
							break;
					}
				break;

			case 1:
			case 2:
				switch (rmval) {
						case 0:
							tempea = regs.wordregs[regbx] + regs.wordregs[regsi] + disp16;
							break;
						case 1:
							tempea = regs.wordregs[regbx] + regs.wordregs[regdi] + disp16;
							break;
						case 2:
							tempea = regs.wordregs[regbp] + regs.wordregs[regsi] + disp16;
							break;
						case 3:
							tempea = regs.wordregs[regbp] + regs.wordregs[regdi] + disp16;
							break;
						case 4:
							tempea = regs.wordregs[regsi] + disp16;
							break;
						case 5:
							tempea = regs.wordregs[regdi] + disp16;
							break;
						case 6:
							tempea = regs.wordregs[regbp] + disp16;
							break;
						case 7:
							tempea = regs.wordregs[regbx] + disp16;
							break;
					}
				break;
		}

	ea = (tempea & 0xFFFF) + (useseg << 4);
#ifdef CPU_SEGMENT_WRAPAROUND
	eawraparound = (tempea & 0xFFFF) == 0xFFFF;
#endif
}

void push (uint16_t pushval) {
	regs.wordregs[regsp] = regs.wordregs[regsp] - 2;
	putmem16 (segregs[regss], regs.wordregs[regsp], pushval);
}

uint16_t pop() {

	uint16_t	tempval;

	tempval = getmem16 (segregs[regss], regs.wordregs[regsp]);
	regs.wordregs[regsp] = regs.wordregs[regsp] + 2;
	return tempval;
}

#ifdef CPU_EMULATE_8080
void push80 (uint16_t pushval) {
	putreg16 (regbp, regs.wordregs[regbp] - 2);
	putmem16 (segregs[regds], regs.wordregs[regbp], pushval);
}

uint16_t pop80() {

	uint16_t	tempval;

	tempval = getmem16 (segregs[regds], regs.wordregs[regbp] );
	putreg16 (regbp, regs.wordregs[regbp] + 2);
	return tempval;
}
#endif

void reset86() {
	segregs[regcs] = 0xF000;
	ip = 0xFFF0;
	hltstate = 0;
#ifdef CPU_EMULATE_8080
	md = 1;
	mdenable = 0;
#endif
}

uint16_t readrm16 (uint8_t rmval) {
	if (mode < 3) {
			getea (rmval);
			return read86 (ea) | ( (uint16_t) read86 (ea + 1) << 8);
		}
	else {
			return getreg16 (rmval);
		}
}

uint8_t readrm8 (uint8_t rmval) {
	if (mode < 3) {
			getea (rmval);
			return read86 (ea);
		}
	else {
			return getreg8 (rmval);
		}
}

void writerm16 (uint8_t rmval, uint16_t value) {
	if (mode < 3) {
			getea (rmval);
			write86 (ea, value & 0xFF);
#ifndef CPU_SEGMENT_WRAPAROUND
			write86 (ea + 1, value >> 8);
#else
			write86 (eawraparound ? ea - 0xFFFF : ea + 1, value >> 8);
#endif
		}
	else {
			putreg16 (rmval, value);
		}
}

void writerm8 (uint8_t rmval, uint8_t value) {
	if (mode < 3) {
			getea (rmval);
			write86 (ea, value);
		}
	else {
			putreg8 (rmval, value);
		}
}

uint8_t op_grp2_8 (uint8_t cnt) {

	uint16_t	s;
	uint16_t	shift;
	uint16_t	oldcf;
	uint16_t	msb;

	s = oper1b;
	oldcf = cf;
#ifdef CPU_LIMIT_SHIFT_COUNT
	cnt &= 0x1F;
#endif
	switch (reg) {
			case 0: /* ROL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						if (s & 0x80) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = s << 1;
						s = s | cf;
					}

				if (cnt == 1) {
						//of = cf ^ ( (s >> 7) & 1);
					if ((s & 0x80) && cf) of = 1; else of = 0;
					} else of = 0;
				break;

			case 1: /* ROR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						cf = s & 1;
						s = (s >> 1) | (cf << 7);
					}

				if (cnt == 1) {
						of = (s >> 7) ^ ( (s >> 6) & 1);
					}
				break;

			case 2: /* RCL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						oldcf = cf;
						if (s & 0x80) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = s << 1;
						s = s | oldcf;
					}

				if (cnt == 1) {
						of = cf ^ ( (s >> 7) & 1);
					}
				break;

			case 3: /* RCR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						oldcf = cf;
						cf = s & 1;
						s = (s >> 1) | (oldcf << 7);
					}

				if (cnt == 1) {
						of = (s >> 7) ^ ( (s >> 6) & 1);
					}
				break;

			case 4:
			case 6: /* SHL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						if (s & 0x80) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = (s << 1) & 0xFF;
					}

				if ( (cnt == 1) && (cf == (s >> 7) ) ) {
						of = 0;
					}
				else {
						of = 1;
					}

				flag_szp8 ( (uint8_t) s);
				break;

			case 5: /* SHR r/m8 */
				if ( (cnt == 1) && (s & 0x80) ) {
						of = 1;
					}
				else {
						of = 0;
					}

				for (shift = 1; shift <= cnt; shift++) {
						cf = s & 1;
						s = s >> 1;
					}

				flag_szp8 ( (uint8_t) s);
				break;

			case 7: /* SAR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						msb = s & 0x80;
						cf = s & 1;
						s = (s >> 1) | msb;
					}

				of = 0;
				flag_szp8 ( (uint8_t) s);
				break;
		}

	return s & 0xFF;
}

uint16_t op_grp2_16 (uint8_t cnt) {

	uint32_t	s;
	uint32_t	shift;
	uint32_t	oldcf;
	uint32_t	msb;

	s = oper1;
	oldcf = cf;
#ifdef CPU_LIMIT_SHIFT_COUNT
	cnt &= 0x1F;
#endif
	switch (reg) {
			case 0: /* ROL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						if (s & 0x8000) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = s << 1;
						s = s | cf;
					}

				if (cnt == 1) {
						of = cf ^ ( (s >> 15) & 1);
					}
				break;

			case 1: /* ROR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						cf = s & 1;
						s = (s >> 1) | (cf << 15);
					}

				if (cnt == 1) {
						of = (s >> 15) ^ ( (s >> 14) & 1);
					}
				break;

			case 2: /* RCL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						oldcf = cf;
						if (s & 0x8000) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = s << 1;
						s = s | oldcf;
					}

				if (cnt == 1) {
						of = cf ^ ( (s >> 15) & 1);
					}
				break;

			case 3: /* RCR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						oldcf = cf;
						cf = s & 1;
						s = (s >> 1) | (oldcf << 15);
					}

				if (cnt == 1) {
						of = (s >> 15) ^ ( (s >> 14) & 1);
					}
				break;

			case 4:
			case 6: /* SHL r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						if (s & 0x8000) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						s = (s << 1) & 0xFFFF;
					}

				if ( (cnt == 1) && (cf == (s >> 15) ) ) {
						of = 0;
					}
				else {
						of = 1;
					}

				flag_szp16 ( (uint16_t) s);
				break;

			case 5: /* SHR r/m8 */
				if ( (cnt == 1) && (s & 0x8000) ) {
						of = 1;
					}
				else {
						of = 0;
					}

				for (shift = 1; shift <= cnt; shift++) {
						cf = s & 1;
						s = s >> 1;
					}

				flag_szp16 ( (uint16_t) s);
				break;

			case 7: /* SAR r/m8 */
				for (shift = 1; shift <= cnt; shift++) {
						msb = s & 0x8000;
						cf = s & 1;
						s = (s >> 1) | msb;
					}

				of = 0;
				flag_szp16 ( (uint16_t) s);
				break;
		}

	return (uint16_t) s & 0xFFFF;
}

void op_div8 (uint16_t valdiv, uint8_t divisor) {
	if (divisor == 0) {
			intcall86 (0);
			return;
		}

	if ( (valdiv / (uint16_t) divisor) > 0xFF) {
			intcall86 (0);
			return;
		}

	regs.byteregs[regah] = valdiv % (uint16_t) divisor;
	regs.byteregs[regal] = valdiv / (uint16_t) divisor;
}

void op_idiv8 (uint16_t valdiv, uint8_t divisor) {

	uint16_t	s1;
	uint16_t	s2;
	uint16_t	d1;
	uint16_t	d2;
	int	sign;

	if (divisor == 0) {
			intcall86 (0);
			return;
		}

	s1 = valdiv;
	s2 = divisor;
	sign = ( ( (s1 ^ s2) & 0x8000) != 0);
	s1 = (s1 < 0x8000) ? s1 : ( (~s1 + 1) & 0xffff);
	s2 = (s2 < 0x8000) ? s2 : ( (~s2 + 1) & 0xffff);
	d1 = s1 / s2;
	d2 = s1 % s2;
	if (d1 & 0xFF00) {
			intcall86 (0);
			return;
		}

	if (sign) {
			d1 = (~d1 + 1) & 0xff;
			d2 = (~d2 + 1) & 0xff;
		}

	regs.byteregs[regah] = (uint8_t) d2;
	regs.byteregs[regal] = (uint8_t) d1;
}

void op_grp3_8() {
	oper1 = signext (oper1b);
	oper2 = signext (oper2b);
	switch (reg) {
			case 0:
			case 1: /* TEST */
				flag_log8 (oper1b & getmem8 (segregs[regcs], ip) );
				StepIP (1);
				break;

			case 2: /* NOT */
				res8 = ~oper1b;
				break;

			case 3: /* NEG */
				res8 = (~oper1b) + 1;
				flag_sub8 (0, oper1b);
				if (res8 == 0) {
						cf = 0;
					}
				else {
						cf = 1;
					}
				break;

			case 4: /* MUL */
				temp1 = (uint32_t) oper1b * (uint32_t) regs.byteregs[regal];
				regs.wordregs[regax] = temp1 & 0xFFFF;
				flag_szp8 ( (uint8_t) temp1);
				if (regs.byteregs[regah]) {
						cf = 1;
						of = 1;
					}
				else {
						cf = 0;
						of = 0;
					}
#ifdef CPU_CLEAR_ZF_ON_MUL
				zf = 0;
#endif
				break;

			case 5: /* IMUL */
				oper1 = signext (oper1b);
				temp1 = signext (regs.byteregs[regal]);
				temp2 = oper1;
				if ( (temp1 & 0x80) == 0x80) {
						temp1 = temp1 | 0xFFFFFF00;
					}

				if ( (temp2 & 0x80) == 0x80) {
						temp2 = temp2 | 0xFFFFFF00;
					}

				temp3 = (temp1 * temp2) & 0xFFFF;
				regs.wordregs[regax] = temp3 & 0xFFFF;
				if (regs.byteregs[regah]) {
						cf = 1;
						of = 1;
					}
				else {
						cf = 0;
						of = 0;
					}
#ifdef CPU_CLEAR_ZF_ON_MUL
				zf = 0;
#endif
				break;

			case 6: /* DIV */
				op_div8 (regs.wordregs[regax], oper1b);
				break;

			case 7: /* IDIV */
				op_idiv8 (regs.wordregs[regax], oper1b);
				break;
		}
}

void op_div16 (uint32_t valdiv, uint16_t divisor) {
	if (divisor == 0) {
			intcall86 (0);
			return;
		}

	if ( (valdiv / (uint32_t) divisor) > 0xFFFF) {
			intcall86 (0);
			return;
		}

	regs.wordregs[regdx] = valdiv % (uint32_t) divisor;
	regs.wordregs[regax] = valdiv / (uint32_t) divisor;
}

void op_idiv16 (uint32_t valdiv, uint16_t divisor) {

	uint32_t	d1;
	uint32_t	d2;
	uint32_t	s1;
	uint32_t	s2;
	int	sign;

	if (divisor == 0) {
			intcall86 (0);
			return;
		}

	s1 = valdiv;
	s2 = divisor;
	s2 = (s2 & 0x8000) ? (s2 | 0xffff0000) : s2;
	sign = ( ( (s1 ^ s2) & 0x80000000) != 0);
	s1 = (s1 < 0x80000000) ? s1 : ( (~s1 + 1) & 0xffffffff);
	s2 = (s2 < 0x80000000) ? s2 : ( (~s2 + 1) & 0xffffffff);
	d1 = s1 / s2;
	d2 = s1 % s2;
	if (d1 & 0xFFFF0000) {
			intcall86 (0);
			return;
		}

	if (sign) {
			d1 = (~d1 + 1) & 0xffff;
			d2 = (~d2 + 1) & 0xffff;
		}

	regs.wordregs[regax] = d1;
	regs.wordregs[regdx] = d2;
}

void op_grp3_16() {
	switch (reg) {
			case 0:
			case 1: /* TEST */
				flag_log16 (oper1 & getmem16 (segregs[regcs], ip) );
				StepIP (2);
				break;

			case 2: /* NOT */
				res16 = ~oper1;
				break;

			case 3: /* NEG */
				res16 = (~oper1) + 1;
				flag_sub16 (0, oper1);
				if (res16) {
						cf = 1;
					}
				else {
						cf = 0;
					}
				break;

			case 4: /* MUL */
				temp1 = (uint32_t) oper1 * (uint32_t) regs.wordregs[regax];
				regs.wordregs[regax] = temp1 & 0xFFFF;
				regs.wordregs[regdx] = temp1 >> 16;
				flag_szp16 ( (uint16_t) temp1);
				if (regs.wordregs[regdx]) {
						cf = 1;
						of = 1;
					}
				else {
						cf = 0;
						of = 0;
					}
#ifdef CPU_CLEAR_ZF_ON_MUL
				zf = 0;
#endif
				break;

			case 5: /* IMUL */
				temp1 = regs.wordregs[regax];
				temp2 = oper1;
				if (temp1 & 0x8000) {
						temp1 |= 0xFFFF0000;
					}

				if (temp2 & 0x8000) {
						temp2 |= 0xFFFF0000;
					}

				temp3 = temp1 * temp2;
				regs.wordregs[regax] = temp3 & 0xFFFF;	/* into register ax */
				regs.wordregs[regdx] = temp3 >> 16;	/* into register dx */
				if (regs.wordregs[regdx]) {
						cf = 1;
						of = 1;
					}
				else {
						cf = 0;
						of = 0;
					}
#ifdef CPU_CLEAR_ZF_ON_MUL
				zf = 0;
#endif
				break;

			case 6: /* DIV */
				op_div16 ( ( (uint32_t) regs.wordregs[regdx] << 16) + regs.wordregs[regax], oper1);
				break;

			case 7: /* DIV */
				op_idiv16 ( ( (uint32_t) regs.wordregs[regdx] << 16) + regs.wordregs[regax], oper1);
				break;
		}
}

void op_grp5() {
	switch (reg) {
			case 0: /* INC Ev */
				oper2 = 1;
				tempcf = cf;
				op_add16();
				cf = tempcf;
				writerm16 (rm, res16);
				break;

			case 1: /* DEC Ev */
				oper2 = 1;
				tempcf = cf;
				op_sub16();
				cf = tempcf;
				writerm16 (rm, res16);
				break;

			case 2: /* CALL Ev */
				push (ip);
				ip = oper1;
				break;

			case 3: /* CALL Mp */
				push (segregs[regcs]);
				push (ip);
				getea (rm);
				ip = (uint16_t) read86 (ea) + (uint16_t) read86 (ea + 1) * 256;
				segregs[regcs] = (uint16_t) read86 (ea + 2) + (uint16_t) read86 (ea + 3) * 256;
				break;

			case 4: /* JMP Ev */
				ip = oper1;
				break;

			case 5: /* JMP Mp */
				getea (rm);
				ip = (uint16_t) read86 (ea) + (uint16_t) read86 (ea + 1) * 256;
				segregs[regcs] = (uint16_t) read86 (ea + 2) + (uint16_t) read86 (ea + 3) * 256;
				break;

			case 6: /* PUSH Ev */
				push (oper1);
				break;
		}
}

uint8_t dolog = 0, didintr = 0;
FILE	*logout;
uint8_t printops = 0;

#ifdef NETWORKING_ENABLED
extern void nethandler();
#endif
extern void diskhandler();
extern void readdisk (uint8_t drivenum, uint16_t dstseg, uint16_t dstoff, uint16_t cyl, uint16_t sect, uint16_t head, uint16_t sectcount);

void intcall86 (uint8_t intnum) {
	static uint16_t lastint10ax;
	uint16_t oldregax;
	didintr = 1;

	if (intnum == 0x19) didbootstrap = 1;

	switch (intnum) {
			case 0x10:
				updatedscreen = 1;
				/*if (regs.byteregs[regah]!=0x0E) {
					printf("Int 10h AX = %04X\n", regs.wordregs[regax]);
				}*/
				if ( (regs.byteregs[regah]==0x00) || (regs.byteregs[regah]==0x10) ) {
						oldregax = regs.wordregs[regax];
						vidinterrupt();
						regs.wordregs[regax] = oldregax;
						if (regs.byteregs[regah]==0x10) return;
						if (vidmode==9) return;
					}
				if ( (regs.byteregs[regah]==0x1A) && (lastint10ax!=0x0100) ) { //the 0x0100 is a cheap hack to make it not do this if DOS EDIT/QBASIC
						regs.byteregs[regal] = 0x1A;
						regs.byteregs[regbl] = 0x8;
						return;
					}
				lastint10ax = regs.wordregs[regax];
				if (regs.byteregs[regah]==0x1B) {
						regs.byteregs[regal] = 0x1B;
						segregs[reges] = 0xC800;
						regs.wordregs[regdi] = 0x0000;
						writew86(0xC8000, 0x0000);
						writew86(0xC8002, 0xC900);
						write86(0xC9000, 0x00);
						write86(0xC9001, 0x00);
						write86(0xC9002, 0x01);
						return;
					}
				break;

#ifndef DISK_CONTROLLER_ATA
			case 0x19: //bootstrap
#ifdef BENCHMARK_BIOS
				running = 0;
#endif
				if (bootdrive<255) { //read first sector of boot drive into 07C0:0000 and execute it
						regs.byteregs[regdl] = bootdrive;
						readdisk (regs.byteregs[regdl], 0x07C0, 0x0000, 0, 1, 0, 1);
						segregs[regcs] = 0x0000;
						ip = 0x7C00;
					}
				else {
						segregs[regcs] = 0xF600;	//start ROM BASIC at bootstrap if requested
						ip = 0x0000;
					}
				return;

			case 0x13:
			case 0xFD:
				diskhandler();
				return;
#endif
#ifdef NETWORKING_OLDCARD
			case 0xFC:
#ifdef NETWORKING_ENABLED
				nethandler();
#endif
				return;
#endif
		}

//	push (makeflagsword() );
#ifdef CPU_EMULATE_8080
	push (makeflagsword() | 0x7000);
#elif defined CPU_SET_HIGH_FLAGS
	push (makeflagsword() | 0xF000);
#else
	push (makeflagsword() );
#endif
	push (segregs[regcs]);
	push (ip);
	segregs[regcs] = getmem16 (0, (uint16_t) intnum * 4 + 2);
	ip = getmem16 (0, (uint16_t) intnum * 4);
	ifl = 0;
	tf = 0;
}

#if defined(NETWORKING_ENABLED)
extern struct netstruct {
	uint8_t	enabled;
	uint8_t	canrecv;
	uint16_t	pktlen;
} net;
#endif
uint64_t	frametimer = 0, didwhen = 0, didticks = 0;
uint32_t	makeupticks = 0;
extern float	timercomp;
uint64_t	timerticks = 0, realticks = 0;
uint64_t	lastcountertimer = 0, counterticks = 10000;
extern uint8_t	nextintr();
extern void	timing();

#ifdef USE_PREFETCH_QUEUE
uint8_t prefetch[6];
uint32_t prefetch_base = 0;
#endif

void exec86 (uint32_t execloops) {

	uint32_t loopcount;
	uint8_t docontinue;
	static uint16_t firstip;
	static uint16_t trap_toggle = 0;

	counterticks = (uint64_t) ( (double) timerfreq / (double) 65536.0);

	for (loopcount = 0; loopcount < execloops; loopcount++) {

			if ( (totalexec & TIMING_INTERVAL) == 0) timing();

			if (trap_toggle) {
					intcall86 (1);
				}

			if (tf) {
					trap_toggle = 1;
				}
			else {
					trap_toggle = 0;
				}

			if (!trap_toggle && (ifl && (i8259.irr & (~i8259.imr) ) ) ) {
					hltstate = 0;
					intcall86 (nextintr() );	/* get next interrupt from the i8259, if any */
				}

			if (hltstate) goto skipexecution;

			/*if ((((uint32_t)segregs[regcs] << 4) + (uint32_t)ip) == 0xFEC59) {
					//printf("Entered F000:EC59, returning to ");
					ip = pop();
					segregs[regcs] = pop();
					decodeflagsword(pop());
					//printf("%04X:%04X\n", segregs[regcs], ip);
					diskhandler();
				}*/

			reptype = 0;
			segoverride = 0;
			useseg = segregs[regds];
			docontinue = 0;
			firstip = ip;

			if ( (segregs[regcs] == 0xF000) && (ip == 0xE066) ) didbootstrap = 0; //detect if we hit the BIOS entry point to clear didbootstrap because we've rebooted

#ifdef CPU_EMULATE_8080
			if (!md) {
				segregs[regcs] = segregs[regcs] & 0xFFFF;
				ip = ip & 0xFFFF;
				savecs = segregs[regcs];
				saveip = ip;
				opcode = getmem8 (segregs[regcs], ip);
				StepIP (1);
				//fprintf(stderr, "?%X? <DE=%X, A=%X, DS=%X>\n", opcode, regs.wordregs[reg80de], regs.byteregs[reg80a], useseg);
				//if(getchar() == 'q') exit(0);
				switch (opcode) {
					case 0x00:	/* 00 NOP */
					case 0x08:	/* 08 NOP */
					case 0x10:	/* 10 NOP */
					case 0x18:	/* 18 NOP */
					case 0x20:	/* 20 NOP */
					case 0x28:	/* 28 NOP */
					case 0x30:	/* 30 NOP */
					case 0x38:	/* 38 NOP */
						break;

					case 0x01:	/* 01 LXI B Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						putreg16 (reg80bc, oper1);
						break;

					case 0x02:	/* 02 STAX B */
						putmem8 (useseg, regs.wordregs[reg80bc], regs.byteregs[reg80a]);
						break;

					case 0x03:	/* 03 INX B */
						regs.wordregs[reg80bc] ++;
						break;

					case 0x04:	/* 04 INR B */
						oldcf = cf;
						oper1b = regs.byteregs[reg80b];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80b] = res8;
						break;

					case 0x05:	/* 05 DCR B */
						oldcf = cf;
						oper1b = regs.byteregs[reg80b];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80b] = res8;
						break;

					case 0x06:	/* 06 MVI B Ib */
						regs.byteregs[reg80b] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x07:	/* 07 RLC */
						oper1b = regs.byteregs[reg80a];
						if (oper1b & 0x80) {
								cf = 1;
							}
						else {
								cf = 0;
							}

						oper1b = oper1b << 1;
						oper1b = oper1b | cf;
						regs.byteregs[reg80a] = oper1b;
						break;

					case 0x09:	/* 09 DAD B */
						oper1 = regs.wordregs[reg80hl];
						oper2 = regs.wordregs[reg80bc];
						op_80add16();
						putreg16 (reg80hl, res16);
						break;

					case 0x0A:	/* 0A LDAX B */
						regs.byteregs[reg80a] = getmem16 (useseg, regs.wordregs[reg80bc] );
						break;

					case 0x0B:	/* 0B DCX B */
						regs.wordregs[reg80bc] --;
						break;

					case 0x0C:	/* 0C INR C */
						oldcf = cf;
						oper1b = regs.byteregs[reg80c];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80c] = res8;
						break;

					case 0x0D:	/* 0D DCR C */
						oldcf = cf;
						oper1b = regs.byteregs[reg80c];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80c] = res8;
						break;

					case 0x0E:	/* 0E MVI C Ib */
						regs.byteregs[reg80c] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x0F:	/* 0F RRC */
						oper1b = regs.byteregs[reg80a];
						cf = oper1b & 1;
						oper1b = (oper1b >> 1) | (cf << 7);
						regs.byteregs[reg80a] = oper1b;
						break;

					case 0x11:	/* 11 LXI D Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						putreg16 (reg80de, oper1);
						break;

					case 0x12:	/* 12 STAX D */
						putmem8 (useseg, regs.wordregs[reg80de], regs.byteregs[reg80a]);
						break;

					case 0x13:	/* 13 INX D */
						regs.wordregs[reg80de] ++;
						break;

					case 0x14:	/* 14 INR D */
						oldcf = cf;
						oper1b = regs.byteregs[reg80d];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80d] = res8;
						break;

					case 0x15:	/* 15 DCR D */
						oldcf = cf;
						oper1b = regs.byteregs[reg80d];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80d] = res8;
						break;

					case 0x16:	/* 16 MVI D Ib */
						regs.byteregs[reg80d] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x17:	/* 17 RAL */
						oper1b = regs.byteregs[reg80a];
						oldcf = cf;
						if (oper1b & 0x80) {
								cf = 1;
							}
						else {
								cf = 0;
							}
						oper1b = oper1b << 1;
						oper1b = oper1b | oldcf;
						regs.byteregs[reg80a] = oper1b;
						break;

					case 0x19:	/* 19 DAD D */
						oper1 = regs.wordregs[reg80hl];
						oper2 = regs.wordregs[reg80de];
						op_80add16();
						putreg16 (reg80hl, res16);
						break;

					case 0x1A:	/* 1A LDAX D */
						regs.byteregs[reg80a] = getmem16 (useseg, regs.wordregs[reg80de] );
						break;

					case 0x1B:	/* 1B DCX D */
						regs.wordregs[reg80de] --;
						break;

					case 0x1C:	/* 1C INR E */
						oldcf = cf;
						oper1b = regs.byteregs[reg80e];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80e] = res8;
						break;

					case 0x1D:	/* 1D DCR E */
						oldcf = cf;
						oper1b = regs.byteregs[reg80e];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80e] = res8;
						break;

					case 0x1E:	/* 1E MVI E Ib */
						regs.byteregs[reg80e] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x1F:	/* 1F RAR */
						oper1b = regs.byteregs[reg80a];
						oldcf = cf;
						cf = oper1b & 1;
						oper1b = (oper1b >> 1) | (oldcf << 7);
						regs.byteregs[reg80a] = oper1b;
						break;

					case 0x21:	/* 21 LXI H Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						putreg16 (reg80hl, oper1);
						break;

					case 0x22:	/* 22 SHLD Ow */
						putmem16 (useseg, getmem16 (segregs[regcs], ip), regs.wordregs[reg80hl] );
						StepIP (2);
						break;

					case 0x23:	/* 13 INX H */
						regs.wordregs[reg80hl] ++;
						break;

					case 0x24:	/* 24 INR H */
						oldcf = cf;
						oper1b = regs.byteregs[reg80h];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80h] = res8;
						break;

					case 0x25:	/* 25 DCR H */
						oldcf = cf;
						oper1b = regs.byteregs[reg80h];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80h] = res8;
						break;

					case 0x26:	/* 26 MVI H Ib */
						regs.byteregs[reg80h] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x27:	/* 27 DAA */
						if ( ( (regs.byteregs[reg80a] & 0xF) > 9) || (af == 1) ) {
								oper1 = regs.byteregs[reg80a] + 6;
								regs.byteregs[reg80a] = oper1 & 255;
								if (oper1 & 0xFF00) {
										cf = 1;
									}
								else {
										cf = 0;
									}

								af = 1;
							}
						else {
								af = 0;
							}

						if ( ( (regs.byteregs[reg80a] & 0xF0) > 0x90) || (cf == 1) ) {
								regs.byteregs[reg80a] = regs.byteregs[reg80a] + 0x60;
								cf = 1;
							}
						else {
								cf = 0;
							}

						regs.byteregs[reg80a] = regs.byteregs[reg80a] & 255;
						flag_szp8 (regs.byteregs[reg80a]);
						break;

					case 0x29:	/* 29 DAD H */
						oper1 = regs.wordregs[reg80hl];
						oper2 = regs.wordregs[reg80hl];
						op_80add16();
						putreg16 (reg80hl, res16);
						break;

					case 0x2A:	/* 2A LHLD Ow */
						putreg16 (reg80hl, getmem16 (useseg, getmem16 (segregs[regcs], ip) ) );
						StepIP (2);
						break;

					case 0x2B:	/* 2B DCX H */
						regs.wordregs[reg80hl] --;
						break;

					case 0x2C:	/* 2C INR L */
						oldcf = cf;
						oper1b = regs.byteregs[reg80l];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80l] = res8;
						break;

					case 0x2D:	/* 2D DCR L */
						oldcf = cf;
						oper1b = regs.byteregs[reg80l];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80l] = res8;
						break;

					case 0x2E:	/* 2E MVI L Ib */
						regs.byteregs[reg80l] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x2F:	/* 2F CMA */
						regs.byteregs[reg80a] ^= 0xff;
						break;

					case 0x31:	/* 31 LXI SP Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						putreg16 (reg80sp, oper1);
						break;

					case 0x32:	/* 32 STA Ob */
						putmem8 (useseg, getmem16 (segregs[regcs], ip), regs.byteregs[reg80a]);
						StepIP (2);
						break;

					case 0x33:	/* 33 INX SP */
						regs.wordregs[reg80sp] ++;
						break;

					case 0x34:	/* 34 INR M */
						oldcf = cf;
						oper1b = getmem8 (useseg, regs.wordregs[reg80hl] );
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						putmem8 (useseg, regs.wordregs[reg80hl], res8);
						break;

					case 0x35:	/* 35 DCR M */
						oldcf = cf;
						oper1b = getmem8 (useseg, regs.wordregs[reg80hl] );
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						putmem8 (useseg, regs.wordregs[reg80hl], res8);
						break;

					case 0x36:	/* 36 MVI M Ib */
						putmem8 (useseg, regs.wordregs[reg80hl], getmem8 (segregs[regcs], ip) );
						StepIP (1);
						break;

					case 0x37:	/* 37 STC */
						cf = 1;
						break;

					case 0x39:	/* 29 DAD SP */
						oper1 = regs.wordregs[reg80hl];
						oper2 = regs.wordregs[reg80sp];
						op_80add16();
						putreg16 (reg80hl, res16);
						break;

					case 0x3A:	/* 2A LDA Ob */
						regs.byteregs[reg80a] = getmem8 (useseg, getmem16 (segregs[regcs], ip) );
						StepIP (2);
						break;

					case 0x3B:	/* 3B DCX SP */
						regs.wordregs[reg80sp] --;
						break;

					case 0x3C:	/* 3C INR A */
						oldcf = cf;
						oper1b = regs.byteregs[reg80a];
						oper2b = 1;
						op_80add8();
						cf = oldcf;
						regs.byteregs[reg80a] = res8;
						break;

					case 0x3D:	/* 3D DCR A */
						oldcf = cf;
						oper1b = regs.byteregs[reg80a];
						oper2b = 1;
						op_80sub8();
						cf = oldcf;
						regs.byteregs[reg80a] = res8;
						break;

					case 0x3E:	/* 3E MVI A Ib */
						regs.byteregs[reg80a] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0x3F:	/* 3F CMC */
						cf ^= 1;
						break;

					case 0x40:	/* 40 MOV B B */
						regs.byteregs[reg80b] = regs.byteregs[reg80b];
						break;

					case 0x41:	/* 41 MOV B C */
						regs.byteregs[reg80b] = regs.byteregs[reg80c];
						break;

					case 0x42:	/* 42 MOV B D */
						regs.byteregs[reg80b] = regs.byteregs[reg80d];
						break;

					case 0x43:	/* 43 MOV B E */
						regs.byteregs[reg80b] = regs.byteregs[reg80e];
						break;

					case 0x44:	/* 44 MOV B H */
						regs.byteregs[reg80b] = regs.byteregs[reg80h];
						break;

					case 0x45:	/* 45 MOV B L */
						regs.byteregs[reg80b] = regs.byteregs[reg80l];
						break;

					case 0x46:	/* 46 MOV B M */
						regs.byteregs[reg80b] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x47:	/* 47 MOV B A */
						regs.byteregs[reg80b] = regs.byteregs[reg80a];
						break;

					case 0x48:	/* 48 MOV C B */
						regs.byteregs[reg80c] = regs.byteregs[reg80b];
						break;

					case 0x49:	/* 49 MOV C C */
						regs.byteregs[reg80c] = regs.byteregs[reg80c];
						break;

					case 0x4A:	/* 4A MOV C D */
						regs.byteregs[reg80c] = regs.byteregs[reg80d];
						break;

					case 0x4B:	/* 4B MOV C E */
						regs.byteregs[reg80c] = regs.byteregs[reg80e];
						break;

					case 0x4C:	/* 4C MOV C H */
						regs.byteregs[reg80c] = regs.byteregs[reg80h];
						break;

					case 0x4D:	/* 4D MOV C L */
						regs.byteregs[reg80c] = regs.byteregs[reg80l];
						break;

					case 0x4E:	/* 4E MOV C M */
						regs.byteregs[reg80c] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x4F:	/* 4F MOV C A */
						regs.byteregs[reg80c] = regs.byteregs[reg80a];
						break;

					case 0x50:	/* 50 MOV D B */
						regs.byteregs[reg80d] = regs.byteregs[reg80b];
						break;

					case 0x51:	/* 51 MOV D C */
						regs.byteregs[reg80d] = regs.byteregs[reg80c];
						break;

					case 0x52:	/* 52 MOV D D */
						regs.byteregs[reg80d] = regs.byteregs[reg80d];
						break;

					case 0x53:	/* 53 MOV D E */
						regs.byteregs[reg80d] = regs.byteregs[reg80e];
						break;

					case 0x54:	/* 54 MOV D H */
						regs.byteregs[reg80d] = regs.byteregs[reg80h];
						break;

					case 0x55:	/* 55 MOV D L */
						regs.byteregs[reg80d] = regs.byteregs[reg80l];
						break;

					case 0x56:	/* 56 MOV D M */
						regs.byteregs[reg80d] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x57:	/* 57 MOV D A */
						regs.byteregs[reg80d] = regs.byteregs[reg80a];
						break;

					case 0x58:	/* 58 MOV E B */
						regs.byteregs[reg80e] = regs.byteregs[reg80b];
						break;

					case 0x59:	/* 59 MOV E C */
						regs.byteregs[reg80e] = regs.byteregs[reg80c];
						break;

					case 0x5A:	/* 5A MOV E D */
						regs.byteregs[reg80e] = regs.byteregs[reg80d];
						break;

					case 0x5B:	/* 5B MOV E E */
						regs.byteregs[reg80e] = regs.byteregs[reg80e];
						break;

					case 0x5C:	/* 5C MOV E H */
						regs.byteregs[reg80e] = regs.byteregs[reg80h];
						break;

					case 0x5D:	/* 5D MOV E L */
						regs.byteregs[reg80e] = regs.byteregs[reg80l];
						break;

					case 0x5E:	/* 5E MOV E M */
						regs.byteregs[reg80e] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x5F:	/* 5F MOV E A */
						regs.byteregs[reg80e] = regs.byteregs[reg80a];
						break;

					case 0x60:	/* 60 MOV H B */
						regs.byteregs[reg80h] = regs.byteregs[reg80b];
						break;

					case 0x61:	/* 61 MOV H C */
						regs.byteregs[reg80h] = regs.byteregs[reg80c];
						break;

					case 0x62:	/* 62 MOV H D */
						regs.byteregs[reg80h] = regs.byteregs[reg80d];
						break;

					case 0x63:	/* 63 MOV H E */
						regs.byteregs[reg80h] = regs.byteregs[reg80e];
						break;

					case 0x64:	/* 64 MOV H H */
						regs.byteregs[reg80h] = regs.byteregs[reg80h];
						break;

					case 0x65:	/* 65 MOV H L */
						regs.byteregs[reg80h] = regs.byteregs[reg80l];
						break;

					case 0x66:	/* 66 MOV H M */
						regs.byteregs[reg80h] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x67:	/* 67 MOV H A */
						regs.byteregs[reg80h] = regs.byteregs[reg80a];
						break;

					case 0x68:	/* 68 MOV L B */
						regs.byteregs[reg80l] = regs.byteregs[reg80b];
						break;

					case 0x69:	/* 69 MOV L C */
						regs.byteregs[reg80l] = regs.byteregs[reg80c];
						break;

					case 0x6A:	/* 6A MOV L D */
						regs.byteregs[reg80l] = regs.byteregs[reg80d];
						break;

					case 0x6B:	/* 6B MOV L E */
						regs.byteregs[reg80l] = regs.byteregs[reg80e];
						break;

					case 0x6C:	/* 6C MOV L H */
						regs.byteregs[reg80l] = regs.byteregs[reg80h];
						break;

					case 0x6D:	/* 6D MOV L L */
						regs.byteregs[reg80l] = regs.byteregs[reg80l];
						break;

					case 0x6E:	/* 6E MOV L M */
						regs.byteregs[reg80l] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x6F:	/* 6F MOV L A */
						regs.byteregs[reg80l] = regs.byteregs[reg80a];
						break;

					case 0x70:	/* 70 MOV M B */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80b]);
						break;

					case 0x71:	/* 71 MOV M C */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80c]);
						break;

					case 0x72:	/* 72 MOV M D */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80d]);
						break;

					case 0x73:	/* 73 MOV M E */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80e]);
						break;

					case 0x74:	/* 74 MOV M H */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80h]);
						break;

					case 0x75:	/* 75 MOV M L */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80l]);
						break;

					case 0x76:	/* 76 HLT */
						ip--;
						break;

					case 0x77:	/* 77 MOV M A */
						putmem8 (useseg, regs.wordregs[reg80hl], regs.byteregs[reg80a]);
						break;

					case 0x78:	/* 78 MOV A B */
						regs.byteregs[reg80a] = regs.byteregs[reg80b];
						break;

					case 0x79:	/* 79 MOV A C */
						regs.byteregs[reg80a] = regs.byteregs[reg80c];
						break;

					case 0x7A:	/* 7A MOV A D */
						regs.byteregs[reg80a] = regs.byteregs[reg80d];
						break;

					case 0x7B:	/* 7B MOV A E */
						regs.byteregs[reg80a] = regs.byteregs[reg80e];
						break;

					case 0x7C:	/* 7C MOV A H */
						regs.byteregs[reg80a] = regs.byteregs[reg80h];
						break;

					case 0x7D:	/* 7D MOV A L */
						regs.byteregs[reg80a] = regs.byteregs[reg80l];
						break;

					case 0x7E:	/* 7E MOV A M */
						regs.byteregs[reg80a] = getmem8 (useseg, regs.wordregs[reg80hl]);
						break;

					case 0x7F:	/* 7F MOV A A */
						regs.byteregs[reg80a] = regs.byteregs[reg80a];
						break;

					case 0x80:	/* 80 ADD B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x81:	/* 81 ADD C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x82:	/* 82 ADD D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x83:	/* 83 ADD E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x84:	/* 84 ADD H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x85:	/* 85 ADD L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x86:	/* 86 ADD M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x87:	/* 87 ADD A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x88:	/* 88 ADC B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x89:	/* 89 ADC C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8A:	/* 8A ADC D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8B:	/* 8B ADC E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8C:	/* 8C ADC H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8D:	/* 8D ADC L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8E:	/* 8E ADC M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x8F:	/* 8F ADC A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x90:	/* 90 SUB B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x91:	/* 91 SUB C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x92:	/* 92 SUB D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x93:	/* 93 SUB E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x94:	/* 94 SUB H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x95:	/* 95 SUB L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x96:	/* 96 SUB M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x97:	/* 97 SUB A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x98:	/* 98 SBB B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x99:	/* 99 SBB C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9A:	/* 9A SBB D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9B:	/* 9B SBB E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9C:	/* 9C SBB H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9D:	/* 9D SBB L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9E:	/* 9E SBB M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0x9F:	/* 9F SBB A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA0:	/* A0 ANA B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA1:	/* A1 ANA C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA2:	/* A2 ANA D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA3:	/* A3 ANA E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA4:	/* A4 ANA H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA5:	/* A5 ANA L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA6:	/* A6 ANA M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA7:	/* A7 ANA A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA8:	/* A8 XRA B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xA9:	/* A9 XRA C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAA:	/* AA XRA D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAB:	/* AB XRA E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAC:	/* AC XRA H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAD:	/* AD XRA L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAE:	/* AE XRA M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xAF:	/* AF XRA A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB0:	/* B0 ANA B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB1:	/* B1 ANA C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB2:	/* B2 ANA D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB3:	/* B3 ANA E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB4:	/* B4 ANA H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB5:	/* B5 ANA L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB6:	/* B6 ANA M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB7:	/* B7 ANA A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xB8:	/* B8 CMP B */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80b];
						op_80sub8();
						break;

					case 0xB9:	/* B9 CMP C */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80c];
						op_80sub8();
						break;

					case 0xBA:	/* BA CMP D */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80d];
						op_80sub8();
						break;

					case 0xBB:	/* BB CMP E */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80e];
						op_80sub8();
						break;

					case 0xBC:	/* BC CMP H */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80h];
						op_80sub8();
						break;

					case 0xBD:	/* BD CMP L */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80l];
						op_80sub8();
						break;

					case 0xBE:	/* BE CMP M */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (useseg, regs.wordregs[reg80hl] );
						op_80sub8();
						break;

					case 0xBF:	/* BF CMP A */
						oper1b = regs.byteregs[reg80a];
						oper2b = regs.byteregs[reg80a];
						op_80sub8();
						break;

					case 0xC0:	/* C0 RNZ */
						if (!zf) {
								ip = pop80();
							}
						break;

					case 0xC1:	/* C1 POP B */
						regs.wordregs[reg80bc] = pop80();
						break;

					case 0xC2:	/* C2 JNZ Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!zf) {
								ip = temp16;
							}
						break;

					case 0xC3:	/* C3 JMP Iw */
					case 0xCB:	/* CB JMP Iw */
						ip = getmem16 (segregs[regcs], ip);
						break;

					case 0xC4:	/* C4 CNZ Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!zf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xC5:	/* C5 PUSH B */
						push80(regs.wordregs[reg80bc]);
						break;

					case 0xC6:	/* C6 ADI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80add8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xC7: /* C7 RST 0 */
						push80 (ip);
						ip = 0x0000;
						break;

					case 0xC8:	/* C8 RZ */
						if (zf) {
								ip = pop80();
							}
						break;

					case 0xC9:	/* C9 RET */
					case 0xD9:	/* D9 RET */
						ip = pop80();
						break;

					case 0xCA:	/* CA JZ Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (zf) {
								ip = temp16;
							}
						break;

					case 0xCC:	/* CC CZ Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (zf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xCD: /* CD CALL Iw */
					case 0xDD: /* CD CALL Iw */
					case 0xFD: /* CD CALL Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						push80 (ip);
						ip = temp16;
						break;

					case 0xCE:	/* CE ACI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80adc8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xCF: /* CF RST 1 */
						push80 (ip);
						ip = 0x0008;
						break;

					case 0xD0:	/* C0 RNC */
						if (!cf) {
								ip = pop80();
							}
						break;

					case 0xD1:	/* D1 POP D */
						regs.wordregs[reg80de] = pop80();
						break;

					case 0xD2:	/* D2 JNC Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!cf) {
								ip = temp16;
							}
						break;

					case 0xD3:	/* D3 OUT Ib */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						portout (oper1b, regs.byteregs[reg80a]);
						break;

					case 0xD4:	/* D4 CNC Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!cf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xD5:	/* D5 PUSH D */
						push80(regs.wordregs[reg80de]);
						break;

					case 0xD6:	/* D6 SUI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xD7: /* D7 RST 2 */
						push80 (ip);
						ip = 0x0010;
						break;

					case 0xD8:	/* D8 RZ */
						if (cf) {
								ip = pop80();
							}
						break;

					case 0xDA:	/* DA JC Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (cf) {
								ip = temp16;
							}
						break;

					case 0xDB:	/* DB IN Ib */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						regs.byteregs[reg80a] = (uint8_t) portin (oper1b);
						break;

					case 0xDC:	/* DC CC Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (cf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xDE:	/* DE SBI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80sbb8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xDF: /* CF RST 3 */
						push80 (ip);
						ip = 0x0018;
						break;

					case 0xE0:	/* E0 RPO */
						if (!pf) {
								ip = pop80();
							}
						break;

					case 0xE1:	/* E1 POP H */
						regs.wordregs[reg80hl] = pop80();
						break;

					case 0xE2:	/* E2 JPO Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!pf) {
								ip = temp16;
							}
						break;

					case 0xE3:	/* E3 XTHL */
						oper1 = getmem16 (segregs[regds], regs.wordregs[reg80sp] );
						putmem16 (segregs[regds], regs.wordregs[reg80sp], regs.wordregs[reg80hl] );
						putreg16 (reg80hl, oper1);
						break;

					case 0xE4:	/* E4 CPO Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!pf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xE5:	/* E5 PUSH H */
						push80(regs.wordregs[reg80hl]);
						break;

					case 0xE6:	/* E6 ANI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80and8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xE7: /* E7 RST 4 */
						push80 (ip);
						ip = 0x0020;
						break;

					case 0xE8:	/* E8 RPE */
						if (pf) {
								ip = pop80();
							}
						break;

					case 0xE9:	/* E9 PCHL */
						ip = regs.wordregs[reg80hl];
						break;

					case 0xEA:	/* EA JPE Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (pf) {
								ip = temp16;
							}
						break;

					case 0xEB:	/* EB XCHG */
						oper1 = regs.wordregs[reg80de];
						putreg16 (reg80de, regs.wordregs[reg80hl] );
						putreg16 (reg80hl, oper1);
						break;

					case 0xEC:	/* EC CPE Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (pf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xED: /* ED CALL Iw */
						temp16 = getmem8 (segregs[regcs], ip);
						switch (temp16) {
								case 0xED:	/* ED ED CALLN Ib */
									StepIP (1);
									oper1b = getmem8 (segregs[regcs], ip);
									StepIP (1);
									intcall86 (oper1b);
									md = 1;
									break;

								case 0xFD:	/* ED FD RETEM */
									ip = pop();
									segregs[regcs] = pop();
									decodeflagsword (pop() );
									mdenable = 0;
									break;

								default:
									temp16 = getmem16 (segregs[regcs], ip);
									StepIP (2);
									push80 (ip);
									ip = temp16;
									break;
						}
						break;

					case 0xEE:	/* EE XRI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80xor8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xEF: /* CF RST 5 */
						push80 (ip);
						ip = 0x0028;
						break;

					case 0xF0:	/* F0 RP */
						if (!sf) {
								ip = pop80();
							}
						break;

					case 0xF1:	/* F1 POP PSW */
						temp16 = pop80();
						decodeflagsword ( (makeflagsword() & 0xFF00) | (temp16 & 0x00FF) );
						regs.byteregs[reg80a] = (temp16 & 0xFF00) >> 8;
						break;

					case 0xF2:	/* F2 JP Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!sf) {
								ip = temp16;
							}
						break;

					case 0xF3:	/* F3 DI */
						ifl = 0;
						break;

					case 0xF4:	/* F4 CP Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (!sf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xF5:	/* F5 PUSH PSW */
						temp16 = (makeflagsword() & 0x00FF) | (regs.byteregs[reg80a] << 8);
						push80(temp16);
						break;

					case 0xF6:	/* F6 ORI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80or8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xF7: /* F7 RST 6 */
						push80 (ip);
						ip = 0x0030;
						break;

					case 0xF8:	/* F8 RM */
						if (sf) {
								ip = pop80();
							}
						break;

					case 0xF9:	/* FB SPHL */
						putreg16 (reg80sp, regs.wordregs[reg80hl] );
						break;

					case 0xFA:	/* FA JM Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (sf) {
								ip = temp16;
							}
						break;

					case 0xFB:	/* FD EI */
						ifl = 1;
						break;

					case 0xFC:	/* FC CM Iw */
						temp16 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if (sf) {
								push80 (ip);
								ip = temp16;
							}
						break;

					case 0xFE:	/* FE CPI Ib */
						oper1b = regs.byteregs[reg80a];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_80sub8();
						regs.byteregs[reg80a] = res8;
						break;

					case 0xFF: /* FF RST 7 */
						push80 (ip);
						ip = 0x0038;
						break;
				}
				totalexec++;
			}
			else
#endif /* CPU_EMULATE_8080 */
			{
			while (!docontinue) {
					segregs[regcs] = segregs[regcs] & 0xFFFF;
					ip = ip & 0xFFFF;
					savecs = segregs[regcs];
					saveip = ip;
#ifdef USE_PREFETCH_QUEUE
					ea = segbase(savecs) + (uint32_t)saveip;
					if ( (ea < prefetch_base) || (ea > (prefetch_base + 5)) ) {
							memcpy (&prefetch[0], &RAM[ea], 6);
							prefetch_base = ea;
						}
					opcode = prefetch[ea - prefetch_base];
#else
					opcode = getmem8 (segregs[regcs], ip);
#endif
					StepIP (1);

					switch (opcode) {
								/* segment prefix check */
							case 0x2E:	/* segment segregs[regcs] */
								useseg = segregs[regcs];
								segoverride = 1;
								break;

							case 0x3E:	/* segment segregs[regds] */
								useseg = segregs[regds];
								segoverride = 1;
								break;

							case 0x26:	/* segment segregs[reges] */
								useseg = segregs[reges];
								segoverride = 1;
								break;

							case 0x36:	/* segment segregs[regss] */
								useseg = segregs[regss];
								segoverride = 1;
								break;

								/* repetition prefix check */
							case 0xF3:	/* REP/REPE/REPZ */
								reptype = 1;
								break;

							case 0xF2:	/* REPNE/REPNZ */
								reptype = 2;
								break;

							default:
								docontinue = 1;
								break;
						}
				}

			totalexec++;

			switch (opcode) {
					case 0x0:	/* 00 ADD Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_add8();
						writerm8 (rm, res8);
						break;

					case 0x1:	/* 01 ADD Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_add16();
						writerm16 (rm, res16);
						break;

					case 0x2:	/* 02 ADD Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_add8();
						putreg8 (reg, res8);
						break;

					case 0x3:	/* 03 ADD Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_add16();
						putreg16 (reg, res16);
						break;

					case 0x4:	/* 04 ADD regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_add8();
						regs.byteregs[regal] = res8;
						break;

					case 0x5:	/* 05 ADD eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_add16();
						regs.wordregs[regax] = res16;
						break;

					case 0x6:	/* 06 PUSH segregs[reges] */
						push (segregs[reges]);
						break;

					case 0x7:	/* 07 POP segregs[reges] */
						segregs[reges] = pop();
						break;

					case 0x8:	/* 08 OR Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_or8();
						writerm8 (rm, res8);
						break;

					case 0x9:	/* 09 OR Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_or16();
						writerm16 (rm, res16);
						break;

					case 0xA:	/* 0A OR Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_or8();
						putreg8 (reg, res8);
						break;

					case 0xB:	/* 0B OR Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_or16();
						if ( (oper1 == 0xF802) && (oper2 == 0xF802) ) {
								sf = 0;	/* cheap hack to make Wolf 3D think we're a 286 so it plays */
							}

						putreg16 (reg, res16);
						break;

					case 0xC:	/* 0C OR regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_or8();
						regs.byteregs[regal] = res8;
						break;

					case 0xD:	/* 0D OR eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_or16();
						regs.wordregs[regax] = res16;
						break;

					case 0xE:	/* 0E PUSH segregs[regcs] */
						push (segregs[regcs]);
						break;

#ifdef CPU_ALLOW_POP_CS //only the 8086/8088 does this.
					case 0xF: //0F POP CS
						segregs[regcs] = pop();
						break;
#elif defined CPU_EMULATE_8080
					case 0xF: //0F POP CS
						oper1b = getmem8 (segregs[regcs], ip);
						switch (oper1b) {
								case 0xFF:	/* 0F FF BRKEM Ib */
									StepIP (1);
									oper2b = getmem8 (segregs[regcs], ip);
									StepIP (1);
									intcall86 (oper2b);
									mdenable = 1;
									md = 0;
									break;

								default:
#ifdef CPU_ALLOW_ILLEGAL_OP_EXCEPTION
									intcall86 (6); /* trip invalid opcode exception (this occurs on the 80186+, 8086/8088 CPUs treat them as NOPs. */
												   /* technically they aren't exactly like NOPs in most cases, but for our pursoses, that's accurate enough. */
#endif
									if (verbose) {
											printf ("Illegal opcode: %02X %02X /%X @ %04X:%04X\n", getmem8(savecs, saveip), getmem8(savecs, saveip+1), (getmem8(savecs, saveip+2) >> 3) & 7, savecs, saveip);
										}
									break;
						}
						break;
#endif

					case 0x10:	/* 10 ADC Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_adc8();
						writerm8 (rm, res8);
						break;

					case 0x11:	/* 11 ADC Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_adc16();
						writerm16 (rm, res16);
						break;

					case 0x12:	/* 12 ADC Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_adc8();
						putreg8 (reg, res8);
						break;

					case 0x13:	/* 13 ADC Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_adc16();
						putreg16 (reg, res16);
						break;

					case 0x14:	/* 14 ADC regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_adc8();
						regs.byteregs[regal] = res8;
						break;

					case 0x15:	/* 15 ADC eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_adc16();
						regs.wordregs[regax] = res16;
						break;

					case 0x16:	/* 16 PUSH segregs[regss] */
						push (segregs[regss]);
						break;

					case 0x17:	/* 17 POP segregs[regss] */
						segregs[regss] = pop();
						break;

					case 0x18:	/* 18 SBB Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_sbb8();
						writerm8 (rm, res8);
						break;

					case 0x19:	/* 19 SBB Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_sbb16();
						writerm16 (rm, res16);
						break;

					case 0x1A:	/* 1A SBB Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_sbb8();
						putreg8 (reg, res8);
						break;

					case 0x1B:	/* 1B SBB Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_sbb16();
						putreg16 (reg, res16);
						break;

					case 0x1C:	/* 1C SBB regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_sbb8();
						regs.byteregs[regal] = res8;
						break;

					case 0x1D:	/* 1D SBB eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_sbb16();
						regs.wordregs[regax] = res16;
						break;

					case 0x1E:	/* 1E PUSH segregs[regds] */
						push (segregs[regds]);
						break;

					case 0x1F:	/* 1F POP segregs[regds] */
						segregs[regds] = pop();
						break;

					case 0x20:	/* 20 AND Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_and8();
						writerm8 (rm, res8);
						break;

					case 0x21:	/* 21 AND Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_and16();
						writerm16 (rm, res16);
						break;

					case 0x22:	/* 22 AND Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_and8();
						putreg8 (reg, res8);
						break;

					case 0x23:	/* 23 AND Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_and16();
						putreg16 (reg, res16);
						break;

					case 0x24:	/* 24 AND regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_and8();
						regs.byteregs[regal] = res8;
						break;

					case 0x25:	/* 25 AND eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_and16();
						regs.wordregs[regax] = res16;
						break;

					case 0x27:	/* 27 DAA */
						if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
								oper1 = regs.byteregs[regal] + 6;
								regs.byteregs[regal] = oper1 & 255;
								if (oper1 & 0xFF00) {
										cf = 1;
									}
								else {
										cf = 0;
									}

								af = 1;
							}
						else {
								//af = 0;
							}

						if ( (regs.byteregs[regal]  > 0x9F) || (cf == 1) ) {
								regs.byteregs[regal] = regs.byteregs[regal] + 0x60;
								cf = 1;
							}
						else {
								//cf = 0;
							}

						regs.byteregs[regal] = regs.byteregs[regal] & 255;
						flag_szp8 (regs.byteregs[regal]);
						break;

					case 0x28:	/* 28 SUB Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_sub8();
						writerm8 (rm, res8);
						break;

					case 0x29:	/* 29 SUB Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_sub16();
						writerm16 (rm, res16);
						break;

					case 0x2A:	/* 2A SUB Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_sub8();
						putreg8 (reg, res8);
						break;

					case 0x2B:	/* 2B SUB Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_sub16();
						putreg16 (reg, res16);
						break;

					case 0x2C:	/* 2C SUB regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_sub8();
						regs.byteregs[regal] = res8;
						break;

					case 0x2D:	/* 2D SUB eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_sub16();
						regs.wordregs[regax] = res16;
						break;

					case 0x2F:	/* 2F DAS */
						if ( ( (regs.byteregs[regal] & 15) > 9) || (af == 1) ) {
								oper1 = regs.byteregs[regal] - 6;
								regs.byteregs[regal] = oper1 & 255;
								if (oper1 & 0xFF00) {
										cf = 1;
									}
								else {
										cf = 0;
									}

								af = 1;
							}
						else {
								af = 0;
							}

						if ( ( (regs.byteregs[regal] & 0xF0) > 0x90) || (cf == 1) ) {
								regs.byteregs[regal] = regs.byteregs[regal] - 0x60;
								cf = 1;
							}
						else {
								cf = 0;
							}

						flag_szp8 (regs.byteregs[regal]);
						break;

					case 0x30:	/* 30 XOR Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						op_xor8();
						writerm8 (rm, res8);
						break;

					case 0x31:	/* 31 XOR Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						op_xor16();
						writerm16 (rm, res16);
						break;

					case 0x32:	/* 32 XOR Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						op_xor8();
						putreg8 (reg, res8);
						break;

					case 0x33:	/* 33 XOR Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						op_xor16();
						putreg16 (reg, res16);
						break;

					case 0x34:	/* 34 XOR regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						op_xor8();
						regs.byteregs[regal] = res8;
						break;

					case 0x35:	/* 35 XOR eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						op_xor16();
						regs.wordregs[regax] = res16;
						break;

					case 0x37:	/* 37 AAA ASCII */
						if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
								regs.byteregs[regal] = regs.byteregs[regal] + 6;
								regs.byteregs[regah] = regs.byteregs[regah] + 1;
								af = 1;
								cf = 1;
							}
						else {
								af = 0;
								cf = 0;
							}

						regs.byteregs[regal] = regs.byteregs[regal] & 0xF;
						break;

					case 0x38:	/* 38 CMP Eb Gb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getreg8 (reg);
						flag_sub8 (oper1b, oper2b);
						break;

					case 0x39:	/* 39 CMP Ev Gv */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getreg16 (reg);
						flag_sub16 (oper1, oper2);
						break;

					case 0x3A:	/* 3A CMP Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						flag_sub8 (oper1b, oper2b);
						break;

					case 0x3B:	/* 3B CMP Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						flag_sub16 (oper1, oper2);
						break;

					case 0x3C:	/* 3C CMP regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						flag_sub8 (oper1b, oper2b);
						break;

					case 0x3D:	/* 3D CMP eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						flag_sub16 (oper1, oper2);
						break;

					case 0x3F:	/* 3F AAS ASCII */
						if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
								regs.byteregs[regal] = regs.byteregs[regal] - 6;
								regs.byteregs[regah] = regs.byteregs[regah] - 1;
								af = 1;
								cf = 1;
							}
						else {
								af = 0;
								cf = 0;
							}

						regs.byteregs[regal] = regs.byteregs[regal] & 0xF;
						break;

					case 0x40:	/* 40 INC eAX */
						oldcf = cf;
						oper1 = regs.wordregs[regax];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regax] = res16;
						break;

					case 0x41:	/* 41 INC eCX */
						oldcf = cf;
						oper1 = regs.wordregs[regcx];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regcx] = res16;
						break;

					case 0x42:	/* 42 INC eDX */
						oldcf = cf;
						oper1 = regs.wordregs[regdx];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regdx] = res16;
						break;

					case 0x43:	/* 43 INC eBX */
						oldcf = cf;
						oper1 = regs.wordregs[regbx];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regbx] = res16;
						break;

					case 0x44:	/* 44 INC eSP */
						oldcf = cf;
						oper1 = regs.wordregs[regsp];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regsp] = res16;
						break;

					case 0x45:	/* 45 INC eBP */
						oldcf = cf;
						oper1 = regs.wordregs[regbp];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regbp] = res16;
						break;

					case 0x46:	/* 46 INC eSI */
						oldcf = cf;
						oper1 = regs.wordregs[regsi];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regsi] = res16;
						break;

					case 0x47:	/* 47 INC eDI */
						oldcf = cf;
						oper1 = regs.wordregs[regdi];
						oper2 = 1;
						op_add16();
						cf = oldcf;
						regs.wordregs[regdi] = res16;
						break;

					case 0x48:	/* 48 DEC eAX */
						oldcf = cf;
						oper1 = regs.wordregs[regax];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regax] = res16;
						break;

					case 0x49:	/* 49 DEC eCX */
						oldcf = cf;
						oper1 = regs.wordregs[regcx];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regcx] = res16;
						break;

					case 0x4A:	/* 4A DEC eDX */
						oldcf = cf;
						oper1 = regs.wordregs[regdx];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regdx] = res16;
						break;

					case 0x4B:	/* 4B DEC eBX */
						oldcf = cf;
						oper1 = regs.wordregs[regbx];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regbx] = res16;
						break;

					case 0x4C:	/* 4C DEC eSP */
						oldcf = cf;
						oper1 = regs.wordregs[regsp];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regsp] = res16;
						break;

					case 0x4D:	/* 4D DEC eBP */
						oldcf = cf;
						oper1 = regs.wordregs[regbp];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regbp] = res16;
						break;

					case 0x4E:	/* 4E DEC eSI */
						oldcf = cf;
						oper1 = regs.wordregs[regsi];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regsi] = res16;
						break;

					case 0x4F:	/* 4F DEC eDI */
						oldcf = cf;
						oper1 = regs.wordregs[regdi];
						oper2 = 1;
						op_sub16();
						cf = oldcf;
						regs.wordregs[regdi] = res16;
						break;

					case 0x50:	/* 50 PUSH eAX */
						push (regs.wordregs[regax]);
						break;

					case 0x51:	/* 51 PUSH eCX */
						push (regs.wordregs[regcx]);
						break;

					case 0x52:	/* 52 PUSH eDX */
						push (regs.wordregs[regdx]);
						break;

					case 0x53:	/* 53 PUSH eBX */
						push (regs.wordregs[regbx]);
						break;

					case 0x54:	/* 54 PUSH eSP */
#ifdef USE_286_STYLE_PUSH_SP
						push (regs.wordregs[regsp]);
#else
						push (regs.wordregs[regsp] - 2);
#endif
						break;

					case 0x55:	/* 55 PUSH eBP */
						push (regs.wordregs[regbp]);
						break;

					case 0x56:	/* 56 PUSH eSI */
						push (regs.wordregs[regsi]);
						break;

					case 0x57:	/* 57 PUSH eDI */
						push (regs.wordregs[regdi]);
						break;

					case 0x58:	/* 58 POP eAX */
						regs.wordregs[regax] = pop();
						break;

					case 0x59:	/* 59 POP eCX */
						regs.wordregs[regcx] = pop();
						break;

					case 0x5A:	/* 5A POP eDX */
						regs.wordregs[regdx] = pop();
						break;

					case 0x5B:	/* 5B POP eBX */
						regs.wordregs[regbx] = pop();
						break;

					case 0x5C:	/* 5C POP eSP */
						regs.wordregs[regsp] = pop();
						break;

					case 0x5D:	/* 5D POP eBP */
						regs.wordregs[regbp] = pop();
						break;

					case 0x5E:	/* 5E POP eSI */
						regs.wordregs[regsi] = pop();
						break;

					case 0x5F:	/* 5F POP eDI */
						regs.wordregs[regdi] = pop();
						break;

#ifndef CPU_8086
					case 0x60:	/* 60 PUSHA (80186+) */
						oldsp = regs.wordregs[regsp];
						push (regs.wordregs[regax]);
						push (regs.wordregs[regcx]);
						push (regs.wordregs[regdx]);
						push (regs.wordregs[regbx]);
						push (oldsp);
						push (regs.wordregs[regbp]);
						push (regs.wordregs[regsi]);
						push (regs.wordregs[regdi]);
						break;

					case 0x61:	/* 61 POPA (80186+) */
						regs.wordregs[regdi] = pop();
						regs.wordregs[regsi] = pop();
						regs.wordregs[regbp] = pop();
						dummy = pop();
						regs.wordregs[regbx] = pop();
						regs.wordregs[regdx] = pop();
						regs.wordregs[regcx] = pop();
						regs.wordregs[regax] = pop();
						break;

					case 0x62: /* 62 BOUND Gv, Ev (80186+) */
						modregrm();
						getea (rm);
						if (signext32 (getreg16 (reg) ) < signext32 ( getmem16 (ea >> 4, ea & 15) ) ) {
								intcall86 (5); //bounds check exception
							}
						else {
								ea += 2;
								if (signext32 (getreg16 (reg) ) > signext32 ( getmem16 (ea >> 4, ea & 15) ) ) {
										intcall86(5); //bounds check exception
									}
							}
						break;

					case 0x68:	/* 68 PUSH Iv (80186+) */
						push (getmem16 (segregs[regcs], ip) );
						StepIP (2);
						break;

					case 0x69:	/* 69 IMUL Gv Ev Iv (80186+) */
						modregrm();
						temp1 = readrm16 (rm);
						temp2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						if ( (temp1 & 0x8000L) == 0x8000L) {
								temp1 = temp1 | 0xFFFF0000L;
							}

						if ( (temp2 & 0x8000L) == 0x8000L) {
								temp2 = temp2 | 0xFFFF0000L;
							}

						temp3 = temp1 * temp2;
						putreg16 (reg, temp3 & 0xFFFFL);
						if (temp3 & 0xFFFF0000L) {
								cf = 1;
								of = 1;
							}
						else {
								cf = 0;
								of = 0;
							}
						break;

					case 0x6A:	/* 6A PUSH Ib (80186+) */
						push (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						break;

					case 0x6B:	/* 6B IMUL Gv Eb Ib (80186+) */
						modregrm();
						temp1 = readrm16 (rm);
						temp2 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if ( (temp1 & 0x8000L) == 0x8000L) {
								temp1 = temp1 | 0xFFFF0000L;
							}

						if ( (temp2 & 0x8000L) == 0x8000L) {
								temp2 = temp2 | 0xFFFF0000L;
							}

						temp3 = temp1 * temp2;
						putreg16 (reg, temp3 & 0xFFFFL);
						if (temp3 & 0xFFFF0000L) {
								cf = 1;
								of = 1;
							}
						else {
								cf = 0;
								of = 0;
							}
						break;

					case 0x6C:	/* 6E INSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem8 (useseg, regs.wordregs[regsi], portin (regs.wordregs[regdx]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0x6D:	/* 6F INSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem16 (useseg, regs.wordregs[regsi], portin16 (regs.wordregs[regdx]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0x6E:	/* 6E OUTSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						portout (regs.wordregs[regdx], getmem8 (useseg, regs.wordregs[regsi]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0x6F:	/* 6F OUTSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						portout16 (regs.wordregs[regdx], getmem16 (useseg, regs.wordregs[regsi]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;
#endif

					case 0x70:	/* 70 JO Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (of) {
								ip = ip + temp16;
							}
						break;

					case 0x71:	/* 71 JNO Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!of) {
								ip = ip + temp16;
							}
						break;

					case 0x72:	/* 72 JB Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (cf) {
								ip = ip + temp16;
							}
						break;

					case 0x73:	/* 73 JNB Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!cf) {
								ip = ip + temp16;
							}
						break;

					case 0x74:	/* 74 JZ Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (zf) {
								ip = ip + temp16;
							}
						break;

					case 0x75:	/* 75 JNZ Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!zf) {
								ip = ip + temp16;
							}
						break;

					case 0x76:	/* 76 JBE Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (cf || zf) {
								ip = ip + temp16;
							}
						break;

					case 0x77:	/* 77 JA Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!cf && !zf) {
								ip = ip + temp16;
							}
						break;

					case 0x78:	/* 78 JS Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (sf) {
								ip = ip + temp16;
							}
						break;

					case 0x79:	/* 79 JNS Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!sf) {
								ip = ip + temp16;
							}
						break;

					case 0x7A:	/* 7A JPE Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (pf) {
								ip = ip + temp16;
							}
						break;

					case 0x7B:	/* 7B JPO Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!pf) {
								ip = ip + temp16;
							}
						break;

					case 0x7C:	/* 7C JL Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (sf != of) {
								ip = ip + temp16;
							}
						break;

					case 0x7D:	/* 7D JGE Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (sf == of) {
								ip = ip + temp16;
							}
						break;

					case 0x7E:	/* 7E JLE Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if ( (sf != of) || zf) {
								ip = ip + temp16;
							}
						break;

					case 0x7F:	/* 7F JG Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!zf && (sf == of) ) {
								ip = ip + temp16;
							}
						break;

					case 0x80:
					case 0x82:	/* 80/82 GRP1 Eb Ib */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						switch (reg) {
								case 0:
									op_add8();
									break;
								case 1:
									op_or8();
									break;
								case 2:
									op_adc8();
									break;
								case 3:
									op_sbb8();
									break;
								case 4:
									op_and8();
									break;
								case 5:
									op_sub8();
									break;
								case 6:
									op_xor8();
									break;
								case 7:
									flag_sub8 (oper1b, oper2b);
									break;
								default:
									break;	/* to avoid compiler warnings */
							}

						if (reg < 7) {
								writerm8 (rm, res8);
							}
						break;

					case 0x81:	/* 81 GRP1 Ev Iv */
					case 0x83:	/* 83 GRP1 Ev Ib */
						modregrm();
						oper1 = readrm16 (rm);
						if (opcode == 0x81) {
								oper2 = getmem16 (segregs[regcs], ip);
								StepIP (2);
							}
						else {
								oper2 = signext (getmem8 (segregs[regcs], ip) );
								StepIP (1);
							}

						switch (reg) {
								case 0:
									op_add16();
									break;
								case 1:
									op_or16();
									break;
								case 2:
									op_adc16();
									break;
								case 3:
									op_sbb16();
									break;
								case 4:
									op_and16();
									break;
								case 5:
									op_sub16();
									break;
								case 6:
									op_xor16();
									break;
								case 7:
									flag_sub16 (oper1, oper2);
									break;
								default:
									break;	/* to avoid compiler warnings */
							}

						if (reg < 7) {
								writerm16 (rm, res16);
							}
						break;

					case 0x84:	/* 84 TEST Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						oper2b = readrm8 (rm);
						flag_log8 (oper1b & oper2b);
						break;

					case 0x85:	/* 85 TEST Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						oper2 = readrm16 (rm);
						flag_log16 (oper1 & oper2);
						break;

					case 0x86:	/* 86 XCHG Gb Eb */
						modregrm();
						oper1b = getreg8 (reg);
						putreg8 (reg, readrm8 (rm) );
						writerm8 (rm, oper1b);
						break;

					case 0x87:	/* 87 XCHG Gv Ev */
						modregrm();
						oper1 = getreg16 (reg);
						putreg16 (reg, readrm16 (rm) );
						writerm16 (rm, oper1);
						break;

					case 0x88:	/* 88 MOV Eb Gb */
						modregrm();
						writerm8 (rm, getreg8 (reg) );
						break;

					case 0x89:	/* 89 MOV Ev Gv */
						modregrm();
						writerm16 (rm, getreg16 (reg) );
						break;

					case 0x8A:	/* 8A MOV Gb Eb */
						modregrm();
						putreg8 (reg, readrm8 (rm) );
						break;

					case 0x8B:	/* 8B MOV Gv Ev */
						modregrm();
						putreg16 (reg, readrm16 (rm) );
						break;

					case 0x8C:	/* 8C MOV Ew Sw */
						modregrm();
						writerm16 (rm, getsegreg (reg) );
						break;

					case 0x8D:	/* 8D LEA Gv M */
						modregrm();
						getea (rm);
						putreg16 (reg, ea - segbase (useseg) );
						break;

					case 0x8E:	/* 8E MOV Sw Ew */
						modregrm();
						putsegreg (reg, readrm16 (rm) );
						break;

					case 0x8F:	/* 8F POP Ev */
						modregrm();
						writerm16 (rm, pop() );
						break;

					case 0x90:	/* 90 NOP */
						break;

					case 0x91:	/* 91 XCHG eCX eAX */
						oper1 = regs.wordregs[regcx];
						regs.wordregs[regcx] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x92:	/* 92 XCHG eDX eAX */
						oper1 = regs.wordregs[regdx];
						regs.wordregs[regdx] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x93:	/* 93 XCHG eBX eAX */
						oper1 = regs.wordregs[regbx];
						regs.wordregs[regbx] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x94:	/* 94 XCHG eSP eAX */
						oper1 = regs.wordregs[regsp];
						regs.wordregs[regsp] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x95:	/* 95 XCHG eBP eAX */
						oper1 = regs.wordregs[regbp];
						regs.wordregs[regbp] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x96:	/* 96 XCHG eSI eAX */
						oper1 = regs.wordregs[regsi];
						regs.wordregs[regsi] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x97:	/* 97 XCHG eDI eAX */
						oper1 = regs.wordregs[regdi];
						regs.wordregs[regdi] = regs.wordregs[regax];
						regs.wordregs[regax] = oper1;
						break;

					case 0x98:	/* 98 CBW */
						if ( (regs.byteregs[regal] & 0x80) == 0x80) {
								regs.byteregs[regah] = 0xFF;
							}
						else {
								regs.byteregs[regah] = 0;
							}
						break;

					case 0x99:	/* 99 CWD */
						if ( (regs.byteregs[regah] & 0x80) == 0x80) {
								regs.wordregs[regdx] = 0xFFFF;
							}
						else {
								regs.wordregs[regdx] = 0;
							}
						break;

					case 0x9A:	/* 9A CALL Ap */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						push (segregs[regcs]);
						push (ip);
						ip = oper1;
						segregs[regcs] = oper2;
						break;

					case 0x9B:	/* 9B WAIT */
						break;

					case 0x9C:	/* 9C PUSHF */
#ifdef CPU_SET_HIGH_FLAGS
						push (makeflagsword() | 0xF000); /* running in native mode */
#else
						push (makeflagsword() );
#endif
						break;

					case 0x9D:	/* 9D POPF */
						temp16 = pop();
						decodeflagsword (temp16);
						break;

					case 0x9E:	/* 9E SAHF */
						decodeflagsword ( (makeflagsword() & 0xFF00) | regs.byteregs[regah]);
						break;

					case 0x9F:	/* 9F LAHF */
						regs.byteregs[regah] = makeflagsword() & 0xFF;
						break;

					case 0xA0:	/* A0 MOV regs.byteregs[regal] Ob */
						regs.byteregs[regal] = getmem8 (useseg, getmem16 (segregs[regcs], ip) );
						StepIP (2);
						break;

					case 0xA1:	/* A1 MOV eAX Ov */
						oper1 = getmem16 (useseg, getmem16 (segregs[regcs], ip) );
						StepIP (2);
						regs.wordregs[regax] = oper1;
						break;

					case 0xA2:	/* A2 MOV Ob regs.byteregs[regal] */
						putmem8 (useseg, getmem16 (segregs[regcs], ip), regs.byteregs[regal]);
						StepIP (2);
						break;

					case 0xA3:	/* A3 MOV Ov eAX */
						putmem16 (useseg, getmem16 (segregs[regcs], ip), regs.wordregs[regax]);
						StepIP (2);
						break;

					case 0xA4:	/* A4 MOVSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem8 (segregs[reges], regs.wordregs[regdi], getmem8 (useseg, regs.wordregs[regsi]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xA5:	/* A5 MOVSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem16 (segregs[reges], regs.wordregs[regdi], getmem16 (useseg, regs.wordregs[regsi]) );
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xA6:	/* A6 CMPSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						oper1b = getmem8 (useseg, regs.wordregs[regsi]);
						oper2b = getmem8 (segregs[reges], regs.wordregs[regdi]);
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 1;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						flag_sub8 (oper1b, oper2b);
						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						if ( (reptype == 1) && !zf) {
								break;
							}
						else if ( (reptype == 2) && (zf == 1) ) {
								break;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xA7:	/* A7 CMPSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						oper1 = getmem16 (useseg,regs.wordregs[regsi]);
						oper2 = getmem16 (segregs[reges], regs.wordregs[regdi]);
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 2;
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						flag_sub16 (oper1, oper2);
						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						if ( (reptype == 1) && !zf) {
								break;
							}

						if ( (reptype == 2) && (zf == 1) ) {
								break;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xA8:	/* A8 TEST regs.byteregs[regal] Ib */
						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						flag_log8 (oper1b & oper2b);
						break;

					case 0xA9:	/* A9 TEST eAX Iv */
						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						flag_log16 (oper1 & oper2);
						break;

					case 0xAA:	/* AA STOSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem8 (segregs[reges], regs.wordregs[regdi], regs.byteregs[regal]);
						if (df) {
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xAB:	/* AB STOSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						putmem16 (segregs[reges], regs.wordregs[regdi], regs.wordregs[regax]);
						if (df) {
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xAC:	/* AC LODSB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						regs.byteregs[regal] = getmem8 (useseg, regs.wordregs[regsi]);
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 1;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xAD:	/* AD LODSW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						oper1 = getmem16 (useseg, regs.wordregs[regsi]);
						regs.wordregs[regax] = oper1;
						if (df) {
								regs.wordregs[regsi] = regs.wordregs[regsi] - 2;
							}
						else {
								regs.wordregs[regsi] = regs.wordregs[regsi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xAE:	/* AE SCASB */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						oper1b = regs.byteregs[regal];
						oper2b = getmem8 (segregs[reges], regs.wordregs[regdi]);
						flag_sub8 (oper1b, oper2b);
						if (df) {
								regs.wordregs[regdi] = regs.wordregs[regdi] - 1;
							}
						else {
								regs.wordregs[regdi] = regs.wordregs[regdi] + 1;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						if ( (reptype == 1) && !zf) {
								break;
							}
						else if ( (reptype == 2) && (zf == 1) ) {
								break;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xAF:	/* AF SCASW */
						if (reptype && (regs.wordregs[regcx] == 0) ) {
								break;
							}

						oper1 = regs.wordregs[regax];
						oper2 = getmem16 (segregs[reges], regs.wordregs[regdi]);
						flag_sub16 (oper1, oper2);
						if (df) {
								regs.wordregs[regdi] = regs.wordregs[regdi] - 2;
							}
						else {
								regs.wordregs[regdi] = regs.wordregs[regdi] + 2;
							}

						if (reptype) {
								regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
							}

						if ( (reptype == 1) && !zf) {
								break;
							}
						else if ( (reptype == 2) & (zf == 1) ) {
								break;
							}

						totalexec++;
						loopcount++;
						if (!reptype) {
								break;
							}

						ip = firstip;
						break;

					case 0xB0:	/* B0 MOV regs.byteregs[regal] Ib */
						regs.byteregs[regal] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB1:	/* B1 MOV regs.byteregs[regcl] Ib */
						regs.byteregs[regcl] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB2:	/* B2 MOV regs.byteregs[regdl] Ib */
						regs.byteregs[regdl] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB3:	/* B3 MOV regs.byteregs[regbl] Ib */
						regs.byteregs[regbl] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB4:	/* B4 MOV regs.byteregs[regah] Ib */
						regs.byteregs[regah] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB5:	/* B5 MOV regs.byteregs[regch] Ib */
						regs.byteregs[regch] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB6:	/* B6 MOV regs.byteregs[regdh] Ib */
						regs.byteregs[regdh] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB7:	/* B7 MOV regs.byteregs[regbh] Ib */
						regs.byteregs[regbh] = getmem8 (segregs[regcs], ip);
						StepIP (1);
						break;

					case 0xB8:	/* B8 MOV eAX Iv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						regs.wordregs[regax] = oper1;
						break;

					case 0xB9:	/* B9 MOV eCX Iv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						regs.wordregs[regcx] = oper1;
						break;

					case 0xBA:	/* BA MOV eDX Iv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						regs.wordregs[regdx] = oper1;
						break;

					case 0xBB:	/* BB MOV eBX Iv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						regs.wordregs[regbx] = oper1;
						break;

					case 0xBC:	/* BC MOV eSP Iv */
						regs.wordregs[regsp] = getmem16 (segregs[regcs], ip);
						StepIP (2);
						break;

					case 0xBD:	/* BD MOV eBP Iv */
						regs.wordregs[regbp] = getmem16 (segregs[regcs], ip);
						StepIP (2);
						break;

					case 0xBE:	/* BE MOV eSI Iv */
						regs.wordregs[regsi] = getmem16 (segregs[regcs], ip);
						StepIP (2);
						break;

					case 0xBF:	/* BF MOV eDI Iv */
						regs.wordregs[regdi] = getmem16 (segregs[regcs], ip);
						StepIP (2);
						break;

					case 0xC0:	/* C0 GRP2 byte imm8 (80186+) */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						writerm8 (rm, op_grp2_8 (oper2b) );
						break;

					case 0xC1:	/* C1 GRP2 word imm8 (80186+) */
						modregrm();
						oper1 = readrm16 (rm);
						oper2 = getmem8 (segregs[regcs], ip);
						StepIP (1);
						writerm16 (rm, op_grp2_16 ( (uint8_t) oper2) );
						break;

					case 0xC2:	/* C2 RET Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						ip = pop();
						regs.wordregs[regsp] = regs.wordregs[regsp] + oper1;
						break;

					case 0xC3:	/* C3 RET */
						ip = pop();
						break;

					case 0xC4:	/* C4 LES Gv Mp */
						modregrm();
						getea (rm);
						putreg16 (reg, read86 (ea) + read86 (ea + 1) * 256);
						segregs[reges] = read86 (ea + 2) + read86 (ea + 3) * 256;
						break;

					case 0xC5:	/* C5 LDS Gv Mp */
						modregrm();
						getea (rm);
						putreg16 (reg, read86 (ea) + read86 (ea + 1) * 256);
						segregs[regds] = read86 (ea + 2) + read86 (ea + 3) * 256;
						break;

					case 0xC6:	/* C6 MOV Eb Ib */
						modregrm();
						writerm8 (rm, getmem8 (segregs[regcs], ip) );
						StepIP (1);
						break;

					case 0xC7:	/* C7 MOV Ev Iv */
						modregrm();
						writerm16 (rm, getmem16 (segregs[regcs], ip) );
						StepIP (2);
						break;

					case 0xC8:	/* C8 ENTER (80186+) */
						stacksize = getmem16 (segregs[regcs], ip);
						StepIP (2);
						nestlev = getmem8 (segregs[regcs], ip);
						StepIP (1);
						push (regs.wordregs[regbp]);
						frametemp = regs.wordregs[regsp];
						if (nestlev) {
								for (temp16 = 1; temp16 < nestlev; temp16++) {
										regs.wordregs[regbp] = regs.wordregs[regbp] - 2;
										push (regs.wordregs[regbp]);
									}

								push (regs.wordregs[regsp]);
							}

						regs.wordregs[regbp] = frametemp;
						regs.wordregs[regsp] = regs.wordregs[regbp] - stacksize;

						break;

					case 0xC9:	/* C9 LEAVE (80186+) */
						regs.wordregs[regsp] = regs.wordregs[regbp];
						regs.wordregs[regbp] = pop();
						break;

					case 0xCA:	/* CA RETF Iw */
						oper1 = getmem16 (segregs[regcs], ip);
						ip = pop();
						segregs[regcs] = pop();
						regs.wordregs[regsp] = regs.wordregs[regsp] + oper1;
						break;

					case 0xCB:	/* CB RETF */
						ip = pop();;
						segregs[regcs] = pop();
						break;

					case 0xCC:	/* CC INT 3 */
						intcall86 (3);
						break;

					case 0xCD:	/* CD INT Ib */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						intcall86 (oper1b);
						break;

					case 0xCE:	/* CE INTO */
						if (of) {
								intcall86 (4);
							}
						break;

					case 0xCF:	/* CF IRET */
						ip = pop();
						segregs[regcs] = pop();
						decodeflagsword (pop() );

						/*
						 * if (net.enabled) net.canrecv = 1;
						 */
						break;

					case 0xD0:	/* D0 GRP2 Eb 1 */
						modregrm();
						oper1b = readrm8 (rm);
						writerm8 (rm, op_grp2_8 (1) );
						break;

					case 0xD1:	/* D1 GRP2 Ev 1 */
						modregrm();
						oper1 = readrm16 (rm);
						writerm16 (rm, op_grp2_16 (1) );
						break;

					case 0xD2:	/* D2 GRP2 Eb regs.byteregs[regcl] */
						modregrm();
						oper1b = readrm8 (rm);
						writerm8 (rm, op_grp2_8 (regs.byteregs[regcl]) );
						break;

					case 0xD3:	/* D3 GRP2 Ev regs.byteregs[regcl] */
						modregrm();
						oper1 = readrm16 (rm);
						writerm16 (rm, op_grp2_16 (regs.byteregs[regcl]) );
						break;

					case 0xD4:	/* D4 AAM I0 */
						oper1 = getmem8 (segregs[regcs], ip);
						StepIP (1);
#ifdef CPU_DECIMAL_ONLY_AAD_AAM /* V20 only does decimal AAM */
						regs.byteregs[regah] = (regs.byteregs[regal] / 10) & 255;
						regs.byteregs[regal] = (regs.byteregs[regal] % 10) & 255;
#else
						if (!oper1) {
								intcall86 (0);
								break;
							}	/* division by zero */

						regs.byteregs[regah] = (regs.byteregs[regal] / oper1) & 255;
						regs.byteregs[regal] = (regs.byteregs[regal] % oper1) & 255;
#endif
						flag_szp16 (regs.wordregs[regax]);
						break;

					case 0xD5:	/* D5 AAD I0 */
						oper1 = getmem8 (segregs[regcs], ip);
						StepIP (1);
#ifdef CPU_DECIMAL_ONLY_AAD_AAM /* V20 only does decimal AAD */
						regs.byteregs[regal] = (regs.byteregs[regah] * 10 + regs.byteregs[regal]) & 255;
						regs.byteregs[regah] = 0;
						flag_szp16 (regs.byteregs[regah] * 10 + regs.byteregs[regal]);
#else
						regs.byteregs[regal] = (regs.byteregs[regah] * oper1 + regs.byteregs[regal]) & 255;
						regs.byteregs[regah] = 0;
						flag_szp16 (regs.byteregs[regah] * oper1 + regs.byteregs[regal]);
#endif
						sf = 0;
						break;

					case 0xD6:	/* D6 XLAT on V20/V30, SALC on 8086/8088 */
#ifndef CPU_NO_SALC
						regs.byteregs[regal] = cf ? 0xFF : 0x00;
						break;
#endif

					case 0xD7:	/* D7 XLAT */
						regs.byteregs[regal] = read86(useseg * 16 + (regs.wordregs[regbx]) + regs.byteregs[regal]);
						break;

					case 0xD8:
					case 0xD9:
					case 0xDA:
					case 0xDB:
					case 0xDC:
					case 0xDE:
					case 0xDD:
					case 0xDF:	/* escape to x87 FPU (unsupported) */
						modregrm();
						break;

					case 0xE0:	/* E0 LOOPNZ Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
						if ( (regs.wordregs[regcx]) && !zf) {
								ip = ip + temp16;
							}
						break;

					case 0xE1:	/* E1 LOOPZ Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
						if (regs.wordregs[regcx] && (zf == 1) ) {
								ip = ip + temp16;
							}
						break;

					case 0xE2:	/* E2 LOOP Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						regs.wordregs[regcx] = regs.wordregs[regcx] - 1;
						if (regs.wordregs[regcx]) {
								ip = ip + temp16;
							}
						break;

					case 0xE3:	/* E3 JCXZ Jb */
						temp16 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						if (!regs.wordregs[regcx]) {
								ip = ip + temp16;
							}
						break;

					case 0xE4:	/* E4 IN regs.byteregs[regal] Ib */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						regs.byteregs[regal] = (uint8_t) portin (oper1b);
						break;

					case 0xE5:	/* E5 IN eAX Ib */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						regs.wordregs[regax] = portin16 (oper1b);
						break;

					case 0xE6:	/* E6 OUT Ib regs.byteregs[regal] */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						portout (oper1b, regs.byteregs[regal]);
						break;

					case 0xE7:	/* E7 OUT Ib eAX */
						oper1b = getmem8 (segregs[regcs], ip);
						StepIP (1);
						portout16 (oper1b, regs.wordregs[regax]);
						break;

					case 0xE8:	/* E8 CALL Jv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						push (ip);
						ip = ip + oper1;
						break;

					case 0xE9:	/* E9 JMP Jv */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						ip = ip + oper1;
						break;

					case 0xEA:	/* EA JMP Ap */
						oper1 = getmem16 (segregs[regcs], ip);
						StepIP (2);
						oper2 = getmem16 (segregs[regcs], ip);
						ip = oper1;
						segregs[regcs] = oper2;
						break;

					case 0xEB:	/* EB JMP Jb */
						oper1 = signext (getmem8 (segregs[regcs], ip) );
						StepIP (1);
						ip = ip + oper1;
						break;

					case 0xEC:	/* EC IN regs.byteregs[regal] regdx */
						oper1 = regs.wordregs[regdx];
						regs.byteregs[regal] = (uint8_t) portin (oper1);
						break;

					case 0xED:	/* ED IN eAX regdx */
						oper1 = regs.wordregs[regdx];
						regs.wordregs[regax] = portin16 (oper1);
						break;

					case 0xEE:	/* EE OUT regdx regs.byteregs[regal] */
						oper1 = regs.wordregs[regdx];
						portout (oper1, regs.byteregs[regal]);
						break;

					case 0xEF:	/* EF OUT regdx eAX */
						oper1 = regs.wordregs[regdx];
						portout16 (oper1, regs.wordregs[regax]);
						break;

					case 0xF0:	/* F0 LOCK */
						break;

					case 0xF4:	/* F4 HLT */
						hltstate = 1;
						break;

					case 0xF5:	/* F5 CMC */
						if (!cf) {
								cf = 1;
							}
						else {
								cf = 0;
							}
						break;

					case 0xF6:	/* F6 GRP3a Eb */
						modregrm();
						oper1b = readrm8 (rm);
						op_grp3_8();
						if ( (reg > 1) && (reg < 4) ) {
								writerm8 (rm, res8);
							}
						break;

					case 0xF7:	/* F7 GRP3b Ev */
						modregrm();
						oper1 = readrm16 (rm);
						op_grp3_16();
						if ( (reg > 1) && (reg < 4) ) {
								writerm16 (rm, res16);
							}
						break;

					case 0xF8:	/* F8 CLC */
						cf = 0;
						break;

					case 0xF9:	/* F9 STC */
						cf = 1;
						break;

					case 0xFA:	/* FA CLI */
						ifl = 0;
						break;

					case 0xFB:	/* FB STI */
						ifl = 1;
						break;

					case 0xFC:	/* FC CLD */
						df = 0;
						break;

					case 0xFD:	/* FD STD */
						df = 1;
						break;

					case 0xFE:	/* FE GRP4 Eb */
						modregrm();
						oper1b = readrm8 (rm);
						oper2b = 1;
						if (!reg) {
								tempcf = cf;
								res8 = oper1b + oper2b;
								flag_add8 (oper1b, oper2b);
								cf = tempcf;
								writerm8 (rm, res8);
							}
						else {
								tempcf = cf;
								res8 = oper1b - oper2b;
								flag_sub8 (oper1b, oper2b);
								cf = tempcf;
								writerm8 (rm, res8);
							}
						break;

					case 0xFF:	/* FF GRP5 Ev */
						modregrm();
						oper1 = readrm16 (rm);
						op_grp5();
						break;

					default:
#ifdef CPU_ALLOW_ILLEGAL_OP_EXCEPTION
						intcall86 (6); /* trip invalid opcode exception (this occurs on the 80186+, 8086/8088 CPUs treat them as NOPs. */
						               /* technically they aren't exactly like NOPs in most cases, but for our pursoses, that's accurate enough. */
#endif
						if (verbose) {
								printf ("Illegal opcode: %02X %02X /%X @ %04X:%04X\n", getmem8(savecs, saveip), getmem8(savecs, saveip+1), (getmem8(savecs, saveip+2) >> 3) & 7, savecs, saveip);
							}
						break;
				}
			}

skipexecution:
			if (!running) {
					return;
				}
		}
}
#endif

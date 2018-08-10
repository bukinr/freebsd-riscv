/*-
 * Copyright (c) 2016 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Portions of this software were developed by SRI International and the
 * University of Cambridge Computer Laboratory under DARPA/AFRL contract
 * FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Portions of this software were developed by the University of Cambridge
 * Computer Laboratory as part of the CTSRD Project, with support from the
 * UK Higher Education Innovation Fund (HEIF).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <ddb/ddb.h>
#include <ddb/db_access.h>
#include <ddb/db_sym.h>

#include <machine/riscvreg.h>
#include <machine/riscv_opcode.h>
#include <machine/riscv_encoding.h>

struct riscv1_op {
	char *name;
	char *fmt;
	int match;
	int mask;
	int (*match_func)(struct riscv1_op *op, uint32_t insn);
};

static int
match_opcode(struct riscv1_op *op, uint32_t insn)
{

	if (((insn ^ op->match) & op->mask) == 0)
		return (1);

	return (0);
}

static struct riscv1_op riscv1_opcodes[] = {
	//{ "mv",		MATCH_ADDI,	MASK_ADDI | MASK_IMM, match_opcode },

	{ "beq",	"s,t,p", 	MATCH_BEQ,	MASK_BEQ,	match_opcode },
	{ "bne",	"s,t,p", 	MATCH_BNE,	MASK_BNE,	match_opcode },
	{ "blt",	"s,t,p", 	MATCH_BLT,	MASK_BLT,	match_opcode },
	{ "bge",	"s,t,p", 	MATCH_BGE,	MASK_BGE,	match_opcode },
	{ "bltu",	"s,t,p", 	MATCH_BLTU,	MASK_BLTU,	match_opcode },
	{ "bgeu",	"s,t,p", 	MATCH_BGEU,	MASK_BGEU,	match_opcode },
	{ "jalr",	"d,o(s)", 	MATCH_JALR,	MASK_JALR,	match_opcode },
	{ "jal",	"d,a",		MATCH_JAL,	MASK_JAL,	match_opcode },
	{ "lui",	"d,u",		MATCH_LUI,	MASK_LUI,	match_opcode },
	{ "auipc",	"d,u",		MATCH_AUIPC,	MASK_AUIPC,	match_opcode },
	{ "addi",	"d,s,j", 	MATCH_ADDI,	MASK_ADDI,	match_opcode },
	{ "slli",	"d,s,>", 	MATCH_SLLI,	MASK_SLLI,	match_opcode },
	{ "slti",	"d,s,j", 	MATCH_SLTI,	MASK_SLTI,	match_opcode },
	{ "sltiu",	"d,s,j", 	MATCH_SLTIU,	MASK_SLTIU,	match_opcode },
	{ "xori",	"d,s,j", 	MATCH_XORI,	MASK_XORI,	match_opcode },
	{ "srli",	"d,s,>", 	MATCH_SRLI,	MASK_SRLI,	match_opcode },
	{ "srai",	"d,s,>", 	MATCH_SRAI,	MASK_SRAI,	match_opcode },
	{ "ori",	"d,s,j", 	MATCH_ORI,	MASK_ORI,	match_opcode },
	{ "andi",	"d,s,j", 	MATCH_ANDI,	MASK_ANDI,	match_opcode },
	{ "add",	"d,s,t", 	MATCH_ADD,	MASK_ADD,	match_opcode },
	{ "sub",	"d,s,t", 	MATCH_SUB,	MASK_SUB,	match_opcode },
	{ "sll",	"d,s,t", 	MATCH_SLL,	MASK_SLL,	match_opcode },
	{ "slt",	"d,s,t", 	MATCH_SLT,	MASK_SLT,	match_opcode },
	{ "sltu",	"d,s,t", 	MATCH_SLTU,	MASK_SLTU,	match_opcode },
	{ "xor",	"d,s,t", 	MATCH_XOR,	MASK_XOR,	match_opcode },
	{ "srl",	"d,s,t", 	MATCH_SRL,	MASK_SRL,	match_opcode },
	{ "sra",	"d,s,t", 	MATCH_SRA,	MASK_SRA,	match_opcode },
	{ "or",		"d,s,t",	MATCH_OR,	MASK_OR,	match_opcode },
	{ "and",	"d,s,t", 	MATCH_AND,	MASK_AND,	match_opcode },
	{ "addiw",	"d,s,j", 	MATCH_ADDIW,	MASK_ADDIW,	match_opcode },
	{ "slliw",	"d,s,<", 	MATCH_SLLIW,	MASK_SLLIW,	match_opcode },
	{ "srliw",	"d,s,<", 	MATCH_SRLIW,	MASK_SRLIW,	match_opcode },
	{ "sraiw",	"d,s,<", 	MATCH_SRAIW,	MASK_SRAIW,	match_opcode },
	{ "addw",	"d,s,t", 	MATCH_ADDW,	MASK_ADDW,	match_opcode },
	{ "subw",	"d,s,t", 	MATCH_SUBW,	MASK_SUBW,	match_opcode },
	{ "sllw",	"d,s,t", 	MATCH_SLLW,	MASK_SLLW,	match_opcode },
	{ "srlw",	"d,s,t", 	MATCH_SRLW,	MASK_SRLW,	match_opcode },
	{ "sraw",	"d,s,t", 	MATCH_SRAW,	MASK_SRAW,	match_opcode },
	{ "lb",		"d,o(s)", 	MATCH_LB,	MASK_LB,	match_opcode },
	{ "lh",		"d,o(s)", 	MATCH_LH,	MASK_LH,	match_opcode },
	{ "lw",		"d,o(s)", 	MATCH_LW,	MASK_LW,	match_opcode },
	{ "ld",		"d,o(s)", 	MATCH_LD,	MASK_LD,	match_opcode },
	{ "lbu",	"d,o(s)", 	MATCH_LBU,	MASK_LBU,	match_opcode },
	{ "lhu",	"d,o(s)", 	MATCH_LHU,	MASK_LHU,	match_opcode },
	{ "lwu",	"d,o(s)", 	MATCH_LWU,	MASK_LWU,	match_opcode },
	{ "sb",		"t,q(s)", 	MATCH_SB,	MASK_SB,	match_opcode },
	{ "sh",		"t,q(s)", 	MATCH_SH,	MASK_SH,	match_opcode },
	{ "sw",		"t,q(s)", 	MATCH_SW,	MASK_SW,	match_opcode },
	{ "sd",		"t,q(s)", 	MATCH_SD,	MASK_SD,	match_opcode },
	{ "fence",	"P,Q",		MATCH_FENCE,	MASK_FENCE,	match_opcode },
	{ "fence.i",	"",		MATCH_FENCE_I,	MASK_FENCE_I,	match_opcode },
	{ "mul",	"d,s,t", 	MATCH_MUL,	MASK_MUL,	match_opcode },
	{ "mulh",	"d,s,t", 	MATCH_MULH,	MASK_MULH,	match_opcode },
	{ "mulhsu",	"d,s,t", 	MATCH_MULHSU,	MASK_MULHSU,	match_opcode },
	{ "mulhu",	"d,s,t", 	MATCH_MULHU,	MASK_MULHU,	match_opcode },
	{ "div",	"d,s,t", 	MATCH_DIV,	MASK_DIV,	match_opcode },
	{ "divu",	"d,s,t", 	MATCH_DIVU,	MASK_DIVU,	match_opcode },
	{ "rem",	"d,s,t", 	MATCH_REM,	MASK_REM,	match_opcode },
	{ "remu",	"d,s,t", 	MATCH_REMU,	MASK_REMU,	match_opcode },
	{ "mulw",	"d,s,t", 	MATCH_MULW,	MASK_MULW,	match_opcode },
	{ "divw",	"d,s,t", 	MATCH_DIVW,	MASK_DIVW,	match_opcode },
	{ "divuw",	"d,s,t", 	MATCH_DIVUW,	MASK_DIVUW,	match_opcode },
	{ "remw",	"d,s,t", 	MATCH_REMW,	MASK_REMW,	match_opcode },
	{ "remuw",	"d,s,t", 	MATCH_REMUW,	MASK_REMUW,	match_opcode },
	{ "amoadd.w",	"d,t,0(s)", 	MATCH_AMOADD_W,	MASK_AMOADD_W,	match_opcode },
	{ "amoxor.w",	"d,t,0(s)", 	MATCH_AMOXOR_W,	MASK_AMOXOR_W,	match_opcode },
	{ "amoor.w",	"d,t,0(s)", 	MATCH_AMOOR_W,	MASK_AMOOR_W,	match_opcode },
	{ "amoand.w",	"d,t,0(s)", 	MATCH_AMOAND_W,	MASK_AMOAND_W,	match_opcode },
	{ "amomin.w",	"d,t,0(s)", 	MATCH_AMOMIN_W,	MASK_AMOMIN_W,	match_opcode },
	{ "amomax.w",	"d,t,0(s)", 	MATCH_AMOMAX_W,	MASK_AMOMAX_W,	match_opcode },
	{ "amominu.w",	"d,t,0(s)", 	MATCH_AMOMINU_W,	MASK_AMOMINU_W,	match_opcode },
	{ "amomaxu.w",	"d,t,0(s)", 	MATCH_AMOMAXU_W,	MASK_AMOMAXU_W,	match_opcode },
	{ "amoswap.w",	"d,t,0(s)", 	MATCH_AMOSWAP_W,	MASK_AMOSWAP_W,	match_opcode },
	{ "lr.w",	"d,0(s)", 	MATCH_LR_W,	MASK_LR_W,	match_opcode },
	{ "sc.w",	"d,t,0(s)", 	MATCH_SC_W,	MASK_SC_W,	match_opcode },
	{ "amoadd.d",	"d,t,0(s)", 	MATCH_AMOADD_D,	MASK_AMOADD_D,	match_opcode },
	{ "amoxor.d",	"d,t,0(s)", 	MATCH_AMOXOR_D,	MASK_AMOXOR_D,	match_opcode },
	{ "amoor.d",	"d,t,0(s)", 	MATCH_AMOOR_D,	MASK_AMOOR_D,	match_opcode },
	{ "amoand.d",	"d,t,0(s)", 	MATCH_AMOAND_D,	MASK_AMOAND_D,	match_opcode },
	{ "amomin.d",	"d,t,0(s)", 	MATCH_AMOMIN_D,	MASK_AMOMIN_D,	match_opcode },
	{ "amomax.d",	"d,t,0(s)", 	MATCH_AMOMAX_D,	MASK_AMOMAX_D,	match_opcode },
	{ "amominu.d",	"d,t,0(s)", 	MATCH_AMOMINU_D,	MASK_AMOMINU_D,	match_opcode },
	{ "amomaxu.d",	"d,t,0(s)", 	MATCH_AMOMAXU_D,	MASK_AMOMAXU_D,	match_opcode },
	{ "amoswap.d",	"d,t,0(s)", 	MATCH_AMOSWAP_D,	MASK_AMOSWAP_D,	match_opcode },
	{ "lr.d",	"d,0(s)", 	MATCH_LR_D,	MASK_LR_D,	match_opcode },
	{ "sc.d",	"d,t,0(s)", 	MATCH_SC_D,	MASK_SC_D,	match_opcode },
	{ "ecall",	"", 		MATCH_ECALL,	MASK_ECALL,	match_opcode },
	{ "ebreak",	"", 		MATCH_EBREAK,	MASK_EBREAK,	match_opcode },
	{ "uret",	"", 		MATCH_URET,	MASK_URET,	match_opcode },
	{ "sret",	"", 		MATCH_SRET,	MASK_SRET,	match_opcode },
	{ "mret",	"", 		MATCH_MRET,	MASK_MRET,	match_opcode },
	{ "dret",	"", 		MATCH_DRET,	MASK_DRET,	match_opcode },
	{ "sfence.vma",	"", 		MATCH_SFENCE_VMA,	MASK_SFENCE_VMA,	match_opcode },
	{ "wfi",	"", 		MATCH_WFI,	MASK_WFI,	match_opcode },
	{ "csrrw",	"d,E,s", 	MATCH_CSRRW,	MASK_CSRRW,	match_opcode },
	{ "csrrs",	"d,E,s", 	MATCH_CSRRS,	MASK_CSRRS,	match_opcode },
	{ "csrrc",	"d,E,s", 	MATCH_CSRRC,	MASK_CSRRC,	match_opcode },
	{ "csrrwi",	"d,E,Z", 	MATCH_CSRRWI,	MASK_CSRRWI,	match_opcode },
	{ "csrrsi",	"d,E,Z", 	MATCH_CSRRSI,	MASK_CSRRSI,	match_opcode },
	{ "csrrci",	"d,E,Z", 	MATCH_CSRRCI,	MASK_CSRRCI,	match_opcode },
	{ "fadd.s",	"D,S,T", 	MATCH_FADD_S,	MASK_FADD_S,	match_opcode },
	{ "fsub.s",	"D,S,T", 	MATCH_FSUB_S,	MASK_FSUB_S,	match_opcode },
	{ "fmul.s",	"D,S,T", 	MATCH_FMUL_S,	MASK_FMUL_S,	match_opcode },
	{ "fdiv.s",	"D,S,T", 	MATCH_FDIV_S,	MASK_FDIV_S,	match_opcode },
	{ "fsgnj.s",	"D,S,T", 	MATCH_FSGNJ_S,	MASK_FSGNJ_S,	match_opcode },
	{ "fsgnjn.s",	"D,S,T", 	MATCH_FSGNJN_S,	MASK_FSGNJN_S,	match_opcode },
	{ "fsgnjx.s",	"D,S,T", 	MATCH_FSGNJX_S,	MASK_FSGNJX_S,	match_opcode },
	{ "fmin.s",	"D,S,T", 	MATCH_FMIN_S,	MASK_FMIN_S,	match_opcode },
	{ "fmax.s",	"D,S,T", 	MATCH_FMAX_S,	MASK_FMAX_S,	match_opcode },
	{ "fsqrt.s",	"D,S",		MATCH_FSQRT_S,	MASK_FSQRT_S,	match_opcode },
	{ "fadd.d",	"D,S,T", 	MATCH_FADD_D,	MASK_FADD_D,	match_opcode },
	{ "fsub.d",	"D,S,T", 	MATCH_FSUB_D,	MASK_FSUB_D,	match_opcode },
	{ "fmul.d",	"D,S,T", 	MATCH_FMUL_D,	MASK_FMUL_D,	match_opcode },
	{ "fdiv.d",	"D,S,T", 	MATCH_FDIV_D,	MASK_FDIV_D,	match_opcode },
	{ "fsgnj.d",	"D,S,T", 	MATCH_FSGNJ_D,	MASK_FSGNJ_D,	match_opcode },
	{ "fsgnjn.d",	"D,S,T", 	MATCH_FSGNJN_D,	MASK_FSGNJN_D,	match_opcode },
	{ "fsgnjx.d",	"D,S,T", 	MATCH_FSGNJX_D,	MASK_FSGNJX_D,	match_opcode },
	{ "fmin.d",	"D,S,T", 	MATCH_FMIN_D,	MASK_FMIN_D,	match_opcode },
	{ "fmax.d",	"D,S,T", 	MATCH_FMAX_D,	MASK_FMAX_D,	match_opcode },
	{ "fcvt.s.d",	"D,S", 		MATCH_FCVT_S_D,	MASK_FCVT_S_D,	match_opcode },
	{ "fcvt.d.s",	"D,S", 		MATCH_FCVT_D_S,	MASK_FCVT_D_S,	match_opcode },
	{ "fsqrt.d",	"D,S", 		MATCH_FSQRT_D,	MASK_FSQRT_D,	match_opcode },
	{ "fadd.q",	"D,S,T", 	MATCH_FADD_Q,	MASK_FADD_Q,	match_opcode },
	{ "fsub.q",	"D,S,T", 	MATCH_FSUB_Q,	MASK_FSUB_Q,	match_opcode },
	{ "fmul.q",	"D,S,T", 	MATCH_FMUL_Q,	MASK_FMUL_Q,	match_opcode },
	{ "fdiv.q",	"D,S,T", 	MATCH_FDIV_Q,	MASK_FDIV_Q,	match_opcode },
	{ "fsgnj.q",	"D,S,T", 	MATCH_FSGNJ_Q,	MASK_FSGNJ_Q,	match_opcode },
	{ "fsgnjn.q",	"D,S,T", 	MATCH_FSGNJN_Q,	MASK_FSGNJN_Q,	match_opcode },
	{ "fsgnjx.q",	"D,S,T", 	MATCH_FSGNJX_Q,	MASK_FSGNJX_Q,	match_opcode },
	{ "fmin.q",	"D,S,T", 	MATCH_FMIN_Q,	MASK_FMIN_Q,	match_opcode },
	{ "fmax.q",	"D,S,T", 	MATCH_FMAX_Q,	MASK_FMAX_Q,	match_opcode },
	{ "fcvt.s.q",	"D,S", 		MATCH_FCVT_S_Q,	MASK_FCVT_S_Q,	match_opcode },
	{ "fcvt.q.s",	"D,S", 		MATCH_FCVT_Q_S,	MASK_FCVT_Q_S,	match_opcode },
	{ "fcvt.d.q",	"D,S", 		MATCH_FCVT_D_Q,	MASK_FCVT_D_Q,	match_opcode },
	{ "fcvt.q.d",	"D,S", 		MATCH_FCVT_Q_D,	MASK_FCVT_Q_D,	match_opcode },
	{ "fsqrt.q",	"D,S", 		MATCH_FSQRT_Q,	MASK_FSQRT_Q,	match_opcode },
	{ "fle.s",	"d,S,T", 	MATCH_FLE_S,	MASK_FLE_S,	match_opcode },
	{ "flt.s",	"d,S,T", 	MATCH_FLT_S,	MASK_FLT_S,	match_opcode },
	{ "feq.s",	"d,S,T", 	MATCH_FEQ_S,	MASK_FEQ_S,	match_opcode },
	{ "fle.d",	"d,S,T", 	MATCH_FLE_D,	MASK_FLE_D,	match_opcode },
	{ "flt.d",	"d,S,T", 	MATCH_FLT_D,	MASK_FLT_D,	match_opcode },
	{ "feq.d",	"d,S,T", 	MATCH_FEQ_D,	MASK_FEQ_D,	match_opcode },
	{ "fle.q",	"d,S,T", 	MATCH_FLE_Q,	MASK_FLE_Q,	match_opcode },
	{ "flt.q",	"d,S,T", 	MATCH_FLT_Q,	MASK_FLT_Q,	match_opcode },
	{ "feq.q",	"d,S,T", 	MATCH_FEQ_Q,	MASK_FEQ_Q,	match_opcode },
	{ "fcvt.w.s",	"d,S", 		MATCH_FCVT_W_S,	MASK_FCVT_W_S,	match_opcode },
	{ "fcvt.wu.s",	"d,S", 		MATCH_FCVT_WU_S,	MASK_FCVT_WU_S,	match_opcode },
	{ "fcvt.l.s",	"d,S", 		MATCH_FCVT_L_S,	MASK_FCVT_L_S,	match_opcode },
	{ "fcvt.lu.s",	"d,S", 		MATCH_FCVT_LU_S,	MASK_FCVT_LU_S,	match_opcode },
	{ "fmv.x.w",	"d,S", 		MATCH_FMV_X_W,	MASK_FMV_X_W,	match_opcode },
	{ "fclass.s",	"d,S", 		MATCH_FCLASS_S,	MASK_FCLASS_S,	match_opcode },
	{ "fcvt.w.d",	"d,S", 		MATCH_FCVT_W_D,	MASK_FCVT_W_D,	match_opcode },
	{ "fcvt.wu.d",	"d,S", 		MATCH_FCVT_WU_D,	MASK_FCVT_WU_D,	match_opcode },
	{ "fcvt.l.d",	"d,S", 		MATCH_FCVT_L_D,	MASK_FCVT_L_D,	match_opcode },
	{ "fcvt.lu.d",	"d,S", 		MATCH_FCVT_LU_D,	MASK_FCVT_LU_D,	match_opcode },
	{ "fmv.x.d",	"d,S", 		MATCH_FMV_X_D,	MASK_FMV_X_D,	match_opcode },
	{ "fclass.d",	"d,S", 		MATCH_FCLASS_D,	MASK_FCLASS_D,	match_opcode },
	{ "fcvt.w.q",	"d,S", 		MATCH_FCVT_W_Q,	MASK_FCVT_W_Q,	match_opcode },
	{ "fcvt.wu.q",	"d,S", 		MATCH_FCVT_WU_Q,	MASK_FCVT_WU_Q,	match_opcode },
	{ "fcvt.l.q",	"d,S", 		MATCH_FCVT_L_Q,	MASK_FCVT_L_Q,	match_opcode },
	{ "fcvt.lu.q",	"d,S", 		MATCH_FCVT_LU_Q,	MASK_FCVT_LU_Q,	match_opcode },
	{ "fmv.x.q",	"d,S", 		MATCH_FMV_X_Q,	MASK_FMV_X_Q,	match_opcode },
	{ "fclass.q",	"d,S", 		MATCH_FCLASS_Q,	MASK_FCLASS_Q,	match_opcode },
	{ "fcvt.s.w",	"D,s", 		MATCH_FCVT_S_W,	MASK_FCVT_S_W,	match_opcode },
	{ "fcvt.s.wu",	"D,s", 		MATCH_FCVT_S_WU,	MASK_FCVT_S_WU,	match_opcode },
	{ "fcvt.s.l",	"D,s", 		MATCH_FCVT_S_L,	MASK_FCVT_S_L,	match_opcode },
	{ "fcvt.s.lu",	"D,s", 		MATCH_FCVT_S_LU,	MASK_FCVT_S_LU,	match_opcode },
	{ "fmv.w.x",	"D,s", 		MATCH_FMV_W_X,	MASK_FMV_W_X,	match_opcode },
	{ "fcvt.d.w",	"D,s", 		MATCH_FCVT_D_W,	MASK_FCVT_D_W,	match_opcode },
	{ "fcvt.d.wu",	"D,s", 		MATCH_FCVT_D_WU,	MASK_FCVT_D_WU,	match_opcode },
	{ "fcvt.d.l",	"D,s", 		MATCH_FCVT_D_L,	MASK_FCVT_D_L,	match_opcode },
	{ "fcvt.d.lu",	"D,s", 		MATCH_FCVT_D_LU,	MASK_FCVT_D_LU,	match_opcode },
	{ "fmv.d.x",	"D,s", 		MATCH_FMV_D_X,	MASK_FMV_D_X,	match_opcode },
	{ "fcvt.q.w",	"D,s", 		MATCH_FCVT_Q_W,	MASK_FCVT_Q_W,	match_opcode },
	{ "fcvt.q.wu",	"D,s", 		MATCH_FCVT_Q_WU,	MASK_FCVT_Q_WU,	match_opcode },
	{ "fcvt.q.l",	"D,s", 		MATCH_FCVT_Q_L,	MASK_FCVT_Q_L,	match_opcode },
	{ "fcvt.q.lu",	"D,s", 		MATCH_FCVT_Q_LU,	MASK_FCVT_Q_LU,	match_opcode },
	{ "fmv.q.x",	"D,s", 		MATCH_FMV_Q_X,	MASK_FMV_Q_X,	match_opcode },
	{ "flw",	"D,o(s)", 	MATCH_FLW,	MASK_FLW,	match_opcode },
	{ "fld",	"D,o(s)", 	MATCH_FLD,	MASK_FLD,	match_opcode },
	{ "flq",	"D,o(s)", 	MATCH_FLQ,	MASK_FLQ,	match_opcode },
	{ "fsw",	"T,q(s)", 	MATCH_FSW,	MASK_FSW,	match_opcode },
	{ "fsd",	"T,q(s)", 	MATCH_FSD,	MASK_FSD,	match_opcode },
	{ "fsq",	"T,q(s)", 	MATCH_FSQ,	MASK_FSQ,	match_opcode },
	{ "fmadd.s",	"D,S,T,R", 	MATCH_FMADD_S,	MASK_FMADD_S,	match_opcode },
	{ "fmsub.s",	"D,S,T,R", 	MATCH_FMSUB_S,	MASK_FMSUB_S,	match_opcode },
	{ "fnmsub.s",	"D,S,T,R", 	MATCH_FNMSUB_S,	MASK_FNMSUB_S,	match_opcode },
	{ "fnmadd.s",	"D,S,T,R", 	MATCH_FNMADD_S,	MASK_FNMADD_S,	match_opcode },
	{ "fmadd.d",	"D,S,T,R", 	MATCH_FMADD_D,	MASK_FMADD_D,	match_opcode },
	{ "fmsub.d",	"D,S,T,R", 	MATCH_FMSUB_D,	MASK_FMSUB_D,	match_opcode },
	{ "fnmsub.d",	"D,S,T,R", 	MATCH_FNMSUB_D,	MASK_FNMSUB_D,	match_opcode },
	{ "fnmadd.d",	"D,S,T,R", 	MATCH_FNMADD_D,	MASK_FNMADD_D,	match_opcode },
	{ "fmadd.q",	"D,S,T,R", 	MATCH_FMADD_Q,	MASK_FMADD_Q,	match_opcode },
	{ "fmsub.q",	"D,S,T,R", 	MATCH_FMSUB_Q,	MASK_FMSUB_Q,	match_opcode },
	{ "fnmsub.q",	"D,S,T,R", 	MATCH_FNMSUB_Q,	MASK_FNMSUB_Q,	match_opcode },
	{ "fnmadd.q",	"D,S,T,R", 	MATCH_FNMADD_Q,	MASK_FNMADD_Q,	match_opcode },
	{ NULL, NULL, 0, 0, NULL },
};

static struct riscv1_op riscv1_copcodes[] = {
	{ "c.nop",	"", 		MATCH_C_NOP,	MASK_C_NOP,	match_opcode },
	{ "c.addi16sp",	"", 		MATCH_C_ADDI16SP,	MASK_C_ADDI16SP,	match_opcode },
	{ "c.jr",	"d", 		MATCH_C_JR,	MASK_C_JR,	match_opcode },
	{ "c.jalr",	"d", 		MATCH_C_JALR,	MASK_C_JALR,	match_opcode },
	{ "c.ebreak",	"", 		MATCH_C_EBREAK,	MASK_C_EBREAK,	match_opcode },
	{ "c.ld",	"Ct,Cl(Cs)", 	MATCH_C_LD,	MASK_C_LD,	match_opcode },
	{ "c.sd",	"Ct,Cl(Cs)", 	MATCH_C_SD,	MASK_C_SD,	match_opcode },
	{ "c.addiw",	"d,Co", 	MATCH_C_ADDIW,	MASK_C_ADDIW,	match_opcode },
	{ "c.ldsp",	"d,Cn(Cc)", 	MATCH_C_LDSP,	MASK_C_LDSP,	match_opcode },
	{ "c.sdsp",	"CV,CN(Cc)", 	MATCH_C_SDSP,	MASK_C_SDSP,	match_opcode },
	{ "c.addi4spn",	"", 		MATCH_C_ADDI4SPN,	MASK_C_ADDI4SPN,	match_opcode },
	{ "c.fld",	"CD,Cl(Cs)", 	MATCH_C_FLD,	MASK_C_FLD,	match_opcode },
	{ "c.lw",	"Ct,Ck(Cs)", 	MATCH_C_LW,	MASK_C_LW,	match_opcode },
	{ "c.flw",	"CD,Ck(Cs)", 	MATCH_C_FLW,	MASK_C_FLW,	match_opcode },
	{ "c.fsd",	"CD,Cl(Cs)", 	MATCH_C_FSD,	MASK_C_FSD,	match_opcode },
	{ "c.sw",	"Ct,Ck(Cs)", 	MATCH_C_SW,	MASK_C_SW,	match_opcode },
	{ "c.fsw",	"CD,Ck(Cs)", 	MATCH_C_FSW,	MASK_C_FSW,	match_opcode },
	{ "c.addi",	"d,Co", 	MATCH_C_ADDI,	MASK_C_ADDI,	match_opcode },
	{ "c.jal",	"Ca", 		MATCH_C_JAL,	MASK_C_JAL,	match_opcode },
	{ "c.li",	"d,Co", 	MATCH_C_LI,	MASK_C_LI,	match_opcode },
	{ "c.lui",	"d,Cu", 	MATCH_C_LUI,	MASK_C_LUI,	match_opcode },
	{ "c.srli",	"Cs,C>", 	MATCH_C_SRLI,	MASK_C_SRLI,	match_opcode },
	{ "c.srai",	"Cs,C>", 	MATCH_C_SRAI,	MASK_C_SRAI,	match_opcode },
	{ "c.andi",	"Cs,Co", 	MATCH_C_ANDI,	MASK_C_ANDI,	match_opcode },
	{ "c.sub",	"Cs,Ct", 	MATCH_C_SUB,	MASK_C_SUB,	match_opcode },
	{ "c.xor",	"Cs,Ct", 	MATCH_C_XOR,	MASK_C_XOR,	match_opcode },
	{ "c.or",	"Cs,Ct", 	MATCH_C_OR,	MASK_C_OR,	match_opcode },
	{ "c.and",	"Cs,Ct", 	MATCH_C_AND,	MASK_C_AND,	match_opcode },
	{ "c.subw",	"Cs,Ct", 	MATCH_C_SUBW,	MASK_C_SUBW,	match_opcode },
	{ "c.addw",	"Cs,Ct", 	MATCH_C_ADDW,	MASK_C_ADDW,	match_opcode },
	{ "c.j",	"Ca",		MATCH_C_J,	MASK_C_J,	match_opcode },
	{ "c.beqz",	"Cs,Cp", 	MATCH_C_BEQZ,	MASK_C_BEQZ,	match_opcode },
	{ "c.bnez",	"Cs,Cp", 	MATCH_C_BNEZ,	MASK_C_BNEZ,	match_opcode },
	{ "c.slli",	"d,C>", 	MATCH_C_SLLI,	MASK_C_SLLI,	match_opcode },
	{ "c.fldsp",	"D,Cn(Cc)", 	MATCH_C_FLDSP,	MASK_C_FLDSP,	match_opcode },
	{ "c.lwsp",	"d,Cm(Cc)", 	MATCH_C_LWSP,	MASK_C_LWSP,	match_opcode },
	{ "c.flwsp",	"D,Cm(Cc)", 	MATCH_C_FLWSP,	MASK_C_FLWSP,	match_opcode },
	{ "c.mv",	"d,CV", 	MATCH_C_MV,	MASK_C_MV,	match_opcode },
	{ "c.add",	"d,CV", 	MATCH_C_ADD,	MASK_C_ADD,	match_opcode },
	{ "c.fsdsp",	"CT,CN(Cc)", 	MATCH_C_FSDSP,	MASK_C_FSDSP,	match_opcode },
	{ "c.swsp",	"CV,CM(Cc)", 	MATCH_C_SWSP,	MASK_C_SWSP,	match_opcode },
	{ "c.fswsp",	"CT,CM(Cc)", 	MATCH_C_FSWSP,	MASK_C_FSWSP,	match_opcode },
	{ NULL, NULL, 0, 0, NULL },
};

struct riscv_op {
	char *name;
	char *type;
	char *fmt;
	int opcode;
	int funct3;
	int funct7; /* Or imm, depending on type. */
};

/*
 * Keep sorted by opcode, funct3, funct7 so some instructions
 * aliases will be supported (e.g. "mv" instruction alias)
 * Use same print format as binutils do.
 */
static struct riscv_op riscv_copcodes[] = {
	{ "c.srli",	"CI",	"Cs,Cw,C>",	1,   4, -1 },
	{ "c.mv",	"CI",	"d,CV",		2,   4, -1 },
	{ NULL, NULL, NULL, 0, 0, 0 }
};

static struct riscv_op riscv_opcodes[] = {
	{ "lb",		"I",	"d,o(s)",	3,   0, -1 },
	{ "lh",		"I",	"d,o(s)",	3,   1, -1 },
	{ "lw",		"I",	"d,o(s)",	3,   2, -1 },
	{ "ld",		"I",	"d,o(s)",	3,   3, -1 },
	{ "lbu",	"I",	"d,o(s)",	3,   4, -1 },
	{ "lhu",	"I",	"d,o(s)",	3,   5, -1 },
	{ "lwu",	"I",	"d,o(s)",	3,   6, -1 },
	{ "ldu",	"I",	"d,o(s)",	3,   7, -1 },
	{ "fence",	"I",	"",		15,  0, -1 },
	{ "fence.i",	"I",	"",		15,  1, -1 },
	{ "mv",		"I",	"d,s",		19,  0,  0 },
	{ "addi",	"I",	"d,s,j",	19,  0, -1 },
	{ "slli",	"R2",	"d,s,>",	19,  1,  0 },
	{ "slti",	"I",	"d,s,j",	19,  2, -1 },
	{ "sltiu",	"I",	"d,s,j",	19,  3, -1 },
	{ "xori",	"I",	"d,s,j",	19,  4, -1 },
	{ "srli",	"R2",	"d,s,>",	19,  5,  0 },
	{ "srai",	"R2",	"d,s,>",	19,  5, 0b010000 },
	{ "ori",	"I",	"d,s,j",	19,  6, -1 },
	{ "andi",	"I",	"d,s,j",	19,  7, -1 },
	{ "auipc",	"U",	"d,u",		23, -1, -1 },
	{ "sext.w",	"I",	"d,s",		27,  0,  0 },
	{ "addiw",	"I",	"d,s,j",	27,  0, -1 },
	{ "slliw",	"R",	"d,s,<",	27,  1,  0 },
	{ "srliw",	"R",	"d,s,<",	27,  5,  0 },
	{ "sraiw",	"R",	"d,s,<",	27,  5, 0b0100000 },
	{ "sb",		"S",	"t,q(s)",	35,  0, -1 },
	{ "sh",		"S",	"t,q(s)",	35,  1, -1 },
	{ "sw",		"S",	"t,q(s)",	35,  2, -1 },
	{ "sd",		"S",	"t,q(s)",	35,  3, -1 },
	{ "sbu",	"S",	"t,q(s)",	35,  4, -1 },
	{ "shu",	"S",	"t,q(s)",	35,  5, -1 },
	{ "swu",	"S",	"t,q(s)",	35,  6, -1 },
	{ "sdu",	"S",	"t,q(s)",	35,  7, -1 },
	{ "lr.w",	"R",	"d,t,0(s)",	47,  2, 0b0001000 },
	{ "sc.w",	"R",	"d,t,0(s)",	47,  2, 0b0001100 },
	{ "amoswap.w",	"R",	"d,t,0(s)",	47,  2, 0b0000100 },
	{ "amoadd.w",	"R",	"d,t,0(s)",	47,  2, 0b0000000 },
	{ "amoxor.w",	"R",	"d,t,0(s)",	47,  2, 0b0010000 },
	{ "amoand.w",	"R",	"d,t,0(s)",	47,  2, 0b0110000 },
	{ "amoor.w",	"R",	"d,t,0(s)",	47,  2, 0b0100000 },
	{ "amomin.w",	"R",	"d,t,0(s)",	47,  2, 0b1000000 },
	{ "amomax.w",	"R",	"d,t,0(s)",	47,  2, 0b1010000 },
	{ "amominu.w",	"R",	"d,t,0(s)",	47,  2, 0b1100000 },
	{ "amomaxu.w",	"R",	"d,t,0(s)",	47,  2, 0b1110000 },
	{ "lr.w.aq",	"R",	"d,t,0(s)",	47,  2, 0b0001000 },
	{ "sc.w.aq",	"R",	"d,t,0(s)",	47,  2, 0b0001100 },
	{ "amoswap.w.aq","R",	"d,t,0(s)",	47,  2, 0b0000110 },
	{ "amoadd.w.aq","R",	"d,t,0(s)",	47,  2, 0b0000010 },
	{ "amoxor.w.aq","R",	"d,t,0(s)",	47,  2, 0b0010010 },
	{ "amoand.w.aq","R",	"d,t,0(s)",	47,  2, 0b0110010 },
	{ "amoor.w.aq",	"R",	"d,t,0(s)",	47,  2, 0b0100010 },
	{ "amomin.w.aq","R",	"d,t,0(s)",	47,  2, 0b1000010 },
	{ "amomax.w.aq","R",	"d,t,0(s)",	47,  2, 0b1010010 },
	{ "amominu.w.aq","R",	"d,t,0(s)",	47,  2, 0b1100010 },
	{ "amomaxu.w.aq","R",	"d,t,0(s)",	47,  2, 0b1110010 },
	{ "amoswap.w.rl","R",	"d,t,0(s)",	47,  2, 0b0000110 },
	{ "amoadd.w.rl","R",	"d,t,0(s)",	47,  2, 0b0000001 },
	{ "amoxor.w.rl","R",	"d,t,0(s)",	47,  2, 0b0010001 },
	{ "amoand.w.rl","R",	"d,t,0(s)",	47,  2, 0b0110001 },
	{ "amoor.w.rl",	"R",	"d,t,0(s)",	47,  2, 0b0100001 },
	{ "amomin.w.rl","R",	"d,t,0(s)",	47,  2, 0b1000001 },
	{ "amomax.w.rl","R",	"d,t,0(s)",	47,  2, 0b1010001 },
	{ "amominu.w.rl","R",	"d,t,0(s)",	47,  2, 0b1100001 },
	{ "amomaxu.w.rl","R",	"d,t,0(s)",	47,  2, 0b1110001 },
	{ "amoswap.d",	"R",	"d,t,0(s)",	47,  3, 0b0000100 },
	{ "amoadd.d",	"R",	"d,t,0(s)",	47,  3, 0b0000000 },
	{ "amoxor.d",	"R",	"d,t,0(s)",	47,  3, 0b0010000 },
	{ "amoand.d",	"R",	"d,t,0(s)",	47,  3, 0b0110000 },
	{ "amoor.d",	"R",	"d,t,0(s)",	47,  3, 0b0100000 },
	{ "amomin.d",	"R",	"d,t,0(s)",	47,  3, 0b1000000 },
	{ "amomax.d",	"R",	"d,t,0(s)",	47,  3, 0b1010000 },
	{ "amominu.d",	"R",	"d,t,0(s)",	47,  3, 0b1100000 },
	{ "amomaxu.d",	"R",	"d,t,0(s)",	47,  3, 0b1110000 },
	{ "lr.d.aq",	"R",	"d,t,0(s)",	47,  3, 0b0001000 },
	{ "sc.d.aq",	"R",	"d,t,0(s)",	47,  3, 0b0001100 },
	{ "amoswap.d.aq","R",	"d,t,0(s)",	47,  3, 0b0000110 },
	{ "amoadd.d.aq","R",	"d,t,0(s)",	47,  3, 0b0000010 },
	{ "amoxor.d.aq","R",	"d,t,0(s)",	47,  3, 0b0010010 },
	{ "amoand.d.aq","R",	"d,t,0(s)",	47,  3, 0b0110010 },
	{ "amoor.d.aq",	"R",	"d,t,0(s)",	47,  3, 0b0100010 },
	{ "amomin.d.aq","R",	"d,t,0(s)",	47,  3, 0b1000010 },
	{ "amomax.d.aq","R",	"d,t,0(s)",	47,  3, 0b1010010 },
	{ "amominu.d.aq","R",	"d,t,0(s)",	47,  3, 0b1100010 },
	{ "amomaxu.d.aq","R",	"d,t,0(s)",	47,  3, 0b1110010 },
	{ "amoswap.d.rl","R",	"d,t,0(s)",	47,  3, 0b0000110 },
	{ "amoadd.d.rl","R",	"d,t,0(s)",	47,  3, 0b0000001 },
	{ "amoxor.d.rl","R",	"d,t,0(s)",	47,  3, 0b0010001 },
	{ "amoand.d.rl","R",	"d,t,0(s)",	47,  3, 0b0110001 },
	{ "amoor.d.rl",	"R",	"d,t,0(s)",	47,  3, 0b0100001 },
	{ "amomin.d.rl","R",	"d,t,0(s)",	47,  3, 0b1000001 },
	{ "amomax.d.rl","R",	"d,t,0(s)",	47,  3, 0b1010001 },
	{ "amominu.d.rl","R",	"d,t,0(s)",	47,  3, 0b1100001 },
	{ "amomaxu.d.rl","R",	"d,t,0(s)",	47,  3, 0b1110001 },
	{ "add",	"R",	"d,s,t",	51,  0,  0 },
	{ "sub",	"R",	"d,s,t",	51,  0,  0b0100000 },
	{ "mul",	"R",	"d,s,t",	51,  0,  0b0000001 },
	{ "sll",	"R",	"d,s,t",	51,  1,  0 },
	{ "slt",	"R",	"d,s,t",	51,  2,  0 },
	{ "sltu",	"R",	"d,s,t",	51,  3,  0 },
	{ "xor",	"R",	"d,s,t",	51,  4,  0 },
	{ "srl",	"R",	"d,s,t",	51,  5,  0 },
	{ "sra",	"R",	"d,s,t",	51,  5,  0b0100000 },
	{ "or",		"R",	"d,s,t",	51,  6,  0 },
	{ "and",	"R",	"d,s,t",	51,  7,  0 },
	{ "lui",	"U",	"d,u",		55, -1, -1 },
	{ "addw",	"R",	"d,s,t",	59,  0,  0 },
	{ "subw",	"R",	"d,s,t",	59,  0,  0b0100000 },
	{ "mulw",	"R",	"d,s,t",	59,  0,  1 },
	{ "sllw",	"R",	"d,s,t",	59,  1,  0 },
	{ "srlw",	"R",	"d,s,t",	59,  5,  0 },
	{ "sraw",	"R",	"d,s,t",	59,  5,  0b0100000 },
	{ "beq",	"SB",	"s,t,p",	99,  0,  -1 },
	{ "bne",	"SB",	"s,t,p",	99,  1,  -1 },
	{ "blt",	"SB",	"s,t,p",	99,  4,  -1 },
	{ "bge",	"SB",	"s,t,p",	99,  5,  -1 },
	{ "bltu",	"SB",	"s,t,p",	99,  6,  -1 },
	{ "bgeu",	"SB",	"s,t,p",	99,  7,  -1 },
	{ "jalr",	"I",	"d,s,j",	103,  0, -1 },
	{ "jal",	"UJ",	"a",		111, -1, -1 },
	{ "eret",	"I",	"",		115,  0, 0b000100000000 },
	{ "sfence.vm",	"I",	"",		115,  0, 0b000100000001 },
	{ "wfi",	"I",	"",		115,  0, 0b000100000010 },
	{ "csrrw",	"I",	"d,E,s",	115,  1, -1},
	{ "csrrs",	"I",	"d,E,s",	115,  2, -1},
	{ "csrrc",	"I",	"d,E,s",	115,  3, -1},
	{ "csrrwi",	"I",	"d,E,Z",	115,  5, -1},
	{ "csrrsi",	"I",	"d,E,Z",	115,  6, -1},
	{ "csrrci",	"I",	"d,E,Z",	115,  7, -1},
	{ NULL, NULL, NULL, 0, 0, 0 }
};

struct csr_op {
	char *name;
	int imm;
};

static struct csr_op csr_name[] = {
	{ "fflags",		0x001 },
	{ "frm",		0x002 },
	{ "fcsr",		0x003 },
	{ "cycle",		0xc00 },
	{ "time",		0xc01 },
	{ "instret",		0xc02 },
	{ "stats",		0x0c0 },
	{ "uarch0",		0xcc0 },
	{ "uarch1",		0xcc1 },
	{ "uarch2",		0xcc2 },
	{ "uarch3",		0xcc3 },
	{ "uarch4",		0xcc4 },
	{ "uarch5",		0xcc5 },
	{ "uarch6",		0xcc6 },
	{ "uarch7",		0xcc7 },
	{ "uarch8",		0xcc8 },
	{ "uarch9",		0xcc9 },
	{ "uarch10",		0xcca },
	{ "uarch11",		0xccb },
	{ "uarch12",		0xccc },
	{ "uarch13",		0xccd },
	{ "uarch14",		0xcce },
	{ "uarch15",		0xccf },
	{ "sstatus",		0x100 },
	{ "stvec",		0x101 },
	{ "sie",		0x104 },
	{ "sscratch",		0x140 },
	{ "sepc",		0x141 },
	{ "sip",		0x144 },
	{ "sptbr",		0x180 },
	{ "sasid",		0x181 },
	{ "cyclew",		0x900 },
	{ "timew",		0x901 },
	{ "instretw",		0x902 },
	{ "stime",		0xd01 },
	{ "scause",		0xd42 },
	{ "sbadaddr",		0xd43 },
	{ "stimew",		0xa01 },
	{ "mstatus",		0x300 },
	{ "mtvec",		0x301 },
	{ "mtdeleg",		0x302 },
	{ "mie",		0x304 },
	{ "mtimecmp",		0x321 },
	{ "mscratch",		0x340 },
	{ "mepc",		0x341 },
	{ "mcause",		0x342 },
	{ "mbadaddr",		0x343 },
	{ "mip",		0x344 },
	{ "mtime",		0x701 },
	{ "mcpuid",		0xf00 },
	{ "mimpid",		0xf01 },
	{ "mhartid",		0xf10 },
	{ "mtohost",		0x780 },
	{ "mfromhost",		0x781 },
	{ "mreset",		0x782 },
	{ "send_ipi",		0x783 },
	{ "miobase",		0x784 },
	{ "cycleh",		0xc80 },
	{ "timeh",		0xc81 },
	{ "instreth",		0xc82 },
	{ "cyclehw",		0x980 },
	{ "timehw",		0x981 },
	{ "instrethw",		0x982 },
	{ "stimeh",		0xd81 },
	{ "stimehw",		0xa81 },
	{ "mtimecmph",		0x361 },
	{ "mtimeh",		0x741 },
	{ NULL,	0 }
};

static char *reg_name[32] = {
	"zero",	"ra",	"sp",	"gp",	"tp",	"t0",	"t1",	"t2",
	"s0",	"s1",	"a0",	"a1",	"a2",	"a3",	"a4",	"a5",
	"a6",	"a7",	"s2",	"s3",	"s4",	"s5",	"s6",	"s7",
	"s8",	"s9",	"s10",	"s11",	"t3",	"t4",	"t5",	"t6"
};

static int32_t
get_cimm(CInstFmt i, char *type, uint32_t *val)
{
	int imm;

	imm = 0;

	if (strcmp(type, "CI") == 0) {
		imm = i.CIType.imm0_4;
		imm |= (i.CIType.imm5 << 5);
		*val = imm;
		//if (imm & (1 << 5))
		//	imm |= (0xfffff << 6);	/* sign extend */
	}

	return (imm);
}

static int32_t
get_imm(InstFmt i, char *type, uint32_t *val)
{
	int imm;

	imm = 0;

	if (strcmp(type, "I") == 0) {
		imm = i.IType.imm;
		*val = imm;
		if (imm & (1 << 11))
			imm |= (0xfffff << 12);	/* sign extend */

	} else if (strcmp(type, "S") == 0) {
		imm = i.SType.imm0_4;
		imm |= (i.SType.imm5_11 << 5);
		*val = imm;
		if (imm & (1 << 11))
			imm |= (0xfffff << 12);	/* sign extend */

	} else if (strcmp(type, "U") == 0) {
		imm = i.UType.imm12_31;
		*val = imm;

	} else if (strcmp(type, "UJ") == 0) {
		imm = i.UJType.imm12_19 << 12;
		imm |= i.UJType.imm11 << 11;
		imm |= i.UJType.imm1_10 << 1;
		imm |= i.UJType.imm20 << 20;
		*val = imm;
		if (imm & (1 << 20))
			imm |= (0xfff << 21);	/* sign extend */

	} else if (strcmp(type, "SB") == 0) {
		imm = i.SBType.imm11 << 11;
		imm |= i.SBType.imm1_4 << 1;
		imm |= i.SBType.imm5_10 << 5;
		imm |= i.SBType.imm12 << 12;
		*val = imm;
		if (imm & (1 << 12))
			imm |= (0xfffff << 12);	/* sign extend */
	}

	return (imm);
}

static int
oprint(struct riscv_op *op, vm_offset_t loc, int rd,
    int rs1, int rs2, uint32_t val, vm_offset_t imm)
{
	char *p;
	int i;

	p = op->fmt;

	db_printf("%s\t", op->name);

	while (*p) {
		if (strncmp("CV", p, 2) == 0)
			db_printf("%s", reg_name[imm]);

		else if (strncmp("d", p, 1) == 0)
			db_printf("%s", reg_name[rd]);

		else if (strncmp("s", p, 1) == 0)
			db_printf("%s", reg_name[rs1]);

		else if (strncmp("t", p, 1) == 0)
			db_printf("%s", reg_name[rs2]);

		else if (strncmp(">", p, 1) == 0)
			db_printf("0x%x", rs2);

		else if (strncmp("E", p, 1) == 0) {
			for (i = 0; csr_name[i].name != NULL; i++)
				if (csr_name[i].imm == val)
					db_printf("%s",
					    csr_name[i].name);
		} else if (strncmp("Z", p, 1) == 0)
			db_printf("%d", rs1);

		else if (strncmp("<", p, 1) == 0)
			db_printf("0x%x", rs2);

		else if (strncmp("j", p, 1) == 0)
			db_printf("%d", imm);

		else if (strncmp("u", p, 1) == 0)
			db_printf("0x%x", imm);

		else if (strncmp("a", p, 1) == 0)
			db_printf("0x%016lx", imm);

		else if (strncmp("p", p, 1) == 0)
			db_printf("0x%016lx", (loc + imm));

		else if (strlen(p) >= 4) {
			if (strncmp("o(s)", p, 4) == 0)
				db_printf("%d(%s)", imm, reg_name[rs1]);
			else if (strncmp("q(s)", p, 4) == 0)
				db_printf("%d(%s)", imm, reg_name[rs1]);
			else if (strncmp("0(s)", p, 4) == 0)
				db_printf("(%s)", reg_name[rs1]);
		}

		while (*p && strncmp(p, ",", 1) != 0)
			p++;

		if (*p) {
			db_printf(", ");
			p++;
		}
	}


	return (0);
}

static int
match_ctype(CInstFmt i, struct riscv_op *op, vm_offset_t loc)
{
	uint32_t val;
	//int found;
	int imm;

	printf("%s: %lx\n", __func__, loc);
	val = 0;
	imm = get_cimm(i, op->type, &val);
	if ((strcmp(op->type, "CI") == 0) && \
	    (op->funct3 == i.CIType.funct3)) {
		oprint(op, loc, i.CIType.rs1, 0, 0, val, imm);
		return (1);
	}

	return (0);
}

static int
match_type(InstFmt i, struct riscv_op *op, vm_offset_t loc)
{
	uint32_t val;
	int found;
	int imm;

	val = 0;
	imm = get_imm(i, op->type, &val);

	if (strcmp(op->type, "U") == 0) {
		oprint(op, loc, i.UType.rd, 0, 0, val, imm);
		return (1);
	}
	if (strcmp(op->type, "UJ") == 0) {
		oprint(op, loc, 0, 0, 0, val, (loc + imm));
		return (1);
	}
	if ((strcmp(op->type, "I") == 0) && \
	    (op->funct3 == i.IType.funct3)) {
		found = 0;
		if (op->funct7 != -1) {
			if (op->funct7 == i.IType.imm)
				found = 1;
		} else
			found = 1;

		if (found) {
			oprint(op, loc, i.IType.rd,
			    i.IType.rs1, 0, val, imm);
			return (1);
		}
	}
	if ((strcmp(op->type, "S") == 0) && \
	    (op->funct3 == i.SType.funct3)) {
		oprint(op, loc, 0, i.SType.rs1, i.SType.rs2,
		    val, imm);
		return (1);
	}
	if ((strcmp(op->type, "SB") == 0) && \
	    (op->funct3 == i.SBType.funct3)) {
		oprint(op, loc, 0, i.SBType.rs1, i.SBType.rs2,
		    val, imm);
		return (1);
	}
	if ((strcmp(op->type, "R2") == 0) && \
	    (op->funct3 == i.R2Type.funct3) && \
	    (op->funct7 == i.R2Type.funct7)) {
		oprint(op, loc, i.R2Type.rd, i.R2Type.rs1,
		    i.R2Type.rs2, val, imm);
		return (1);
	}
	if ((strcmp(op->type, "R") == 0) && \
	    (op->funct3 == i.RType.funct3) && \
	    (op->funct7 == i.RType.funct7)) {
		oprint(op, loc, i.RType.rd, i.RType.rs1,
		    i.RType.rs2, val, imm);
		return (1);
	}

	return (0);
}

static int
oprint1(struct riscv1_op *op, vm_offset_t loc, int insn)
{
	uint32_t rd, rs1, rs2;
	uint32_t val;
	int found;
	int imm;
	char *p;
	int i;

	p = op->fmt;

	db_printf("%s\t", op->name);

	while (*p) {
		if (strncmp("d", p, 1) == 0) {
			rd = (insn >> 7) & 0x1f;
			db_printf("%s", reg_name[rd]);

		} else if (strncmp("s", p, 1) == 0) {
			rs1 = (insn >> 15) & 0x1f;
			db_printf("%s", reg_name[rs1]);

		} else if (strncmp("t", p, 1) == 0) {
			rs2 = (insn >> 20) & 0x1f;
			db_printf("%s", reg_name[rs2]);

		} else if (strncmp("p", p, 1) == 0) {
			imm = ((insn >> 8) & 0xf) << 1;
			imm |= ((insn >> 25) & 0x3f) << 5;
			imm |= ((insn >> 7) & 0x1) << 11;
			imm |= ((insn >> 31) & 0x1) << 12;
			if (imm & (1 << 12))
				imm |= (0xfffff << 12);	/* sign extend */
			db_printf("0x%016lx", (loc + imm));

		} else if (strncmp("o(s)", p, 4) == 0) {
			rs1 = (insn >> 15) & 0x1f;
			imm = (insn >> 20) & 0xfff;
			if (imm & (1 << 11))
				imm |= (0xfffff << 12);	/* sign extend */
			db_printf("%d(%s)", imm, reg_name[rs1]);

		} else if (strncmp("q(s)", p, 4) == 0) {
			rs1 = (insn >> 15) & 0x1f;
			imm = (insn >> 7) & 0x1f;
			imm |= (insn >> 25) & 0x7f;
			if (imm & (1 << 11))
				imm |= (0xfffff << 12);	/* sign extend */
			db_printf("%d(%s)", imm, reg_name[rs1]);

		} else if (strncmp("0(s)", p, 4) == 0) {
			rs1 = (insn >> 15) & 0x1f;
			db_printf("(%s)", reg_name[rs1]);

		} else if (strncmp("a", p, 1) == 0) {
			/* imm[20|10:1|11|19:12] << 12 */
			imm = ((insn >> 21) & 0x3ff) << 1;
			imm |= ((insn >> 20) & 0x1) << 11;
			imm |= ((insn >> 12) & 0xff) << 12;
			imm |= ((insn >> 31) & 0x1) << 20;
			if (imm & (1 << 20))
				imm |= (0xfff << 20);	/* sign extend */
			db_printf("0x%lx", (loc + imm));

		} else if (strncmp("u", p, 1) == 0) {
			/* imm[31:12] << 12 */
			imm = (insn >> 12) & 0xfffff;
			if (imm & (1 << 20))
				imm |= (0xfff << 20);	/* sign extend */
			db_printf("0x%lx", imm);

		} else if (strncmp("j", p, 1) == 0) {
			/* imm[11:0] << 20 */
			imm = (insn >> 20) & 0xfff;
			if (imm & (1 << 11))
				imm |= (0xfffff << 12); /* sign extend */
			db_printf("%d", imm);

		} else if (strncmp(">", p, 1) == 0) {
			val = (insn >> 20) & 0x3f;
			db_printf("0x%x", val);

		} else if (strncmp("<", p, 1) == 0) {
			val = (insn >> 20) & 0x1f;
			db_printf("0x%x", val);

		} else if (strncmp("E", p, 1) == 0) {
			val = (insn >> 20) & 0xfff;
			found = 0;
			for (i = 0; csr_name[i].name != NULL; i++)
				if (csr_name[i].imm == val) {
					db_printf("%s",
					    csr_name[i].name);
					found = 1;
				}
			if (found == 0)
				db_printf("csr?");

		} else if (strncmp("Ct", p, 2) == 0) {
			rd = (insn >> 2) & 0x7;
			rd |= 0x8;
			db_printf("%s", reg_name[rd]);

		} else if (strncmp("Cs", p, 2) == 0) {
			rs1 = (insn >> 7) & 0x7;
			rs1 |= 0x8;
			db_printf("%s", reg_name[rs1]);

		} else if (strncmp("Cl(Cs)", p, 6) == 0) {
			rs1 = (insn >> 7) & 0x7;
			rs1 |= 0x8;
			imm = ((insn >> 10) & 0x7) << 3;
			imm |= ((insn >> 5) & 0x3) << 6;
			if (imm & (1 << 8))
				imm |= 0xffffff << 8;
			db_printf("%d(%s)", imm, reg_name[rs1]);

		} else if (strncmp("Ck(Cs)", p, 6) == 0) {
			rs1 = (insn >> 7) & 0x7;
			rs1 |= 0x8;
			imm = ((insn >> 10) & 0x7) << 3;
			imm |= ((insn >> 6) & 0x1) << 2;
			imm |= ((insn >> 5) & 0x1) << 6;
			if (imm & (1 << 8))
				imm |= 0xffffff << 8;
			db_printf("%d(%s)", imm, reg_name[rs1]);

		} else if (strncmp("Cn(Cc)", p, 6) == 0) {
			imm = ((insn >> 5) & 0x3) << 3;
			imm |= ((insn >> 12) & 0x1) << 5;
			imm |= ((insn >> 2) & 0x7) << 6;
			if (imm & (1 << 8))
				imm |= 0xffffff << 8;
			db_printf("%d(sp)", imm);

		} else if (strncmp("CN(Cc)", p, 6) == 0) {
			imm = ((insn >> 10) & 0x7) << 3;
			imm |= ((insn >> 7) & 0x7) << 6;
			if (imm & (1 << 8))
				imm |= 0xffffff << 8;
			db_printf("%d(sp)", imm);

		} else if (strncmp("Cu", p, 2) == 0) {
			imm = ((insn >> 2) & 0x1f) << 0;
			imm |= ((insn >> 12) & 0x1) << 5;
			if (imm & (1 << 5))
				imm |= (0x7ffffff << 5);	/* sign extend */
			db_printf("0x%lx", imm);

		} else if (strncmp("Co", p, 2) == 0) {
			imm = ((insn >> 2) & 0x1f) << 0;
			imm |= ((insn >> 12) & 0x1) << 5;
			if (imm & (1 << 5))
				imm |= (0x7ffffff << 5);	/* sign extend */
			db_printf("%d", imm);

		} else if (strncmp("Ca", p, 2) == 0) {
			/* imm[11|4|9:8|10|6|7|3:1|5] << 2 */
			imm = ((insn >> 3) & 0x7) << 1;
			imm |= ((insn >> 11) & 0x1) << 4;
			imm |= ((insn >> 2) & 0x1) << 5;
			imm |= ((insn >> 7) & 0x1) << 6;
			imm |= ((insn >> 6) & 0x1) << 7;
			imm |= ((insn >> 9) & 0x3) << 8;
			imm |= ((insn >> 8) & 0x1) << 10;
			imm |= ((insn >> 12) & 0x1) << 11;
			if (imm & (1 << 11))
				imm |= (0xfffff << 12);	/* sign extend */
			db_printf("0x%lx", (loc + imm));

		} else if (strncmp("CV", p, 2) == 0) {
			rs1 = (insn >> 2) & 0x1f;
			db_printf("%s", reg_name[rs1]);

		} else if (strncmp("P", p, 1) == 0) {
			if (insn & (1 << 27))
				db_printf("i");
			if (insn & (1 << 26))
				db_printf("o");
			if (insn & (1 << 25))
				db_printf("r");
			if (insn & (1 << 24))
				db_printf("w");

		} else if (strncmp("Q", p, 1) == 0) {
			if (insn & (1 << 23))
				db_printf("i");
			if (insn & (1 << 22))
				db_printf("o");
			if (insn & (1 << 21))
				db_printf("r");
			if (insn & (1 << 20))
				db_printf("w");
		}

		while (*p && strncmp(p, ",", 1) != 0)
			p++;

		if (*p) {
			db_printf(",");
			p++;
		}
	}

	//db_printf("%s: %s\n", op->name, op->fmt);

	return (0);
}

vm_offset_t
db_disasm(vm_offset_t loc, bool altfmt)
{
	struct riscv1_op *op1;
	struct riscv_op *op;
	CInstFmt ci;
	InstFmt i;
	int j;

	uint32_t insn;

	insn = db_get_value(loc, 4, 0);

	for (j = 0; riscv1_opcodes[j].name != NULL; j++) {
		op1 = &riscv1_opcodes[j];
		if (op1->match_func(op1, insn)) {
			oprint1(op1, loc, insn);
			return(loc + 4);
		}
	};

	insn = db_get_value(loc, 2, 0);

	for (j = 0; riscv1_copcodes[j].name != NULL; j++) {
		op1 = &riscv1_copcodes[j];
		if (op1->match_func(op1, insn)) {
			oprint1(op1, loc, insn);
			return(loc + 2);
		}
	};

	return(loc);
	//return(loc + INSN_SIZE);

	/* Try compressed first */
	ci.half = db_get_value(loc, 2, 0);

	//printf("ci.half %x\n", ci.half);

	/* First match opcode */
	for (j = 0; riscv_copcodes[j].name != NULL; j++) {
		op = &riscv_copcodes[j];
		if (op->opcode == ci.CRType.opcode) {
			if (match_ctype(ci, op, loc)) {
				db_printf("\n");
				return(loc + 2);
			}
		}
	}

	i.word = db_get_value(loc, INSN_SIZE, 0);

	/* First match opcode */
	for (j = 0; riscv_opcodes[j].name != NULL; j++) {
		op = &riscv_opcodes[j];
		if (op->opcode == i.RType.opcode) {
			if (match_type(i, op, loc))
				break;
		}
	}

	db_printf("\n");
	return(loc + INSN_SIZE);
}

/*********************************************************
 *                                                       *
 *  EE511 Midterm Project                                *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *  April, 2018                                          *
 *                                                       *
 *  Written by : Daesung Kim (dskim@ics.kaist.ac.kr)     *
 *                                                       *
 *********************************************************/

#include "iss.h"

/* newly implemented functions */
void lsl_i(uint16_t inst);
void lsr_i(uint16_t inst);
void asr_i(uint16_t inst);
void add_r1(uint16_t inst);
void sub_r(uint16_t inst);
void add_3i(uint16_t inst);
void sub_3i(uint16_t inst);
void mov_i(uint16_t inst);
void cmp_i(uint16_t inst);
void add_8i(uint16_t inst);
void sub_8i(uint16_t inst);

void and_r(uint16_t inst);
void eor_r(uint16_t inst);
void lsl_r(uint16_t inst);
void lsr_r(uint16_t inst);
void asr_r(uint16_t inst);
void adc_r(uint16_t inst);
void sbc_r(uint16_t inst);
void ror_r(uint16_t inst);
void tst_r(uint16_t inst);
void rsb_i(uint16_t inst);
void cmp_r1(uint16_t inst);
void cmn_r(uint16_t inst);
void orr_r(uint16_t inst);
void mul(uint16_t inst);
void bic_r(uint16_t inst);
void mvn_r(uint16_t inst);

void add_r2(uint16_t inst);
void cmp_r2(uint16_t inst);
void mov_r1(uint16_t inst);
void bx(uint16_t inst);
void blx(uint16_t inst);

void ldr_pc_rel(uint16_t inst);

void str_r(uint16_t inst);
void strh_r(uint16_t inst);
void strb_r(uint16_t inst);
void ldrsb_r(uint16_t inst);
void ldr_r(uint16_t inst);
void ldrh_r(uint16_t inst);
void ldrb_r(uint16_t inst);
void ldrsh_r(uint16_t inst);
void str_5i(uint16_t inst);
void ldr_5i(uint16_t inst);
void strb_i(uint16_t inst);
void ldrb_i(uint16_t inst);
void strh_i(uint16_t inst);
void ldrh_i(uint16_t inst);
void str_8i(uint16_t inst);
void ldr_8i(uint16_t inst);

void adr(uint16_t inst);
void add_sp_i1(uint16_t inst);

void add_sp_i2(uint16_t inst);
void sub_sp_i(uint16_t inst);
void sxth(uint16_t inst);
void sxtb(uint16_t inst);
void uxth(uint16_t inst);
void uxtb(uint16_t inst);
void push(uint16_t inst);
void rev(uint16_t inst);
void rev16(uint16_t inst);
void revsh(uint16_t inst);
void pop(uint16_t inst);

void stm(uint16_t inst);
void ldm(uint16_t inst);

void b_conditional(uint16_t inst);



/* given functions */
void b_unconditional(uint16_t inst);
void bl(uint32_t inst);

void process(uint16_t inst)
{
  uint16_t inst2;
  uint32_t inst32;

  /* your code here */
  /* 
	 LSLi
	 LSRi
	 ASRi
	 ADDr1
	 SUBr
	 ADD3i
	 SUB3i
	 MOVi
	 CMPi
	 ADD8i
	 SUB8i
   */
  if (INST(15, 14) == 0x0) {
	  /* LSLi */
	  if (INST(13, 11) == 0x0) {
		  lsl_i(inst);
	  }
	  /* LSRi */
	  else if (INST(13, 11) == 0x1) {
		  lsr_i(inst);
	  }
	  /* ASRi */
	  else if (INST(13, 11) == 0x2) {
		  asr_i(inst);
	  }
	  else if (INST(13, 11) == 0x3) {
		  /* ADDr1 */
		  if (INST(10, 9) == 0x0) {
			  add_r1(inst);
		  }
		  /* SUBr */
		  else if (INST(10,9) == 0x1) {
			  sub_r(inst);
		  }
		  /* ADD3i */
		  else if (INST(10,9) == 0x2) {
			  add_3i(inst);
		  }
		  /* SUB3i */
		  else if (INST(10,9) == 0x3) {
			  sub_3i(inst);
		  }
	  }
	  /* MOVi */
	  else if (INST(13, 11) == 0x4) {
		  mov_i(inst);
	  }
	  /* CMPi */
	  else if (INST(13, 11) == 0x5) {
		  cmp_i(inst);
	  }
	  /* ADD8i */
	  else if (INST(13, 11) == 0x6) {
		  add_8i(inst);
	  }
	  /* SUB8i */
	  else if (INST(13, 11) == 0x7) {
		  sub_8i(inst);
	  }
  }

  /*
	 ANDr
	 EORr
	 LSLr
	 LSRr
	 ASRr
	 ADCr
	 SBCr
	 RORr
	 TSTr
	 RSBi
	 CMPr1
	 CMNr
	 ORRr
	 MUL
	 BICr
	 MVNr
   */
  else if (INST(15, 10) == 0x10) {
	  /* ANDr */
	  if (INST(9, 6) == 0x0) {
		  and_r(inst);
	  }
	  /* EORr */
	  else if (INST(9, 6) == 0x1) {
		  eor_r(inst);
	  }
	  /* LSLr */
	  else if (INST(9, 6) == 0x2) {
		  lsl_r(inst);
	  }
	  /* LSRr */
	  else if (INST(9, 6) == 0x3) {
		  lsr_r(inst);
	  }
	  /* ASRr */
	  else if (INST(9, 6) == 0x4) {
		  asr_r(inst);
	  }
	  /* ADCr */
	  else if (INST(9, 6) == 0x5) {
		  adc_r(inst);
	  }
	  /* SBCr */
	  else if (INST(9, 6) == 0x6) {
		  sbc_r(inst);
	  }
	  /* RORr */
	  else if (INST(9, 6) == 0x7) {
		  ror_r(inst);
	  }
	  /* TSTr */
	  else if (INST(9, 6) == 0x8) {
		  tst_r(inst);
	  }
	  /* RSBi */
	  else if (INST(9, 6) == 0x9) {
		  rsb_i(inst);
	  }
	  /* CMPr1 */
	  else if (INST(9, 6) == 0xA) {
		  cmp_r1(inst);
	  }
	  /* CMNr */
	  else if (INST(9, 6) == 0xB) {
		  cmn_r(inst);
	  }
	  /* ORRr */
	  else if (INST(9, 6) == 0xC) {
		  orr_r(inst);
	  }
	  /* MUL */
	  else if (INST(9, 6) == 0xD) {
		  mul(inst);
	  }
	  /* BICr */
	  else if (INST(9, 6) == 0xE) {
		  bic_r(inst);
	  }
	  /* MVNr */
	  else if (INST(9, 6) == 0xF) {
		  mvn_r(inst);
	  }
  }

  /*
	 ADDr2
	 CMPr2
	 MOVr1
	 BX
	 BLX
   */
  else if (INST(15, 10) == 0x11) {
	  /* ADDr2 */
	  if (INST(9, 8) == 0x0) {
		  add_r2(inst);
	  }
	  /* CMPr2 */
	  else if (INST(9, 8) == 0x1) {
		  cmp_r2(inst);
	  }
	  /* MOVr1 */
	  else if (INST(9, 8) == 0x2) {
		  mov_r1(inst);
	  }
	  else if (INST(9, 8) == 0x3) {
		  /* BX */
		  if (INST_(7) == 0x0) {
			  bx(inst);
		  }
		  /* BLX */
		  else if (INST_(7) == 0x1) {
			  blx(inst);
		  }
	  }
  }

  /*
	 LDR(PC-relative)
   */
  else if (INST(15, 11) == 0x09) {
	  ldr_pc_rel(inst);
  }

  /*
	 STRr
	 STRHr
	 STRBr
	 LDRSBr
	 LDRr
	 LDRHr
	 LDRBr
	 LDRSHr
   */
  else if (INST(15, 12) == 0x05) {
	  /* STRr */
	  if (INST(11, 9) == 0x0) {
		  str_r(inst);
	  }
	  /* STRHr */
	  else if (INST(11, 9) == 0x1) {
		  strh_r(inst);
	  }
	  /* STRBr */
	  else if (INST(11, 9) == 0x2) {
		  strb_r(inst);
	  }
	  /* LDRSBr */
	  else if (INST(11, 9) == 0x3) {
		  ldrsb_r(inst);
	  }
	  /* LDRr */
	  else if (INST(11, 9) == 0x4) {
		  ldr_r(inst);
	  }
	  /* LDRHr */
	  else if (INST(11, 9) == 0x5) {
		  ldrh_r(inst);
	  }
	  /* LDRBr */
	  else if (INST(11, 9) == 0x6) {
		  ldrb_r(inst);
	  }
	  /* LDRSHr */
	  else if (INST(11, 9) == 0x7) {
		  ldrsh_r(inst);
	  }
  }

  /*
	 STR5i
	 LDR5i
	 STRBi
	 LDRBi
   */
  else if (INST(15, 13) == 0x03) {
	  /* STR5i */
	  if (INST(12, 11) == 0x0) {
		  str_5i(inst);
	  }
	  /* LDR5i */
	  else if (INST(12, 11) == 0x1) {
		  ldr_5i(inst);
	  }
	  /* STRBi */
	  else if (INST(12, 11) == 0x2) {
		  strb_i(inst);
	  }
	  /* LDRBi */
	  else if (INST(12, 11) == 0x3) {
		  ldrb_i(inst);
	  }
  }

  /*
	 STRHi
	 LDRHi
	 STR8i
	 LDR8i
   */
  else if (INST(15, 13) == 0x04) {
	  /* STRHi */
	  if (INST(12, 11) == 0x0) {
		  strh_i(inst);
	  }
	  /* LDRHi */
	  else if (INST(12, 11) == 0x1) {
		  ldrh_i(inst);
	  }
	  /* STR8i */
	  else if (INST(12, 11) == 0x2) {
		  str_8i(inst);
	  }
	  /* LDR8i */
	  else if (INST(12, 11) == 0x3) {
		  ldr_8i(inst);
	  }
  }

  /*
	 ADR
   */
  else if (INST(15, 11) == 0x14) {
	  adr(inst);
  }

  /*
	 ADDSPi1
   */
  else if (INST(15, 11) == 0x15) {
	  add_sp_i1(inst);
  }

  /*
	 ADDSPi2
	 SUBSPi
	 SXTH
	 SXTB
	 UXTH
	 UXTB
	 PUSH
	 REV
	 REV16
	 REVSH
	 POP
   */
  else if (INST(15, 12) == 0x0B) {
	  if (INST(11, 8) == 0x0) {
		  /* ADDSPi2 */
		  if (INST_(7) == 0x0) {
			  add_sp_i2(inst);
		  }
		  /* SUBSPi */
		  else if (INST_(7) == 0x1) {
			  sub_sp_i(inst);
		  }
	  }
	  else if (INST(11, 8) == 0x2) {
		  /* SXTH */
		  if (INST(7, 6) == 0x0) {
			  sxth(inst);
		  }
		  /* SXTB */
		  else if (INST(7, 6) == 0x1) {
			  sxtb(inst);
		  }
		  /* UXTH */
		  else if (INST(7, 6) == 0x2) {
			  uxth(inst);
		  }
		  /* UXTB */
		  else if (INST(7, 6) == 0x3) {
			  uxtb(inst);
		  }
	  }
	  /* PUSH */
	  else if (INST(11, 9) == 0x2) {
		  push(inst);
	  }
	  else if (INST(11, 9) == 0x5) {
		  if (INST_(8) == 0x0) {
			  /* REV */
			  if (INST(7, 6) == 0x0) {
				  rev(inst);
			  }
			  /* REV16 */
			  else if (INST(7, 6) == 0x1) {
				  rev16(inst);
			  }
			  /* REVSH */
			  else if (INST(7, 6) == 0x3) {
				  revsh(inst);
			  }
		  }
	  }
	  /* POP */
	  else if (INST(11, 9) == 0x6) {
		  pop(inst);
	  }
  }

  /*
	 STM
   */
  else if (INST(15, 11) == 0x18) {
	  stm(inst);
  }

  /*
	 LDM
   */
  else if (INST(15, 11) == 0x19) {
	  ldm(inst);
  }

  /*
	 B (conditional)
   */
  else if (INST(15, 12) == 0x0D) {
	  b_conditional(inst);
  }

  /* Already Implemented. */
  else if (INST(15, 11) == 0x1C) {
	  b_unconditional(inst);
  }
  else if (INST(15, 11) == 0x1E) {
	  inst2 = read_halfword(PC + 2);
	  inst32 = ((uint32_t) inst << 16) | ((uint32_t) inst2);
	  if (extract16_(inst2, 14) && extract16_(inst2, 12))
		bl(inst32);
  }
}

/* add additional functions for other instructions */

/* newly implemented functions */
void lsl_i(uint16_t inst)
{

}

void lsr_i(uint16_t inst)
{

}

void asr_i(uint16_t inst)
{

}

void add_r1(uint16_t inst)
{

}

void sub_r(uint16_t inst)
{

}

void add_3i(uint16_t inst)
{

}

void sub_3i(uint16_t inst)
{

}

void mov_i(uint16_t inst)
{

}

void cmp_i(uint16_t inst)
{

}

void add_8i(uint16_t inst)
{

}

void sub_8i(uint16_t inst)
{

}

{

}

void and_r(uint16_t inst)
{

}

void eor_r(uint16_t inst)
{

}

void lsl_r(uint16_t inst)
{

}

void lsr_r(uint16_t inst)
{

}

void asr_r(uint16_t inst)
{

}

void adc_r(uint16_t inst)
{

}

void sbc_r(uint16_t inst)
{

}

void ror_r(uint16_t inst)
{

}

void tst_r(uint16_t inst)
{

}

void rsb_i(uint16_t inst)
{

}

void cmp_r1(uint16_t inst)
{

}

void cmn_r(uint16_t inst)
{

}

void orr_r(uint16_t inst)
{

}

void mul(uint16_t inst)
{

}

void bic_r(uint16_t inst)
{

}

void mvn_r(uint16_t inst)
{

}


void add_r2(uint16_t inst)
{

}

void cmp_r2(uint16_t inst)
{

}

void mov_r1(uint16_t inst)
{

}

void bx(uint16_t inst)
{

}

void blx(uint16_t inst)
{

}


void ldr_pc_rel(uint16_t inst)
{

}


void str_r(uint16_t inst)
{

}

void strh_r(uint16_t inst)
{

}

void strb_r(uint16_t inst)
{

}

void ldrsb_r(uint16_t inst)
{

}

void ldr_r(uint16_t inst)
{

}

void ldrh_r(uint16_t inst)
{

}

void ldrb_r(uint16_t inst)
{

}

void ldrsh_r(uint16_t inst)
{

}

void str_5i(uint16_t inst)
{

}

void ldr_5i(uint16_t inst)
{

}

void strb_i(uint16_t inst)
{

}

void ldrb_i(uint16_t inst)
{

}

void strh_i(uint16_t inst)
{

}

void ldrh_i(uint16_t inst)
{

}

void str_8i(uint16_t inst)
{

}

void ldr_8i(uint16_t inst)
{

}


void adr(uint16_t inst)
{

}

void add_sp_i1(uint16_t inst)
{

}


void add_sp_i2(uint16_t inst)
{

}

void sub_sp_i(uint16_t inst)
{

}

void sxth(uint16_t inst)
{

}

void sxtb(uint16_t inst)
{

}

void uxth(uint16_t inst)
{

}

void uxtb(uint16_t inst)
{

}

void push(uint16_t inst)
{

}

void rev(uint16_t inst)
{

}

void rev16(uint16_t inst)
{

}

void revsh(uint16_t inst)
{

}

void pop(uint16_t inst)
{

}


void stm(uint16_t inst)
{

}

void ldm(uint16_t inst)
{

}


void b_conditional(uint16_t inst)
{

}

/* Given functions */

void b_unconditional(uint16_t inst)
{
  uint32_t imm11 = INST(10, 0);
  uint32_t address;

  address = PC + 4 + signExtend32((imm11 << 1), 12);
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

void bl(uint32_t inst)
{
  uint32_t S = INST32_(10 + 16);
  uint32_t imm10 = INST32(9 + 16, 0 + 16);
  uint32_t J1 = INST32_(13);
  uint32_t J2 = INST32_(11);
  uint32_t imm11 = INST32(10, 0);
  uint32_t I1, I2, imm32, address;

  I1 = !(J1 ^ S);
  I2 = !(J2 ^ S);
  imm32 = sign_extend((S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1), 25);

  LR = PC | 0x00000001;

  address = PC + 4 + imm32;
  branch = 1;
  PC = address & 0xFFFFFFFE;
}


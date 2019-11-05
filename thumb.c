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
#include <stdbool.h>
/* Shifts applied to a register. */
enum SRType {SRType_LSL, SRType_LSR, SRType_ASR, SRType_ROR, SRType_RRX};

/* static funcitons */
static int LowestSetBit(uint32_t x);
static int BitCount(uint32_t x);
static void Shift(uint32_t* result, uint32_t value , int value_bit, enum SRType srtype, int amount, int carry_in);
static void Shift_C(uint32_t* result, int* carry_out, uint32_t value , int value_bit, enum SRType srtype, int amount, int carry_in);
static void LSL_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount);
static void LSR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount);
static void ASR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount);
static void ROR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount);
static void RRX_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount);
static void AddWithCarry(uint32_t* result, int* carry, int* overflow, uint32_t x, uint32_t y, int cin, int N);

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
  uint32_t imm5 = INST(10, 6);
  uint32_t d = INST(2, 0);
  uint32_t m = INST(5, 3);
  bool setflags = true;

  uint32_t shift_n = imm5;
  uint32_t Rm = R[m];

  uint32_t result;
  uint32_t carry;

  if (imm5 == 0x0)
	{
	  /* SEE MOV (register) */
	  result = Rm;
	  R[d] = (uint32_t)result;
	  APSR.N = extract32_(R[d], 31);
	  APSR.Z = (R[d] == 0);
	}
  else 
	{
	  /* result, carry = Shift_C(R[m], SRType_LSL, shift_n, APSR.C */
	  Shift_C(&result, &carry, R[m], 32, SRType_LSL, shift_n, APSR.C);

	  /* store values */
	  R[d] = result;

	  /* setflags */
	  APSR.N = extract32_(R[d], 31);
	  APSR.Z = (R[d] == 0);
	  APSR.C = carry;
	  // APSR.V unchanged.
	}
}

void lsr_i(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t m = INST(5, 3);
  uint32_t imm5 = INST(10, 6);
  uint32_t shift_n = imm5;

  uint32_t result;
  int carry;
  /* (result, carry) = Shift_C(R[m], SRType_LSR, shift_n, APSR.C */
  Shift_C(&result, &carry, R[m], 32, SRType_LSR, shift_n, APSR.C);
  
  R[d] = result;
  
  /* set flags */
  APSR.N = extract32_(R[d], 31);
  APSR.Z = (R[d] == 0);
  APSR.C = carry;
  /* APSR.V unchanged */
}

void asr_i(uint16_t inst)
{
  /* Encoding */
  uint32_t imm5 = INST(10, 6);
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);
  uint32_t shift_n = imm5;

  uint32_t result;
  int carry;

  Shift_C(&result, &carry, R[m], 32, SRType_ASR, shift_n, APSR.C);

  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged. 
}

void add_r1(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t d = INST(2, 0);
  uint32_t shifted;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;
  /* shifted = Shift(R[m], shift_t, shift_n, APSR.C) */
  
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  AddWithCarry(&result, &carry, &overflow, R[n], shifted, 0, 32);

  if (d == 15)
	{
	  /* ALUWritePC(result) */
	  PC = result & 0xFFFFFFFE;
	}
  else
	{
	  R[d] = result;

	  APSR.N = extract32_(result, 31);
	  APSR.Z = (result == 0);
	  APSR.C = carry;
	  APSR.V = overflow;
	}
}

void sub_r(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t d = INST(2, 0);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(shifted), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], ~shifted, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void add_3i(uint16_t inst)
{
  /* Encoding */
  uint32_t imm3 = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t d = INST(2, 0);
  uint32_t imm32 = zeroExtend32(imm3);

  uint32_t result;
  int carry, overflow;
  AddWithCarry(&result, &carry, &overflow, R[n], imm32, 0, 32);

  /* store value */
  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void sub_3i(uint16_t inst)
{
  /* Encoding */
  uint32_t imm3 = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t d = INST(2, 0);
  uint32_t imm32 = zeroExtend32(imm3);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(imm32), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], ~imm32, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void mov_i(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(10, 8);
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8); /* zero extend */
  int carry = APSR.C;
  
  uint32_t result = imm32;
  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  /* APSR.V unchanged */
}

void cmp_i(uint16_t inst)
{
  /* encoding */
  int32_t n = INST(10, 8);
  int32_t imm8 = INST(7, 0);
  int32_t imm32 = zeroExtend32(imm8);

  /* result, carry, overflow = AddWithCarry(R[n], NOT(imm32), '1')*/ 
  uint32_t result;
  int carry, overflow;
  AddWithCarry(&result, &carry, &overflow, R[n], ~imm32, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void add_8i(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(10, 8);
  uint32_t n = d;
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8);

 /* result, carry, overflow = AddWithCarry(R[n], imm32, '0')*/ 
  uint32_t result;
  int carry, overflow;
  AddWithCarry(&result, &carry, &overflow, R[n], imm32, 0, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void sub_8i(uint16_t inst)
{
  uint32_t d = INST(10, 8);
  uint32_t n = d;
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8);

  /* result, carry, overflow = AddWithCarry(R[n], NOT(imm32), '1')*/
  uint32_t result;
  int carry, overflow;
  AddWithCarry(&result, &carry, &overflow, R[n], ~imm32, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void and_r(uint16_t inst)
{
  /* Encoding */
  int32_t d = INST(2, 0);
  int32_t n = d;
  int32_t m = INST(5, 3);

  /* shifted_t = SRType_LSL */
  int32_t shifted_n = 0;

  int32_t shifted, carry, result;

  /* result, carry = Shift_C(R[m], SRType_LSL, shift_n, APSR.C */
  Shift_C(&result, &carry, R[m], 32, SRType_LSL, shifted_n, APSR.C);
 
  /* and operation */
  result = R[n] & shifted;
  /* store value to R[d] */
  R[d] = result;

  /* setflags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged. 
}

void eor_r(uint16_t inst)
{
  /* encoding */
  int32_t d = INST(2, 0);
  int32_t n = d;
  int32_t m = INST(5, 3);

  /* shift_t, shift_n = (SRTypeLSL, 0) */
  int32_t shift_n = 0;
  int32_t shifted, result, carry;

  /* (shifted, carry) = Shift_C(R[m], shift_t, shift_n, APSR_C) */
  Shift_C(&result, &carry, R[m], 32, SRType_LSL, shift_n, APSR.C);
  
  /* eor operation */
  result = R[n] ^ shifted;
  /* store value to R[d] */
  R[d] = result;

  /* setflags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged. 
}

void lsl_r(uint16_t inst)
{
  /* Encoding */
  int32_t d = INST(2, 0);
  int32_t n = d;
  int32_t m = INST(5, 3);

  int32_t shift_n = extract32(R[m], 7, 0);

  int32_t result;
  int carry;
  /* result, carry = Shift_C(R[m], SRType_LSL, shift_n, APSR.C */
  Shift_C(&result, &carry, R[m], 32, SRType_LSL, shift_n, APSR.C);
  
  /* store values */
  R[d] = (int32_t) result;

  /* setflags */
  APSR.N = extract32_(R[d], 31);
  APSR.Z = (R[d] == 0);
  APSR.C = carry;
  // APSR.V unchanged.
}

void lsr_r(uint16_t inst)
{
  /* Encoding */
  int32_t d = INST(2, 0);
  int32_t n = d;
  int32_t m = INST(5, 3);

  int32_t shift_n = extract32(R[m], 7, 0);

  int32_t result;
  int carry;
  /* result, carry = Shift_C(R[n], SRType_LSR, shift_n, APSR.C */
  Shift_C(&result, &carry, R[n], 32, SRType_LSR, shift_n, APSR.C);

  /* store value */
  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged.
}

void asr_r(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t n = d;
  uint32_t m = INST(5, 3);

  uint32_t shifted_n = extract32(R[m], 7, 0);

  uint32_t result;
  int carry;
  /* (result, carry) = Shift_C(R[n], SRType_ASR, shift_n, APSR.C */
  Shift_C(&result, &carry, R[n], 32, SRType_ASR, shifted_n, APSR.C);

  /* store value */
  R[d] = result;
  
  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged 
}

void adc_r(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t n = d;
  uint32_t m = INST(5, 3);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], shifted, APSR.C, 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], shifted, APSR.C, 32);

  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void sbc_r(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t n = INST(2, 0);
  uint32_t d = INST(2, 0);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(shifted), APSR.C, 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], ~shifted, APSR.C, 32);

  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void ror_r(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t n = d;
  uint32_t m = INST(5, 3);
  uint32_t shift_n = extract32(R[m], 7, 0);
  
  uint32_t result;
  int carry;
  Shift_C(&result, &carry, R[n], 32, SRType_ROR, shift_n, APSR.C);

  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged. 
}

void tst_r(uint16_t inst)
{
  uint32_t n = INST(2, 0);
  uint32_t m = INST(5, 3);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted, result;
  int carry;
  Shift_C(&shifted, &carry, R[m] , 32, shift_t, shift_n, APSR.C);

  result = R[n] & shifted;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  //APSR.V = overflow;
}

void rsb_i(uint16_t inst)
{
  uint32_t d = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t imm32 = 0x00000000;

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(shifted), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, ~R[n], imm32, 1, 32);

  R[d] = result;

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void cmp_r1(uint16_t inst)
{
  /* encoding */
  uint32_t m = INST(5, 3);
  uint32_t n = INST(2, 0);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(shifted), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], ~shifted, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void cmn_r(uint16_t inst)
{
  /* encoding */
  uint32_t m = INST(5, 3);
  uint32_t n = INST(2, 0);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], shifted, '0', 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], shifted, 0, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void orr_r(uint16_t inst)
{
  /* encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);
  uint32_t n = d;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t result, shifted;
  int carry;
  /* shifted, carry = Shift_C(R[m], shift_t, shift_n, APSR.C */
  Shift_C(&shifted, &carry, R[m], 32, shift_t, shift_n, APSR.C);
  result = R[n] | shifted;

  R[d] = result;

  /* setflags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged.
}

void mul(uint16_t inst)
{
  uint32_t d = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t m = d;

  int32_t op1 = R[n];
  int32_t op2 = R[m];

  uint64_t result = op1 * op2;

  R[d] = (uint32_t)( result & 0xFFFFFFFF);

  /* set flags */
  APSR.N = extract32_(R[d], 31);
  APSR.Z = (R[d] == 0);
  // APSR.C unchanged
  // APSR.V unchanged
}

void bic_r(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t n = d;
  uint32_t m = INST(5, 3);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t result, shifted;
  int carry;
  /* (result, carry) = Shift_C(R[m], shift_t, shift_n, APSR.C */
  Shift_C(&shifted, &carry, R[m], 32, shift_t, shift_n, APSR.C);

  result = R[n] & (~shifted);

  R[d] = result;
  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged 
}

void mvn_r(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(2, 0);
  uint32_t m = INST(5, 3);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t result, shifted;
  int carry;
  /* (result, carry) = Shift_C(R[m], shift_t, shift_n, APSR.C */
  Shift_C(&shifted, &carry, R[m], 32, shift_t, shift_n, APSR.C);
  result = ~shifted;

  R[d] = result;
  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  // APSR.V unchanged 
}


void add_r2(uint16_t inst)
{
  /* Encoding */
  uint32_t DN = INST_(7);
  uint32_t m = INST(6, 3);
  uint32_t d = INST(2, 0);
  uint32_t n = d;

  if ( (((DN<<4) | R[n]) == 0xD) || (R[m] == 0xD))
	{
	  /* SEE ADD (SP plus register) */
	}
  else
	{
	  uint32_t Rd = zeroExtend32( (DN << 3) | R[d]);
	  uint32_t Rm = zeroExtend32(R[m]);

	  enum SRType shift_t = SRType_LSL;
	  uint32_t shift_n = 0;

	  uint32_t shifted;
	  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

	  uint32_t result;
	  int carry, overflow;
	  /* (result, carry, overflow) = AddWithCarry(R[n], shifted, '0', 32) */
	  AddWithCarry(&result, &carry, &overflow, R[n], shifted, 0, 32);
	  if (d == 15)
		{
		  PC = result & 0xFFFFFFFE;
		}
	  else
		{
		  R[d] = result;

		  /* set flags */
		  APSR.N = extract32_(result, 31);
		  APSR.Z = (result == 0);
		  APSR.C = carry;
		  APSR.V = overflow;
		}
	}
}

void cmp_r2(uint16_t inst)
{
  /* Encoding */
  uint32_t N = INST_(7);
  uint32_t m = INST(6, 3);
  uint32_t n = INST(2, 0);

  uint32_t Rm = INST(5, 3);
  uint32_t Rn = zeroExtend32(R[m]);
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t shifted;
  Shift(&shifted, R[m] , 32, shift_t, shift_n, APSR.C);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(R[n], NOT(shifted), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, R[n], ~shifted, 1, 32);

  /* set flags */
  APSR.N = extract32_(result, 31);
  APSR.Z = (result == 0);
  APSR.C = carry;
  APSR.V = overflow;
}

void mov_r1(uint16_t inst)
{
  /* Encoding */
  uint32_t D = INST_(7);
  uint32_t m = INST(6, 3);
  uint32_t d = INST(2, 0);
  d = (D<<3) | d;

  uint32_t result = R[m];

  if (d == 15)
	{
	  /* ALUWritePC(result) */
	  /* BranchWritePC(result) */
	  branch = 1;
	  PC = result & 0xFFFFFFFE;
	}
  else
	{
	  R[d] = result;

	  /* no setflags. */
	}
}

void bx(uint16_t inst)
{
  uint32_t m = INST(6, 3);
  /* BXWritePC(R[m]) */

  uint32_t address = R[m];
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

void blx(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(6, 3);
  uint32_t address;

  address = R[m];
  LR = (PC - 2) | 0x00000001;

  /* BLXWritePC */
  branch = 1;
  PC = address & 0xFFFFFFFE;

}


void ldr_pc_rel(uint16_t inst)
{
  uint32_t t = INST(10, 8);
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8 << 2);

  bool add = true;

  uint32_t base = (PC >> 2) << 2;
  // add is true
  uint32_t address = base + imm32;

  /* R[t] = MemU[address, 4] */
  R[t] = read_word(address);
}


void str_r(uint16_t inst)
{
  /* encoding */
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t m = INST(8, 6);

  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  uint32_t address = R[n] + offset;
  /* MemU[address, 4] = R[t] */
  write_word(address, R[t]);
}

void strh_r(uint16_t inst)
{
  /* encoding */
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t m = INST(8, 6);

  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  uint32_t address = R[n] + offset;
  /* MemU[address, 2] = R[t]<15, 0> */
  write_halfword(address, extract32(R[t], 15, 0));
}

void strb_r(uint16_t inst)
{
  /* encoding */
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t m = INST(8, 6);

  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  uint32_t address = R[n] + offset;
  /* MemU[address, 1] = R[t]<7, 0> */
  write_byte(address, extract32(R[t], 7, 0));

}

void ldrsb_r(uint16_t inst)
{
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  /* add is true */
  uint32_t offset_addr = R[n] + offset;
  /* index is true */
  uint32_t address = offset_addr;
  /* R[t] = signExtend( MemU[address, 1], 32) */
  R[t] = signExtend32( read_byte(address), 8);
}

void ldr_r(uint16_t inst)
{
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  /* add is true */
  uint32_t offset_addr = R[n] + offset;
  /* index is true */
  uint32_t address = offset_addr;
  
  /* R[t] = MemU[address, 4] */
  R[t] = read_word(address);

  /* wback is false */
}

void ldrh_r(uint16_t inst)
{
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  /* add is true */
  uint32_t offset_addr = R[n] + offset;
  /* index is true */
  uint32_t address = offset_addr;
  /* data = MemU[address, 2] */
  uint32_t data = read_halfword(address);
  /* wback is false*/
  R[t] = zeroExtend32(data);

}

void ldrb_r(uint16_t inst)
{
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  /* add is true */
  uint32_t offset_addr = R[n] + offset;
  /* index is true */
  uint32_t address = offset_addr;
  
  /* R[t] = MemU[address, 1] */
  R[t] = zeroExtend32(read_byte(address));
}

void ldrsh_r(uint16_t inst)
{
  uint32_t m = INST(8, 6);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  uint32_t offset;
  /* offset = Shift(R[m], shift_t, shift_n, APSR.C) */
  Shift(&offset, R[m] , 32, shift_t, shift_n, APSR.C);
  /* add is true */
  uint32_t offset_addr = R[n] + offset;
  /* index is true */
  uint32_t address = offset_addr;
  uint32_t data = read_halfword(address);
  /* R[t] = signExtend( MemU[address, 1], 32) */
  R[t] = signExtend32( data, 16);
}

void str_5i(uint16_t inst)
{
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5);
  bool index = true;
  bool add = true;
  bool wback = false;

  /* add is true */
  uint32_t offset = R[n] + imm32;
  /* index is true */
  uint32_t address = offset;
  /* MemU[address, 4] = R[t] */
  write_word(address, R[t]);
  /* wback is false */
}

void ldr_5i(uint16_t inst)
{
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;

  /* add is true */
  uint32_t offset_addr = R[n] + imm32;
  /* index is true */
  uint32_t address = offset_addr;
  
  /* R[t] = MemU[address, 4] */
  R[t] = read_word(address);

  /* wback is false */
}

void strb_i(uint16_t inst)
{
  /* encoding */
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5);

  bool index = true;
  bool add = true;
  bool wback = false;

  /* add is true*/
  uint32_t offset = R[n] + imm32;
  /* index is true*/
  uint32_t address = offset;
  /* MemU[address, 1] = R[t]<7, 0> */
  write_byte(address, extract32(R[t], 7, 0));
  /* wback is false */
}

void ldrb_i(uint16_t inst)
{
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  /* add is true */
  uint32_t offset_addr = R[n] + imm32;
  /* index is true */
  uint32_t address = offset_addr;
  
  /* R[t] = MemU[address, 1] */
  R[t] = zeroExtend32(read_byte(address));

  /* wback is false */
}

void strh_i(uint16_t inst)
{
  /* encoding */
  uint32_t t = INST(2, 0);
  uint32_t n = INST(5, 3);
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5);

  bool index = true;
  bool add = true;
  bool wback = false;

  uint32_t offset;
  
  // add is true
  offset = R[n] + imm32;
  // index is true
  uint32_t address = offset;
  /* MemU[address, 2] = R[t]<15, 0> */
  write_halfword(address, extract32(R[t], 15, 0));

  // wback is false
}

void ldrh_i(uint16_t inst)
{
  uint32_t imm5 = INST(10, 6);
  uint32_t imm32 = zeroExtend32(imm5 << 1);
  uint32_t n = INST(5, 3);
  uint32_t t = INST(2, 0);
  bool index = true;
  bool add = true;
  bool wback = false;
  enum SRType shift_t = SRType_LSL;
  int shift_n = 0;

  /* add is true */
  uint32_t offset_addr = R[n] + imm32;
  /* index is true */
  uint32_t address = offset_addr;
  /* data = MemU[address, 2] */
  uint32_t data = read_halfword(address);
  /* wback is false*/
  R[t] = zeroExtend32(data);
}

void str_8i(uint16_t inst)
{
  uint32_t t = INST(10, 8);
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8);
  bool index = true;
  bool add = true;
  bool wback = false;

  /* add is true */
  int n = 13;
  uint32_t offset = R[n] + imm32;
  /* index is true */
  uint32_t address = offset;
  /* MemU[address, 4] = R[t] */
  write_word(address, R[t]);
  /* wback is false */
}

void ldr_8i(uint16_t inst)
{
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = zeroExtend32(imm8 << 2);
  uint32_t t = INST(10, 8);
  bool index = true;
  bool add = true;
  bool wback = false;

  /* add is true */
  int n = 13;
  uint32_t offset_addr = R[n] + imm32;
  /* index is true */
  uint32_t address = offset_addr;
  
  /* R[t] = MemU[address, 4] */
  R[t] = read_word(address);

  /* wback is false */

}


void adr(uint16_t inst)
{
  uint32_t d = INST(10, 8);
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32 = imm8 << 2;

  /* add is true.
     result = Align(PC,4) + imm32 */
  uint32_t result = (PC & 0xFFFFFFFC)  + imm32;
  R[d] = result;
}

void add_sp_i1(uint16_t inst)
{
  /* Encoding */
  uint32_t d = INST(10, 8);
  uint32_t imm8 = INST(7, 0);
  uint32_t imm32;
  uint32_t Rd;
  imm32 = zeroExtend32(imm8 << 2); /* zero extend */

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(SP, imm32, '0', 32) */
  AddWithCarry(&result, &carry, &overflow, SP, imm32, 0, 32);

  R[d] = result;
  /* no flag setting form of the instruction supported.
  APSR.C = carry;
  APSR.V = overflow;
  */
}


void add_sp_i2(uint16_t inst)
{
  uint32_t d = 13;
  bool setflags = false;
  uint32_t imm7 = INST(6, 0);
  uint32_t imm32;
  uint32_t Rd;
  imm32 = zeroExtend32(imm7 << 2); /* zero extend */

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(SP, imm32, '0', 32) */
  AddWithCarry(&result, &carry, &overflow, SP, imm32, 0, 32);

  R[d] = result;
  /* no flag setting form of the instruction supported.
	 APSR.C = carry;
	 APSR.V = overflow;
   */
}

void sub_sp_i(uint16_t inst)
{
  int d = 13;
  uint32_t imm7 = INST(6, 0);
  uint32_t imm32 = zeroExtend32(imm7 << 2);

  uint32_t result;
  int carry, overflow;
  /* (result, carry, overflow) = AddWithCarry(SP, NOT(imm32), '1', 32) */
  AddWithCarry(&result, &carry, &overflow, SP, ~imm32, 1, 32);

  R[d] = result;
  /* not flag setting form of the instruction supported */
}

void sxth(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);
  int rotation = 0;
  uint32_t rotated;

  /* rotated = ROR(R[m], rotation) */
  rotated = R[m];

  R[d] = signExtend32(extract32(rotated, 15, 0),  16);
}

void sxtb(uint16_t inst)
{
  uint32_t d = INST(2, 0);
  uint32_t m = INST(5, 3);
  int rotation = 0;
  uint32_t rotated;

  /* rotated = ROR(R[m], rotation) */
  rotated = R[m];

  R[d] = signExtend32(extract32(rotated, 7, 0), 8);
}

void uxth(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);
  int rotation = 0;
  uint32_t rotated;

  /* rotated = ROR(R[m], rotation) */
  rotated = R[m];

  R[d] = zeroExtend32(extract32(rotated, 15, 0));
}

void uxtb(uint16_t inst)
{
  uint32_t d = INST(2, 0);
  uint32_t m = INST(5, 3);
  int rotation = 0;
  uint32_t rotated;

  /* rotated = ROR(R[m], rotation) */
  rotated = R[m];

  R[d] = zeroExtend32(extract32(rotated, 7, 0));
}

void push(uint16_t inst)
{
  uint32_t M = INST_(8);
  uint32_t register_list = INST(7, 0);
  uint32_t registers = (M << 14)  | register_list;

  uint32_t address = SP - 4 * BitCount(registers);

  for (int i=0; i<15; i++)
	{
	  if (extract32_(registers, i) == 1)
		{
			  write_word(address, R[i]);
			  address = address + 4;
		}
	}

  SP = SP - 4 * BitCount(registers);
}

void rev(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);

  uint32_t result;
  uint32_t first = extract32(R[m], 7, 0) << 24;
  uint32_t second = extract32(R[m], 15, 8) << 16;
  uint32_t third = extract32(R[m], 23, 16) << 8;
  uint32_t fourth = extract32(R[m], 31, 24);
  result = first | second | third | fourth;

  R[d] = result;
}

void rev16(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);

  uint32_t result;
  uint32_t first = extract32(R[m], 23, 16) << 24;
  uint32_t second = extract32(R[m], 31, 24) << 16;
  uint32_t third = extract32(R[m], 7, 0) << 8;
  uint32_t fourth = extract32(R[m], 15, 8);
  result = first | second | third | fourth;

  R[d] = result;
}

void revsh(uint16_t inst)
{
  /* Encoding */
  uint32_t m = INST(5, 3);
  uint32_t d = INST(2, 0);

  uint32_t result;
  uint32_t first = (signExtend32(extract32(R[m], 7, 0), 8) & 0xFFFFFF) << 8;
  uint32_t second = extract32(R[m], 15, 8);
  result = first | second;

  R[d] = result;
}

void pop(uint16_t inst)
{

}


void stm(uint16_t inst)
{
  uint32_t n = INST(10, 8);
  uint32_t reg_list = INST(7, 0);
  uint32_t registers = zeroExtend32(reg_list);
  bool wback = true;
  uint32_t address = R[n];

  /* LowestSetBit(x) is the minimum bit number of any of its bits that are
   * ones. If all of its bits are zeros, LowestSetBits(x) = N */
  for(int i=0; i<15; i++)
	{
	  if (extract32_(registers, i) == 1)
		{
		  if (i == n && wback && i != LowestSetBit(registers))
			{
			  // MemA[address, 4] = bit(32) UNKNOWN;

			}
		  else
			{
			  // MemA[address, 4] = R[i]
			  write_word(address, R[i]);
			}
		  address = address + 4;
		}
	}

  if (wback)
	{
	  // R[n] = R[n] + 4 * BitCount(registers)
	  R[n] = R[n] + 4 * BitCount(registers);
	}
}

void ldm(uint16_t inst)
{
  uint32_t n = INST(10, 8);
  uint32_t reg_list = INST(7, 0);
  uint32_t registers = zeroExtend32(reg_list);
  bool wback = (extract32_(registers, n) == 0);

  uint32_t address = R[n];

  for(int i=0; i<7; i++)
	{
	  if (extract32_(registers, i) == 1)
		{
		  R[i] = read_word(address);
		  address = address + 4;
		}
	}

  if (wback)
	{
	  // R[n] = R[n] + 4 * BitCount(registers)
	  R[n] = R[n] + 4 * BitCount(registers);
	}

}


void b_conditional(uint16_t inst)
{
  uint32_t imm8 = INST(7, 0);
  uint32_t cond = INST(11, 8);
  uint32_t address;

  if (cond == 0xE)
	{
	  /* SEE UDF */

	}
  else if (cond == 0xF)
	{
	  /* SEE SVC */

	}

  address = PC + signExtend32((imm8 << 1), 9);
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

/* Given functions */

void b_unconditional(uint16_t inst)
{
  uint32_t imm11 = INST(10, 0);
  uint32_t address;

  address = PC + signExtend32((imm11 << 1), 12);
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

  address = PC + imm32;
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

/* static functions */
static int BitCount(uint32_t x)
{
  int count = 0;
  while (x != 0)
	{
	  if (x % 2 == 1)
		count ++;
	  x /= 2;
	}
  return count;
}

static int LowestSetBit(uint32_t x)
{
  int lowest = 32;
  for (int i=0; i<32; i++)
	{
	  if (extract32_(x, i) == 1)
		{
		  lowest = i;
		  break;
		}
	}
  return lowest;
}

static void Shift(uint32_t* result, uint32_t value , int value_bit, enum SRType srtype, int amount, int carry_in)
{
  int dummy;
  Shift_C(result, &dummy, value, value_bit, srtype, amount, carry_in);
}

static void Shift_C(uint32_t* result, int* carry_out, uint32_t value , int value_bit, enum SRType srtype, int amount, int carry_in)
{
  if (amount == 0)
	{
	  *result = value;
	  *carry_out = carry_in;
	}
  else
	{
	  switch (srtype)
		{
		case SRType_LSL:
		  LSL_C(result, carry_out, value, value_bit, amount);
		  break;
		case SRType_LSR:
		  LSR_C(result, carry_out, value, value_bit, amount);
		  break;
		case SRType_ASR:
		  ASR_C(result, carry_out, value, value_bit, amount);
		  break;
		case SRType_ROR:
		  ROR_C(result, carry_out, value, value_bit, amount);
		  break;
		case SRType_RRX:
		  RRX_C(result, carry_out, value, value_bit, amount);
		  break;
		default:
		  // type error. do nothing.
		  break;
		}
	}
}

static void LSL_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount)
{
  uint64_t extended_x = value << amount;
  *result = (uint32_t)(extended_x % ( 1 << value_bit));
  *carry_out = extract32_(*result, value_bit);
}

static void LSR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount)
{
  uint64_t extended_x = value >> amount;
  uint64_t mask = ((1 << value_bit) - 1) << amount;
  *result = (uint32_t)(extended_x & mask) >> amount;
  *carry_out = extended_x & (1<<(value_bit-1)) >> (value_bit-1);
}

static void ASR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount)
{
  uint32_t extended_x = signExtend32(value, value_bit);
  *result = extract32(extended_x, value_bit + amount -1, amount);
  *carry_out = extract32_(extended_x, amount -1);
}

static void ROR_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount)
{
  uint32_t m = amount % value_bit;
  /* result = LSR(x, m) OR LSL(x, N-m) */
  int lsr = value >> m;
  int lsl = value << (value_bit - m);
  *result = lsr | lsl;
  /* carry_out = result <N-1> */
  *carry_out = extract32_(*result, value_bit -1);
}

static void RRX_C(uint32_t* result, int* carry_out, uint32_t value, int value_bit, int amount)
{
  /* amount is carry_in now. */
  *result = (amount << (value_bit-1)) | extract32(value, value_bit-1, 1);
  *carry_out = extract32_(value, 0);
}

/* N : x,y's bit. */
static void AddWithCarry(uint32_t* result, int* carry, int* overflow, uint32_t x, uint32_t y, int cin, int N)
{
  uint64_t unsigned_sum = x + y + (uint32_t)cin;
  int64_t signed_sum = signExtend32(x, N) + signExtend32(y, N) +
	(uint32_t)cin;
  
  *result = (uint32_t)(unsigned_sum & 0xFFFFFFFF );
  
  if ((*result) == unsigned_sum)
	*carry = 0;
  else
	*carry = 1;

  if (signExtend32(*result, 31) == signed_sum)
	*overflow = 0;
  else
	*overflow = 1;
}

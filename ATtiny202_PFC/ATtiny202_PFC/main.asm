; Filename	:ATtiny202_PFC.asm
; Author	:KKT
; Date		:2020/10/11
; Device	:ATtiny202-SSNR
; IDE		:Atmel Studio 7 (Version 7.0.2397)
; Debugger	:Pickit4
; Note		:Digital Power Factor Corrector with ATtiny202

.INCLUDE <tn202def.inc>
.INCLUDE "mymacro.inc"

;****************************************************************
; Configuration
;****************************************************************
; @ Output Voltage(Unit [V])
.EQU CFG_Vout		=380

; @ Software MOSFET average current limit threshold(Unit [mA])
.EQU CFG_Ilimit_sw	=4000

;****************************************************************
; Constant definition
;****************************************************************
.EQU C_FCPU		=20000000		; CPU clock speed in Hz
.EQU C_VREF_ADC		=1100			; ADC reference voltage in mV
.EQU C_PWM_PER		=200			; PWM period in CPU clock
.EQU C_PIN_V_Rs		=7			; PA7
.EQU C_PIN_PWM		=6			; PA6
.EQU C_PIN_V_FB		=3			; PA3
.EQU C_PIN_V_AC		=2			; PA2
;.EQU C_PIN_V_Rs_avg	=1			; PA1
.EQU C_SMPLCOUNT_INIT	=104			; 

;****************************************************************
; Register definition
;****************************************************************
.DEF ZERO		=R2		; Zero register
.DEF R_STACK_SREG	=R3		; SREG stack
.DEF R_PWR_RATE		=R13		; Output power rate
.DEF R_ADCRES_Vout	=R14		; ADC0 conversion result
.DEF R_SMPLCOUNT_Vout	=R15		; Vout sample counter
.DEF W0			=R16		; work0
.DEF W1			=R17		; work1
.DEF W2			=R18		; work2
.DEF W3			=R19		; work3
.DEF W4			=R20		; work4
.DEF W5			=R21		; work5
.DEF ARG0		=R22		; argument0
.DEF ARG1		=R23		; argument1
.DEF R_ADC_STATUS	=R24		; ADC status
.DEF R_FLAG		=R25		; Operation flag

;****************************************************************
; RAM definition
;****************************************************************
.DSEG
/*
struct {
    uint8_t SV;
    uint8_t PV;
    int16_t PastP;
    uint8_t GainP;
    uint8_t GainI;
    uint16_t MV;
} ST_PIPARAM;
*/
st_PIparam_Current:	    .BYTE 8
st_PIparam_Vout:	    .BYTE 8

; Vout averaging buffer
u8_index_Vout_buf:	    .BYTE 1
.EQU C_SIZE_Vout_buf	    =18
u8_Vout_buf:	    	    .BYTE C_SIZE_Vout_buf

.CSEG
.ORG 0x000
; Interrupt Vector
RJMP RESET
RJMP CRCSCAN_NMI
RJMP BOD_VLM
RJMP PORTA_PORT
RJMP UNHANDLED
RJMP UNHANDLED
RJMP RTC_OVF_CMP
RJMP RTC_PIT
RJMP TCA0_LUNF_OVF
RJMP TCA0_HUNF
RJMP TCA0_LCMP0_CMP0
RJMP TCA0_LCMP1_CMP1
RJMP TCA0_LCMP2_CMP2
RJMP TCB0_INT
RJMP UNHANDLED
RJMP UNHANDLED
RJMP AC0_AC
RJMP ADC0_RESRDY
RJMP ADC0_WCOMP
RJMP TWI0_TWIS
RJMP TWI0_TWIM
RJMP SPI0_INT
RJMP USART0_RXC
RJMP USART0_DRE
RJMP USART0_TXC
RJMP NVMCTRL_EE

; Unhandled interrupt
CRCSCAN_NMI:
BOD_VLM:
PORTA_PORT:
RTC_OVF_CMP:
RTC_PIT:
TCA0_LUNF_OVF:
TCA0_HUNF:
TCA0_LCMP0_CMP0:
TCA0_LCMP1_CMP1:
TCA0_LCMP2_CMP2:
TCB0_INT:
AC0_AC:
;ADC0_RESRDY:
ADC0_WCOMP:
TWI0_TWIS:
TWI0_TWIM:
SPI0_INT:
USART0_RXC:
USART0_DRE:
USART0_TXC:
NVMCTRL_EE:
UNHANDLED:
    RETI

; Reset
RESET:
    ; Clock Config
    OUTI CPU_CCP, (0xD8)
    OUTI CLKCTRL_MCLKCTRLB, (0x10)

    ; PortA Config
    OUTI VPORTA_DIR, (1 << C_PIN_PWM)

    ; CCL Config
    LDIW ZL, ZH, CCL_base
    STDI Z + CCL_SEQCTRL0_offset, 0x04
    STDI Z + CCL_TRUTH0_offset, 0x04
    STDI Z + CCL_LUT0CTRLC_offset, 0x00
    STDI Z + CCL_LUT0CTRLB_offset, 0x88
    STDI Z + CCL_LUT0CTRLA_offset, 0x89
    STDI Z + CCL_TRUTH1_offset, 0x03
    STDI Z + CCL_LUT1CTRLC_offset, 0x00
    STDI Z + CCL_LUT1CTRLB_offset, 0x88
    STDI Z + CCL_LUT1CTRLA_offset, 0x81
    STDI Z + CCL_CTRLA_offset, 0x01

    ; TCA0 Config
    LDIW ZL, ZH, TCA0_base
    STDI Z + TCA_SINGLE_CMP2BUF_offset, 78
    STDI Z + TCA_SINGLE_CMP2BUF_offset + 1, 0
    STDI Z + TCA_SINGLE_CMP1BUF_offset, 100
    STDI Z + TCA_SINGLE_CMP1BUF_offset + 1, 0
    STDI Z + TCA_SINGLE_CMP0BUF_offset, 100
    STDI Z + TCA_SINGLE_CMP0BUF_offset + 1, 0
    STDI Z + TCA_SINGLE_PERBUF_offset, low(C_PWM_PER)
    STDI Z + TCA_SINGLE_PERBUF_offset + 1, high(C_PWM_PER)
    STDI Z + TCA_SINGLE_CTRLD_offset, 0x00
    STDI Z + TCA_SINGLE_CTRLB_offset, 0x03
    STDI Z + TCA_SINGLE_INTCTRL_offset, 0x00
    STDI Z + TCA_SINGLE_CTRLA_offset, (TCA_SPLIT_CLKSEL_DIV1_gc | TCA_SPLIT_ENABLE_bm)

    ; VREF Config
    OUTI VREF_CTRLA, (0x14)
    ;OUTI VREF_CTRLA, (0x04)

    ; AC0 Config
    LDIW ZL, ZH, AC0_base
    ;STDI Z + AC_MUXCTRLA_offset, (0x02)
    ;STDI Z + AC_CTRLA_offset, (0x07)

    ; ADC0 Config
    LDIW ZL, ZH, ADC0_base
    STDI Z + ADC_CALIB_offset, (0x00)
    STDI Z + ADC_INTCTRL_offset, (0x01)
    STDI Z + ADC_EVCTRL_offset, (0x01)
    STDI Z + ADC_MUXPOS_offset, (0x07)
    STDI Z + ADC_SAMPCTRL_offset, (0x00)
    STDI Z + ADC_CTRLE_offset, (0x00)
    STDI Z + ADC_CTRLD_offset, (0x00)
    STDI Z + ADC_CTRLC_offset, (0x42)
    STDI Z + ADC_CTRLB_offset, (0x00)
    STDI Z + ADC_CTRLA_offset, (0x05)

    ; EVSYS Config
    OUTI EVSYS_SYNCCH0, 0x06
    OUTI EVSYS_ASYNCUSER1, 0x01

    ; RAM init
    OUTI (st_PIparam_Current), 64
    OUTI (st_PIparam_Current + 4), 30
    OUTI (st_PIparam_Current + 5), 40

    OUTI (st_PIparam_Vout), 100
    OUTI (st_PIparam_Vout + 4), 8
    OUTI (st_PIparam_Vout + 5), 8

    LDI W0, C_SMPLCOUNT_INIT
    MOV R_SMPLCOUNT_Vout, W0

    SEI

; PA6:LED dbg
; PA1:DBG
MAIN:
    BST R_FLAG, 0
    BRTC MAIN
    CBR R_FLAG, 0x01
    ; Vout operation
    LDIW ZL, ZH, u8_index_Vout_buf
    MOV ARG0, R_ADCRES_Vout
    LDI ARG1, C_SIZE_Vout_buf
    RCALL s_insert_to_buffer
    LDIW ZL, ZH, u8_Vout_buf
    LDI ARG0, C_SIZE_Vout_buf
    RCALL s_get_sum_buffer
    LSR W1
    ROR W0; / 2
    LSR W1
    ROR W0; / 2
    LSR W1
    ROR W0; / 2
    LSR W1
    ROR W0; / 2
    LDIW YL, YH, st_PIparam_Vout
    RCALL S_GET_PI_MV
    TST W1
    BRPL PC + 2
    CLR W1
    MOV R_PWR_RATE, W1
    RJMP MAIN

; ADC0 result ready interrupt service routine
ADC0_RESRDY:
    IN R_STACK_SREG, CPU_SREG
    PUSH W0
    TST R_ADC_STATUS
    BREQ LL_ADC_RESRDY_0
    CPI R_ADC_STATUS, 1
    BREQ LL_ADC_RESRDY_1
    RJMP LL_ADC_RESRDY_2

LL_ADC_RESRDY_0:
    ; ADC conversion complete on PA3(Vout), next is PA2(Vac)
    OUTI ADC0_MUXPOS, (0x02)
    LDS R_ADCRES_Vout, ADC0_RES
    INC R_ADC_STATUS
    DEC R_SMPLCOUNT_Vout
    BRNE PC + 4
    LDI W0, C_SMPLCOUNT_INIT
    MOV R_SMPLCOUNT_Vout, W0
    SBR R_FLAG, 0x01
    RJMP LL_ADC_RESRDY_END

LL_ADC_RESRDY_1:
; ADC conversion complete on PA2(Vac), next is PA7(Is)
    OUTI ADC0_MUXPOS, (0x07)
    LDS W0, ADC0_RES
    PUSH R0
    PUSH R1
    MUL R_PWR_RATE, W0
    LSL R0
    ROL R1
    STS st_PIparam_Current + 0, R1
    POP R1
    POP R0
    INC R_ADC_STATUS
    RJMP LL_ADC_RESRDY_END

LL_ADC_RESRDY_2:
; ADC conversion complete on PA7(Is), next is PA3(Vout)
    OUTI ADC0_MUXPOS, (0x03)
    LDS W0, ADC0_RES
    CLR R_ADC_STATUS
    PUSH W1
    PUSH W2
    PUSH R0
    PUSH R1
    PUSH YL
    PUSH YH
    LDIW YL, YH, st_PIparam_Current
    RCALL S_GET_PI_MV
    TST W1
    BRPL PC + 3
    CLR W0
    CLR W1
    LSL W0
    ROL W1
    CPI W1, (C_PWM_PER - 10)
    BRCS PC + 2
    LDI W1, (C_PWM_PER - 10)
    LDIW YL, YH, TCA0_base
    LDI W0, 100
    LSR W1
    ADC W0, W1
    STD Y + TCA_SINGLE_CMP1BUF_offset, W0
    STD Y + TCA_SINGLE_CMP1BUF_offset + 1, ZERO
    LDI W0, 100
    SUB W0, W1
    STD Y + TCA_SINGLE_CMP0BUF_offset, W0
    STD Y + TCA_SINGLE_CMP0BUF_offset + 1, ZERO
    POP YH
    POP YL
    POP R1
    POP R0
    POP W2
    POP W1
    
LL_ADC_RESRDY_END:
    POP W0
    OUT CPU_SREG, R_STACK_SREG
    RETI

; YH:YL->Pointer for param
; W0:PV
S_GET_PI_MV:
    LDD W2, Y + 0; SV
    SUBSAT W2, W0; W2 = SV - PV
    LDD W0, Y + 6
    LDD W1, Y + 7; W[1:0] = past MV
    LDD R0, Y + 2
    LDD R1, Y + 3; R[1:0] = past dP
    SUBWSAT W0, W1, R0, R1; W[1:0] = past MV - past P
    PUSH W1
    LDD W1, Y + 4; W1 = Gain_P
    MULSU W2, W1; R[1:0] = error * Gain_P
    STD Y + 2, R0
    STD Y + 3, R1; param P[1:0] write back
    POP W1
    ADDWSAT W0, W1, R0, R1; W[1:0] += param P
    PUSH W1
    LDD W1, Y + 5; W1 = Gain_I
    MULSU W2, W1; R[1:0] = error * Gain_I
    POP W1
    ADDWSAT W0, W1, R0, R1; W[1:0] += param I
    STD Y + 6, W0
    STD Y + 7, W1; MV write back
    RET

; Insert new val to ring buffer
; Z:ptr to buffer index
; ARG0:new val
; ARG1:buffer size
s_insert_to_buffer:
    LD W0, Z
    INC W0
    CP W0, ARG1
    BRCS PC + 2
    CLR W0
    ST Z+, W0
    ADD ZL, W0
    CLR W0
    ADC ZH, W0
    ST Z, ARG0
    RET

; Get sum of buffer with decimation
; Z:ptr to buffer(unsigned only)
; W[1:0]:return val
; W2:min
; W3:max
; W4:work
; ARG0:buffsize
s_get_sum_buffer:
    SER W2
    CLR W3
    CLR W0
    CLR W1
LL_cal_sum:
    LD W4, Z+
    CP W4, W2; buff[n] - min
    BRCC PC + 2
    MOV W2, W4; update min
    CP W4, W3; buff[n] - max
    BRCS PC + 2
    MOV W3, W4; update max
    ADD W0, W4
    CLR W4
    ADC W1, W4
    DEC ARG0
    BRNE LL_cal_sum
    SUB W0, W2
    CLR W2
    SBC W1, W2
    SUB W0, W3
    SBC W1, W2
    RET
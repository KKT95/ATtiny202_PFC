
; Out immediately
.MACRO OUTI
    LDI W0, @1
.IF (@0 > 0x3F)
    STS @0, W0
.ELSE
    OUT @0, W0
.ENDIF
.ENDMACRO

; Subtract with saturation(unsigned - unsigned)
.MACRO SUBSAT
    SUB @0, @1
    BRCC PC + 3
    BRMI PC + 4
    LDI @0, (0x80)
    BRPL PC + 2
    LDI @0, (0x7F)
.ENDMACRO

; Add 16bit with saturation(signed + signed)
.MACRO ADDWSAT
    ADD @0, @2
    ADC @1, @3
    BRVC PC + 7
    BRPL PC + 3
    LDI @0, (0xFF)
    LDI @1, (0x7F)
    BRMI PC + 3
    LDI @0, (0x00)
    LDI @1, (0x80)
.ENDMACRO

; Subtract 16bit with saturation(signed - signed)
.MACRO SUBWSAT
    SUB @0, @2
    SBC @1, @3
    BRVC PC + 7
    BRPL PC + 3
    LDI @0, (0xFF)
    LDI @1, (0x7F)
    BRMI PC + 3
    LDI @0, (0x00)
    LDI @1, (0x80)
.ENDMACRO

; Load direct immediately(word long)
.MACRO LDIW
    LDI @0, low(@2)
    LDI @1, high(@2)
.ENDMACRO

; STD Immediately
.MACRO STDI
    LDI W0, @1
	STD @0, W0
.ENDMACRO
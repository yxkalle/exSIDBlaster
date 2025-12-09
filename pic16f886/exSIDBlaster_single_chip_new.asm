; Firmware version 11 for exSID USB Rev.B
; (C) 2015-2016 T. Varene
; License: CC BY-NC-SA 4.0

; Designed for MPASM
; This program is optimized for timing accuracy, speed and size, in that order.
; Changelog at the bottom of the file.

; SIDs are driven by a timer set up to generate the 1MHz square signal.
; Since the SID requires that read/write operations are aligned to the clock,
; the code is designed to always run an exact multiple of SID clock cycles thus
; keeping the code synchronized to the timer output.

; NOTE: While the SID will apparently respond to rising clock edge, some voices
; may not properly work that way. Thus clocking on falling edge (per datasheet)
; appears to be more reliable.

; This program assumes full operation within Page 0 (compatible with PIC16F722/882)
; PCLATH is preset in init routine assuming PCL is directly modified only in the address jump table

; This program was initially written for PIC16F88X and later adapted to PIC16F72X
; PIC16F72X support is only for "educational" purposes and is not actively supported.

	TITLE		"exSID rev.C Firmware v11 - Single chip version"

#define	FWVERS		.11
#define HWVERS		"S"

	list		p=16f886
	errorlevel	-302
 #include		p16f886.inc
 #define		__PIC16F88X
 
 ; Suppress MPASM Warning 302 (Register not in Bank 0)

; Configuration bits
;   FOSC_EC	- enables external 24MHz clock from FTDI
;   WTDE_OFF	- watchdog is not kicked by code
;   PWRTE_ON	- power-on timer enabled
;   MCLRE_ON	- /MCLR enabled for delayed boot after secondary power rises
;   CP_OFF	- No code protection
;   CPD_OFF	- No data code protection
;   BOREN_ON	- Brown-out reset enabled
;   IESO_OFF	- Internal/external clock switchover disabled
;   FCMEN_OFF	- Fail-safe clock monitor disabled
;   LVP_OFF	- LVP disabled: frees up RB3
;   BOR40V	- Brown-out at 4.0V
;   WRT_OFF	- Write protection off
	__CONFIG _CONFIG1, _FOSC_EC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
	__CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

#define	SIDCTL		PORTC	; SID bus control port (except reset)
#define	SIDADR		PORTA	; SID bus address port
#define	SIDDAT		PORTB	; SID bus data port
#define	SIDDDR		TRISB	; SID bus data direction

#define	EXTCLK		RA7

#define	CTL_CS		RC0
#define	CTL_RW		RC1
#define	CTLRST		RA5
#define	CTLCLK		RC2		; SID CLK on SIDCTL
#define	FTDIRX		RC6
#define	FTDITX		RC7

; -- ADDED DEFINE FOR MASKING --
#define ADDRBITM	(1 << CTLRST)

; By default RW is held low. The read routine must bring it back low after exec.
; Chip select bitmask.
#define	CSBITM		(1 << CTL_CS)

#define	JMPTBA		0x0700		; Jump table base address. 0x07xx stays in Page 0 and ensures no screw up with GOTO/CALL

; Cross-bank data registers
	CBLOCK	0x70
TEMPBUF						; typically holds current address
WAITCNT:2					; SID clock loop counter
LPMBYTE						; Pgm memory LSB
HPMBYTE						; Pgm memory MSB
	ENDC

; Program
	ORG	0x0
__INIT

	BANKSEL	OPTION_REG
	MOVLW	B'00000000'		; OPTION bits, nothing fancy
	MOVWF	OPTION_REG

	; Disable interrupts
	BANKSEL INTCON
	CLRF	INTCON			; Disable interrupts and clear IFs
	CLRF	PIE1			; Disable peripheral interrupts
	CLRF	PIE2			; Disable peripheral interrupts

	; Disable analog
	BANKSEL	ANSEL
	CLRF	ANSEL			; Clear ANSEL
	CLRF	ANSELH			; Clear ANSELH

	; Preset data
	BANKSEL PORTA
	CLRF	SIDADR			; Clear SID address
	CLRF	SIDDAT			; Clear SID data
	MOVLW	CSBITM			; disable SID as initial condition
	MOVWF	SIDCTL			; Clear SID control except CS (preset high). This enables SID /reset

	MOVLW	high JMPTBA
	MOVWF	PCLATH			; Preset PCLATH for jump table == MSB of JMPTBA

	MOVLW	.20
	MOVWF	WAITCNT			; Initial reset time

	; Enable outputs
	BANKSEL	TRISA
	MOVLW	B'1' << EXTCLK
	MOVWF	TRISA			; TRISA   <0:4>:address <7>:clk12 [in]
	CLRF	TRISB			; TRISB   <0:7>:data
	MOVLW	B'1' << FTDITX
	MOVWF	TRISC			; TRISC   <0>:cs0 <1>:rw <2>:clk <3>: rst <6>:tx <7>:rx [in]

	; UART setup: In high baudrate, at 2Mpbs, chars arrive at max 200kHz (2000/(1+8+1)).
	; We have exactly CYCCHR=5 SID cycles between chars ("CYCCHR"), 10 cycles between tuples.
	; We have a max of 100kHz SID instruction rate (address + data)
	; @24MHz FOSC, we have 30 PIC cycles between chars.
	; In low baudrate config, at 750kbps words arrive at 75kHz, we have just over
	; 13 SID cycles between words (13.33), and 37.5kHz instruction rate.
	; For reference, C64 scanline is < 16kHz
	; This also means that since we don't have flow control, routines must
	; ensure that the receive buffer will always be emptied in a timely fashion,
	; i.e. no more than 2*CYCCHR SID cycles will pass between two fetches.
	; NB: since at 750kbps words timings aren't exact multiples of SID clock cycles,
	; this has a negative impact on timing accuracy with a drift accumulating over time.
	; it can be compensated, preferably in host software, by adjusting every 3 SIDclk.
	; NOTE: The PIC FIFO buffering allows reception of two complete characters and
	; the start of a third character before software must start servicing the EUSART receiver.
	BANKSEL	BAUDCTL
	MOVLW   B'01001000'		; BAUDCTL: Use 16bit BRG
	MOVWF   BAUDCTL
	BANKSEL SPBRG
	MOVLW   .2				; 2Mbps 0% error @24MHz on 16F88X
	MOVWF	SPBRG
	MOVLW	B'00100100'
	MOVWF	TXSTA			; TXSTA: Enable transmitter
	BANKSEL RCSTA
	MOVLW	B'10010000'
	MOVWF	RCSTA			; RCSTA: Enable receiver

	; Set up T1/PWM output (SIDCLK)
	; * PWM registers configuration
	; * Fosc = 24000000 Hz
	; * Fpwm = 1000000.00 Hz
	; * Duty Cycle = 50 %
	; * Resolution is 4 bits
	; * Prescaler is 1
	; */
	;PR2 = 0b00000101 ;
	;T2CON = 0b00000100 ;
	;CCPR1L = 0b00000011 ;
	;CCP1CON = 0b00001100 ;

	BANKSEL	TRISC		; BANK 1
	BCF	TRISC, RC2		; Make sure pin is an output
	MOVLW	B'00000101'
	MOVWF	PR2

	BANKSEL	CCP1CON		; BANK 0
	CLRF	CCP1CON		; CCP Module is off
	CLRF	CCP2CON		; for extra safety, manually disable CCP2 as well
	MOVLW	B'00000011'
	MOVWF	CCPR1L		; Duty Cycle is 50% of PWM Period
	MOVLW	B'00001100'	; PWM mode, 2 LSbs of Duty cycle = 00
	MOVWF	CCP1CON
	CLRF	PIR1		; Clear peripheral interrupts Flags
	CLRF	TMR2		; Clear Timer2
	MOVLW	B'00000100'
	MOVWF	T2CON		; Timer2 starts to increment, SIDCLK rise

	; Timing SIDCLKs from here
	; The instruction executing on SIDCLK front is 1/4 cycle ahead of the actual edge.
	; @24MHz, that's 40ns

	; This NOP can be used as a global offset on timings: Clock edges happen 1 INSN earlier
;	NOP

	BSF	SIDADR, CTLRST	; Clear reset condition before playing the startup tune
	; start address must be set before the jump
	BCF	STATUS,	RP0
	BSF	STATUS, RP1	; BANK 2, SC low

	MOVLW	high __TUNEDATA
	MOVWF	EEADRH
	MOVLW	low __TUNEDATA

	MOVWF	EEADR
	CALL	__INITTUNE	; SC low

sid_reset				; 1 SIDCLK loop: SID /RST for WAITCNT SIDCLKs (min 10 per datasheet)
	BCF	STATUS,	RP0
	BCF	STATUS,	RP1		; Back in BANK 0, because __INITTUNE returns from BANK 2
	BCF	SIDADR,	CTLRST	; Re-enable reset

	DECFSZ	WAITCNT,F
	GOTO	sid_reset
	BSF	SIDADR,	CTLRST	; SIDCLK. Clear reset condition.

	; WARN: We are in BANK 0 from now on: the rest of the code assumes this

	; @24MHz: INSNCLK = 6MHz => INSN period = 166ns | 1 SIDCLK = 6 cycles = 1000ns/1MHz

__MAIN
	; We service incoming serial data via polling to ensure proper SIDCLK timing
	GOTO	$+1
	GOTO	$+1
	GOTO	__GET_ADDR	; SIDCLK

; Long delayed data write
; When called, WAITCNT contains 0000ddd0, ddd == delay in SIDCLKs, non-null
; Execution time: CYCCHR + WAITCNT SIDCLKs max. Write is effective after CYCCHR+WAITCNT SIDCLKs
; WAITCNT must NOT exceed CYCCHR: maximum execution time after clearing RCREG MUST BE <= CYCCHR
__WRITE_REGD	
	; W currently holds the address (from BTABLE)
	ANDLW	B'11100000'	; W: ddd00000
	MOVWF	WAITCNT		; WAITCNT: ddd00000
	SWAPF	WAITCNT, F	; WAITCNT: 0000ddd0 => ddd is delay cycles

wrd_data	; Wait for data byte
	GOTO	$+1
	NOP
	
	BTFSS	PIR1,	RCIF
	GOTO	wrd_data
	MOVF	RCREG,	W	; SIDCLK

wrd_waitone				; WAITCNT is twice the desired value, decrement 2-by-2 on each loop
	DECF	WAITCNT,F

	NOP
	MOVWF	SIDDAT		; Done here to compensate shifted GOTO
	
	DECFSZ	WAITCNT,F
	GOTO	wrd_waitone
						; SIDCLK
	GOTO	__wri		; this GOTO will be 1 cycle late, so we shift inside __WRITE_REGI

; Refer to SID datasheet for timing considerations

; Immediate data write
; SID writes are latched on falling SIDCLK edge
; Execution time: CYCCHR + 2 SIDCLKs max
__WRITE_REGI
wri_data					; Wait for data byte
	GOTO	$+1				; 1 op / 2 cycles
	NOP						; 1 op / 1 cycle

	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	wri_data		; 1 op / 2 cycles
	MOVF	RCREG,	W		; 1 op / 1 cycle	; SIDCLK fall

	; Copy received byte to data port
	MOVWF	SIDDAT			; 1 op / 1 cycle
	; SID control lines
__wri	
	MOVLW   ~CSBITM	; 1 op / 1 cycle
	NOP

	NOP
	ANDWF	SIDCTL,	F		; 1 op / 1 cycle
	MOVLW	CSBITM			; 1 op / 1 cycle	; SIDCLK fall: write effective

	IORWF	SIDCTL,	F		; 1 op / 1 cycle
	
	NOP						; 1 op / 1 cycle
	GOTO	$+1				; 1 op / 2 cycles
	GOTO	__GET_ADDR		; 1 op / 2 cycles	; SIDCLK
	
__ERR_OERR
	BCF	RCSTA, CREN		; Disable UART Receiver (Clears OERR)
	BSF	RCSTA, CREN		; Re-enable UART Receiver
	; Fall through to ga_waddr to try receiving again...

; Get requested address
; In order for cycle-accurate writes to work, execution time after clearing RCREG
; MUST NOT take more than CYCCHR/2
; Execution time: CYCCHR + 1 + 1 (jump table) SIDCLKs max
__GET_ADDR
ga_waddr
	;GOTO	$+1
	NOP
	
	BTFSC	RCSTA, OERR		; Check if OERR bit is set (Skip if Clear)
	GOTO	__ERR_OERR		; If Set, Jump to Error Handler
	
	BTFSS	PIR1,	RCIF
	GOTO	ga_waddr 
	MOVF	RCREG,	W		; SIDCLK
	
	MOVWF	TEMPBUF			; Copy address to TEMPBUF, used by jump table
	NOP
	
	GOTO	BTABLE			; SIDCLK
	; JUMP TABLE BASED ON CONTENT OF TEMPBUF, aka ADDRESS

; Delayed data read
; Code flow continues into __READ_REGI to save a goto and maintain clock alignment
; Execution time: WAITCNT SIDCLKs. Ensures read happens after exactly WAITCNT SIDclks
__READ_REGD
	; W contains address from BTABLE
	ANDLW	B'11100000'	; W: ddd00000
	MOVWF	WAITCNT		; WAITCNT: ddd00000
	SWAPF	WAITCNT, F	; WAITCNT: 0000ddd0 => ddd is delay cycles

rrd_waitone
	; WAITCNT is twice the desired value, decrement 2-by-2 on each loop
	DECF	WAITCNT,F
	GOTO	$+1

	DECFSZ	WAITCNT,F
	GOTO	rrd_waitone
	NOP						; SIDCLK

; SID reads are latched before falling SIDCLK edge within max 350ns of /CS
; Immediate data read
; Execution time: 3 SIDCLKs.
; Register is read on current clock cycle
; Transfer time: CYCCHR + 1 1/2 SIDCLK.
__READ_REGI
	MOVLW	~CSBITM
	BSF		SIDCTL,	CTL_RW	; set RW high ahead of clock rise
	ANDWF	SIDCTL,	F		; SC high

	; Set data port to input. Apparently this can be done after SIDCTL. Saves a full cycle
	BSF	STATUS,	RP0
	COMF	SIDDDR,	F
	BCF		STATUS, RP0		; SC low, data valid

	NOP
	MOVF	SIDDAT,	W
	MOVWF	TXREG			; start RS232 1/2 SID cycle	; SC high

	MOVLW 	CSBITM
	IORWF	SIDCTL,	F
	BCF		SIDCTL,	CTL_RW	; SIDCLK low

	; Set data port back to output
	BSF		STATUS,	RP0
	CLRF	SIDDDR
	BCF		STATUS, RP0		; SC high

	NOP
	GOTO	__GET_ADDR		; SIDCLK low

__IOCTLS
; Reset IOCTL
; Trigger hardware SID reset
; Execution time: 1 SIDCLKs (+reset time)
; This will result in 1+20+1 SIDCLKs minimum total time (reset loop + rsync)
; Caution must thus be taken when calling this routine to not overflow the RX buffer
IOCTRS
	MOVLW	.20	; per datasheet, min 10 SIDCLKs for reset
	MOVWF	WAITCNT
	GOTO	$+1
	GOTO	sid_reset		; SIDCLK

; Firmware version IOCTL
; Returns current firmware version
; Execution time: 1 SIDCLKs (+send time)
IOCTFV
	MOVLW	FWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR		; SIDCLK

; Hardware version IOCTL
; Returns current hardware version
; Execution time: 1 SIDCLKs (+send time)
IOCTHV
	MOVLW	HWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR		; SIDCLK

; Long Delay IOCTL
; Delay value MUST BE >0
; TODO: implement as a timer interrupt?
; Execution time:
; Total SIDCLK count: 2*CYCCHR [addr+data] + WAITCNT(250*2 + 1) + CYCCHR [tx]
;			3*CYCCHR + WAITCNT(500 + 1)
IOCTLD
	GOTO $+1
	NOP

	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	IOCTLD			; 1 op / 2 cycles
	MOVF	RCREG,	W		; 1 op / 1 cycle	; SIDCLK

	; 2*CYCCHR before we arrive here: address decoding + own read
	MOVWF	WAITCNT+1
	GOTO	$+1

icld_waitlong
	MOVLW	0xFA			; wait x times 250 SIDCLKs
	MOVWF	WAITCNT
	NOP						; SIDCLK

icld_waitcycle
	GOTO	$+1
	GOTO	$+1
	GOTO	$+1				; SIDCLK

	GOTO	$+1
	NOP

	DECFSZ	WAITCNT,F
	GOTO	icld_waitcycle
	NOP						; SIDCLK
	; This loop executes in 2*SIDCLK.

	DECFSZ	WAITCNT+1,F
	GOTO	icld_waitlong
	NOP
	; When here, we are at SIDCLK/2

	CLRF	TXREG			; 1 op	dummy write / 1 CYCCHR before fully transmitted
	GOTO	__GET_ADDR		; SIDCLK

; A simple start tune routine that reads data from program memory
; Currently only handles writes and delay: addresses are written to the SIDs,
; unless it starts with 001xxxxx, in which case address<0:4> + data byte are
; used to form a 13bit delay value in multiples of 10 SIDCLKs.
;
; Note: this works because the PIC can store 14 bits per address location, and
; the SID address space fits 5 bits. We can thus pack SID address (5 bits) + SID
; data (8 bits) on a single location and have 1 extra bit for other use.
__INITTUNE
	BANKSEL EECON1		; 2 cycles Bank 3
	BSF	EECON1, EEPGD	; Point to PROGRAM memory
	BSF	EECON1, RD		; Toggle Read
	NOP					; Contrary to what the Datasheet says, the next instruction
						; after BSF EECON1,RD does *NOT* execute normally
	NOP	;SC				; Instruction here is ignored as program
						; memory is read in second cycle after BSF EECON1,RD

	; Fetch data read
	BCF	STATUS, RP0	; Bank 2
	MOVF	EEDAT,	W	; W = LSB of Program Memory
	MOVWF	LPMBYTE
	MOVF	EEDATH, W	; W = MSB of Program EEDAT
	MOVWF	HPMBYTE

	; prepare next memory address
	INCF	EEADRH,	F	; SC
	INCFSZ	EEADR,	F
	DECF	EEADRH,	F

	; Routine exit condition
	; 0x3F address byte marks end of tune: can't use 0x3Fxx delays
	SUBLW	0x3F

	BTFSC	STATUS, Z
	RETURN				; SC if Z (exit) -- XXX returns in Bank 2
	BCF	STATUS,	RP1		; SC if !Z - BANK 0 necessary for the rest of the code

	; Check if delay is requested
	BTFSC	HPMBYTE,5
	GOTO	st_delay

	; Copy received address to address port
	MOVF	HPMBYTE,W	; sc h
	IORLW	ADDRBITM	; Force RA5 High (Masking for startup tune)
	MOVWF	SIDADR

	; Copy received data to data port
	MOVF	LPMBYTE,W
	MOVWF	SIDDAT		; SC l

	; SID control lines
	GOTO	$+1

	MOVLW	~CSBITM
	ANDWF	SIDCTL,	F
	MOVLW	CSBITM		; SIDCLK fall

	IORWF	SIDCTL,	F
	GOTO	$+1

	NOP
	GOTO	__INITTUNE	; SC

	; Expects non-zero delay. delay = 1+1+10*cnt cycles
	; MSB can only be coded on 5 bits.
st_delay
	MOVF	HPMBYTE,W
	ANDLW	B'00011111'
	MOVWF	HPMBYTE		; SC low

st_dloop2				; 10 SIDCLKs (0.01 ms) loop
	MOVF	LPMBYTE,W
	BTFSC	STATUS, Z
	DECF	HPMBYTE,F	; if low = 0, dec high
	DECF	LPMBYTE,F
	MOVF	HPMBYTE,W
	IORWF	LPMBYTE,W	; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1			; SC

	GOTO	$+1
	NOP

	BTFSS	STATUS, Z
	GOTO	st_dloop2	; SC if !Z
	NOP					; SC if Z

	GOTO	$+1
	GOTO	$+1
	GOTO	__INITTUNE	; SC - both numbers 0, we're done.

; Delay (approx) macro, Time in ms
Mdelay		MACRO	Time
	if (Time > .79)
	error "Time cannot be more than 79ms"
	endif
	dw	(Time * .100) | 0x2000
	ENDM

; Clear all 25 writable SID registers
Mreset		MACRO
	local Reg = 0
	while (Reg < .25)
	dw	Reg << 8
Reg++
	endw
	ENDM

Mreg		MACRO Reg, Val
	dw	(Reg << 8) | Val
	ENDM

; Voice Macros (Voice = 1, 2, or 3)
Mvoice_Freq	MACRO Voice, Val16
	dw	(((Voice - 1) * 7) << 8) | (Val16 & 0xFF)
	dw	((((Voice - 1) * 7) + 1) << 8) | (Val16 >> 8)
	ENDM

Mvoice_PW	MACRO Voice, Val12
	dw	((((Voice - 1) * 7) + 2) << 8) | (Val12 & 0xFF)
	dw	((((Voice - 1) * 7) + 3) << 8) | ((Val12 >> 8) & 0x0F)
	ENDM

Mvoice_Ctrl MACRO Voice, Val8
	dw	((((Voice - 1) * 7) + 4) << 8) | Val8
	ENDM

Mvoice_AD	MACRO Voice, Val8
	dw	((((Voice - 1) * 7) + 5) << 8) | Val8
	ENDM

Mvoice_SR	MACRO Voice, Val8
	dw	((((Voice - 1) * 7) + 6) << 8) | Val8
	ENDM

; Filter Macros
Mcutoff		MACRO Val16
	dw	0x1500 | (Val16 & 0x07)
	dw	0x1600 | (Val16 >> 3)
	ENDM

; Master Volume
Mvol		MACRO Val8
	Mreg	0x18, Val8
	ENDM

__TUNEDATA	ORG	0x0200
	Mdelay				.50			; 50ms (Wait for power stabilization)
	Mdelay				.50			; 50ms

	Mreset							; Clear SID registers

	Mvoice_Ctrl			1, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Ctrl			2, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mvol 				0x1F		; Master Volume 15 | Filter: Low Pass
	Mdelay				.20			; 20ms

	Mvoice_Freq			1, 0x045A	; C-2
	Mvoice_PW			1, 0x320	;
	Mvoice_Ctrl			1, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_AD			1, 0x09		; Attack 0, Decay 9
	Mvoice_SR			1, 0x9A		; Sustain 9, Release 10
	Mvoice_Freq			2, 0x2E76	; F-5
	Mvoice_PW			2, 0x420	;
	Mvoice_Ctrl			2, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_AD			2, 0x09		; Attack 0, Decay 9
	Mvoice_SR			2, 0x99		; Sustain 9, Release 9
	Mvoice_Freq			3, 0x0751	; A-2
	Mvoice_Ctrl			3, 0x11		; Triangle, Start Note (Gate Open)
	Mvoice_SR			3, 0xE0		; Sustain 14, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			3, 0x00		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Ctrl			1, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0x60	;
	Mvoice_Ctrl			2, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Freq			3, 0x0127
	Mvoice_SR			3, 0x00		; Sustain 0, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0xE0	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mvoice_PW			2, 0x500	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0x60	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x80	;
	Mvoice_Freq			3, 0x0EA2	; A-3
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mvoice_SR			3, 0xE0		; Sustain 14, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0xE0	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x500	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mvoice_PW			2, 0x600	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x4E0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mvoice_PW			2, 0x5E0	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0xC0	;
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mreg				0x09, 0x60	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0x20	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x00	;
	Mvoice_Ctrl			3, 0x11		; Triangle, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x3E0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mvoice_PW			2, 0x4E0	;
	Mvoice_Freq			3, 0x5CED	; F-6
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0xC0	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0xA0	;
	Mvoice_Freq			3, 0x0127	; C#0
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x4E24	; D-6
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x5CED	; F-6
	Mreg				0x09, 0x00	;
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x2E76	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x3E05	; A#5
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mvoice_Ctrl			2, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_Freq			3, 0x0EA2	; A-3
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x80	;
	Mvoice_Ctrl			2, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xC0	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xE0	;
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mvoice_PW			2, 0x500	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x80	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x320	;
	Mvoice_Ctrl			1, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0xA0	;
	Mvoice_Freq			3, 0x0751	; A-2
	Mvoice_Ctrl			3, 0x11		; Triangle, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xC0	;
	Mvoice_Freq			3, 0x2E76	; F-5
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xE0	;
	Mvoice_Ctrl			3, 0x00		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Ctrl			1, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Freq			2, 0x57B6	; E-6
	Mvoice_PW			2, 0x600	;
	Mvoice_Freq			3, 0x0127
	Mvoice_SR			3, 0x00		; Sustain 0, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mvoice_PW			2, 0x5E0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x00	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mvoice_PW			2, 0x4E0	;
	Mvoice_Freq			3, 0x0751	; A-2
	Mvoice_Ctrl			3, 0x11		; Triangle, Start Note (Gate Open)
	Mvoice_SR			3, 0xE0		; Sustain 14, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xC0	;
	Mvoice_Freq			3, 0x2E76	; F-5
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xA0	;
	Mvoice_Ctrl			3, 0x00		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x500	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x80	;
	Mvoice_Freq			3, 0x0127
	Mvoice_SR			3, 0x00		; Sustain 0, Release 0
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x4E0	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x20	;
	Mvoice_Freq			3, 0x0EA2	; A-3
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mvoice_SR			3, 0xE0		; Sustain 14, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x00	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x20	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x60	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x3E0	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x80	;
	Mvoice_Ctrl			3, 0x11		; Triangle, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xA0	;
	Mvoice_Freq			3, 0x5CED	; F-6
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xC0	;
	Mvoice_Ctrl			3, 0x00		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0xE0	;
	Mvoice_Freq			3, 0x0127
	Mvoice_SR			3, 0x00		; Sustain 0, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mvoice_PW			2, 0x500	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x40	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0xA0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0xE0	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mvoice_PW			2, 0x600	;
	Mvoice_Freq			3, 0x0EA2	; A-3
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mvoice_SR			3, 0xE0		; Sustain 14, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mvoice_PW			2, 0x5E0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0xA0	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mreg				0x09, 0x60	;
	Mvoice_Ctrl			3, 0x80		; Noise, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			3, 0x81		; Noise, Start Note (Gate Open)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x57B6	; E-6
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2BDB	; E-5
	Mreg				0x09, 0x00	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x3427	; Note: G-5
	Mvoice_PW			2, 0x4E0	;
	Mvoice_Ctrl			3, 0x08		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x459D	; C-6
	Mreg				0x09, 0xC0	;
	Mdelay				.20			; 20ms

	Mvoice_Ctrl			3, 0x00		; Silent, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mvoice_Freq			1, 0x02E7	; F-1
	Mvoice_PW			1, 0x320	;
	Mvoice_Ctrl			1, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_Freq			2, 0x2E76	; F-5
	Mvoice_PW			2, 0x790	;
	Mvoice_Ctrl			2, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_AD			2, 0x06		; Attack 0, Decay 6
	Mvoice_SR			2, 0x6A		; Sustain 6, Release 10
	Mvoice_Freq			3, 0x0127
	Mvoice_SR			3, 0x00		; Sustain 0, Release 0
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mreg				0x09, 0x20	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x1D45	; A-4
	Mvoice_PW			2, 0x6B0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Ctrl			1, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Freq			2, 0x173B	; F-4
	Mreg				0x09, 0x40	;
	Mvoice_Ctrl			2, 0x50		; Triangle + Pulse, Stop Note (Gate Closed)
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mvoice_PW			2, 0x5D0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mreg				0x09, 0x60	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x1D65	; A-4
	Mvoice_PW			2, 0x4F0	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x177B	; F-4
	Mreg				0x09, 0x80	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x2E96	; F-5
	Mreg				0x09, 0x10	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mvoice_PW			2, 0x3A0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x1D25	; A-4
	Mreg				0x09, 0x30	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x16FB	; F-4
	Mvoice_PW			2, 0x2C0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E16	; F-5
	Mreg				0x09, 0x50	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x226F	; C-5
	Mvoice_PW			2, 0x1E0	;
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x1D05	; A-4
	Mreg				0x09, 0x70	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x500	;
	Mvoice_Freq			2, 0x171B	; F-4
	Mreg				0x09, 0x00	;
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x4E0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x22EF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x1D85	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x175B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x22AF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x1D05	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x16DB	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x3E0	;
	Mvoice_Freq			2, 0x2E16	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x228F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x1D25	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x173B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E96	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x230F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x1D65	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x173B	; F-4
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x2E56	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x228F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x1CE5	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x16DB	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2E36	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x22AF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x1D45	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x175B	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x2EB6	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x22EF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x1D45	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x171B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2E36	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x226F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x1CE5	; A-4
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x16FB	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x320	;
	Mvoice_Ctrl			1, 0x41		; Pulse, Start Note (Gate Open)
	Mvoice_Freq			2, 0x2E56	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x1D65	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Ctrl			1, 0x40		; Pulse, Stop Note (Gate Closed)
	Mvoice_Freq			2, 0x177B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E96	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x1D25	; A-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x16FB	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x2E16	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x226F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x1D05	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x171B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x22EF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x1D85	; A-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x500	;
	Mvoice_Freq			2, 0x175B	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x4E0	;
	Mvoice_Freq			2, 0x2E76	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x22AF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x1D05	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x16DB	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E16	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x228F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x1D25	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x173B	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x3E0	;
	Mvoice_Freq			2, 0x2E96	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x230F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x1D65	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x173B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x2E56	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x228F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x1CE5	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x00	;
	Mvoice_Freq			2, 0x16DB	; F-4
	Mdelay				.20			; 20ms

	Mvoice_Freq			2, 0x2E36	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x22AF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x1D45	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x175B	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2EB6	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x22EF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x1D45	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xE0	;
	Mvoice_Freq			2, 0x171B	; F-4
	Mdelay				.20			; 20ms

	Mvoice_PW			1, 0x400	;
	Mvoice_Freq			2, 0x2E36	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x20	;
	Mvoice_Freq			2, 0x226F	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x40	;
	Mvoice_Freq			2, 0x1CE5	; A-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x60	;
	Mvoice_Freq			2, 0x16FB	; F-4
	Mdelay				.20			; 20ms

	Mreg				0x02, 0x80	;
	Mvoice_Freq			2, 0x2E56	; F-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xA0	;
	Mvoice_Freq			2, 0x22CF	; C-5
	Mdelay				.20			; 20ms

	Mreg				0x02, 0xC0	;
	Mvoice_Freq			2, 0x1D65	; A-4

	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	Mdelay				.50			; 50ms
	
	dw	0x3FFF						; END

; Branch table. IOCTL are interleaved with valid SID addresses
; Execution time: 6 cycles / 1 SIDCLK
; XXX predelay 5 cycles: I suspect the MOVWF PCL requires 2 cycles to take effect

; --- CHANGED for MASKING ---
; We have 5 instructions now instead of 3.
; We moved ORG back by 2 bytes (3->5)
BTABLE	ORG	JMPTBA-5
	MOVF	TEMPBUF,W
	IORLW	ADDRBITM	; Force RA5 High (1 cycle)
	MOVWF	SIDADR		; Output address (Write happens at correct time)
	MOVF	TEMPBUF,W
	MOVWF	PCL			; PCLATH must be set before the jump
	FILL	(GOTO	__WRITE_REGI), 0x19	; 25 valid write addresses up to 0x18, data
	FILL	(GOTO	__READ_REGI), 0x4	; 4 valid read addresses up to 0x1C, no data
	NOP				; 0x1D UNUSED
	NOP				; 0x1E UNUSED
	NOP				; 0x1F UNUSED
	; Wrapped addresses for delayed I/O start here
	FILL	(GOTO	__WRITE_REGD), 0x19	; calling delayed write
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x3D UNUSED
	NOP				; 0x3E UNUSED
	NOP				; 0x3F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x5D UNUSED
	NOP				; 0x5E UNUSED
	NOP				; 0x5F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x7D UNUSED
	NOP				; 0x7E UNUSED
	NOP				; 0x7F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	__GET_ADDR					; 0x9D IOCTD1 CYCCHR SIDCLK delay, no data
	GOTO	IOCTLD						; 0x9E IOCTLD long delay, amount in data, returns 1 byte
	NOP				; 0x9F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	__MAIN						; 0xBD We removed CS logic. We only have one SID
	GOTO	__MAIN						; 0xBE
	GOTO	__MAIN						; 0xBF
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0xDD UNUSED
	NOP				; 0xDE UNUSED
	NOP				; 0xDF UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	IOCTFV						; 0xFD Firmware version, no data, returns 1 byte
	GOTO	IOCTHV						; 0xFE Hardware version, no data, returns 1 byte
__END	GOTO	IOCTRS					; 0xFF Reset, no data

; !!! 0x07FF Program must stay in Page 0 !!!
	END

; CHANGELOG

; Version 10:	- Initial release, support for 750kbps/1Mbps
; Version 11:	- Switch to integer versioning (1.0 became 10)
;		- Support for 2Mbps on 88X
;		- Support for rev.C hardware
;		- Improved SID clocking
;		- Fixed init tune timing bug
;		- Switched to deep note init tune
;		- Fixed timing bug (buffer overrun when consecutive delayed writes with maximum adjustment)

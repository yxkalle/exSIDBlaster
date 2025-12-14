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

	title		"exSID rev.C Firmware v11"
	
#define	FWVERS		.11
#define HWVERS		"S"

	list		p=16f886
	errorlevel	-207, -302
 #include		p16f886.inc
 ; #define		__PIC16F88X

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
	
#define	SIDCTL		PORTC		; SID bus control port
#define	SIDADR		PORTA		; SID bus address port
#define	SIDRST		PORTA		; SID reset address port
#define	SIDDAT		PORTB		; SID bus data port
#define	SIDDDR		TRISB		; SID bus data direction

#define	EXTCLK		RA7

#define	CTLCS1		RC0
#define	CTL_RW		RC1
;#define	CTLCS0		RC3
#define	CTLRST		RA5			; Resides in PORTA

#define	CTLCLK		RC2			; SID CLK on SIDCTL
#define	FTDIRX		RC6
#define	FTDITX		RC7
	
; By default RW is held low. The read routine must bring it back low after exec.
; Chip select bitmask.
;#define	CSBBITM		(B'1' << CTLCS0 | B'1' << CTLCS1)
;#define	CS0BITM		(B'1' << CTLCS0)
#define	CS1BITM		(B'1' << CTLCS1)
#define	RSTBITM		(B'1' << CTLRST)

#define	JMPTBA		H'0700'		; Jump table base address. 0x07xx stays in Page 0 and ensures no screw up with GOTO/CALL

	; Cross-bank data registers
	CBLOCK	H'70'
		TEMPBUF					; typically holds current address
		WAITCNT:2				; SID clock loop counter
		CSENCTL					; CS Enable bits
		CSDICTL					; CS Disable bits
		LPMBYTE					; Pgm memory LSB
		HPMBYTE					; Pgm memory MSB
	ENDC


; Program
__INIT	ORG	0	

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
	CLRF	SIDADR			; Clear SID address. This enables SID /reset
	CLRF	SIDDAT			; Clear SID data
	MOVLW	CS1BITM			; disable sid 1 as initial condition
	MOVWF	SIDCTL			; Clear SID control except CS1 (preset high).
	
	; By default we enable 'sid 1' operation
	MOVLW	~CS1BITM
	MOVWF	CSENCTL			; CS enable bits
	MOVLW	CS1BITM
	MOVWF	CSDICTL			; CS disable bits
	
	MOVLW	high JMPTBA
	MOVWF	PCLATH			; Preset PCLATH for jump table == MSB of JMPTBA
	
	MOVLW	.20
	MOVWF	WAITCNT			; Initial reset time
	
	; Enable outputs
	BANKSEL	TRISA
	MOVLW	B'1' << EXTCLK
	MOVWF	TRISA			; TRISA   <0:4>:address <5>:rst <7>:clk12 [in]
	CLRF	TRISB			; TRISB   <0:7>:data
	MOVLW	B'1' << FTDITX
	MOVWF	TRISC			; TRISC   <0>:cs1 <1>:rw <2>:clk <6>:tx <7>:rx [in]
	
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
	CLRF	SPBRGH
	MOVLW   .2			; 2Mbps 0% error @24MHz on 16F88X
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
	BCF	TRISC, RC2	; Make sure pin is an output
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
	NOP

	BSF	SIDRST,	CTLRST	; Clear reset condition before playing the startup tune
	; start address must be set before the jump
	BCF	STATUS,	RP0
	BSF	STATUS, RP1	; BANK 2, SC low
	
	MOVLW	high __TUNEDATA
	MOVWF	EEADRH
	MOVLW	low __TUNEDATA
	
	MOVWF	EEADR		
	CALL	__INITTUNE	; SC low
	
_sid_reset	; 1 SIDCLK loop: SID /RST for WAITCNT SIDCLKs (min 10 per datasheet)
	BCF	STATUS,	RP0
	BCF	STATUS,	RP1	; Back in BANK 0, because __INITTUNE returns from BANK 2
	BCF	SIDRST,	CTLRST	; Re-enable reset

	DECFSZ	WAITCNT,F
	GOTO	_sid_reset
	BSF	SIDRST,	CTLRST	; SIDCLK. Clear reset condition.
	
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
_wrd_data	; Wait for data byte
	GOTO	$+1
	NOP
	
	BTFSS	PIR1,	RCIF
	GOTO	_wrd_data
	MOVF	RCREG,	W				; SIDCLK

_wrd_waitone
	MOVWF	SIDDAT	; Done here to compensate shifted GOTO
	DECF	WAITCNT,F
	NOP
	
	DECFSZ	WAITCNT,F
	GOTO	_wrd_waitone
							; SIDCLK
	GOTO	_wri	; this GOTO will be 1 cycle late, so we shift inside __WRITE_REGI		

	
; Refer to SID datasheet for timing considerations

; Immediate data write
; SID writes are latched on falling SIDCLK edge
; Execution time: CYCCHR + 2 SIDCLKs max
__WRITE_REGI
_wri_data	; Wait for data byte
	GOTO	$+1		; 1 op / 2 cycles
	NOP			; 1 op / 1 cycle
	
	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	_wri_data	; 1 op / 2 cycles
	MOVF	RCREG,	W	; 1 op / 1 cycle	; SIDCLK fall

	; Copy received byte to data port
	MOVWF	SIDDAT		; 1 op / 1 cycle
	; SID control lines
_wri	MOVF	CSENCTL,W	; 1 op / 1 cycle	
	NOP
	
	NOP
	ANDWF	SIDCTL,	F	; 1 op / 1 cycle	
	MOVF	CSDICTL,W	; 1 op / 1 cycle	; SIDCLK fall: write effective

	IORWF	SIDCTL,	F	; 1 op / 1 cycle 
	NOP			; 1 op / 1 cycle
	GOTO	$+1		; 1 op / 2 cycles
	GOTO	__GET_ADDR	; 1 op / 2 cycles	; SIDCLK


; Get requested address
; In order for cycle-accurate writes to work, execution time after clearing RCREG
; MUST NOT take more than CYCCHR/2
; Execution time: CYCCHR + 1 + 1 (jump table) SIDCLKs max
; EVERY CALL HERE MUST TAKE 2 CYCLES LESS SINCE WE'VE ADDED 2 TO BTABLE
_ga_waddr
	GOTO	$+1
__GET_ADDR
	NOP
	
	BTFSS	PIR1,	RCIF
	GOTO	_ga_waddr 
	MOVF	RCREG,	W				; SIDCLK
	
	MOVWF	TEMPBUF		; Copy address to TEMPBUF, used by jump table
	ANDLW	B'11100000'	; W: ddd00000
	MOVWF	WAITCNT		; WAITCNT: ddd00000
	
	SWAPF	WAITCNT, F	; WAITCNT: 0000ddd0 => ddd is delay cycles
	GOTO	BTABLE					; SIDCLK
	; JUMP TABLE BASED ON CONTENT OF TEMPBUF, aka ADDRESS


; Delayed data read
; Code flow continues into __READ_REGI to save a goto and maintain clock alignment
; Execution time: WAITCNT SIDCLKs. Ensures read happens after exactly WAITCNT SIDclks
__READ_REGD
_rrd_waitone
	DECF	WAITCNT,F
	GOTO	$+1
	
	DECFSZ	WAITCNT,F
	GOTO	_rrd_waitone
	NOP						; SIDCLK
	
; SID reads are latched before falling SIDCLK edge within max 350ns of /CS
; Immediate data read
; Execution time: 3 SIDCLKs.
; Register is read on current clock cycle
; Transfer time: CYCCHR + 1 1/2 SIDCLK.
__READ_REGI
	MOVF	CSENCTL,W
	BSF	SIDCTL,	CTL_RW	; set RW high ahead of clock rise
	ANDWF	SIDCTL,	F				; SC high
	
	; Set data port to input. Apparently this can be done after SIDCTL. Saves a full cycle
	BSF	STATUS,	RP0
	COMF	SIDDDR,	F
	BCF	STATUS, RP0				; SC low, data valid
	
	NOP
	MOVF	SIDDAT,	W
	MOVWF	TXREG		; start RS232 1/2 SID cycle	; SC high
	
	MOVF	CSDICTL,W 
	IORWF	SIDCTL,	F
	BCF	SIDCTL,	CTL_RW				; SIDCLK low

	; Set data port back to output
	BSF	STATUS,	RP0
	CLRF	SIDDDR
	BCF	STATUS, RP0				; SC high

	NOP
	GOTO	__GET_ADDR				; SIDCLK low

	
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
	GOTO	_sid_reset				; SIDCLK

; Select SID0 IOCTL
; Set bitmasks for SID0-only operation
; Execution time: 1 SIDCLKs
;IOCTS0
;	MOVLW	~CS0BITM
;	MOVWF	CSENCTL			; Write enable bits
;	MOVLW	CS0BITM
;
;	MOVWF	CSDICTL			; Write disable bits
;	GOTO	__GET_ADDR				;SIDCLK
	

; Select SID1 IOCTL
; Set bitmasks for SID1-only operation
; Execution time: 1 SIDCLKs
IOCTS1
	MOVLW	~CS1BITM
	MOVWF	CSENCTL			; Write enable bits
	MOVLW	CS1BITM
	
	MOVWF	CSDICTL			; Write disable bits
	GOTO	__GET_ADDR				; SIDCLK

; Select both SIDs IOCTL
; Set bitmasks for dual SID operation
; Execution time: 1 SIDCLKs
;IOCTSB
;	MOVLW	~CSBBITM
;	MOVWF	CSENCTL			; Write enable bits
;	MOVLW	CSBBITM
;	
;	MOVWF	CSDICTL			; Write disable bits
;	GOTO	__GET_ADDR				; SIDCLK

; Firmware version IOCTL
; Returns current firmware version
; Execution time: 1 SIDCLKs (+send time)
IOCTFV
	MOVLW	FWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR				; SIDCLK

; Hardware version IOCTL
; Returns current hardware version
; Execution time: 1 SIDCLKs (+send time)
IOCTHV
	MOVLW	HWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR				; SIDCLK

; Long Delay IOCTL
; Delay value MUST BE >0
; TODO: implement as a timer interrupt?
; Execution time:
; Total SIDCLK count: 2*CYCCHR [addr+data] + WAITCNT(250*2 + 1) + CYCCHR [tx]
;			3*CYCCHR + WAITCNT(500 + 1)
IOCTLD
	GOTO	$+1
	NOP
	
	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	IOCTLD		; 1 op / 2 cycles
	MOVF	RCREG,	W	; 1 op / 1 cycle	; SIDCLK

	; 2*CYCCHR before we arrive here: address decoding + own read
	MOVWF	WAITCNT+1
	GOTO	$+1
	
_icld_waitlong
	MOVLW	.250			; wait x times 250 SIDCLKs
	MOVWF	WAITCNT
	NOP						; SIDCLK
	
_icld_waitcycle
	GOTO	$+1
	GOTO	$+1
	GOTO	$+1					; SIDCLK
	
	GOTO	$+1
	NOP
	
	DECFSZ	WAITCNT,F
	GOTO	_icld_waitcycle
	NOP						; SIDCLK
	; This loop executes in 2*SIDCLK.
	
	DECFSZ	WAITCNT+1,F
	GOTO	_icld_waitlong
	NOP
	; When here, we are at SIDCLK/2
	
	CLRF	TXREG		; 1 op	dummy write / 1 CYCCHR before fully transmitted
	GOTO	__GET_ADDR				; SIDCLK
	
	
; A simple start tune routine that reads data from program memory
; Currently only handles writes and delay: addresses are written to the SIDs,
; unless it starts with 001xxxxx, in which case address<0:4> + data byte are
; used to form a 13bit delay value in multiples of 10 SIDCLKs.
; Since this routine is called before external input can mess with the device,
; by default it will play the tune on both SIDs simultaneously.
;
; Note: this works because the PIC can store 14 bits per address location, and
; the SID address space fits 5 bits. We can thus pack SID address (5 bits) + SID
; data (8 bits) on a single location and have 1 extra bit for other use.
__INITTUNE
	BANKSEL EECON1		; 2 cycles Bank 3
	BSF	EECON1, EEPGD	; Point to PROGRAM memory
	BSF	EECON1, RD	; Toggle Read
	NOP			; Contrary to what the Datasheet says, the next instruction
				; after BSF EECON1,RD does *NOT* execute normally
	NOP	;SC		; Instruction here is ignored as program
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
	; H'3F' address byte marks end of tune: can't use 0x3Fxx delays
	SUBLW	H'3F'
	
	BTFSC	STATUS, Z
	RETURN			; SC if Z (exit) -- XXX returns in Bank 2
	BCF		STATUS, RP1	; SC if !Z - BANK 0 necessary for the rest of the code

	; Check if delay is requested
	BTFSC	HPMBYTE, 5
	GOTO	_st_delay
	
	; Copy received address to address port
	MOVF	HPMBYTE,W	; sc h
	IORLW   RSTBITM		; Keep reset clear
	MOVWF	SIDADR
	
	; Copy received data to data port
	MOVF	LPMBYTE,W	; SC l
	MOVWF	SIDDAT

	; SID control lines
	; GOTO	$+1			; Removed to make room for reset pin masking
	NOP
	NOP					; sc h
	
	MOVF	CSENCTL,W	
	ANDWF	SIDCTL,	F
	MOVF	CSDICTL,W	; SIDCLK fall

	IORWF	SIDCTL,	F
	GOTO	$+1
	
	NOP
	GOTO	__INITTUNE	; SC
	
	; Expects non-zero delay. delay = 1+1+10*cnt cycles
	; MSB can only be coded on 5 bits.
_st_delay
	MOVF	HPMBYTE,W
	ANDLW	B'00011111'
	MOVWF	HPMBYTE		; SC low	

_st_dloop2	; 10 SIDCLKs (0.01 ms) loop
	MOVF	LPMBYTE,W
	BTFSC	STATUS, Z
	DECF	HPMBYTE,F	; if low = 0, dec high
	DECF	LPMBYTE,F
	MOVF	HPMBYTE,W
	IORWF	LPMBYTE,W	; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC
	
	GOTO	$+1
	NOP
	
	BTFSS	STATUS, Z
	GOTO	_st_dloop2	; SC if !Z
	NOP			; SC if Z
	
	GOTO	$+1
	GOTO	$+1	
	GOTO	__INITTUNE	; SC - both numbers 0, we're done.
				
; Delay (approx), Time in ms
; H'3F' address byte marks end of tune - we can't use H'3F'xx delays
SID_DELAY			MACRO Time
	IF				(Time < 1) || (Time > .79)
		ERROR		"SID_DELAY: Time cannot be less than 1 or more than 79ms."
	ENDIF
	DW				H'2000' | ((Time * .100) & H'1FFF')
	ENDM

; Clear all 25 writeable SID registers
SID_RESET			MACRO
	LOCAL			_reg = 0
	WHILE			_reg < .25
		DW			_reg << 8
		_reg++
	ENDW
	ENDM
	
; Voice frequency
SID_FREQ			MACRO Voice, Value
	IF 				(Value < 0) || (Value > H'FFFF')
		ERROR 		"SID_FREQ: Value must be between 0 and 65535."
	ENDIF
	IF 				(Voice < 1) || (Voice > 3)
		ERROR 		"SID_FREQ: Voice must be between 1 and 3."
	ENDIF
	DW				((((Voice - 1) * 7) << 8) & H'1F00') | (Value & H'FF')
	DW				(((((Voice - 1) * 7) + 1) << 8) & H'1F00') | ((Value >> 8) & H'FF')
	ENDM

; Voice pulse width (low byte)
SID_PWLO			MACRO Voice, Value
	IF 				(Voice < 1) || (Voice > 3)
		ERROR 		"SID_PW: Voice must be between 1 and 3."
	ENDIF
	DW				(((((Voice - 1) * 7) + 2) << 8) & H'1F00') | (Value & H'FF')
	ENDM

; Voice pulse-width (low byte + high nibble)
SID_PW				MACRO Voice, Value
	IF 				(Value < 0) || (Value > H'FFF')
		ERROR 		"SID_PW: Value must be between 0 and 4095."
	ENDIF
	SID_PWLO		Voice, Value
	DW	 			(((((Voice - 1) * 7) + 3) << 8) & H'1F00') | ((Value >> 8) & H'F')
	ENDM

; Voice control
SID_CR				MACRO Voice, Value
	IF 				(Value < 0) || (Value > H'FF')
		ERROR 		"SID_CR: Value must be between 0 and 255."
	ENDIF
	IF 				(Voice < 1) || (Voice > 3)
		ERROR 		"SID_CR: Voice must be between 1 and 3."
	ENDIF
	DW				(((((Voice - 1) * 7) + 4) << 8) & H'1F00') | (Value & H'FF')
	ENDM

; Voice ADsr (attack + delay)
SID_AD				MACRO Voice, Value
	IF 				(Value < 0) || (Value > H'FF')
		ERROR 		"SID_AD: Value must be between 0 and 255."
	ENDIF
	IF 				(Voice < 1) || (Voice > 3)
		ERROR 		"SID_AD: Voice must be between 1 and 3."
	ENDIF
	DW				(((((Voice - 1) * 7) + 5) << 8) & H'1F00') | (Value & H'FF')
	ENDM

; Voice adSR (sustain + release)
SID_SR				MACRO Voice, Value
	IF 				(Value < 0) || (Value > H'FF')
		ERROR 		"SID_CSR Value must be between 0 and 255."
	ENDIF
	IF 				(Voice < 1) || (Voice > 3)
		ERROR 		"SID_SR: Voice must be between 1 and 3."
	ENDIF
	DW				(((((Voice - 1) * 7) + 6) << 8) & H'1F00') | (Value & H'FF')
	ENDM

; Filter mode + master volume
SID_VOL_MODE		MACRO Volume, Mode
	IF 				(Volume < 0) || (Volume > H'F')
		ERROR 		"SID_VOL_MODE: Volume must be between 0 and 15."
	ENDIF
	; <0>:Low pass <1>:Band pass <2>:High pass <3>: Chan 3 off
	IF 				(Mode < 0) || (Mode > H'F')
		ERROR 		"SID_VOL_MODE: Mode must be between 0 and 15."
	ENDIF
	DW				H'1800' | ((Mode << 4) & H'F0') | (Volume & H'F')
	ENDM

; BOOT JINGLE
; The "GET READY" tune from Giana Sisters by Chris Huelsbeck.
; Duration is about 4s
__TUNEDATA	ORG	H'200'
	; Wait for power stabilization
	SID_DELAY		.50			; 50ms
	SID_DELAY		.50			; 100ms

	SID_RESET					; Clear SID registers

	SID_CR			1, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_CR			2, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_VOL_MODE	.15, 1		; Master Volume 15 | Filter: Low Pass
	SID_DELAY		.20			; 20ms

	SID_FREQ		1, H'045A'	; C-2
	SID_PW			1, H'320'	;
	SID_CR			1, H'41'	; Pulse, Start Note (Gate Open)
	SID_AD			1, H'09'	; Attack 0, Decay 9
	SID_SR			1, H'9A'	; Sustain 9, Release 10
	SID_FREQ		2, H'2E76'	; F-5
	SID_PW			2, H'420'	;
	SID_CR			2, H'41'	; Pulse, Start Note (Gate Open)
	SID_AD			2, H'09'	; Attack 0, Decay 9
	SID_SR			2, H'99'	; Sustain 9, Release 9
	SID_FREQ		3, H'0751'	; A-2
	SID_CR			3, H'11'	; Triangle, Start Note (Gate Open)
	SID_SR			3, H'E0'	; Sustain 14, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'40'	;
	SID_CR			3, H'00'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_CR			1, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'60'	;
	SID_CR			2, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_FREQ		3, H'0127'	;
	SID_SR			3, H'00'	; Sustain 0, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'E0'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PW			2, H'500'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'60'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'80'	;
	SID_FREQ		3, H'0EA2'	; A-3
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_SR			3, H'E0'	; Sustain 14, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'E0'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'500'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PW			2, H'600'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'4E0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PW			2, H'5E0'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'C0'	;
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PWLO		2, H'60'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'20'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'00'	;
	SID_CR			3, H'11'	; Triangle, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'3E0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PW			2, H'4E0'	;
	SID_FREQ		3, H'5CED'	; F-6
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'C0'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'A0'	;
	SID_FREQ		3, H'0127'	; C#0
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'40'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'4E24'	; D-6
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'5CED'	; F-6
	SID_PWLO		2, H'00'	;
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'2E76'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'3E05'	; A#5
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_CR			2, H'41'	; Pulse, Start Note (Gate Open)
	SID_FREQ		3, H'0EA2'	; A-3
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'80'	;
	SID_CR			2, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'C0'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'E0'	;
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PW			2, H'500'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'40'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'80'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'320'	;
	SID_CR			1, H'41'	; Pulse, Start Note (Gate Open)
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'A0'	;
	SID_FREQ		3, H'0751'	; A-2
	SID_CR			3, H'11'	; Triangle, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'C0'	;
	SID_FREQ		3, H'2E76'	; F-5
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'E0'	;
	SID_CR			3, H'00'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_CR			1, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_FREQ		2, H'57B6'	; E-6
	SID_PW			2, H'600'	;
	SID_FREQ		3, H'0127'	;
	SID_SR			3, H'00'	; Sustain 0, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PW			2, H'5E0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'00'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PW			2, H'4E0'	;
	SID_FREQ		3, H'0751'	; A-2
	SID_CR			3, H'11'	; Triangle, Start Note (Gate Open)
	SID_SR			3, H'E0'	; Sustain 14, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'C0'	;
	SID_FREQ		3, H'2E76'	; F-5
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'A0'	;
	SID_CR			3, H'00'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'500'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'80'	;
	SID_FREQ		3, H'0127'	;
	SID_SR			3, H'00'	; Sustain 0, Release 0
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'4E0'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'40'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'20'	;
	SID_FREQ		3, H'0EA2'	; A-3
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_SR			3, H'E0'	; Sustain 14, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'00'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'20'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'60'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'3E0'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'80'	;
	SID_CR			3, H'11'	; Triangle, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'A0'	;
	SID_FREQ		3, H'5CED'	; F-6
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'C0'	;
	SID_CR			3, H'00'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'E0'	;
	SID_FREQ		3, H'0127'	;
	SID_SR			3, H'00'	; Sustain 0, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PW			2, H'500'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'40'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'A0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'E0'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PW			2, H'600'	;
	SID_FREQ		3, H'0EA2'	; A-3
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_SR			3, H'E0'	; Sustain 14, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PW			2, H'5E0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'A0'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PWLO		2, H'60'	;
	SID_CR			3, H'80'	; Noise, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'40'	;
	SID_CR			3, H'81'	; Noise, Start Note (Gate Open)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'57B6'	; E-6
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2BDB'	; E-5
	SID_PWLO		2, H'00'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'3427'	; G-5
	SID_PW			2, H'4E0'	;
	SID_CR			3, H'08'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'459D'	; C-6
	SID_PWLO		2, H'C0'	;
	SID_DELAY		.20			; 20ms

	SID_CR			3, H'00'	; Silent, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_FREQ		1, H'02E7'	; F-1
	SID_PW			1, H'320'	;
	SID_CR			1, H'41'	; Pulse, Start Note (Gate Open)
	SID_FREQ		2, H'2E76'	; F-5
	SID_PW			2, H'790'	;
	SID_CR			2, H'41'	; Pulse, Start Note (Gate Open)
	SID_AD			2, H'06'	; Attack 0, Decay 6
	SID_SR			2, H'6A'	; Sustain 6, Release 10
	SID_FREQ		3, H'0127'	;
	SID_SR			3, H'00'	; Sustain 0, Release 0
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_PWLO		2, H'20'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'1D45'	; A-4
	SID_PW			2, H'6B0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_CR			1, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_FREQ		2, H'173B'	; F-4
	SID_PWLO		2, H'40'	;
	SID_CR			2, H'50'	; Triangle + Pulse, Stop Note (Gate Closed)
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_PW			2, H'5D0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_PWLO		2, H'60'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'1D65'	; A-4
	SID_PW			2, H'4F0'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'177B'	; F-4
	SID_PWLO		2, H'80'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'2E96'	; F-5
	SID_PWLO		2, H'10'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_PW			2, H'3A0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'1D25'	; A-4
	SID_PWLO		2, H'30'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'16FB'	; F-4
	SID_PW			2, H'2C0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E16'	; F-5
	SID_PWLO		2, H'50'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'226F'	; C-5
	SID_PW			2, H'1E0'	;
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'1D05'	; A-4
	SID_PWLO		2, H'70'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'500'	;
	SID_FREQ		2, H'171B'	; F-4
	SID_PWLO		2, H'00'	;
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'4E0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'22EF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'1D85'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'175B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'22AF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'1D05'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'16DB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'3E0'	;
	SID_FREQ		2, H'2E16'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'228F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'1D25'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'173B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E96'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'230F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'1D65'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'173B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'2E56'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'228F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'1CE5'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'16DB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2E36'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'22AF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'1D45'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'175B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'2EB6'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'22EF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'1D45'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'171B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2E36'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'226F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'1CE5'	; A-4
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'16FB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'320'	;
	SID_CR			1, H'41'	; Pulse, Start Note (Gate Open)
	SID_FREQ		2, H'2E56'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'1D65'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_CR			1, H'40'	; Pulse, Stop Note (Gate Closed)
	SID_FREQ		2, H'177B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E96'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'1D25'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'16FB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'2E16'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'226F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'1D05'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'171B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'22EF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'1D85'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'500'	;
	SID_FREQ		2, H'175B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'4E0'	;
	SID_FREQ		2, H'2E76'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'22AF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'1D05'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'16DB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E16'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'228F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'1D25'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'173B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'3E0'	;
	SID_FREQ		2, H'2E96'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'230F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'1D65'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'173B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'2E56'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'228F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'1CE5'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'00'	;
	SID_FREQ		2, H'16DB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_FREQ		2, H'2E36'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'22AF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'1D45'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'175B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2EB6'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'22EF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'1D45'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'E0'	;
	SID_FREQ		2, H'171B'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PW			1, H'400'	;
	SID_FREQ		2, H'2E36'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'20'	;
	SID_FREQ		2, H'226F'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'40'	;
	SID_FREQ		2, H'1CE5'	; A-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'60'	;
	SID_FREQ		2, H'16FB'	; F-4
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'80'	;
	SID_FREQ		2, H'2E56'	; F-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'A0'	;
	SID_FREQ		2, H'22CF'	; C-5
	SID_DELAY		.20			; 20ms

	SID_PWLO		1, H'C0'	;
	SID_FREQ		2, H'1D65'	; A-4
	
	; Wait for any sustained note to fade out
	SID_DELAY		.75			;  75ms
	SID_DELAY		.75			; 150ms
	SID_DELAY		.75			; 225ms
	SID_DELAY		.75			; 300ms
	SID_DELAY		.75			; 375ms
	SID_DELAY		.75			; 450ms
	SID_DELAY		.50			; 500ms

	DW	H'3FFF'					; END

; Branch table. IOCTL are interleaved with valid SID addresses
; Execution time: 6 cycles / 1 SIDCLK
; XXX predelay 5 cycles: I suspect the MOVWF PCL requires 2 cycles to take effect
BTABLE	ORG	JMPTBA-5	; Added 2 operations to keep the Reset pin clear
	MOVF	TEMPBUF, W
	IORLW	RSTBITM		; Force RA5 High (1 cycle)
	MOVWF	SIDADR		; Output address (Write happens at correct time)
	MOVF	TEMPBUF, W	; Reload unaltered TEMPBUF into W
	MOVWF	PCL			; PCLATH must be set before the jump
	FILL	(GOTO	__WRITE_REGI),	H'19'	; 25 valid write addresses up to H'18', data
	FILL	(GOTO	__READ_REGI),	4		; 4 valid read addresses up to H'1C', no data
	NOP				; H'1D' UNUSED
	NOP				; H'1E' UNUSED
	NOP				; H'1F' UNUSED
	; Wrapped addresses for delayed I/O start here
	FILL	(GOTO	__WRITE_REGD),	H'19'	; calling delayed write
	FILL	(GOTO	__READ_REGD),	4
	NOP				; H'3D' UNUSED
	NOP				; H'3E' UNUSED
	NOP				; H'3F' UNUSED
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4
	NOP				; H'5D' UNUSED
	NOP				; H'5E' UNUSED
	NOP				; H'5F' UNUSED
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4
	NOP				; H'7D' UNUSED
	NOP				; H'7E' UNUSED
	NOP				; H'7F' UNUSED
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4
	GOTO	__GET_ADDR						; H'9D' IOCTD1 CYCCHR SIDCLK delay, no data
	GOTO	IOCTLD							; H'9E' IOCTLD long delay, amount in data, returns 1 byte
	NOP				; H'9F' UNUSED
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4
;	GOTO	IOCTS0							; H'BD' IOCTS0 select Chip 0, no data
;	GOTO	IOCTSB							; H'BF' IOCTSB select both chips, no data
	GOTO	IOCTS1							; H'BD' IOCTS1 select Chip 1, no data
	GOTO	IOCTS1							; H'BE' IOCTS1 select Chip 1, no data
	GOTO	IOCTS1							; H'BF' IOCTS1 select Chip 1, no data
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4
	NOP										; H'DD' UNUSED
	NOP										; H'DE' UNUSED
	NOP										; H'DF' UNUSED
	FILL	(GOTO	__WRITE_REGD),	H'19'
	FILL	(GOTO	__READ_REGD),	4	
	GOTO	IOCTFV							; H'FD' Firmware version, no data, returns 1 byte
	GOTO	IOCTHV							; H'FE' Hardware version, no data, returns 1 byte
__END	
	GOTO	IOCTRS							; H'FF' Reset, no data

; !!! H'07FF' Program must stay in Page 0 !!!		
	END
	
; CHANGELOG
	
; Version 10:	- Initial release, support for 750kbps/1Mbps
; Version 11:	- Switch to integer versionning (1.0 became 10)
;		- Support for 2Mbps on 88X
;		- Support for rev.C hardware
;		- Improved SID clocking
;		- Fixed init tune timing bug
;		- Switched to deep note init tune
;		- Fixed timing bug (buffer overrun when consecutive delayed writes with maximum adjustment)

;   	TITLE		"Code for Railcom to CBUS module  FLiM node for CBUS"
; filename CANRCOM_test5	 	12/10/20
; This code is for a PIC  18F25K80

; Assembly options
	LIST	P=18F25K80,r=hex,N=75,C=120,T=ON

	include		"p18f25K80.inc"
	include		"..\cbuslib\cbusdefs8r.inc"

; uses  "..\cbuslib\cbusdefs8r"

; based on CANRC522 and snds DDES frames. Four data bytes / frame.
; User teaches a short event for the 'device number'

; 

; Uses 16 MHz resonator and PLL for 64 MHz clock
; The setup timer is TMR0. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only except for bootloader
; Uses a CANMIO board with a CT on a small board connected to the 10 way IDC header.
; As test 1 but mod for short loco addresses and equal Ch 1 times.
; As test 3 but with channel 2 added
; Now test 5. Has dual frequency bootloader and 'name' OpCode.



OPC_DDES    EQU	0xFA	; Short data frame aka device data event (device id plus 5 data bytes)

#define FLIM_ONLY

;This code doesn't use the event handler.

MTYP_CANRCOM equ	.66		;definition for now.

S_PORT 	equ	PORTA	;setup switch  Change as needed
S_BIT	equ	2


LED_PORT 	equ	LATB  	;change as needed
OUTPORT  	equ	LATC	;PORTC. Used for SPI etc.
GREEN		equ		7	;PB7 is the green LED on the PCB
YELLOW		equ		6	;PB6 is the yellow LED on the PCB



OLD_EN_NUM  equ	.32		;old number of allowed events (hangover from earlier module code)
EN_NUM	equ	1		;each event is a two byte DN, one for each reader
EV_NUM  equ 1		;number of allowed EVs per event. 1 . 
Modstat equ 1		;Module state address in EEPROM

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 5
MINOR_VER   equ "A"
BETA_VER	equ .101        ; Set BETA version here, set to 0 for release version
EVT_NUM     equ EN_NUM      ; Number of events
EVperEVT    equ EV_NUM      ; Event variables per event
NV_NUM      equ .2       	; Number of node variables  
NODEFLGS    equ PF_CONSUMER + PF_PRODUCER + PF_BOOT
CPU_TYPE    equ P18F25K80
MODULE_ID	equ MTYP_CANRCOM	;.66			;Till in cbusdefs.

	
	;definitions  Change these to suit hardware.
	



#DEFINE OR0 	PORTC,0		;Loco orientation bit 0. If neither bit 0 or 1 is set, No Railcom
#DEFINE OR1		PORTC,1		;Loco orientation bit 1.
#DEFINE SER_IN	PORTC,7		;RailCom serial data in

#DEFINE RAIL	PORTC,2		;RailCom adapter input

#DEFINE	CH1		PORTC,5		;Channel 1 time for test
#DEFINE TXD		PORTB,2		;CAN transmit pin. 
#DEFINE RXD		PORTB,3		;CAN receive  pin. 

#DEFINE	Md_IDconf	Datmode,7	;for CAN_ID conflict

; definitions used by bootloader

#define	MODE_SELF_VERIFY	;Enable self verification of written data (undefine if not wanted)

#define	HIGH_INT_VECT	0x0808		;HP interrupt vector redirect. Change if target is different
#define	LOW_INT_VECT	0x0818		;LP interrupt vector redirect. Change if target is different.
#define	RESET_VECT	0x0800			;start of target
#define	CAN_CD_BIT	RXB0EIDL,0		;Received control / data select bit
#define	CAN_PG_BIT	RXB0EIDL,1		;Received PUT / GET bit
#define	CANTX_CD_BIT	TXB0EIDL,0	;Transmit control/data select bit
#define	CAN_TXB0SIDH	B'10000000'	;Transmitted ID for target node
#define	CAN_TXB0SIDL	B'00001000'
#define	CAN_TXB0EIDH	B'00000000'	;
#define	CAN_TXB0EIDL	B'00000100'
#define	CAN_RXF0SIDH	B'00000000'	;Receive filter for target node
#define	CAN_RXF0SIDL	B'00001000'
#define	CAN_RXF0EIDH	B'00000000'
#define	CAN_RXF0EIDL	B'00000111'
#define	CAN_RXM0SIDH	B'11111111'	;Receive masks for target node
#define	CAN_RXM0SIDL	B'11101011'
#define	CAN_RXM0EIDH	B'11111111'
#define	CAN_RXM0EIDL	B'11111000'
;#define	CAN_BRGCON1		B'00001111'	;CAN bit rate controls. 16 MHz resonator
#define	CANBIT_RATE		B'00001111'	 ;CAN bit rate while runnung. For 16 MHz clock
#define CANBIT_BL		B'00000011'  ;CAN bit rate for bootloader
#define	CAN_BRGCON2		B'10011110'
#define	CAN_BRGCON3		B'00000011'
#define	CAN_CIOCON		B'00100000'	;CAN I/O control	
;	************************************************************ ** * * * * * * * * * * * * * * *
;	************************************************************ ** * * * * * * * * * * * * * * *
#ifndef	EEADRH		
#define	EEADRH	EEADR+ 1	
#endif			
#define	TRUE	1	
#define	FALSE	0	
#define	WREG1	PRODH	; Alternate working register
#define	WREG2	PRODL	
#define	MODE_WRT_UNLCK	_bootCtlBits, 0	; Unlock write and erase
#define	MODE_ERASE_ONLY	_bootCtlBits, 1	; Erase without write
#define	MODE_AUTO_ERASE	_bootCtlBits, 2	; Enable auto erase before write
#define	MODE_AUTO_INC	_bootCtlBits, 3	; Enable auto inc the address
#define	MODE_ACK		_bootCtlBits, 4	; Acknowledge mode
#define	ERR_VERIFY		_bootErrStat, 0	; Failed to verify if set
#define	CMD_NOP			0x00	
#define	CMD_RESET		0x01	
#define	CMD_RST_CHKSM	0x02	
#define	CMD_CHK_RUN		0x03
#define CMD_BOOT_TEST 	0x04
#define	CANTX			0x02			;port B RB2	


;set config registers. Note. CANRCOM uses Port B for the CAN. 



	CONFIG	FCMEN = OFF, FOSC = HS1, IESO = OFF, PLLCFG = OFF
	CONFIG	PWRTEN = ON, BOREN = SBORDIS, BORV=0, SOSCSEL = DIG
	CONFIG	WDTEN=OFF
	CONFIG	MCLRE = ON, CANMX = PORTB
	CONFIG	BBSIZ = BB1K 
	
	CONFIG	XINST = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
	


;	processor uses  16 MHz. Resonator with HSPLL to give a clock of 64MHz  (V4 onwards)



;This code doesn't use the event handler.



;********************************************************************************
;	RAM addresses used by boot. can also be used by application.

	CBLOCK 0
	_bootCtlMem
	_bootAddrL		; Address info
	_bootAddrH		
	_bootAddrU		
	_unused0		;(Reserved)
	_bootCtlBits	; Boot Mode Control bits
	_bootSpcCmd		; Special boot commands
	_bootChkL		; Chksum low byte fromPC
	_bootChkH		; Chksum hi byte from PC		
	_bootCount		
	_bootChksmL		; 16 bit checksum
	_bootChksmH		
	_bootErrStat	;Error Status flags
	ENDC
	
	; end of bootloader RAM


		CBLOCK 0
	
	Byte1		;bytes from EUSART
	Byte2
	Dat1		;after LUT and rearrange for all 4 bytes
	Dat2
	Dat3
	Dat4
	Adr1		;loco address
	Adr2

	Flags		;for loco address calcs
	Count
	Count1
	Count2
	T1count		;in setup for T1
	T4count		;rolls over every 10 mSec
	Debcnt		;debounce counter
	Loco		;flags for orientation
	Loco1		;last byte if there are 4
	IDbyte		;for RailCom ID in top nybble
	ID_old
	Ch_no		;channel number for RailCom

	DNhi		;device number for reader
	DNlo
	Nvtemp
	NV2			;unoccupied delay
	NV2_tmp
	Datmode		;for CBUS
	IDcount		;used in self allocation of CAN ID.
	Temp		;temp for various
	W_temp		;temp for W REG
	EVflags
	Sflag		;flag for send once
	Bflag		;fot both hi amd lo address bytes

	Tx1con
	Tx1sidh
	Tx1sidl
	Tx1eidh
	Tx1eidl
	Tx1dlc
	Tx1d0		;buffer for CBUS DDES nmessage
	Tx1d1
	Tx1d2
	Tx1d3
	Tx1d4
	Tx1d5
	Tx1d6
	Tx1d7

	Dlc

;	Used for self enum

	Roll		;rolling bit for enum
	In_roll		;rolling bit for input sense


	NN_temph
	NN_templ
	
	Fsr_temp0L
	Fsr_temp0H 
	Fsr_temp1L
	Fsr_temp1H 
	Fsr_temp2L
	Fsr_temp2H
	W_tempL			;in lo pri IRQ
	St_tempL
	PCH_tempL
	PCH_tempH

	IDtemph
	IDtempl
	CanID_tmp
	Keepcnt 
	Latcount

	TempCANCON
	TempCANSTAT
	TempINTCON
	TempECAN
	Fsr_tmp1Le	;temp store for FSR1
	Fsr_tmp1He 
	Enum0		;bits for new enum scheme.
	Enum1
	Enum2
	Enum3
	Enum4
	Enum5
	Enum6
	Enum7
	Enum8
	Enum9
	Enum10
	Enum11
	Enum12
	Enum13

;data area to store data for event learning and event matching

	ev_opc	;incomong op code
	ev0		;event number from learn command and from received event
	ev1
	ev2
	ev3
	ENidx
	EVidx
	EVdata

		ENDC

		CBLOCK 0x100		;bank 1

;	lookup table for 4/8 decoding
	
	Lut		


	
	ENDC	

;program start	





;****************************************************************
;	This is the bootloader
; ***************************************************************************** 
;_STARTUPCODE	0x00
	ORG 0x0000
; *****************************************************************************
	bra	_CANInit
	bra	_StartWrite
; ***************************************************************************** 

	ORG 0x0008
; *****************************************************************************

	goto	HIGH_INT_VECT

; ***************************************************************************** 

	ORG 0x0018
; *****************************************************************************

	goto	LOW_INT_VECT 

; ************************************************************** 
;	Code start
; **************************************************************
	ORG 0x0020
;_CAN_IO_MODULE CODE
; ************************************************************ ** * * * * * * * * * * * * * * * 
; Function: VOID _StartWrite(WREG _eecon_data)
;PreCondition: Nothing
;Input: _eecon_data
;Output: Nothing. Self write timing started.
;Side Effects: EECON1 is corrupted; WREG is corrupted.
;Stack Requirements: 1 level.
;Overview: Unlock and start the write or erase sequence to protected
;	memory. Function will wait until write is finished.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_StartWrite
	movwf 	EECON1
	btfss 	MODE_WRT_UNLCK	; Stop if write locked
	return
	movlw 	0x55	; Unlock
	movwf 	 EECON2 
	movlw	 0xAA 
	movwf 	 EECON2
	bsf	 EECON1, WR	; Start the write
	nop
	btfsc 	EECON1, WR	; Wait (depends on mem type)
	bra	$ - 2
 	return
; ************************************************************ ** * * * * * * * * * * * * * * *

; Function: _bootChksm _UpdateChksum(WREG _bootChksmL)
;
; PreCondition: Nothing
; Input: _bootChksmL
; Output: _bootChksm. This is a static 16 bit value stored in the Access Bank.
; Side Effects: STATUS register is corrupted.
; Stack Requirements: 1 level.
; Overview: This function adds a byte to the current 16 bit checksum
;	count. WREG should contain the byte before being called.
;
;	The _bootChksm value is considered a part of the special
;	register set for bootloading. Thus it is not visible. ;
;*************************************************************** * * * * * * * * * * * *
_UpdateChksum:
	addwf	_bootChksmL,	F ; Keep a checksum
	btfsc	STATUS,	C
	incf	_bootChksmH,	F
	return
;************************************************************ ** * * * * * * * * * * * * * * *
;
;	Function:	VOID _CANInit(CAN,	BOOT)
;
;	PreCondition: Enter only after a reset has occurred.
; Input: CAN control information, bootloader control information ; Output: None.
; Side Effects: N/A. Only run immediately after reset.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	This routine tests the boot flags to determine if boot mode is
;	desired or normal operation is desired. If boot mode then the
;	routine initializes the CAN module defined by user input. It
;	also resets some registers associated to bootloading.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CANInit:

	clrf	EECON1
	setf	EEADR	; Point to last location of EEDATA
	setf	EEADRH
	bsf	EECON1, RD	; Read the control code
	nop
	incfsz EEDATA, W

	goto	RESET_VECT


	clrf	_bootSpcCmd 	; Reset the special command register
	movlw 	0x1C		; Reset the boot control bits
	movwf 	_bootCtlBits 
	movlb	d'14'		; Set Bank 14 for K series
;	bcf 	TRISB, CANTX 	; Set the TX pin to output 
	movlw 	CAN_RXF0SIDH 	; Set filter 0
	movwf 	RXF0SIDH
	movlw 	CAN_RXF0SIDL 
	movwf 	RXF0SIDL
	comf	WREG		; Prevent filter 1 from causing a receive event





	movwf	RXF1SIDL	;		
	movlw	CAN_RXF0EIDH	
	movwf	RXF0EIDH	
	movlw	CAN_RXF0EIDL	
	movwf	RXF0EIDL	
	movlw	CAN_RXM0SIDH	;	Set mask
	movwf	RXM0SIDH	
	movlw	CAN_RXM0SIDL	
	movwf	RXM0SIDL	
	movlw	CAN_RXM0EIDH	
	movwf	RXM0EIDH	
	movlw	CAN_RXM0EIDL	
	movwf	RXM0EIDL

	movlw	CANBIT_BL		;	Set bit rate for bootloader
	movwf	BRGCON1	
	movlw	CAN_BRGCON2	
	movwf	BRGCON2	
	movlw	CAN_BRGCON3	
	movwf	BRGCON3	
	movlb	.15
	clrf	ANCON0
	clrf	ANCON1
;	movlb	0
	movlw	CAN_CIOCON	;	Set IO
	movwf	CIOCON	
	
	clrf	CANCON	; Enter Normal mode

	bcf	TRISB,7
	bcf	TRISB,6
	bsf	LATB,7		;green LED on
	bsf	LATB,6		;yellow LED on


; ************************************************************ ** * * * * * * * * * * * * * * * 
; This routine is essentially a polling loop that waits for a
; receive event from RXB0 of the CAN module. When data is
; received, FSR0 is set to point to the TX or RX buffer depending
; upon whether the request was a 'put' or a 'get'.
; ************************************************************ ** * * * * * * * * * * * * * * * 
_CANMain
	
	bcf	RXB0CON, RXFUL	; Clear the receive flag
_wait	clrwdt			; Clear WDT while waiting		
	btfss 	RXB0CON, RXFUL	; Wait for a message	
	bra	_wait



_CANMainJp1
	lfsr	0, RXB0D0
	movf	RXB0DLC, W 
	andlw 	0x0F
	movwf 	_bootCount 
	movwf 	WREG1
	bz	_CANMain 
_CANMainJp2				;?
	


; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _ReadWriteMemory()
;
; PreCondition:Enter only after _CANMain().
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;This is the memory I/O engine. A total of eight data bytes are received and decoded. In addition two control bits are received, put/get and control/data.
;A pointer to the buffer is passed via FSR0 for reading or writing. 
;The control register set contains a pointer, some control bits and special command registers.
;Control
;<PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT>< SPCMD><CPDTL><CPDTH>
;Data
;<PG>< CD>< DATA0>< DATA1>< DATA2>< DATA3>< DATA4>< DATA5>< DATA6>< DATA7>
;PG bit:	Put = 0, Get = 1
;CD bit:	Control = 0, Data = 1

; ************************************************************ ** * * * * * * * * * * * * * * *
_ReadWriteMemory:
	btfsc	CAN_CD_BIT	; Write/read data or control registers
	bra	_DataReg
; ************************************************************ ** * * * * * * * * * * * * * * * ; This routine reads or writes the bootloader control registers,
; then executes any immediate command received.
_ControlReg
	lfsr	1, _bootAddrL		;_bootCtlMem
_ControlRegLp1

	movff 	POSTINC0, POSTINC1 
	decfsz 	WREG1, F
	bra	_ControlRegLp1

; ********************************************************* 
; This is a no operation command.
	movf	_bootSpcCmd, W		; NOP Command
	bz	_CANMain
;	bz	_SpecialCmdJp2		; or send an acknowledge

; ********************************************************* 
; This is the reset command.
	xorlw 	CMD_RESET		; RESET Command 
	btfss 	STATUS, Z
	bra		_SpecialCmdJp4
	setf	EEADR		; Point to last location of EEDATA
	setf	EEADRH
	clrf	EEDATA		; and clear the data (at 0x3FF for now)
	movlw 	b'00000100'	; Setup for EEData
	rcall 	_StartWrite
	bcf		LED_PORT,6		;yellow LED off
	reset
; *********************************************************
; This is the Selfcheck reset command. This routine 
; resets the internal check registers, i.e. checksum and 
; self verify.
_SpecialCmdJp4
	movf	_bootSpcCmd, W 
	xorlw 	CMD_RST_CHKSM
	bnz		_SpecialCmdJp1
	clrf	_bootChksmH
	clrf	_bootChksmL
	bcf		ERR_VERIFY		
	clrf	_bootErrStat
	bra		_CANMain
; RESET_CHKSM Command
; Reset chksum
; Clear the error verify flag

;This is the Test and Run command. The checksum is
; verified, and the self-write verification bit is checked. 
; If both pass, then the boot flag is cleared.
_SpecialCmdJp1
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_CHK_RUN 
	bnz	_SpecialCmdJp3
	movf	_bootChkL, W	; Add the control byte
	addwf	 _bootChksmL, F
	bnz	_SpecialCmdJp2
	movf	_bootChkH, W 
	addwfc	_bootChksmH, F
	bnz	_SpecialCmdJp2
	btfsc 	ERR_VERIFY		; Look for verify errors
	bra	_SpecialCmdJp2

	bra		_CANSendOK	;send OK message


_SpecialCmdJp2

	bra	_CANSendNOK	; or send an error acknowledge


_SpecialCmdJp3
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_BOOT_TEST 
	bnz	_CANMain
	bra	_CANSendBoot

; ************************************************************** * * * * * * * * * * * * * * * 
; This is a jump routine to branch to the appropriate memory access function.
; The high byte of the 24-bit pointer is used to determine which memory to access. 
; All program memories (including Config and User IDs) are directly mapped. 
; EEDATA is remapped.
_DataReg
; *********************************************************
_SetPointers
	movf	_bootAddrU, W	; Copy upper pointer
	movwf 	TBLPTRU
	andlw 	0xF0	; Filter
	movwf 	WREG2
	movf	_bootAddrH, W	; Copy the high pointer
	movwf 	TBLPTRH
	movwf 	EEADRH
	movf	_bootAddrL, W	; Copy the low pointer
	movwf 	TBLPTRL
	movwf	 EEADR
	btfss 	MODE_AUTO_INC	; Adjust the pointer if auto inc is enabled
	bra	_SetPointersJp1
	movf	_bootCount, W	; add the count to the pointer
	addwf	 _bootAddrL, F 
	clrf	WREG
	addwfc	 _bootAddrH, F 
	addwfc	 _bootAddrU, F 

_SetPointersJp1			;?

_Decode
	movlw 	0x30
	cpfslt 	WREG2
	bra	_DecodeJp1



	bra	_PMEraseWrite

_DecodeJp1
	movf	WREG2,W
	xorlw 	0x30
	bnz	_DecodeJp2



	bra	_CFGWrite 
_DecodeJp2
	movf	WREG2,W 
	xorlw 0xF0
	bnz	_CANMain
	bra	_EEWrite

	

; Program memory < 0x300000
; Config memory = 0x300000
; EEPROM data = 0xF00000
	
; ************************************************************ ** * 
; ************************************************************** * 
; Function: VOID _PMRead()
;	VOID _PMEraseWrite ()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
; the source data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;These are the program memory read/write functions. Erase is available through control flags. An automatic erase option is also available.
; A write lock indicator is in place to ensure intentional write operations.
;Note: write operations must be on 8-byte boundaries and must be 8 bytes long. Also erase operations can only occur on 64-byte boundaries.
; ************************************************************ ** * * * * * * * * * * * * * * *



_PMEraseWrite:
	btfss 	MODE_AUTO_ERASE
	bra	_PMWrite
_PMErase:
	movf	TBLPTRL, W
	andlw	b'00111111'
	bnz	_PMWrite
_PMEraseJp1
	movlw	b'10010100' 
	rcall 	_StartWrite 
_PMWrite:
	btfsc 	MODE_ERASE_ONLY


	bra	_CANMain 

	movf	TBLPTRL, W
	andlw	b'00000111'
	bnz	_CANMain 
	movlw 	0x08
	movwf WREG1

_PMWriteLp1					; Load the holding registers
	movf	POSTINC0, W 
	movwf 	TABLAT
	rcall	 _UpdateChksum 	; Adjust the checksum
	tblwt*+
	decfsz	 WREG1, F
	bra	_PMWriteLp1

#ifdef MODE_SELF_VERIFY 
	movlw	 0x08
	movwf 	WREG1 
_PMWriteLp2
	tblrd*-			; Point back into the block
	movf	POSTDEC0, W 
	decfsz	 WREG1, F
	bra	_PMWriteLp2
	movlw	 b'10000100' 	; Setup writes
	rcall	_StartWrite 	; Write the data
	movlw 	0x08
	movwf 	WREG1
_PMReadBackLp1
	tblrd*+			; Test the data
	movf	TABLAT, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY 
	decfsz 	WREG1, F
	bra	_PMReadBackLp1	; Not finished then repeat
#else
	tblrd*-			; Point back into the block
				 ; Setup writes
	movlw 	b'10000100' 	; Write the data
	rcall 	_StartWrite 	; Return the pointer position
	tblrd*+
#endif

	bra	_CANMain


; ************************************************************** * * * * * * * * * * * * * * *
 ; Function: VOID _CFGWrite()
;	VOID _CFGRead()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of the source data. 
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	These are the Config memory read/write functions. Read is
;	actually the same for standard program memory, so any read
;	request is passed directly to _PMRead.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CFGWrite

#ifdef MODE_SELF_VERIFY		; Write to config area
	movf	INDF0, W		; Load data
#else
	movf	POSTINC0, W
#endif
	movwf 	TABLAT
	rcall 	_UpdateChksum	; Adjust the checksum
	tblwt*			; Write the data
	movlw	b'11000100' 
	rcall 	_StartWrite
	tblrd*+			; Move the pointers and verify
#ifdef MODE_SELF_VERIFY 
	movf	TABLAT, W 
	xorwf 	POSTINC0, W

#endif
	decfsz 	WREG1, F
	bra	_CFGWrite	; Not finished then repeat

	bra	_CANMain 



; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _EERead()
;	VOID _EEWrite()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
 ;	the source data.
; Input:	None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	This is the EEDATA memory read/write functions.
;
; ************************************************************ ** * * * * * * * * * * * * * * *


_EEWrite:

#ifdef MODE_SELF_VERIFY
	movf	INDF0, W
#else
	movf	POSTINC0, W 
#endif

	movwf 	EEDATA
	rcall 	_UpdateChksum 
	movlw	b'00000100' 
	rcall	 _StartWrite

#ifdef MODE_SELF_VERIFY 
	clrf	EECON1
	bsf	EECON1, RD
	nop
	movf	EEDATA, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY
#endif

	infsnz	 EEADR, F 
	incf 	EEADRH, F 
	decfsz 	WREG1, F
	bra	_EEWrite


	bra	_CANMain 
	

; Read the data

; Adjust EEDATA pointer
; Not finished then repeat
; Load data
; Adjust the checksum 
; Setup for EEData
; and write
; Read back the data ; verify the data ; and adjust pointer
; Adjust EEDATA pointer
; Not finished then repeat

; ************************************************************** * * * * * * * * * * * * * * *
; Function: VOID _CANSendAck()
;	VOID _CANSendResponce ()
;
; PreCondition:TXB0 must be preloaded with the data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	These routines are used for 'talking back' to the source. The
;	_CANSendAck routine sends an empty message to indicate
;	acknowledgement of a memory write operation. The
;	_CANSendResponce is used to send data back to the source. ;
; ************************************************************ ** * * * * * * * * * * * * * * *



_CANSendMessage
	btfsc 	TXB0CON,TXREQ 
	bra	$ - 2
	movlw 	CAN_TXB0SIDH 
	movwf 	TXB0SIDH
	movlw 	CAN_TXB0SIDL 
	movwf 	TXB0SIDL
	movlw 	CAN_TXB0EIDH 
	movwf 	TXB0EIDH	

	movlw	CAN_TXB0EIDL
	movwf	TXB0EIDL
	bsf	CANTX_CD_BIT
	btfss	CAN_CD_BIT 
	bcf	CANTX_CD_BIT
	bsf	TXB0CON, TXREQ
    	bra	 _CANMain	; Setup the command bit

_CANSendOK				;send OK message 
	movlw	1			;a 1 is OK
	movwf	TXB0D0
	movwf	TXB0DLC
	bra		_CANSendMessage
	
_CANSendNOK				;send not OK message
	clrf	TXB0D0		;a 0 is not OK
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage

_CANSendBoot
	movlw	2			;2 is confirm boot mode
	movwf	TXB0D0
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage
    
; Start the transmission

; 	End of bootloader

; *****************************************************************************
;
;	start vector

	ORG		0800h
loadadr
		nop						;for debug
		goto	setup			;main setup sequence
	
	ORG 0808h
		goto	hpint			;high priority interrupt

;CBUS module info:

		
		ORG		0810h			;node type parameters

myName	db	"RCOM   "   


		ORG		0818h	
		goto	lpint			;low priority interrupt
		
		org		0820h			;these have to be here for FCU

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
			db	MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
            db  0,0,0,0,CPUM_MICROCHIP,BETA_VER
sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 ; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA_VER

cksum       dw  PRCKSUM     ; Checksum of parameters

;**********************************************************************************

	ORG		0840h			;start of program


setup	call	setsub

		call	setup1

;		specific to CANRCOM

setid	

		clrf	EEADR
		call	newid			;put ID into Tx1buf, TXB2 and ID number store
		btfss	Datmode,2
		bra		seten_f

		bra		main

seten_f	
		bcf		LED_PORT,GREEN		;green off
		btfss	Datmode,2			;flashing?
		
		bsf		LED_PORT,YELLOW		;Yellow LED on.
		bcf		RXB0CON,RXFUL
		bcf		Datmode,0

;		get DN  

		
		movlw	LOW DevNo			;get reader number
		movwf	EEADR
		call	eeread				;get DNhi
		movwf	DNhi
		incf	EEADR				;get DNlo

		call 	eeread
		movwf	DNlo

;		get NVs

		movlw	LOW NVstart			;layout setup routine 
		movwf	EEADR
		call	eeread				;get NV
		movwf	Nvtemp				;need to define NV and what it does
		incf	EEADR
		call	eeread
		movwf	NV2_tmp

		goto	main
	


;	High priotity interrupt. Here if gated pulse longer than rollover.Triggered by PIR1,TMR1IF
;	Ignores too short pulses and sets start of EUSART looking for Channel 1 serial data		


hpint	bcf		PIE1,TMR1IE			;no interrupt
		bcf		T1GCON,TMR1GE		;not gated
		bcf		T1CON,TMR1ON		;stop timer
		
		btfsc	Nvtemp,1			;which channel?
		bra		ch1
		btfsc	Nvtemp,2
		bra		ch2
		bra		hi_loop1			;do nothing
	
ch1		clrf	PIR1				;clear flags
		movlw	0xE4				;time for channel 1 data
		movwf	TMR1H
		movlw	0x34
		movwf	TMR1L
		bsf		LATC,5				;flag up  for scope timing test
		bsf		T1CON,TMR1ON		;restart for read
		
		bsf		RCSTA1,CREN			;start EUSART1
b1_loop btfsc	PIR1,TMR1IF			;time out?
		bra		end_loop
		movf	PORTC,W				;get orientation
		andlw	B'00000011'
		bz		b1_loop1			;both low?
		iorwf	Loco				;save orientation bit.
		movlw	B'00000011'
		subwf	Loco,W
		bnz		b1_loop1
		bcf		Loco,0
		bcf		Loco,1
		

b1_loop1	
		btfss	PIR1,RC1IF			;any serial byte?
		bra		b1_loop				;wait for now
		movf	RCREG1,W			;get serial byte to Byte 1
		movwf	Byte1
		movlw	1
		movwf	Flags
		bcf		PIR1,RC1IF			;clear EUSART flag for next
b1_loop2	btfsc	PIR1,TMR1IF			;time out?
		bra		end_loop
		btfss	PIR1,RC1IF
		bra		b1_loop2				;wait for now
		movf	RCREG1,W
		movwf	Byte2
		movlw	2
		movwf	Flags				;bytes waiting.
		bcf		PIR1,RC1IF


	
	
end_loop btfss	PIR1,TMR1IF			;time out?
		bra		end_loop

		bcf		LATC,5				;flag down. End of Channel 1 time

		bcf		RCSTA1,CREN			;stop EUSART
	
		bcf		T1CON,TMR1ON		;stop it 
		clrf	PIR1

		movlw	0xB6				;reload
		movwf	TMR1H
		movlw	0xA1
		movwf	TMR1L
		bsf		T1CON,TMR1ON		;start it
		movlw	2
		subwf	Flags,W
		bnz		hi_loop				;not got bytes

do_data	movlw	1
		movwf	FSR0H			;sort out 4/8 bytes
		movf	Byte1,W
		movwf	FSR0L
		movf	INDF0,W
		btfsc	WREG,7			;valid 4/8?
		bra		do_2			;abort
		movwf	Dat1			;decoded byte 1
		movf	Byte2,W
		movwf	FSR0L
		movf	INDF0,W
		btfsc	WREG,7
		bra		do_2
		movwf	Dat2
		btfss	Dat1,3			;is low byte  ID = 1
		bra		do_1
		rrncf	Dat1			;move bottom two bytes to 7,6 of second byte
		rrncf	Dat1
		movf	Dat1,W
		andlw	B'00001111'
		movwf	IDbyte			;has ID byte
		movf	Dat1,W
		andlw	B'11000000'		;clear unwanted bits
		iorwf	Dat2,W
		movwf	Adr2			;has low address byte
		bsf		Bflag,0			;flag low address 
do_4	movlw	3
		subwf	Bflag,W			;got both?
		bz		do_3
		
do_2	nop
		clrf	Flags			;done
		bra		hi_loop			;back for second byte
do_1	btfss	Dat1,2			;is it hi byte?  ID= 2	(add more here for a consist)
		bra		do_2			;is an error so abort

		rrncf	Dat1			;move bottom two bytes to 7,6 of second byte
		rrncf	Dat1
		movf	Dat1,W
		andlw	B'00001111'
		movwf	IDbyte			;has ID byte
		movf	Dat1,W
		andlw	B'11000000'		;clear unwanted bits
	
		iorwf	Dat2,W
		movwf	Adr1
		bnz		do_5			;top byte = 0?
		movlw	.128			;is it a long?
		subwf	Adr2,W			;is it more than 127?
		bnn		do_5			;is a long address					
		bra		do_6
do_5	movlw	0xC0			;for NMRA compatibility. Adr1 has hi byte
		iorwf	Adr1

do_6	bsf		Bflag,1			;for hi byte
		clrf	Flags			;done
		bra		do_4
do_3	
		
		btfss	Datmode,3		;is it running mode?
		bra		hi_loop
;		movlw	B'00000010'		;Channel of Railcom
;		movwf	Ch_no
		swapf	IDbyte,F
		call	set_DDES		;set output frame and send
			
do_7;	btfss	Sflag,0			;send once
	;	call	sendTXb			;as sendTXa but wait till sent
		clrf	Flags

		bsf		Sflag,0			;set flag for once only

;		btfsc	Nvtemp,2
;		bra		ch2_1				;do channel 2 as well
;		bra		hi_loop

hi_loop	
		btfss	PIR1,TMR1IF
		bra		hi_loop

		
hi_loop1	bcf		T1CON,TMR1ON		;stop it
		bcf		PIR1,TMR1IF
		

	
hi_loop2		movlw	0xF6				;reset timer for short pulses
		movwf	TMR1H
		clrf	TMR1L
		bcf		LATC,5				;clear anyway	
	
		bsf		T1GCON,TMR1GE		;gated
		bsf		T1GCON,T1GGO		;gate set
		bsf		PIE1,TMR1IE			;re-enable interrupts

		bsf		T1CON,TMR1ON		;start timer
	
		
	
	
	
		retfie	1

ch2_1	bcf		Nvtemp,1		;don't do Ch 1 again



ch2		nop						;as channel 1 but do once
		clrf	PIR1			;clear flags
		movlw	0xE5			;time for channel 2 data delay
		movwf	TMR1H
		movlw	0x00
		movwf	TMR1L
		bsf		T1CON,TMR1ON		;restart for read
ch2_2	btfss	PIR1,TMR1IF		;loop
		bra		ch2_2
		bcf		T1CON,TMR1ON		;stop ot
		clrf	PIR1			;clear flags
		movlw	0xBC			;time for channel 2 data
		movwf	TMR1H
		movlw	0x00
		movwf	TMR1L
		bsf		LATC,5				;flag up  for scope timing test
		bsf		T1CON,TMR1ON		;restart for read
		
		bsf		RCSTA1,CREN			;start EUSART1
b2_loop btfsc	PIR1,TMR1IF			;time out?
		bra		end2_loop
		
		

b2_loop1	
		btfss	PIR1,RC1IF			;any serial byte?
		bra		b2_loop				;wait for now
		movf	RCREG1,W			;get serial byte to Byte 1
		movwf	Byte1
		movlw	1
		movwf	Flags
		bcf		PIR1,RC1IF			;clear EUSART flag for next
b2_loop2	btfsc	PIR1,TMR1IF			;time out?
		bra		end2_loop
		btfss	PIR1,RC1IF
		bra		b2_loop2				;wait for now
		movf	RCREG1,W
		movwf	Byte2
		movlw	2
		movwf	Flags				;bytes waiting.
		bcf		PIR1,RC1IF


	
	
end2_loop btfss	PIR1,TMR1IF			;time out?
		bra		end2_loop

		bcf		LATC,5				;flag down. End of Channel 2 time
		bcf		RCSTA1,CREN			;stop EUSART
	bcf		T1CON,TMR1ON		;stop it 
		clrf	PIR1

	
		movlw	2
		subwf	Flags,W
		bnz		hi_loop2				;not got bytes



do2_data	movlw	1
		movwf	FSR0H			;sort out 4/8 bytes
		movf	Byte1,W
		movwf	FSR0L
		movf	INDF0,W
		btfsc	WREG,7			;valid 4/8?
		bra		do2_2			;abort
		movwf	Dat1			;decoded byte 1
		movf	Byte2,W
		movwf	FSR0L
		movf	INDF0,W
		btfsc	WREG,7
		bra		do2_2
		movwf	Dat2
	
	
		rrncf	Dat1			;move bottom two bits to 7,6 of second byte	
		rrncf	Dat1
		movf	Dat1,W
		andlw	B'00001111'
		movwf	IDbyte			;has ID byte
		movf	Dat1,W
		andlw	B'11000000'		;clear unwanted bits
		iorwf	Dat2,W
		movwf	Adr1			;has data byte
		
do2_1	


	
		
		btfss	Datmode,3		;is it running mode?
		bra		hi_loop2		;no
		movlw	B'00000100'		;Channel of Railcom
		swapf	IDbyte,F
		call	set_DDES		;set output frame and send
	
;		movf	IDbyte,W		;same as before?
;		subwf	ID_old,W
;		bz		do2_1a
;		bcf		Sflag,0
;do2_1a	btfss	Sflag,0			;send once
	call	sendTXb			;as sendTXa but wait till sent
		bsf		Sflag,0			;set flag for once only	
do2_1b		movff	IDbyte,ID_old
		
		
		
		


		
do2_2	nop
		clrf	Flags			;done
		bra		hi_loop2		;back 	
		



;************************************************************

;**************************************************************
;
;		low priority interrupt. Used for CAN transmit error / latency and 'busy' frame.
;		Busy frame is a max priority, zero data frame, preloaded in TXB0.
;		Latency count (number of tries to transmit) is preset in code to .10
	
				ORG 0B00h

lpint	movwf	W_tempL					;save critical variables
		movff	STATUS,St_tempL
		movff	CANCON,TempCANCON
		movff	CANSTAT,TempCANSTAT
	
		movff	FSR0L,Fsr_temp0L		;save FSR0
		movff	FSR0H,Fsr_temp0H
		movff	FSR1L,Fsr_temp1L		;save FSR1
		movff	FSR1H,Fsr_temp1H

		btfsc	PIR5,ERRIF
		bra		txerr				;transmit error?
		btfss	PIR5,FIFOWMIF		;FIFO error?
		bra		no_fifo
		bcf		PIR5,FIFOWMIF		;clear FIFO flag
		bcf		PIR5,TXB0IF			;clear busy frame flag
		movlb	.15
		bsf		TXB0CON,TXREQ		;send busy frame
		bcf		PIE5,FIFOWMIE		;disable FIFO interrupt 
		bsf		PIE5,TXB0IE			;enable IRQ for busy frame sent
		movlb	0
		bra		back1	
	
no_fifo	bcf		PIR5,TXB0IF			;clear busy frame flag
		bcf		PIE5,TXB0IE			;no busy frame IRQ
		bsf		PIE5,FIFOWMIE		;wait for next FIFO IRQ
		bra		back1


		
		;Transmit error routine here. Only acts on lost arbitration	

txerr	movlb	.15					;change bank			
		btfss	TXB1CON,TXLARB
		bra		errbak				;not lost arb.
	
		movf	Latcount,F			;is it already at zero?
		bz		errbak
		decfsz	Latcount,F
		bra		errbak
		bcf		TXB1CON,TXREQ
		movlw	B'00111111'
		andwf	TXB1SIDH,F			;change priority
txagain bsf		TXB1CON,TXREQ		;try again
					
errbak	bcf		RXB1CON,RXFUL
		movlb	0
		bcf		RXB0CON,RXFUL		;ready for next
		
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL		
		bra		back1


		
back	bcf		RXB0CON,RXFUL		;ready for next
	
	
back1	clrf	PIR5				;clear all flags
		movf	CANCON,W			;recover variables
		andlw	B'11110001'
		iorwf	TempCANCON,W
		
		movwf	CANCON
		movff	PCH_tempH,PCLATH
		movff	Fsr_temp0L,FSR0L		;recover FSR0
		movff	Fsr_temp0H,FSR0H

		movff	Fsr_temp1L,FSR1L		;recover FSR1
		movff	Fsr_temp1H,FSR1H
		movf	W_tempL,W
		movff	St_tempL,STATUS	
		
		retfie	
	
;******************************************************************

;	main program loop

main	btfsc	COMSTAT,7			;any CAN frame?
		call 	getcan
		btfsc	Datmode,0
		call	packet

				;do the CAN message.

		bcf		Datmode,0
		bcf		RXB0CON,RXFUL		;clear CAN RX buffer for next
		btfsc	S_PORT,S_BIT		;PB in?
		bra		main1
		call	set1				;yes so cancel
		sublw	0
		bz		main1
		call	setup1				;out of main and back to SLiM
		bra		main
		
main1	
	
		btfsc	T1GCON,T1GGO		;has got a pulse?
		bra		main2
		bcf		T1CON,TMR1ON		;stop timer
		
		call	occ_snd				;send an occupied frame
		clrf	T4count	
		movf	NV2_tmp,W
		movwf	NV2		

		bra		main3
main2	btfss	PIR4,TMR4IF
		bra		main
		bcf		PIR4,TMR4IF
		btg		LATC,3				;timer flag
		decfsz	T4count
		bra		main
		decfsz	NV2					;unocc  delay in NV2
		bra		main
		btfss	Sflag,2				;has sent?
		call	un_occ				;send unoccupied
		bra		main3
		
	
	

		;reset timer for short pulses

main3	
		bcf		T1CON,TMR1ON		;stop timer
		bcf		PIR1,TMR1IF			;clear flag
		movlw	0xF6				;timer 1 set
		movwf	TMR1H
		clrf	TMR1L
				
	
		movlw	B'11011000'
		movwf	T1GCON				;set timer 1 for gated mode
		bsf		PIE1,TMR1IE			;enable interrupt
		bsf		T1CON,TMR1ON		;restart timer

		bra		main

;		***********************************************

;
;		start of subroutines

;******************************************************************

;read a EEPROM byte, EEADR  must be set before this sub.
;		
eeread	bcf		EECON1,EEPGD	
		bcf		EECON1,CFGS		
		bsf		EECON1,RD
		nop	
		nop					;needed for K series PICs
		movf	EEDATA,W		;returns with data in W
		return

;*****************************************************************



;	write to EEPROM, EEADR must be set before this sub.
;	data to write in W

eewrite	movwf	EEDATA		
		bcf		EECON1,EEPGD	
		bcf		EECON1,CFGS
		bsf		EECON1,WREN
		movff	INTCON,TempINTCON
		
		clrf	INTCON	;disable interrupts
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf		EECON1,WR

eetest	btfsc	EECON1,WR				;check it has written
		bra		eetest
		bcf		PIR2,EEIF
		bcf		EECON1,WREN
		movff	TempINTCON,INTCON		;reenable interrupts
		
		return	

getcan	movf	CANCON,W			;look for a CAN frame. Used in main outine
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W			;uses the ECAN in mode 2
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can
		btfss	RXB0DLC,RXRTR		;is it RTR?
		bra		get_3
		call	isRTR
		bra		no_can
		

	
get_3	movf	RXB0DLC,F
		bnz		get_2			;ignore zero length frames
		bra		no_can 
get_2	bsf		Datmode,0		;valid message frame. Datmode bit 0 indicates a CAN frame waiting.
		call	copyev			;save to buffer. Saves OpCode and 4 bytes.	
		return

no_can	bcf		RXB0CON,RXFUL	;no valid CAN frame so clear for next.
		return

;***********************************************************************

;********************************************************************
								;main packet handling is here
								;add more commands for incoming frames as needed
								;test OPC and branch if match to 'opc_x'
								;at opc_x, goto opc routine which is a sub and returns to 
								;code that called 'packet'.
		
packet
		movlw	OPC_ASRQ		;request state of block
		subwf	ev_opc,W
		bz		asrq_x
	
;		now all other OPCs
		
		movlw	OPC_BOOT		;reboot
		subwf	ev_opc,W
		bz		boot_x
		movlw	OPC_ENUM		;re-enumerate
		subwf	ev_opc,W
		bz		enum_x
		movlw	OPC_RQNPN
		subwf	ev_opc,W
		bz		rqnpn_x			;read individual parameters

		movlw	OPC_CANID		;force new CAN_ID
		subwf	ev_opc,W
		bz		canid_x
		movlw	OPC_SNN			;set NN 
		subwf	ev_opc,W
		bz		snn_x
		movlw	OPC_QNN			; QNN
		subwf	ev_opc,w
		bz		qnn_x
		movlw	OPC_RQNP					
		subwf	ev_opc,W
		bz		rqnp_x			;read node parameters
		movlw	OPC_RQMN		
		subwf	ev_opc,w
		bz		rqmn_x			;read module name		
		movlw	OPC_NNLRN		;set to learn mode 
		subwf	ev_opc,W
		bz		nnlrn_x		
		movlw	OPC_NNULN		;clear learn mode 
		subwf	ev_opc,W
		bz		nnuln_x
		movlw	OPC_NNCLR		;clear all events on 0x55
		subwf	ev_opc,W
		bz		nnclr_x
		movlw	OPC_NNEVN		;read number of events left
		subwf	ev_opc,W
		bz		nnevn_x
		movlw	OPC_EVLRN		;is it set event?
		subwf	ev_opc,W
		bz		evlrn_x			;do learn
		movlw	OPC_REVAL
		subwf	ev_opc,W
		bz		reval_x
		movlw	OPC_EVULN		;is it unset event
		subwf	ev_opc,W			
		bz		evuln_x
		movlw	OPC_REQEV		;read event variables
		subwf	ev_opc,W
		bz		reqev_x
		movlw	OPC_NVSET		;set NV
		subwf	ev_opc,W
		bz		nvset_x
		movlw	OPC_NVRD		;read NVs
		subwf	ev_opc,W
		bz		nvrd_x
	
		movlw	OPC_NERD		;is it read events
		subwf	ev_opc,W
		bz		nerd_x

		movlw	OPC_RQEVN
		subwf	ev_opc,W
		bz		rqevn_x

		movlw	OPC_RQDDS		;request DDES data
		subwf	ev_opc,W
		bz		rqdds_x

		return					;no match of OPC

		; Too long for a branch so use gotos

;go_on_x goto	go_on
asrq_x	goto 	asrq
snn_x	goto	snn
boot_x	goto 	boot
enum_x	goto 	enum

rqnpn_x goto	rqnpn
canid_x goto	canid
rqnp_x	goto	rqnp
rqmn_x	goto	rqmn
nnlrn_x goto	nnlrn
nnuln_x	goto	nnuln
evlrn_x	goto	evlrn

reval_x	goto	reval
rqevn_x	goto	rqevn
nnclr_x	goto	nnclr
nvset_x	goto 	nvset
nvrd_x	goto	nvrd
nnevn_x	goto	nnevn
rqdds_x	goto	rqdds


qnn_x	goto	qnn


evuln_x	goto	evuln
reqev_x	goto	reqev


nerd_x	goto	nerd


;**************************************************************
		

;		Read LUT from EEPROM to ram

read_LUT	movlw	0			;number in LUT
		movwf	Count
		lfsr	FSR0,Lut		;LUT in ram
		clrf	EEADRH			;after boot test
		bsf		EEADRH,0		;LUT is in page 1 of EEPROM
		clrf	EEADR
		decf	EEADR			;one less to allow for the inc.
lut_loop	incf	EEADR	
		call	eeread
		movwf	POSTINC0
		decfsz	Count			;end?
		bra		lut_loop
		clrf	EEADRH			;back to page 0
		return

;***************************************************************

set_DDES movf	DNhi,F			;is DN = 0
		 bnz	set_DDES1
		movf	DNlo,F
		bnz		set_DDES1
		retlw	1						;invalid DN
	
set_DDES1	
		btfsc	Sflag,0
		return
		movlw	OPC_DDES		;set up CBUS frame to send
		movwf	Tx1d0
		movff	DNhi,Tx1d1			;needs DN here
		movff	DNlo,Tx1d2
		movff	IDbyte,Tx1d3		;use channel number for now
		movff	Adr1,Tx1d4		;loco address hi byte
		movff	Adr2,Tx1d5		;loco address lo byte
		movff	Loco,Tx1d6
		movff	Loco1,Tx1d7
		movlw	.8
		movwf	Dlc
		btfss	Sflag,0
		call	sendTXb
		bsf		Sflag,0
		clrf	Flags
		
		return

;******************************************************************

;		Utility routines used by various subroutines

;		Part of event learning.

learn2	;btfss	Mode,1			;FLiM?
		btfss	Datmode,3
		return	
		btfsc	Datmode,5
		bra		unlearn				;
		movf	RXB0D6,W		;get which reader
		decf	WREG			;starts at 0. Needs check here?
		rlncf	WREG			;two bytes per reader
		movwf	EEADR			;set to write
		movlw	LOW DevNo
		addwf	EEADR
		movf	RXB0D3,W		;upper byte of DN
		movwf	DNhi			;save it
		call	eewrite			;put in
		incf	EEADR
		movf	RXB0D4,W		;next byte
		movwf	DNlo
		call	eewrite
		

l_out1	bcf		Datmode,6
l_out2	bcf		RXB0CON,RXFUL
		movlw	OPC_WRACK
		call	nnrel		;send WRACK
		bcf		Datmode,0
		return



;**********************************************************************

;		Unlearn an event  (there is only one)

unlearn movlw	LOW DevNo
		movwf	EEADR
		movlw	0
		movwf	DNhi			;save it
		call	eewrite			;put in
		incf	EEADR
		movlw	0		;next byte
		movwf	DNlo
		call	eewrite
		bcf		Datmode,5   ; clear unlearn
		bra		l_out2		;back


;		Set node to learn mode


setlrn	call	thisNN			;set to learn mode
		sublw	0
		bnz		setl_1
		bsf		Datmode,4
setl_1	return					;not this NN

;**************************************************************************


;		check if command is for this node

thisNN	movf	NN_temph,W
		subwf	RXB0D1,W
		bnz		not_NN
		movf	NN_templ,W
		subwf	RXB0D2,W
		bnz		not_NN
		retlw 	0			;returns 0 if match
not_NN	retlw	1
							


		
;**************************************************************************

		

;		request frame for new NN or ack if not virgin
		
nnreq	movlw	OPC_RQNN		
nnrel	movwf	Tx1d0
		movff	NN_temph,Tx1d1
		movff	NN_templ,Tx1d2
		movlw	3
		movwf	Dlc
		call	sendTX
		return

;*************************************************************

;		Send contents of Tx1 buffer via CAN TXB1

sendTX1	movff	FSR1L,Fsr_temp1L		;save FSRs
		movff	FSR1H,Fsr_temp1H
		movff	FSR0L,Fsr_temp0L
		movff	FSR0H,Fsr_temp0H


		lfsr	FSR0,Tx1con			;shift buffer to TXB1 registers
		lfsr	FSR1,TXB1CON
		
		movlb	.15				;check for buffer access
ldTX2	btfsc	TXB1CON,TXREQ	; Tx buffer available...?
		bra		ldTX2			;... not yet  so loop
		movlb	0
		
ldTX1	movf	POSTINC0,W
		movwf	POSTINC1		;load TXB1 registers
		movlw	Tx1d7+1
		cpfseq	FSR0L
		bra		ldTX1

		
		movlb	.15				;bank 15
tx1test	btfsc	TXB1CON,TXREQ	;test if clear to send
		bra		tx1test
		bsf		TXB1CON,TXREQ	;OK so send
		
tx1done	movlb	0				;bank 0
		movff	Fsr_temp1L,FSR1L		;recover FSRs
		movff	Fsr_temp1H,FSR1H
		movff	Fsr_temp0L,FSR0L
		movff	Fsr_temp0H,FSR0H
		return					;successful send
;*******************************************************************

;		as sendTX1 bur wait till sent before returning. 

sendTX2	movff	FSR1L,Fsr_temp1L		;save FSRs
		movff	FSR1H,Fsr_temp1H
		movff	FSR0L,Fsr_temp0L
		movff	FSR0H,Fsr_temp0H


		lfsr	FSR0,Tx1con			;shift buffer to TXB1 registers
		lfsr	FSR1,TXB1CON
		
		movlb	.15				;check for buffer access
ldTX2a	btfsc	TXB1CON,TXREQ	; Tx buffer available...?
		bra		ldTX2a			;... not yet  so loop
		movlb	0
		
ldTX2b	movf	POSTINC0,W
		movwf	POSTINC1		;load TXB1 registers
		movlw	Tx1d7+1
		cpfseq	FSR0L
		bra		ldTX2b

		
		movlb	.15				;bank 15
tx2test	btfsc	TXB1CON,TXREQ	;test if clear to send
		bra		tx2test
		bsf		TXB1CON,TXREQ	;OK so send

tx2c	btfss	TXB1CON,TXBIF	;has it sent?
		bra		tx2c			;no so wait
		bcf		TXB1CON,TXBIF
		
tx2done	movlb	0				;bank 0
		movff	Fsr_temp1L,FSR1L		;recover FSRs
		movff	Fsr_temp1H,FSR1H
		movff	Fsr_temp0L,FSR0L
		movff	Fsr_temp0H,FSR0H
		return					;successful send


;*********************************************************************
;		send a CAN frame
;		entry at sendTX puts the current NN in the frame - for producer events
;		entry at sendTXa needs Tx1d1 and Tx1d2 setting first
;		Latcount is the number of CAN send retries before priority is increased
;		the CAN_ID is pre-loaded in the Tx1 buffer 
;		Dlc must be loaded by calling source to the data length value
		
sendTX	movff	NN_temph,Tx1d1		;get NN
		movff	NN_templ,Tx1d2

sendTXa	movf	Dlc,W				;get data length
		movwf	Tx1dlc
		movlw	B'00001111'		;clear old priority
		andwf	Tx1sidh,F
		movlw	B'10110000'
		iorwf	Tx1sidh			;low priority
		movlw	.10
		movwf	Latcount
		call	sendTX1			;send frame
		return			


sendTXb	movf	Dlc,W				;get data length
		movwf	Tx1dlc
		movlw	B'00001111'		;clear old priority
		andwf	Tx1sidh,F
		movlw	B'10110000'
		iorwf	Tx1sidh			;low priority
		movlw	.10
		movwf	Latcount
		call	sendTX2			;send frame and wait
		return			


		
;*********************************************************************
;		put in NN from command. 

putNN	movff	RXB0D1,NN_temph	;get new Node Number
		movff	RXB0D2,NN_templ
		movlw	LOW NodeID		;put in EEPROM
		movwf	EEADR
		movf	RXB0D1,W
		call	eewrite
		incf	EEADR
		movf	RXB0D2,W
		call	eewrite
		movlw	Modstat			;set module status to running mode and store it.
		movwf	EEADR
		movlw	B'00001000'		;Module status has NN set  (bit 3)
		movwf	Datmode			;running mode
		call	eewrite			;save mode

		
		return

;***************************************************************************	

newid	movlw	LOW CANid		;put new CAN_ID etc in EEPROM
		movwf	EEADR
		call	eeread
		movwf	CanID_tmp			
		call	shuffle			;rearrange bits so there is a single CAN_ID byte
		movlw	B'11110000'
		andwf	Tx1sidh
		movf	IDtemph,W		;set current ID into CAN buffer
		iorwf	Tx1sidh			;leave priority bits alone
		movf	IDtempl,W
		movwf	Tx1sidl			;only top three bits used
		movlw 	Modstat
		movwf	EEADR
		call	eeread
		movwf	Datmode
		movlw	LOW DevNo
		movwf	EEADR
		call	eeread
		movwf	DNhi
		incf	EEADR
		call	eeread
		movwf	DNlo
		movlw	LOW NodeID
		movwf	EEADR
		call	eeread
		movwf	NN_temph		;get stored NN
		incf	EEADR
		call	eeread
		movwf	NN_templ
		movlw	LOW NVstart
		movwf	EEADR
		call	eeread
		movwf	Nvtemp
	
		
		movlb	.15				;put CAN_ID into TXB2 for enumeration response to RTR
new_1	btfsc	TXB2CON,TXREQ
		bra		new_1
		clrf	TXB2SIDH
		movf	IDtemph,W
		movwf	TXB2SIDH
		movf	IDtempl,W
		movwf	TXB2SIDL
		movlw	0xB0
		iorwf	TXB2SIDH		;set priority
		clrf	TXB2DLC			;no data, no RTR


		return

;*********************************************************************	

;
;		shuffle for standard ID. Puts 7 bit ID into IDtemph and IDtempl for CAN frame

shuffle	movff	CanID_tmp,IDtempl		;get 7 bit ID
		swapf	IDtempl,F
		rlncf	IDtempl,W
		andlw	B'11100000'
		movwf	IDtempl					;has sidl
		movff	CanID_tmp,IDtemph
		rrncf	IDtemph,F
		rrncf	IDtemph,F
		rrncf	IDtemph,W
		andlw	B'00001111'
		movwf	IDtemph					;has sidh
		return

;*********************************************************************************

;		reverse shuffle for incoming ID. sidh and sidl into one byte.

shuffin	movff	RXB0SIDL,IDtempl
		swapf	IDtempl,F
		rrncf	IDtempl,W
		andlw	B'00000111'
		movwf	IDtempl
		movff	RXB0SIDH,IDtemph
		rlncf	IDtemph,F
		rlncf	IDtemph,F
		rlncf	IDtemph,W
		andlw	B'01111000'
		iorwf	IDtempl,W			;returns with ID in W
		return
;************************************************************************************

;		self enumeration as separate subroutine

self_en	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
		movff	FSR1H,Fsr_tmp1He 
		bsf		Datmode,1		;set to 'setup' mode
		movlw	.14
		movwf	Count
		lfsr	FSR0, Enum0
clr_en
		clrf	POSTINC0
		decfsz	Count
		bra		clr_en

		movlw	0x00			;set T0 to 128 mSec (may need more?)
		movwf	TMR0H			;this waits till all other nodes have answered with their CAN_ID
		movlw	0x00
		movwf	TMR0L
		movlw	B'00000100'		;clock div  32 (0.5 uSec clock)									
		movwf	T0CON			;enable timer 0
		bsf		T0CON,TMR0ON
		bcf		INTCON,TMR0IF
		
								;now send an RTR frame so all other nodes will send their CAN_IDs
		movlb	.15
		movlw	B'10111111'		;fixed node, default ID  
		movwf	TXB1SIDH
		movlw	B'11100000'
		movwf	TXB1SIDL
		movlw	B'01000000'		;RTR frame
		movwf	TXB1DLC
rtr_snd	btfsc	TXB1CON,TXREQ
		bra		rtr_snd
		bsf		TXB1CON,TXREQ
rtr_go	btfsc	TXB1CON,TXREQ		;wait till sent
		bra		rtr_go
		clrf	TXB1DLC				;no more RTR frames
		movlb	0
				
;	wait for answers	

self_en1	btfsc	INTCON,TMR0IF		;setup timer out?
		bra		en_done
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan1
		bra		self_en1		;no CAN
	

getcan1	movf	CANCON,W		;process answers
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can1
		
		
en_1	btfss	Datmode,1			;setup mode?
		bra		no_can1				;must be in setup mode
		movf	RXB0DLC,F
		bnz		no_can1				;only zero length frames
		call	setmode
		bra		no_can1	

no_can1	bcf		RXB0CON,RXFUL
		bra		self_en1			;loop till timer out 

en_done	bcf		T0CON,TMR0ON		;timer off
		bcf		INTCON,TMR0IF		;clear flag

; now sort out the new CAN_ID

		clrf	IDcount
		incf	IDcount,F			;ID starts at 1
		clrf	Roll
		bsf		Roll,0
		lfsr	FSR1,Enum0			;set FSR to start
here1	incf	INDF1,W				;find a space
		bnz		here
		movlw	8
		addwf	IDcount,F
		incf	FSR1L
		bra		here1
here	movf	Roll,W
		andwf	INDF1,W
		bz		here2
		rlcf	Roll,F
		incf	IDcount,F
		bra		here
here2	movlw	.100				;limit to ID
		cpfslt	IDcount
		bra		segful				;segment full
		
here3	movlw	LOW CANid		;put new ID in EEPROM
		movwf	EEADR
		movf	IDcount,W
		call	eewrite
		movf	IDcount,W
		call	newid			;put new ID in various buffers

			
		movff	Fsr_tmp1Le,FSR1L	;
		movff	Fsr_tmp1He,FSR1H 
		return					;finished	

segful	movlw	7		;segment full, no CAN_ID allocated
		call	errsub	;error
		setf	IDcount
		bcf		IDcount,7
		bra		here3

;********************************************************


isRTR	btfsc	Datmode,1		;setup mode?
		return					;back

		btfss	Datmode,3		;FLiM?
		return
		movlb	.15
isRTR1	btfsc	TXB2CON,TXREQ	
		bra		isRTR1		
		bsf		TXB2CON,TXREQ	;send ID frame - preloaded in TXB2

		movlb	0
		return

;****************************************************************

;
setmode	tstfsz	RXB0DLC
		return				;only zero length frames for setup
		
		swapf	RXB0SIDH,W			;get ID into one byte
		rrcf	WREG
		andlw	B'01111000'			;mask
		movwf	Temp
		swapf	RXB0SIDL,W
		rrncf	WREG
		andlw	B'00000111'
		iorwf	Temp,W
		movwf	IDcount				;has current incoming CAN_ID

		lfsr	FSR1,Enum0			;set enum to table
enum_st	clrf	Roll				;start of enum sequence
		bsf		Roll,0
		movlw	8
enum_1	cpfsgt	IDcount
		bra		enum_2
		subwf	IDcount,F			;subtract 8
		incf	FSR1L				;next table byte
		bra		enum_1
enum_2	dcfsnz	IDcount,F
		bra		enum_3
		rlncf	Roll,F
		bra		enum_2
enum_3	movf	Roll,W
		iorwf	INDF1,F
		bcf		RXB0CON,RXFUL		;clear read


		return

;****************************************************************

; force self enumeration

enum	call	thisNN
		sublw	0
		bnz		notNN1			;not there
		call	self_en			;do self enum
		movlw	OPC_NNACK
		call	nnrel			;send confirm frame
		bcf		RXB0CON,RXFUL
		movlw	B'00001000'		;back to normal running
		movwf	Datmode
notNN1	return

;**********************************************************************

snn		btfss	Datmode,2		;in NN set mode?
		return					;no
		call	putNN			;put in NN
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'
		call	eewrite			;set to normal status
		bcf		Datmode,1		;out of setup
		bcf		Datmode,2
		bsf		Datmode,3		;run mode

	
		movlw	.10
		movwf	Keepcnt			;for keep alive
		movlw	OPC_NNACK
		call	nnrel 			;confirm NN set
startNN	bsf		LED_PORT,YELLOW	;LED ON
		bcf		LED_PORT,GREEN	;LED off
		return

;******************************************************************	

;		Start bootloader

boot	
		call	thisNN
		sublw	0
		bnz		retboot			;not there
		
reboot1	movlw	0xFF
		movwf	EEADR
		movlw	0x3F
		movwf	EEADRH
		movlw	0xFF
		call	eewrite			;set last EEPROM byte to 0xFF
		reset					;software reset to bootloader
								;should clear return stack

retboot	return	

;****************************************************************

rqnpn		
		call	thisNN			;read parameter by index
		sublw	0
		bnz		npnret
		call	para1rd
npnret	return

;*************************************************************

;		send module name - 7 bytes

rqmn	btfss	Datmode,2		;setup mode only
		return
		movlw	OPC_NAME
		movwf	Tx1d0
		movlw	LOW myName
		movwf	TBLPTRL
		movlw	HIGH myName
		movwf	TBLPTRH			;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
name1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		name1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa
		return
		

;**********************************************************

;		Set a CAN_ID

canid	call	thisNN
		sublw	0
		bnz		canret				;abort
		movff	RXB0D3,IDcount
		call	here2				;put in as if it was enumerated
		movlw	OPC_NNACK
		call	nnrel				;acknowledge new CAN_ID
canret	return


;*********************************************************************

rqnp	btfss	Datmode,2		;only in setup mode
		return
		call	parasend
		return

;*******************************************************************

;		Learn a NN

nnlrn	call	thisNN
		sublw	0
		bnz		nnlret			;abort
		bsf		Datmode,4

nnlret	return

;************************************************************************

;		Unlearn a NN

nnuln	call	thisNN
		sublw	0
		bnz		notret
		bcf		Datmode,4
		bcf		Datmode,7		;OK to scan
		
notln1									;leave in learn mode
		bcf		Datmode,5
	
notret	return
	
;***********************************************************************

;		Clear a node.  Must be in learn mode for safety

nnclr	call	thisNN
		sublw	0
		bnz		clrret
		btfss	Datmode,4
		bra		clrerr
		call	clr_sub
		
		movlw	OPC_WRACK
		call	nnrel		;send WRACK
		bra		notln1
clrret	return

clrerr	movlw	2			;not in learn mode
		call	errmsg
		return

;**********************************************************************

;		Set a NV

nvset	call	thisNN
		sublw	0
		bnz		notnv			;not this node
		call	putNV
		movlw	OPC_WRACK
		call	nnrel		;send WRACK
notnv	return

;*********************************************************************



;		Learn an event

evlrn	btfss	Datmode,4		;is in learn mode?
		return					;return if not
		movf	EVidx,w			;check EV index
		bz		noEV1
		movlw	EV_NUM+1
		cpfslt	EVidx
		bra		noEV1
		bra		learn2


noEV1	movlw	6
		call	errmsg
		return

;***********************************************************

;		Unlearn an event

evuln	
		btfss	Datmode,4
		return				;prevent error message
		bsf		Datmode,5
		bra		learn2	


;**************************************************************

;	error message send

errmsg	call	errsub
		bra		errret
errmsg1	call	errsub
		bcf		Datmode,6
		bra		errret
errmsg2	call	errsub
		
errret	clrf	PCLATH
		return		

errsub	movwf	Tx1d3		;main error message send. Error no. in WREG
		movlw	OPC_CMDERR
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
		return

;**************************************************************

rqevn	call	thisNN				;read event numbers
		sublw	0
		bnz		rqevret
		call	rqevn1				;send event number
rqevret return

;**************************************************************

;			Sends number of events. 

rqevn1		movlw	1			;always 1 event
			movwf	Tx1d3
			movlw	OPC_NUMEV		;number of events
			movwf	Tx1d0
			movlw	4
			movwf	Dlc
			call	sendTX	
			return

;*******************************************

evsend					;called from REVAL. Sends EV by index

		movlw	OPC_NEVAL		;0xB5
		movwf	Tx1d0
		movff	ENidx,Tx1d3		;event index (1 to 4)
		movff	EVidx,Tx1d4		;event variable index. Always 1
		movff	ENidx,Tx1d5		;EV value is always the event index
		movlw	6
		movwf	Dlc
		call	sendTX
		return

notEV	movlw	6		;invalid EN#
		call	errsub
		return

;***********************************************************

;
;			clears reader DN and restores defaults

clr_sub		movlw	LOW DevNo
			movwf	EEADR
			movlw	1				;2 bytes
			movwf	Count			;1st byte
clrsub1		movlw	0
			call	eewrite			
			incf	EEADR,F
			decfsz	Count
			bra		clrsub1			;loop till done
			movlw	LOW DevNo			;restore defaults
			movf	EEADR
			incf	EEADR			;two byte values
			movlw	1				;default
			call	eewrite
			
		
			return

;*********************************************************


;		Request an EV

reqev	btfss	Datmode,4
		return					;prevent error message
		movf	EVidx,w			;check EV index
		bz		rdev1
		movlw	EV_NUM+1
		cpfslt	EVidx

rdev1	bra		noEV1
		bsf		Datmode,6
		bra		learn2

;********************************************

;	query NN

qnn		movf	NN_temph,W		;respond if NN is not zero
		addwf	NN_templ,W
		btfss	STATUS,Z
		call	whoami
		return	

;***********************************************************************
			
reval	call	thisNN
		sublw	0
		bnz		revret			;abort
		movff	RXB0D3, ENidx
		movff	RXB0D4, EVidx
		call	evsend
revret	return




;******************************************************



;	not really needed

nnevn	call	thisNN
		sublw	0
		bnz		nnnret

nnnret	return

;********************************************************************

nerd	call	thisNN
		sublw	0
		bnz		nrdret			;abort
		call	readen
nrdret	return

;**************************************************************************

;			Read back the stored event (reader number)

readen	
			movlw	OPC_ENRSP		;ENRSP
			movwf	Tx1d0
			movlw	0
			movwf	Tx1d3
			movwf	Tx1d4
			movlw	1
			movwf	Tx1d7		;to atart
			movlw	8
			movwf	Dlc
			movlw	LOW DevNo
			movwf	EEADR
rden1		call	eeread
			movwf	Tx1d5
			incf	EEADR
			call	eeread
			movwf	Tx1d6
			call	sendTX		;send event 1

			return


;******************************************


;	Request for read of node variable	

nvrd		call 	thisNN		;is it here?
			sublw	0
			bnz		nvrd1		;no
			movlw	LOW	NVstart
			addwf	ev2,W		;add index
			movwf	EEADR
			decf	EEADR,F		;index starts at 1, buffer at 0
			call	eeread
			movwf	Tx1d4		;NV val to transmit buffer
			movff	ev2,Tx1d3	;transfer index
			movlw	OPC_NVANS	;NVANS
			movwf	Tx1d0
			movlw	5
			movwf	Dlc
			call	sendTX		;send answer
nvrd1		return

;***********************************************************

;	A new NV so put it in EEPROM

putNV	call	thisNN  		;is it here?
		sublw	0
		bnz		no_NV
		movlw	NV_NUM + 1		;put new NV in EEPROM 
		cpfslt	ev2
		return
		movf	ev2,W
		bz		no_NV
		decf	WREG			;NVI starts at 1
		addlw	LOW NVstart
		movwf	EEADR
		movf	ev3,W
	
		call	eewrite	
		
no_NV	return
		

;******************************************************************


;		send individual parameter

;		Index 0 sends no of parameters

para1rd	movf	RXB0D3,W
		sublw	0
		bz		numParams
		movlw	PRMCOUNT
		movff	RXB0D3, Temp
		decf	Temp
		cpfslt	Temp
		bra		pidxerr
		movlw	OPC_PARAN
		movwf	Tx1d0
		movlw	7			;FLAGS index in nodeprm
		cpfseq	Temp
		bra		notFlags			
		call	getflags
		movwf	Tx1d4
		bra		addflags
notFlags		
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	HIGH nodeprm
		movwf	TBLPTRH		;relocated code
		clrf	TBLPTRU
		decf	RXB0D3,W
		addwf	TBLPTRL
		bsf		EECON1,EEPGD
		tblrd*
		movff	TABLAT,Tx1d4
addflags						
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return	
		
numParams
		movlw	OPC_PARAN
		movwf	Tx1d0
		movlw	PRMCOUNT
		movwf	Tx1d4
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return
		
pidxerr
		movlw	.10				;error
		call	errsub
		return
		
getflags						; create flags byte
		movlw	PF_CONSUMER
		btfsc	Datmode,3
		iorlw	4				; set bit 2
		movwf	Temp
		bsf		Temp,3			;set bit 3, we are bootable
		movf	Temp,W
		return
		
		
;**********************************************************

;		send node parameter bytes (7 maximum)

parasend	
		movlw	OPC_PARAMS
		movwf	Tx1d0
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	8
		movwf	TBLPTRH			;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
para1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		para1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa			;send CBUS message
		return

;**************************************************************************

; returns Node Number, Manufacturer Id, Module Id and Flags

whoami
		call	ldely		;wait for other nodes
		movlw	OPC_PNN
		movwf	Tx1d0
		movlw	MAN_NO		;Manufacturer Id
		movwf	Tx1d3
		movlw	MODULE_ID		; Module Id
		movwf	Tx1d4
		call	getflags
		movwf	Tx1d5
		movlw	6
		movwf	Dlc
		call	sendTX
		return

	
;**************************************************************

;	RQDDS  Request data as in DDES for Railcom reader. Response is DDRS (0xFB)

rqdds	movlw	OPC_DDRS		;load Tx1 with data
		movwf	Tx1d0
		movff	DNhi,Tx1d1
		movff	DNlo,Tx1d2
		movff	Nvtemp,Tx1d3
		movff	Adr1,Tx1d4
		movff	Adr2,Tx1d5
		movff	Loco,Tx1d6
		clrf	Tx1d7			;for now
		movlw	8
		movwf	Dlc
		call	sendTX
		return	

;**********************************************************

;request state of block (occ or unocc)

asrq	btfss	Sflag,1			;occupied
		bra		asrq1
		call	occ_sn1			;send occupied
		return
asrq1	btfss	Sflag,2			;unoccupied
		return
		call	un_occ1

		return

;*************************************************************

		;Time for PB debounce

pb_time	call	pb_deb				;PB debounce
		movf	WREG
		bnz		bounce				;was bounce
		movlw	3					;loops for delay
		movwf	Count
		clrf	TMR0H
		clrf	TMR0L
		movlw	B'00000111'			;set timer 0
		movwf	T0CON
		bsf		T0CON,TMR0ON		;start timer	1 second
pb_1a	btfsc	S_PORT,S_BIT		;PB released?
		bra		pb_2				;yes. Is too short 
		btfss	INTCON,TMR0IF		;timer out?
		bra		pb_1a
		bcf		INTCON,TMR0IF
		decfsz	Count
		bra		pb_1a				;keep looking
		retlw	0
pb_2	retlw 	1
bounce	retlw	2

;***************************************************************

;		Pushbutton debounce
	
pb_deb	btfss	PIR2,TMR3IF		;uses timer 3
		bra		pb_deb			;loop
		bcf		PIR2,TMR3IF
		btfss	S_PORT,S_BIT	;test PB again
		bra		pb_ok			;still on
		retlw 1					;bounce
pb_ok
		retlw 0					;is OK

;***********************************************************

;	Flashes yellow LED in setup

flash	btfss	PIR2,TMR3IF		;Timer 3 out?
		return					;no
		bcf		PIR2,TMR3IF		;reset timer
		decfsz	T1count			;loop counter
		bra		flash
		movlw	.16
		movwf	T1count			;reset counter
		
flash1	btg		LED_PORT,YELLOW	;flash yellow

		return

;*************************************************************************

unflash	bsf		LED_PORT,YELLOW	; yellow steady
		
		return

;*************************************************************************

		; copy event data to safe buffer

copyev	movff	RXB0D0, ev_opc
		movff	RXB0D1, ev0
		movff	RXB0D2, ev1
		movff	RXB0D3, ev2
		movff	RXB0D4, ev3
		movff	RXB0D5, EVidx      	; only used by learn and some read cmds
		movff	RXB0D6, EVdata		; only used by learn cmd

		movlw	OPC_ASON
		subwf	RXB0D0,W
		bz		short				;is a short event
		movlw	OPC_ASOF
		subwf	RXB0D0,W
		bz		short
		movlw	OPC_ASRQ
		subwf	RXB0D0,W
		bz		short
		return

short	clrf	ev0					;here if a short event
		clrf	ev1
		return

;*************************************************************
;		Send an 'occupied' frame

occ_snd	btfsc	Sflag,1			;has sent an occupied?
		bra		no_occ
occ_sn1	movf	DNhi,F			;is DN = 0
		bnz		occ_sn2
		movf	DNlo,F
		bnz		occ_sn2
		bra		no_occ						;invalid DN
occ_sn2	bsf		Sflag,1			;do once
		movlb	.15				;bank 15
		;		put info into TXB0 for occupied frame

		clrf	TXB0SIDH
		movf	IDtemph,W
		movwf	TXB0SIDH
		movf	IDtempl,W
		movwf	TXB0SIDL
		movlw	0xB0
		iorwf	TXB0SIDH		;set priority
		movlw	5
		movwf	TXB0DLC	
		movlw	OPC_ASON
		movwf	TXB0D0
		movff	NN_temph,TXB0D1
		movff	NN_templ,TXB0D2
		movff	DNhi,TXB0D3
		movff	DNlo,TXB0D4
	

		bcf		TXB0D0,0		;ON event
tx0test	btfsc	TXB0CON,TXREQ	;test if clear to send
		bra		tx0test
		bsf		TXB0CON,TXREQ	;OK so send
tx0sent btfss	TXB0CON,TXBIF
		bra		tx0sent			;has it sent
		bcf		TXB0CON,TXBIF
	
		
		movlb	0				;bank 0
no_occ	bcf		Sflag,2			;clear unocc flag
		bcf		T1CON,TMR1ON		;stop timer
		movlw	0xF6				;reset timer for short pulses
		movwf	TMR1H
		clrf	TMR1L
		
	
		bsf		T1GCON,TMR1GE		;gated
		bsf		T1GCON,T1GGO		;gate set

		bsf		T1CON,TMR1ON		;start timer		
		bsf		PIE1,TMR1IE			;re-enable interrupts
		return

;****************************************************************
;		Send an unoccupied frame

un_occ 	;btfsc	Sflag,1			;has sent an occupied?
	;	bra		no_occ
		movf	DNhi,F			;is DN = 0
		bnz		un_occ1
		movf	DNlo,F
		bnz		un_occ1
		bra		un_occ2
un_occ1	bcf		PIE1,TMR1IE
		bsf		Sflag,2			;do once
		movlb	.15				;bank 15
		;		put info into TXB0 for occupied frame

		clrf	TXB0SIDH
		movf	IDtemph,W
		movwf	TXB0SIDH
		movf	IDtempl,W
		movwf	TXB0SIDL
		movlw	0xB0
		iorwf	TXB0SIDH		;set priority
		movlw	5
		movwf	TXB0DLC	
		movlw	OPC_ASON
		movwf	TXB0D0
		movff	NN_temph,TXB0D1
		movff	NN_templ,TXB0D2
		movff	DNhi,TXB0D3
		movff	DNlo,TXB0D4
	

		bsf		TXB0D0,0		;OFF event
tx0tes1	btfsc	TXB0CON,TXREQ	;test if clear to send
		bra		tx0tes1
		bsf		TXB0CON,TXREQ	;OK so send
tx0sen1 btfss	TXB0CON,TXBIF
		bra		tx0sen1			;has it sent
		bcf		TXB0CON,TXBIF
		
		movlb	0				;bank 0
un_occ2	bcf		Sflag,1		;clear occ flag
		bcf		Sflag,0		;allow loco frame when next occupied
		clrf	Bflag		;for next loco	
		movf	NV2_tmp,W
		movwf	NV2	

		bcf		T1CON,TMR1ON		;stop timer
		movlw	0xF6				;reset timer for short pulses
		movwf	TMR1H
		clrf	TMR1L
		
	
		bsf		T1GCON,TMR1GE		;gated
		bsf		T1GCON,T1GGO		;gate set

		bsf		T1CON,TMR1ON		;start timer
				
		return

;*************************************************



;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return
			
;*********************************************************

;		a delay routine
			
dely	movlw	.10
		movwf	Count1
dely2	clrf	Count
dely1	decfsz	Count,F
		goto	dely1
		decfsz	Count1
		bra		dely2
		return	

;***************************************************

;initial set to running mode. Gets NN etc and sets to run

slimset	clrf	NN_temph
		clrf	NN_templ
		call	clr_sub				;set defaults for events 
		bcf		LED_PORT,YELLOW		;yellow LED off
		bsf		LED_PORT,GREEN		;green LED on. Green for SLiM
		
set1	btfsc	S_PORT,S_BIT		;look for PB on 
		bra		set1				;loop till on
		call	pb_time				;time the on state
		btfsc	Datmode,3
		goto	set2				;is pb in main loop
		sublw	0					;is it held in long enough?
		bnz		abort				;bounce or too short
		bcf		LED_PORT,GREEN		;green off
set1a	btfss	S_PORT,S_BIT		;button released?
		bra		set1a				;loop
		call	self_en				;do an enumerate
		bcf		Datmode,1
		bsf		Datmode,2
		movlw	Modstat
		movwf	EEADR
		movlw	B'00000100'
		call	eewrite	
		call	nnreq				;request node number RQNN
set1b	call	flash				;flash yellow
		btfsc	S_PORT,S_BIT		;PB again?
		bra		set1g				;go on
		bcf		LED_PORT,YELLOW		;in case it was on.
		call	pb_time				;how long is it in?
		sublw	0
		bnz		set1g				;too short
		bra		slimset				;try again
set1g	btfsc	COMSTAT,7			;look for CAN input. 
		call	getcan				;wait for answer
set1c	btfss	Datmode,0			;got an answer
		bra		set1b				;loop till		(maybe needs an abort mechanism?)
		movlw	OPC_RQNP			;request for parameters
		subwf	ev_opc,W
		bz		set1e
		movlw	OPC_RQMN			;is it a name request?
		subwf	ev_opc,W
		bz		set1h				;send name

		movlw	OPC_SNN				;set new NN
		subwf	ev_opc,W
		bz		set1f
set1d	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		bra		set1c				;look again

set1e	call	parasend			;send parameters to FCU
		bra		set1d				;wait for SNN

set1h	call 	rqmn				;send name
		goto	set1d

set1f	call	putNN				;put in new NN. Sets Datmode to 8
		
		call	newid				;move all ID to EEPROM etc
		call	unflash				;yellow on 
		movlw	OPC_NNACK
		call	nnrel				;send NNACK
		bcf		RXB0CON,RXFUL		;clear CAN 
		bcf		Datmode,0
		bsf		Sflag,2
		retlw	0					;continue setup as if running mode
abort	bcf		LED_PORT,YELLOW
		retlw	1					;too short or a bounce


set2	movwf	W_temp							;here if in running mode
		
		sublw	1
		bz		set2a 
		movf	W_temp,W
		sublw	2
		bz		set_bak2

		bra		set_off				;is	held so  cancel run mode 
set2a	
		call	self_en				;do an enumerate
		call	nnreq				;request node number RQNN
set2b	call	flash				;flash yellow
		btfss	S_PORT,S_BIT		;pb in again?
		bra		set_bk1				;set back to main
		btfsc	COMSTAT,7			;look for CAN input. 
		call	getcan				;wait for answer
set2c	btfss	Datmode,0			;got an answer
		bra		set2b				;loop till		(maybe needs an abort mechanism?)
		movlw	OPC_SNN				;set new NN
		subwf	ev_opc,W
		bz		set2f
set2d	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		bra		set2b				;look again



set2f	call	putNN				;put in new NN. Sets Datmode to 8
;		call	newid				;move all ID to EEPROM etc
set2g	call	unflash				;yellow on 
		bcf		Datmode,1			;out of NN waiting
		bcf		Datmode,2
		movlw	OPC_NNACK
		movwf	Tx1d0
		call	nnrel				;send NNACK
		goto	main					;back to main

set_bk1	btfss	S_PORT,S_BIT		;released 
		bra		set_bk1
		bra		set2g				;back

set_bak2 goto 	main				;bounce so do nothing 		

		
		




set_off	bcf		LED_PORT,YELLOW
		bsf		LED_PORT,GREEN
		btfss	S_PORT,S_BIT		;released
		bra		set_off
		clrf	Datmode
		movlw	LOW Modstat
		movwf	EEADR
		movf	Datmode,W
		call	eewrite
		movlw	OPC_NNREL
		movwf	Tx1d0				;send release
		call	nnrel
		movlw	0
		incf	EEADR
		call	eewrite				;clear old NN
		movlw	0
		incf	EEADR
		call	eewrite
		
		retlw	1
		
;*****************************************************************	

		

setup1	bcf		LED_PORT,YELLOW	;clear if on
		movlw	Modstat			;get setup status
		movwf	EEADR
		call	eeread
		movwf	Datmode
		sublw	8				;is it in run mode?
		bz		setup2			;yes	
		movlw	0				;else set it
		movwf	Datmode			;not set yet
		call	eewrite
		call	slimset			;wait for setup PB
		movf	WREG			;is it long enough
		bnz		setup1			;no
setup2	return



;********************************************************************	

setsub		movlb	.15

		bsf		OSCTUNE,PLLEN
	
		clrf	ANCON0			;disable A/D
		clrf	ANCON1
		clrf	CM1CON			;disable comparator
		clrf	CM2CON
		clrf	INTCON2			
		bsf		INTCON2,7		;weak pullups off
		clrf	WPUB			;pullups clear
		movlb	0	


		movlw	B'00000100'		;PORTA has the  FLiM PB on RA2
		movwf	TRISA

		movlw	B'00000000'		;used for LEDs
		movwf	TRISB

		bsf		RCON,IPEN

		movlw	B'10000111'		;serial in on portc,7. RC0 and RC1 are loco orientation.
		movwf	TRISC			;gate on RC2
	
	

		call	read_LUT		;move LUT to RAM
		

		lfsr	FSR0, 0			; clear page 1
		
nextram	clrf	POSTINC0		
		tstfsz	FSR0L
		bra		nextram
	
		clrf	INTCON			;no interrupts yet
	
		movlw	B'00000111'		;set timer 0 for block occupied delay
		movwf	T0CON
		clrf	TMR0H
		clrf	TMR0L

		movlw	B'11011000'
		movwf	T1GCON			;set timet 1 for gated mode
		movlw	B'01000110'
		movwf	T1CON			;set timer 1

		movlw	0xF6
		movwf	TMR1H
		clrf	TMR1L
		bsf		IPR1,TMR1IP
		bsf		PIE1,TMR1IE		;high pri for TMR1
	
		bcf		PIR1,TMR1IF		;clear flag
		bsf		T1CON,TMR1ON
		clrf	INTCON			;no interrupts yet
		movlw	B'00000100'		
		movwf	TXSTA1			;Baud rate is High
		movlw	B'10000000'
		movwf	RCSTA1			;set EUSART1 receive
		movlw	B'00000000'
		movwf	BAUDCON1
		clrf	SPBRGH1
		movlw	.15
		movwf	SPBRG1			;set baud rate for 250K
		bcf		RCSTA1,CREN		; EUSART1

;	next segment is essential.
		
	
		clrf	BSR				;set to bank 0
		clrf	EECON1			;no accesses to program memory	
		clrf	Datmode
		clrf	Latcount
		bsf		CANCON,7		;CAN to config mode
		movlw	B'10110000'
		movwf	ECANCON	
		bsf		ECANCON,5		;CAN mode 2 
		movf	ECANCON,W
		movwf	TempECAN 

		movlb	.14
		clrf	BSEL0			;8 frame FIFO
		clrf	RXB0CON
		clrf	RXB1CON
		clrf	B0CON
		clrf	B1CON
		clrf	B2CON
		clrf	B3CON
		clrf	B4CON
		clrf	B5CON
		
			
		movlw	CANBIT_RATE		;set CAN bit rate at 125000 
		movwf	BRGCON1
		movlw	B'10011110'		;set phase 1 etc
		movwf	BRGCON2
		movlw	B'00000011'		;set phase 2 etc
		movwf	BRGCON3
		movlb	0

		movlw	B'00100000'
		movwf	CIOCON			;CAN to high when off


		
mskload	lfsr	FSR0,RXM0SIDH		;Clear masks, point to start
mskloop	clrf	POSTINC0		

		cpfseq	FSR0L			;0xEFF is last mask address
		bra		mskloop
		
		clrf	CANCON			;out of CAN setup mode
		clrf	CCP1CON

		clrf	IPR5			;low priority CAN RX and Tx error interrupts


		clrf	T3GCON	
		movlw	B'00110010'		;Timer 3 set Timer 3 for LED flash
		movwf	T3CON
		movlw	0x00
		movwf	TMR3H			;Timer 3 is a 16 bit timer
		movwf	TMR3L
		bsf		T3CON,TMR3ON	;run timer
		movlw	.16
		movwf	T1count			;flash delay counter
		movlw	4
		movwf	Debcnt	

		bsf		PIE5,ERRIE		;Tx error enable
		bsf		PIE5,FIFOWMIE	;FIFO full interrupt

		clrf	Tx1con	

	;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
	
		clrf	INTCON3			;
		clrf	T3GCON
			
		clrf	PIR1
		clrf	PIR2			;for timer 3
		clrf	PIR3
		clrf	PIR4
		clrf	PIR5

		movlw	.39
		movwf	PR4				;10mSec timer x256
		movlw	B'00000111'
		movwf	T4CON			;set timer 4
		
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR5			;clear all ECAN flags

		clrf	EEADRH			;upper EEPROM page to 0

		movlb	.15				;set busy frame
		clrf	TXB0SIDH
		clrf	TXB0SIDL
		clrf	TXB0DLC
		movlb 0


		movlw	B'11000000'		;start interrupts
		movwf	INTCON
		return




	

;*******************************************************************************


	ORG 0xF00000			;EEPROM data. Defaults
	
CANid	de	B'01111111',0x00	;CAN id default and module status
NodeID	de	0x00,0x00			;Node ID

	ORG 0xF00010
DevNo	de	0x00,0x01		;two byte DN
	ORG 0xF00020
NVstart	de	0x02,0x7F		;two NVs, initialised to 0x06 and 7F

							;NV1 is 0 = do nothing
							;       2 = Ch 1 only
							;		4 = Ch 2 only
							;       6 = Ch 1 and Ch 2 

							;NV2 is unoccupied delay





;		LUT for 4/8 decoding

	ORG 0xF00100

Lut_start			

		de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x33,0xFF,0xFF,0xFF,0x34,0xFF,0x35,0x36,0xFF
		de  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3A,0xFF,0xFF,0xFF,0x3B,0xFF,0x3C,0x37,0xFF
		de	0xFF,0xFF,0xFF,0x3F,0xFF,0x3D,0x38,0xFF,0xFF,0x3E,0x39,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x24,0xFF,0xFF,0xFF,0x23,0xFF,0x22,0x21,0xFF
		de	0xFF,0xFF,0xFF,0x1F,0xFF,0x1E,0x20,0xFF,0xFF,0x1D,0x1C,0xFF,0x1B,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0x19,0xFF,0x18,0x1A,0xFF,0xFF,0x17,0x16,0xFF,0x15,0xFF,0xFF,0xFF
		de	0xFF,0x25,0x14,0xFF,0x13,0xFF,0xFF,0xFF,0x32,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x0E,0xFF,0x0D,0x0C,0xFF
		de	0xFF,0xFF,0xFF,0x0A,0xFF,0x09,0x0B,0xFF,0xFF,0x08,0x07,0xFF,0x06,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0x04,0xFF,0x03,0x05,0xFF,0xFF,0x02,0x01,0xFF,0x00,0xFF,0xFF,0xFF
		de	0xFF,0x0F,0x10,0xFF,0x11,0xFF,0xFF,0xFF,0x12,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0xFF,0xFF,0x2B,0x30,0xFF,0xFF,0xFF,0x2F,0xFF,0x31,0xFF,0xFF,0xFF
		de	0xFF,0x29,0x2E,0xFF,0x2D,0xFF,0xFF,0xFF,0x2C,0x2A,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0x28,0xFF,0x27,0xFF,0xFF,0xFF,0x26,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
		de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

; Note. ACK, NACK abd BUSY are stord as 0xFF for now. 


		ORG	0xF003FE
		de		0,0		;for boot load


		end
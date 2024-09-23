;   	TITLE		"Code for Railcom to CBUS module  FLiM node for CBUS"
; Original filename CANRCOM_test5	 	12/10/20

; This code is for a PIC  18F25K80

; Assembly options
	LIST	P=18F25K80,r=hex,N=75,C=120,T=ON

	include		"p18f25K80.inc"

;	include		"..\cbuslib\cbusdefs8r.inc"

	include		"c:\Users\ardma\Documents\Merg\defs\cbusdefs.inc"
; uses  "..\cbuslib\cbusdefs8v"

; based on CANRC522 and sends DDES frames. Four data bytes / frame.
; User teaches a short event for the 'device number'

; Uses 16 MHz resonator and PLL for 64 MHz clock
; The setup timer is TMR0. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only except for bootloader
; Uses a CANMIO board with a CT on a small board connected to the 
;	10 way IDC header.
; As test 1 but mod for short loco addresses and equal Ch 1 times.
; As test 3 but with channel 2 added
; Now test 5. Has dual frequency bootloader and 'name' OpCode.

; New file name CANRCOM_test6		10/08/24 - MPS
; This code test_6 version of CANRCOM has been structured an tested
; and proved working but the hardware causes and issue with the trigger
;	window created by the pic. A 10nF cap. connected across the CT 
;	secondary goes some way to relieving the problem. MPS

; Version 7 will contain re-writtern RalCom decoding section. MPS 

; Version 10 the DDES data is in accordance with RFID structure.
; This version also handles the third pair of datagrams that provide Info1
;	via ID3 (CV28). MPS

;	OPC_DDES    EQU	0xFA	; Short data frame aka device data event 
;									  (device id plus 5 data bytes)

#DEFINE FLIM_ONLY

; This code doesn't use the event handler.

S_PORT 		EQU	PORTA; setup switch  Change as needed
S_BIT			EQU	2

LED_PORT 	EQU	LATB  	; change as needed
OUTPORT  	EQU	LATC	; PORTC. Used for SPI etc.
GREEN		EQU	7		; PB7 is the green LED on the PCB
YELLOW		EQU	6		; PB6 is the yellow LED on the PCB

OLD_EN_NUM	EQU	d'32'	; old number of allowed events (hangover from earlier module code)
EN_NUM			EQU	1		; each event is a two byte DN, one for each detector
EV_NUM  			EQU 	1		; number of allowed EVs per event. 1 . 
Modstat 			EQU 	1		; Module state address in EEPROM

MAN_NO		EQU  	d'165'    	;manufacturer number
MAJOR_VER	EQU	10
MINOR_VER	EQU	"A"
BETA_VER	EQU	d'101'     	; Set BETA version here, set to 0 for release version
EVT_NUM     EQU	EN_NUM  ; Number of events
EVperEVT		EQU	EV_NUM  	; Event variables per event
NV_NUM		EQU	d'2'       	; Number of node variables  
NODEFLGS	EQU	d'1' + d'2' + d'8'
CPU_TYPE    EQU	d'13'
MODULE_ID	EQU	d'66'		;Till in cbusdefs.

	
; definitions  Change these to suit hardware.

#DEFINE OR0 		PORTC,0		; Loco orientation bit 0. If neither bit 0 or 1 is set, No Railcom
#DEFINE OR1		PORTC,1		; Loco orientation bit 1.
#DEFINE SER_IN	PORTC,7		; RailCom serial data in

#DEFINE RAIL		PORTC,2		; RailCom adapter input

#DEFINE	CH1		PORTC,5		; Channel 1 time for test
#DEFINE TXD		PORTB,2		; CAN transmit pin. 
#DEFINE RXD		PORTB,3		; CAN receive  pin. 

#DEFINE	Md_IDconf	Datmode,7	; for CAN_ID conflict

; definitions used by bootloader

#DEFINE	MODE_SELF_VERIFY	; Enable self verification of written data (undefine if not wanted)

#DEFINE	HIGH_INT_VECT	0x0808		; HP interrupt vector redirect. Change if target is different
#DEFINE	LOW_INT_VECT	0x0818		; LP interrupt vector redirect. Change if target is different.
#DEFINE	RESET_VECT		0x0800		; start of target
#DEFINE	CAN_CD_BIT		RXB0EIDL,0	; Received control / data select bit
#DEFINE	CAN_PG_BIT		RXB0EIDL,1	; Received PUT / GET bit
#DEFINE	CANTX_CD_BIT	TXB0EIDL,0	; Transmit control/data select bit
#DEFINE	CAN_TXB0SIDH	B'10000000'	; Transmitted ID for target node
#DEFINE	CAN_TXB0SIDL	B'00001000'
#DEFINE	CAN_TXB0EIDH	B'00000000'	
#DEFINE	CAN_TXB0EIDL	B'00000100'
#DEFINE	CAN_RXF0SIDH	B'00000000'	; Receive filter for target node
#DEFINE	CAN_RXF0SIDL	B'00001000'
#DEFINE	CAN_RXF0EIDH	B'00000000'
#DEFINE	CAN_RXF0EIDL	B'00000111'
#DEFINE	CAN_RXM0SIDH	B'11111111'	; Receive masks for target node
#DEFINE	CAN_RXM0SIDL	B'11101011'
#DEFINE	CAN_RXM0EIDH	B'11111111'
#DEFINE	CAN_RXM0EIDL	B'11111000'
;#DEFINE	CAN_BRGCON1	B'00001111'	; CAN bit rate controls. 16 MHz resonator
#DEFINE	CANBIT_RATE	B'00001111'	; CAN bit rate while runnung. For 16 MHz clock
#DEFINE 	CANBIT_BL		B'00000011'	; CAN bit rate for bootloader
#DEFINE	CAN_BRGCON2	B'10011110'
#DEFINE	CAN_BRGCON3	B'00000011'
#DEFINE	CAN_CIOCON	B'00100000'	; CAN I/O control	
;**************************************************************
#ifndef	EEADRH		
#DEFINE	EEADRH	EEADR+ 1	
#endif			
#DEFINE	TRUE	1	
#DEFINE	FALSE	0	
#DEFINE	WREG1	PRODH						; Alternate working register
#DEFINE	WREG2	PRODL	
#DEFINE	MODE_WRT_UNLCK	 _bootCtlBits, 0	; Unlock write and erase
#DEFINE	MODE_ERASE_ONLY _bootCtlBits, 1	; Erase without write
#DEFINE	MODE_AUTO_ERASE _bootCtlBits, 2	; Enable auto erase before write
#DEFINE	MODE_AUTO_INC	 _bootCtlBits, 3	; Enable auto inc the address
#DEFINE	MODE_ACK			 _bootCtlBits, 4	; Acknowledge mode
#DEFINE	ERR_VERIFY			 _bootErrStat, 0	; Failed to verify if set
#DEFINE	CMD_NOP			0x00	
#DEFINE	CMD_RESET			0x01	
#DEFINE	CMD_RST_CHKSM	0x02	
#DEFINE	CMD_CHK_RUN		0x03
#DEFINE CMD_BOOT_TEST 	0x04
#DEFINE	CANTX				0x02			;port B RB2	

; Set config registers. Note. CANRCOM uses Port B for the CAN. 

	CONFIG	FCMEN = OFF, FOSC = HS1, IESO = OFF, PLLCFG = OFF
	CONFIG	PWRTEN = ON, BOREN = SBORDIS, BORV=0, SOSCSEL = DIG
	CONFIG	WDTEN=OFF
	CONFIG	MCLRE = ON, CANMX = PORTB
	CONFIG	BBSIZ = BB1K 
	
	CONFIG	XINST = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF

; processor uses  16 MHz. Resonator with HSPLL to give a clock of 64MHz
;	 (V4 onwards)

; This code doesn't use the event handler.

;**************************************************************
;	RAM addresses used by boot. can also be used by application.

	CBLOCK 0
		_bootCtlMem
		_bootAddrL	; Address info
		_bootAddrH		
		_bootAddrU		
		_unused0		; (Reserved)
		_bootCtlBits	; Boot Mode Control bits
		_bootSpcCmd; Special boot commands
		_bootChkL	; Chksum low byte fromPC
		_bootChkH	; Chksum hi byte from PC		
		_bootCount		
		_bootChksmL; 16 bit checksum
		_bootChksmH		
		_bootErrStat	; Error Status flags
	ENDC
	
; end of bootloader RAM

	CBLOCK 0
		Datagram1	; 1st byte from EUSART
		Datagram2	; 2nd byte from EUSART
		Dgm1			; 1st byte after 4/8 conversion of Datagram1
		Dgm2			; 2nd byte after 4/8 conversion of Datagram2
		Adr1			; loco address high (CV17)
		Adr2			; loco address low	(CV1/18)
		DgmFlags		; Number of datagrams received in a DCC packet
		Count
		Count1		; Used in delay routines
		Count2		; Used in delay routines
		T1count		; In setup for T1 (Yellow LED flash rate)
		T4count		; rolls over every 10 mSec
		Debcnt		; debounce counter
		Ori				; flags for orientation
		Ori1			; last byte if there are 4
		IDbyte		; RailCom ID
		ID_old
		Ch_no			; channel number for RailCom
		DNhi			; device number hi for decoder
		DNlo			; device number lo for decoder
		Nvtemp
		NV2			; unoccupied delay
		NV2_tmp
		Datmode		; for CBUS
		IDcount		; used in self allocation of CAN ID.
		Temp			; temp for various
		W_temp		; temp for W REG
		EVflags		
		Sflag			; Send once flag - 0 RailCom - 1 Occupied - 2 Unoccupied
		Bflag			; for hi and lo address bytes as well info1 if there 
		Lspd			; Loco speed
		reada			; Reader type (1 = RailCom)
		Info1			; Received decoder information byte (CV28)
		IDcnt			; Identifier count
		Loc_flgs		; Loco Identification flags
		Tx1con
		Tx1sidh
		Tx1sidl
		Tx1eidh
		Tx1eidl
		Tx1dlc
		Tx1d0			; buffer for CBUS DDES message
		Tx1d1
		Tx1d2
		Tx1d3
		Tx1d4
		Tx1d5
		Tx1d6
		Tx1d7
		Dlc
;	Used for self enum
		Roll			; rolling bit for enum
		In_roll			; rolling bit for input sense
		NN_temph
		NN_templ
		Fsr_temp0L
		Fsr_temp0H 
		Fsr_temp1L
		Fsr_temp1H 
		Fsr_temp2L
		Fsr_temp2H
		W_tempL		; in lo pri IRQ
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
		Fsr_tmp1Le	; temp store for FSR1
		Fsr_tmp1He 
		Enum0		; bits for new enum scheme.
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

; data area to store data for event learning and event matching

		ev_opc	; incomong op code
		ev0		; event number from learn command and from received event
		ev1
		ev2
		ev3
		ENidx
		EVidx
		EVdata
	ENDC

	CBLOCK 0x100		; bank 1
		Lut					; lookup table for 4/8 decoding
	ENDC	

;program start	

;**************************************************************
;	This is the bootloader
; ************************************************************* 
; _STARTUPCODE	0x00
	ORG 0x0000
; *************************************************************
	BRA	_CANInit
	BRA	_StartWrite
; ************************************************************* 

	ORG 0x0008
; *************************************************************

	GOTO		HIGH_INT_VECT

; ************************************************************* 

	ORG 0x0018
; *************************************************************

	GOTO		LOW_INT_VECT 

; ************************************************************* 
;	Code start
; *************************************************************
	ORG 	0x0020
;_CAN_IO_MODULE CODE
; ************************************************************* 
; Function: VOID _StartWrite(WREG _eecon_data)
; PreCondition: Nothing
; Input: _eecon_data
; Output: Nothing. Self write timing started.
; Side Effects: EECON1 is corrupted; WREG is corrupted.
; Stack Requirements: 1 level.
; Overview: Unlock and start the write or erase sequence to protected
;	memory. Function will wait until write is finished.
; *************************************************************
_StartWrite
	MOVWF 	EECON1
	BTFSS 	MODE_WRT_UNLCK	; Stop if write locked
	RETURN
	MOVLW 	0x55					; Unlock
	MOVWF 	EECON2 
	MOVLW	0xAA 
	MOVWF  EECON2
	BSF	 	EECON1, WR			; Start the write
	NOP
	BTFSC 	EECON1, WR			; Wait (depends on mem type)
	BRA		$ - 2
 	RETURN
; *************************************************************

; Function: _bootChksm _UpdateChksum(WREG _bootChksmL)

; PreCondition: Nothing
; Input: _bootChksmL
; Output: _bootChksm. This is a static 16 bit value stored in the Access Bank.
; Side Effects: STATUS register is corrupted.
; Stack Requirements: 1 level.
; Overview: This function adds a byte to the current 16 bit checksum
;	count. WREG should contain the byte before being called.

;	The _bootChksm value is considered a part of the special
;	register set for bootloading. Thus it is not visible. ;
;**************************************************************
_UpdateChksum:
	ADDWF	_bootChksmL,	F 	; Keep a checksum
	BTFSC	STATUS,	C
	INCF		_bootChksmH,	F
	RETURN
;**************************************************************

;	Function:	VOID _CANInit(CAN,	BOOT)

; PreCondition: Enter only after a RESET has occurred.
; Input: CAN control information, bootloader control information 
;	Output: None.
; Side Effects: N/A. Only run immediately after RESET.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	RETURN when called. It has been written in a linear form to
;	save space.Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied. ;
;	This routine tests the boot flags to determine if boot mode is
;	desired or normal operation is desired. If boot mode then the
;	routine initializes the CAN module defined by user input. It
;	also resets some registers associated to bootloading.

; *************************************************************
_CANInit:

	CLRF		EECON1
	SETF		EEADR				; Point to last location of EEDATA
	SETF		EEADRH
	BSF		EECON1, RD			; Read the control code
	NOP
	INCFSZ 	EEDATA, W

	GOTO		RESET_VECT

	CLRF		_bootSpcCmd 	; Reset the special command register
	MOVLW 	0x1C				; Reset the boot control bits
	MOVWF 	_bootCtlBits 
	MOVLB	d'14'				; Set Bank 14 for K series
;	BCF 		TRISB, CANTX 	; Set the TX pin to output 
	MOVLW 	CAN_RXF0SIDH 	; Set filter 0
	MOVWF 	RXF0SIDH
	MOVLW 	CAN_RXF0SIDL 
	MOVWF 	RXF0SIDL
	COMF		WREG				; Prevent filter 1 from causing a receive event

	MOVWF	RXF1SIDL	;		
	MOVLW	CAN_RXF0EIDH	
	MOVWF	RXF0EIDH	
	MOVLW	CAN_RXF0EIDL	
	MOVWF	RXF0EIDL	
	MOVLW	CAN_RXM0SIDH	;	Set mask
	MOVWF	RXM0SIDH	
	MOVLW	CAN_RXM0SIDL	
	MOVWF	RXM0SIDL	
	MOVLW	CAN_RXM0EIDH	
	MOVWF	RXM0EIDH	
	MOVLW	CAN_RXM0EIDL	
	MOVWF	RXM0EIDL

	MOVLW	CANBIT_BL		; Set bit rate for bootloader
	MOVWF	BRGCON1	
	MOVLW	CAN_BRGCON2	
	MOVWF	BRGCON2	
	MOVLW	CAN_BRGCON3	
	MOVWF	BRGCON3	
	MOVLB	d'15'
	CLRF		ANCON0
	CLRF		ANCON1
;	MOVLB	0
	MOVLW	CAN_CIOCON	; Set IO
	MOVWF	CIOCON	
	
	CLRF		CANCON			; Enter Normal mode

	BCF		TRISB,7
	BCF		TRISB,6
	BSF		LATB,7			; green LED on
	BSF		LATB,6			; yellow LED on


; *************************************************************
; This routine is essentially a polling loop that waits for a
; receive event from RXB0 of the CAN module. When data is
; received, FSR0 is set to point to the TX or RX buffer depending
; upon whether the request was a 'put' or a 'get'.
; ************************************************************* 
_CANMain
	
	BCF		RXB0CON, RXFUL	; Clear the receive flag
_wait:
	CLRWDT							; Clear WDT while waiting		
	BTFSS 	RXB0CON, RXFUL	; Wait for a message	
	BRA		_wait

_CANMainJp1
	LFSR		0, RXB0D0
	MOVF		RXB0DLC, W 
	ANDLW 	0x0F
	MOVWF 	_bootCount 
	MOVWF 	WREG1
	BZ			_CANMain 
_CANMainJp2				;?

; ************************************************************* 
; Function: VOID _ReadWriteMemory()

; PreCondition:Enter only after _CANMain().
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	RETURN when called. It has been written in a linear form to
;	save space.Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied.
; This is the memory I/O engine. A total of eight data bytes are received 
;	and decoded. In addition two control bits are received, put/get and 
;	control/data.
; A pointer to the buffer is passed via FSR0 for reading or writing. 
; The control register set contains a pointer, some control bits and special
;	command registers.
; Control
; <PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT>< SPCMD><CPDTL><CPDTH>
; Data
; <PG>< CD>< DATA0>< DATA1>< DATA2>< DATA3>< DATA4>< DATA5>< DATA6>< DATA7>
; PG bit:	Put = 0, Get = 1
; CD bit:	Control = 0, Data = 1

; *************************************************************
_ReadWriteMemory:
	BTFSC		CAN_CD_BIT			; Write/read data or control registers
	BRA		_DataReg
; *************************************************************
; This routine reads or writes the bootloader control registers,
; then executes any immediate command received.
_ControlReg
	LFSR	1, _bootAddrL			; _bootCtlMem
_ControlRegLp1

	MOVFF 	POSTINC0, POSTINC1 
	DECFSZ 	WREG1, F
	BRA		_ControlRegLp1

; ********************************************************* 
; This is a no operation command.
	MOVF	_bootSpcCmd, W		; NOP Command
	BZ		_CANMain
;	BZ		_SpecialCmdJp2			; or send an acknowledge

; ********************************************************* 
; This is the RESET command.
	XORLW 	CMD_RESET			; RESET Command 
	BTFSS 	STATUS, Z
	BRA		_SpecialCmdJp4
	SETF		EEADR				; Point to last location of EEDATA
	SETF		EEADRH
	CLRF		EEDATA				; and clear the data (at 0x3FF for now)
	MOVLW 	b'00000100'			; Setup for EEData
	RCALL 	_StartWrite
	BCF		LED_PORT,6			; yellow LED off
	RESET
; *********************************************************
; This is the Selfcheck RESET command. This routine 
; resets the internal check registers, i.e. checksum and 
; self verify.
_SpecialCmdJp4
	MOVF		_bootSpcCmd, W 
	XORLW 	CMD_RST_CHKSM
	BNZ		_SpecialCmdJp1
	CLRF		_bootChksmH
	CLRF		_bootChksmL
	BCF		ERR_VERIFY		
	CLRF		_bootErrStat
	BRA		_CANMain
; RESET_CHKSM Command
; Reset chksum
; Clear the error verify flag

;This is the Test and Run command. The checksum is
; verified, and the self-write verification bit is checked. 
; If both pass, then the boot flag is cleared.
_SpecialCmdJp1
	MOVF		_bootSpcCmd, W	; RUN_CHKSM Command
	XORLW 	CMD_CHK_RUN 
	BNZ		_SpecialCmdJp3
	MOVF		_bootChkL, W		; Add the control byte
	ADDWF	 _bootChksmL, F
	BNZ		_SpecialCmdJp2
	MOVF		_bootChkH, W 
	ADDWFC	_bootChksmH, F
	BNZ		_SpecialCmdJp2
	BTFSC 	ERR_VERIFY			; Look for verify errors
	BRA		_SpecialCmdJp2

	BRA		_CANSendOK		;send OK message

_SpecialCmdJp2

	BRA		_CANSendNOK		; or send an error acknowledge

_SpecialCmdJp3
	MOVF		_bootSpcCmd, W	; RUN_CHKSM Command
	XORLW 	CMD_BOOT_TEST 
	BNZ		_CANMain
	BRA		_CANSendBoot

; *************************************************************
; This is a jump routine to branch to the appropriate memory access function.
; The high byte of the 24-bit pointer is used to determine which memory to
;	access. 
; All program memories (including Config and User IDs) are directly mapped. 
; EEDATA is remapped.
_DataReg
; *************************************************************
_SetPointers
	MOVF		_bootAddrU, W		; Copy upper pointer
	MOVWF 	TBLPTRU
	ANDLW 	0xF0	; Filter
	MOVWF 	WREG2
	MOVF		_bootAddrH, W		; Copy the high pointer
	MOVWF 	TBLPTRH
	MOVWF 	EEADRH
	MOVF		_bootAddrL, W		; Copy the low pointer
	MOVWF 	TBLPTRL
	MOVWF	EEADR
	BTFSS 	MODE_AUTO_INC	; Adjust the pointer if auto inc is enabled
	BRA		_SetPointersJp1
	MOVF		_bootCount, W		; add the count to the pointer
	ADDWF 	_bootAddrL, F
	CLRF		WREG
	ADDWFC	 _bootAddrH, F 
	ADDWFC	 _bootAddrU, F 

_SetPointersJp1			;?

_Decode
	MOVLW 	0x30
	CPFSLT 	WREG2
	BRA		_DecodeJp1

	BRA		_PMEraseWrite
	
_DecodeJp1
	MOVF		WREG2,W
	XORLW 	0x30
	BNZ		_DecodeJp2

	BRA		_CFGWrite 
_DecodeJp2
	MOVF		WREG2,W 
	XORLW 	0xF0
	BNZ		_CANMain
	BRA		_EEWrite

; Program memory < 0x300000
; Config memory = 0x300000
; EEPROM data = 0xF00000
; ************************************************************* 
; Function: VOID _PMRead()
;	VOID _PMEraseWrite ()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and
;	address of the source data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	RETURN when called. They have been written in a linear form to
;	save space.Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied.
; These are the program memory read/write functions. Erase is available
;	through control flags. An automatic erase option is also available.
; A write lock indicator is in place to ensure intentional write operations.
; Note: write operations must be on 8-byte boundaries and must be 8 bytes
;	long. Also erase operations can only occur on 64-byte boundaries.
; *************************************************************

_PMEraseWrite:
	BTFSS 	MODE_AUTO_ERASE
	BRA		_PMWrite
_PMErase:
	MOVF		TBLPTRL, W
	ANDLW	b'00111111'
	BNZ		_PMWrite
_PMEraseJp1
	MOVLW	b'10010100' 
	RCALL	_StartWrite 
_PMWrite:
	BTFSC 	MODE_ERASE_ONLY

	BRA		_CANMain 

	MOVF		TBLPTRL, W
	ANDLW	b'00000111'
	BNZ		_CANMain 
	MOVLW 	0x08
	MOVWF 	WREG1

_PMWriteLp1:						; Load the holding registers
	MOVF		POSTINC0, W 
	MOVWF 	TABLAT
	RCALL	_UpdateChksum 	; Adjust the checksum
	TBLWT*+
	DECFSZ	 WREG1, F
	BRA		_PMWriteLp1

#ifdef MODE_SELF_VERIFY 
	MOVLW	 0x08
	MOVWF 	WREG1 
_PMWriteLp2:
	TBLRD*-							; Point back into the block
	MOVF		POSTDEC0, W 
	DECFSZ	WREG1, F
	BRA		_PMWriteLp2
	MOVLW	b'10000100' 		; Setup writes
	RCALL	_StartWrite 			; Write the data
	MOVLW 	0x08
	MOVWF 	WREG1
_PMReadBackLp1:
	TBLRD*+							; Test the data
	MOVF		TABLAT, W 
	XORWF 	POSTINC0, W
	BTFSS	STATUS, Z
	BSF		ERR_VERIFY 
	DECFSZ 	WREG1, F
	BRA		_PMReadBackLp1	; Not finished then repeat
#else
	TBLRD*-							; Point back into the block
; Setup writes
	MOVLW 	b'10000100' 			; Write the data
	RCALL 	_StartWrite 			; Return the pointer position
	TBLRD*+
#endif

	BRA		_CANMain

; *************************************************************
 ; Function: VOID _CFGWrite()
;	VOID _CFGRead()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of the source data. 
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	RETURN when called. They have been written in a linear form to
;	save space. Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied.
;
;	These are the Config memory read/write functions. Read is
;	actually the same for standard program memory, so any read
;	request is passed directly to _PMRead.
;
; *************************************************************
_CFGWrite:

#ifdef MODE_SELF_VERIFY		; Write to config area
	MOVF		INDF0, W				; Load data
#else
	MOVF		POSTINC0, W
#endif
	MOVWF 	TABLAT
	RCALL 	_UpdateChksum		; Adjust the checksum
	TBLWT*							; Write the data
	MOVLW	b'11000100' 
	RCALL 	_StartWrite
	TBLRD*+						; Move the pointers and verify
#ifdef MODE_SELF_VERIFY 
	MOVF		TABLAT, W 
	XORWF 	POSTINC0, W

#endif
	DECFSZ 	WREG1, F
	BRA		_CFGWrite			; Not finished then repeat

	BRA		_CANMain 

; ************************************************************* 
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
;	RETURN when called. They have been written in a linear form to
;	save space. Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied.
;
;	This is the EEDATA memory read/write functions.
;
; *************************************************************

_EEWrite:

#ifdef MODE_SELF_VERIFY
	MOVF		INDF0, W
#else
	MOVF		POSTINC0, W 
#endif

	MOVWF 	EEDATA
	RCALL 	_UpdateChksum 
	MOVLW	b'00000100' 
	RCALL	_StartWrite

#ifdef MODE_SELF_VERIFY 
	CLRF		EECON1
	BSF		EECON1, RD
	NOP
	MOVF		EEDATA, W 
	XORWF 	POSTINC0, W
	BTFSS	STATUS, Z
	BSF		ERR_VERIFY
#endif

	INFSNZ	EEADR, F 
	INCF 		EEADRH, F 
	DECFSZ 	WREG1, F
	BRA		_EEWrite

	BRA		_CANMain 
	
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

; *************************************************************
; Function: VOID _CANSendAck()
;	VOID _CANSendResponce ()
;
; PreCondition:TXB0 must be preloaded with the data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	RETURN when called. They have been written in a linear form to
;	save space. Thus 'CALL' and 'RETURN' instructions are not
;	included, but rather they are implied. ;
;	These routines are used for 'talking back' to the source. The
;	_CANSendAck routine sends an empty message to indicate
;	acknowledgement of a memory write operation. The
;	_CANSendResponce is used to send data back to the source. ;
; *************************************************************

_CANSendMessage:
	BTFSC 	TXB0CON,TXREQ 
	BRA		$ - 2
	MOVLW 	CAN_TXB0SIDH 
	MOVWF 	TXB0SIDH
	MOVLW 	CAN_TXB0SIDL 
	MOVWF 	TXB0SIDL
	MOVLW 	CAN_TXB0EIDH 
	MOVWF 	TXB0EIDH	

	MOVLW	CAN_TXB0EIDL
	MOVWF	TXB0EIDL
	BSF		CANTX_CD_BIT
	BTFSS	CAN_CD_BIT 
	BCF		CANTX_CD_BIT
	BSF		TXB0CON, TXREQ
    	BRA	 	_CANMain			; Setup the command bit

_CANSendOK:						; send OK message 
	MOVLW	1						; a 1 is OK
	MOVWF	TXB0D0
	MOVWF	TXB0DLC
	BRA		_CANSendMessage
	
_CANSendNOK:						; send not OK message
	CLRF		TXB0D0				; a 0 is not OK
	MOVLW	1
	MOVWF	TXB0DLC
	BRA		_CANSendMessage

_CANSendBoot:
	MOVLW	2						 ; 2 is confirm boot mode
	MOVWF	TXB0D0
	MOVLW	1
	MOVWF	TXB0DLC
	BRA		_CANSendMessage
    
; Start the transmission

; 	End of bootloader

; *************************************************************

;	start vector

	ORG	0x0800
loadadr:
	NOP								; for debug
	GOTO		setup					; main setup sequence
	
	ORG	0x0808
	GOTO		hpint					; high priority interrupt

;	CBUS module info:
		
	ORG	0x0810					; node type parameters

myName	DB	"RCOM   "   

	ORG	0x0818	
	GOTO		lpint					; low priority interrupt
		
	ORG	0x0820					; these have to be here for FCU

nodeprm:
	DB		MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
	DB		MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN; Main parameters
	DW	RESET_VECT     			; Load address for module code above bootloader
	DW	0           					; Top 2 bytes of 32 bit address not used
	DB		0,0,0,0,CPUM_MICROCHIP,BETA_VER
sparprm:
     FILL 	0,prmcnt-$ 				; Unused parameter space set to zero

PRMCOUNT	EQU sparprm-nodeprm		; Number of parameter bytes implemented

	ORG 0x0838

prmcnt		DW  PRMCOUNT		; Number of parameters implemented
nodenam		DW  myName		; Pointer to module type name
				DW  0 				; Top 2 bytes of 32 bit address not used


PRCKSUM  	EQU MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA_VER

cksum       	DW  PRCKSUM     	; Checksum of parameters

;**************************************************************

	ORG		0x0840				; start of program

setup:
	CALL		setsub

	CALL		setup1

;		specific to CANRCOM

setid:	
	CLRF		EEADR
	CALL		newid					; put ID into Tx1buf, TXB2 and ID number store
	BTFSS	Datmode,2
	BRA		seten_f

	BRA		main

seten_f:
	BCF		LED_PORT,GREEN	; green off
	BTFSS	Datmode,2			; flashing?
		
	BSF		LED_PORT,YELLOW	; Yellow LED on.
	BCF		RXB0CON,RXFUL
	BCF		Datmode,0

;****************Get DN****************************************  

	MOVLW	LOW DevNo			; get detector number
	MOVWF	EEADR
	CALL		eeread				; get DNhi
	MOVWF	DNhi
	INCF		EEADR				; get DNlo

	CALL 		eeread
	MOVWF	DNlo

;***************Get NVs****************************************

	MOVLW	LOW NVstart			; layout setup routine 
	MOVWF	EEADR
	CALL		eeread				; get NV
	MOVWF	Nvtemp				; need to define NV and what it does
	INCF		EEADR
	CALL		eeread
	MOVWF	NV2_tmp

	GOTO		main
;**************************************************************
; High priotity interrupt. Here if gated pulse longer than rollover.
; Triggered by PIR1,TMR1IF.
; Ignores too short pulses and sets start of EUSART looking for Channel 1
; serial data'		
;**************************************************************

hpint:
	BCF		PIE1,TMR1IE			; no interrupt
	BCF		T1GCON,TMR1GE	; not gated
	BCF		T1CON,TMR1ON		; stop timer
	BTFSC	Nvtemp,1				; which channel?
	 BRA		ch1					; Channel 1
;	BTFSC		Nvtemp,2
;	 BRA		ch2					; Channel 2
	BRA		hi_loop1				; do nothing

;*********************Channel 1*********************************
	
ch1:
	CLRF		PIR1					; clear flags
	MOVLW	0xE4					; time for channel 1 data
	MOVWF	TMR1H
	MOVLW	0x34
	MOVWF	TMR1L
	BSF		LATC,5				; flag up  for scope timing test
	BSF		T1CON,TMR1ON		; restart for read
	BSF		RCSTA1,CREN		; start EUSART1
b1_loop:
	BTFSC	PIR1,TMR1IF			; time out?
	 BRA		end_loop
	MOVF		PORTC,W				; get orientation
	ANDLW	B'00000011'
	BZ			b1_loop1				; both low?
	IORWF	Ori						; save orientation bit.
	MOVLW	B'00000011'
	SUBWF	Ori,W
	BNZ		b1_loop1
	BCF		Ori,0
	BCF		Ori,1

;***************Receive 1st Datagram*****************************
b1_loop1:
	BCF		Loc_flgs,0
	BCF		Loc_flgs,1		
	BTFSC	Ori,0					; Move Orientation to Loc_flgs for transmission
	 BSF		Loc_flgs,0
	BTFSC		Ori,1
	 BSF		Loc_flgs,1
	BTFSS	PIR1,RC1IF			; any serial byte?
	BRA		b1_loop				; wait for now
	MOVF		RCREG1,W			; get serial byte (1st Datagram)
	MOVWF	Datagram1			; 1st datagram
	BSF		DgmFlags,0			; 1st byte flagged as received
	BCF		PIR1,RC1IF			; clear EUSART flag for next datagram
;***************Receive 2nd Datagram*****************************
b1_loop2:
	BTFSC	PIR1,TMR1IF			; time out?
	 BRA		end_loop
	BTFSS	PIR1,RC1IF			
	 BRA		b1_loop2				; wait for data 
	MOVF		RCREG1,W			; get serial byte (2nd Datagram)
	MOVWF	Datagram2			; 2nd datagram
	BSF		DgmFlags,1			; 2nd byte flagged as received
	BCF		PIR1,RC1IF			; clear EUSART flag
;***************Stop EUSART************************************	
end_loop:
	BTFSS	PIR1,TMR1IF			; time out?
	 BRA		end_loop
	BCF		LATC,5				; flag down. End of Channel 1 time
	BCF		RCSTA1,CREN		; stop EUSART
	BCF		T1CON,TMR1ON		; stop it 
	CLRF		PIR1
	MOVLW	0xB6					; reload
	MOVWF	TMR1H				
	MOVLW	0xA1
	MOVWF	TMR1L
	BSF		T1CON,TMR1ON		; start it
	MOVLW	0x03					; 0000 0011
	SUBWF	DgmFlags,W			; Number of datagrams in 1 DCC packet
	BNZ		hi_loop				; not got both bytes

;********Both Datagrams in check they are VALID Data**************** 
proc_data:	
	MOVLW	0x01					; Read type RailCom (b'0000-0001)
	MOVWF	reada					; Reader variable (bit0)
	MOVLW	1
	MOVWF	FSR0H					; setup for 4/8 testing
	MOVF		Datagram1,W		; 1st RAW Datagram
	MOVWF	FSR0L					; Find table entery
	MOVF		INDF0,W				; Copy table result to Wreg 
	BTFSC	WREG,7				; test if 4/8 data is valid
	 BRA		get_pkt				; invalid - abort
	MOVWF	Dgm1					; Datagram 1 decoded 

	MOVF		Datagram2,W		; 2nd RAW Datagram
	MOVWF	FSR0L					; Find table entery
	MOVF		INDF0,W				; Copy table result to Wreg
	BTFSC	WREG,7				; test if 4/8 data is valid
	 BRA		get_pkt				; invalid - abort
	MOVWF	Dgm2					; Datagram 2 decoded
	MOVLW	b'10000111'			; 
	MOVWF	IDcnt
	BTFSC	Dgm1,1
	 BSF		Dgm2,6
	BTFSC	Dgm1,1
	 BSF		Dgm2,7
	
;**************Get Identifier************************************

	RRNCF	Dgm1					; Remove bit 0 of Dgm1 
	RRNCF	Dgm1					; Remove bit 1 of Dgm1
	MOVF		Dgm1,W 
	ANDLW	B'00001111'			; Filter only identifier bits
	MOVWF	IDbyte				; IDbyte now only contains identifer

;***************Locate Identifier Data****************************	

	MOVLW	b'10000111'			; bit7 = Extra pass bit2 = ID3, bit1 = ID2 bit0 = ID1		
	MOVWF	IDcnt
	CALL 		tst_ident				; Test identifer byte
;***************Test if all data is Collected*************************
	MOVF		IDcnt,0
	SUBWF	Bflag,W				; got both datagrams
	BZ			snd_data				; send out data

;*****************GET NEXT PACKET****************************** 
get_pkt:
	NOP
	CLRF		DgmFlags				; Clear datagram flag indicators
	BRA		hi_loop				; back around the loop

;****************SEND DATA FOR TRANSMISSION********************
snd_data:
	BTFSS	Datmode,3			; is it running mode?
	 BRA		hi_loop 
	CALL		set_DDES				; create output frame and send
	CLRF		DgmFlags				; clear datagram flags
	BSF		Sflag,0				; Set send once flag (RailCom bit)

hi_loop:	
	BTFSS	PIR1,TMR1IF
	 BRA		hi_loop
		
hi_loop1:
	BCF		T1CON,TMR1ON		; stop it
	BCF		PIR1,TMR1IF
	
hi_loop2:
	MOVLW	0xF6					; RESET timer for short pulses
	MOVWF	TMR1H
	CLRF		TMR1L
	BCF		LATC,5				; clear anyway	
	
	BSF		T1GCON,TMR1GE	; gated
	BSF		T1GCON,T1GGO		; gate set
	BSF		PIE1,TMR1IE			; re-enable interrupts
	BSF		T1CON,TMR1ON		; start timer
	
	RETFIE	1						; Shadow regesters loaded - WS, STATUSS and BSR
;**********************IDENTIFY ID******************************
tst_ident:
	MOVF		IDbyte,0				; Move Identifier for testing
	SUBLW 	0x01
	BZ			CV17					; Extended address identified
	MOVF		IDbyte,0
	SUBLW	0x02
	BZ			CV1					; Primary address identified
	MOVF		IDbyte,0
	SUBLW	0x03
	BZ			info1					; Info1 data identified			
	RETURN

CV17:
	BTFSS	Bflag,7
	 BRA		x2Pass				; Bflag bit7 = 0 
	BSF		Bflag,1				; high address flag		
	BSF		Bflag,2				; temporary set ID3 flag
	MOVFF 	Dgm2,Adr1			; Move Dgm2 to Adr1 for transmission
	RETURN

;***********SETUP 2nd PASS for CV17****************************** 
x2Pass:
	BSF		Bflag,7				; Blag bit7 = 1
	RETURN

CV1:
	BSF		Bflag,0				; low address flag 
	MOVFF 	Dgm2,Adr2			; Move Dgm2 to Adr2 for transmission	
	RETURN

info1:
	BSF		Bflag,2			; decoder info1 flag		
	BSF		Bflag,7
	MOVFF	Dgm2,Info1		; Move Dgm2 to Ifor1 to be analysed
	CLRF		Loc_flgs			; Clear last DDES flags
;***************Orientation Correction*****************************
	BTFSC 	Info1,0			; Check Orientation correction
	 BRA		no_chg1			; Orientation as wired	
	BTFSC	Ori,0
	 BCF		Ori,0				; Ori,0 was 1		Orientation reversal
	BTFSC	Ori,1
	BSF		Ori,0				; Ori,1 was 0			  "			  "
no_chg1:
; ****************No change to Orentation*************************
	BTFSC	Ori,0				; Check detected locos Orientation  
	 BSF		Loc_flgs,0		; Copy Ori to Loc_flgs (reversed)
	BTFSC	Ori,1
	 BSF		Loc_flgs,1		;    "			  "	(reversed)
;******************Travel Direction*******************************
	BSF		Loc_flgs,2		; Fwd	(default)
	BCF		Loc_flgs,3		;   "
	BTFSS	Info1,1			; Travel direction
	 BRA		no_chg2			; Forward		
	BCF		Loc_flgs,2		; Rev
	BSF		Loc_flgs,3		;   "	
no_chg2:
;******************Driving Status********************************
	BSF		Loc_flgs,4		; Stationary	(default)
	BCF		Loc_flgs,5		;      "
	BTFSS	Info1,2			; Driving
	 BRA		no_chg3			; Stationary
	BCF		Loc_flgs,4		; Moving
	BSF		Loc_flgs,5		;    "	
no_chg3:
	RETURN

;**************************************************************
; low priority interrupt. Used for CAN transmit error / latency and 'busy' frame.
; Busy frame is a max priority, zero data frame, preloaded in TXB0.
; Latency count (number of tries to transmit) is preset in code to .10
	
	ORG 0x0B00

lpint:
	MOVWF	W_tempL				;save critical variables
	MOVFF	STATUS,St_tempL
	MOVFF	CANCON,TempCANCON
	MOVFF	CANSTAT,TempCANSTAT
	
	MOVFF	FSR0L,Fsr_temp0L	;save FSR0
	MOVFF	FSR0H,Fsr_temp0H
	MOVFF	FSR1L,Fsr_temp1L	;save FSR1
	MOVFF	FSR1H,Fsr_temp1H

	BTFSC	PIR5,ERRIF
	BRA		txerr					; transmit error?
	BTFSS	PIR5,FIFOWMIF		; FIFO error?
	BRA		no_fifo
	BCF		PIR5,FIFOWMIF		; clear FIFO flag
	BCF		PIR5,TXB0IF			; clear busy frame flag
	MOVLB	d'15'
	BSF		TXB0CON,TXREQ	; send busy frame
	BCF		PIE5,FIFOWMIE		; disable FIFO interrupt 
	BSF		PIE5,TXB0IE			; enable IRQ for busy frame sent
	MOVLB	0
	BRA		back1	
	
no_fifo:
	BCF		PIR5,TXB0IF			; clear busy frame flag
	BCF		PIE5,TXB0IE			; no busy frame IRQ
	BSF		PIE5,FIFOWMIE		; wait for next FIFO IRQ
	BRA		back1
		
;*** Transmit error routine here. Only acts on lost arbitration************	

txerr:
	MOVLB	d'15'					; change bank			
	BTFSS	TXB1CON,TXLARB
	BRA		errbak					; not lost arb.
	
	MOVF		Latcount,F			; is it already at zero?
	BZ			errbak
	DECFSZ	Latcount,F
	BRA		errbak
	BCF		TXB1CON,TXREQ
	MOVLW	B'00111111'
	ANDWF	TXB1SIDH,F			; change priority
txagain:
 	BSF		TXB1CON,TXREQ	; try again
					
errbak:
	BCF		RXB1CON,RXFUL
	MOVLB	0
	BCF		RXB0CON,RXFUL	; ready for next
		
	BCF		COMSTAT,RXB0OVFL; clear overflow flags if set
	BCF		COMSTAT,RXB1OVFL		
	BRA		back1
		
back:
	BCF		RXB0CON,RXFUL	; ready for next
	
back1:
	CLRF		PIR5					; clear all flags
	MOVF		CANCON,W			; recover variables
	ANDLW	B'11110001'
	IORWF	TempCANCON,W
		
	MOVWF	CANCON
	MOVFF	PCH_tempH,PCLATH
	MOVFF	Fsr_temp0L,FSR0L	; recover FSR0
	MOVFF	Fsr_temp0H,FSR0H

	MOVFF	Fsr_temp1L,FSR1L	; recover FSR1
	MOVFF	Fsr_temp1H,FSR1H
	MOVF		W_tempL,W
	MOVFF	St_tempL,STATUS	
		
	RETFIE	
	
;**********Main program loop*************************************

main:
	BTFSC	COMSTAT,7			; any CAN frame?
	 CALL 	getcan
	BTFSC	Datmode,0
	 CALL		packet

; ********Do the CAN message***********************************

	BCF		Datmode,0
	BCF		RXB0CON,RXFUL	; clear CAN RX buffer for next
	BTFSC	S_PORT,S_BIT		; Push button pressed ?
	 BRA		main1					; No
	CALL		set1					; Yes so cancel
	SUBLW	0
	BZ			main1
	CALL		setup1				; out of main and back to SLiM
	BRA		main
		
main1:	
	BTFSC	T1GCON,T1GGO		; has got a pulse?
	BRA		main2
	BCF		T1CON,TMR1ON		; stop timer
		
	CALL		occ_snd				; send an occupied frame
	CLRF		T4count	
	MOVF		NV2_tmp,W
	MOVWF	NV2		
	BRA		main3

main2:
	BTFSS	PIR4,TMR4IF
	 BRA		main
	BCF		PIR4,TMR4IF
	BTG		LATC,3				; timer flag
	DECFSZ	T4count
	 BRA		main
	DECFSZ	NV2					; unocc  delay in NV2
	BRA		main
	BTFSS	Sflag,2				; has sent? (Unoccupied)
	 CALL		un_occ				; send unoccupied
	BRA		main3

;*******RESET timer for short pulses*******************************

main3:	
	BCF		T1CON,TMR1ON		; stop timer
	BCF		PIR1,TMR1IF			; clear flag
	MOVLW	0xF6					; timer 1 set
	MOVWF	TMR1H
	CLRF		TMR1L
	
	MOVLW	B'11011000'
	MOVWF	T1GCON				; set timer 1 for gated mode
	BSF		PIE1,TMR1IE			; enable interrupt
	BSF		T1CON,TMR1ON		; restart timer

	BRA		main

;********Start of subroutines**************************************

; read a EEPROM byte, EEADR  must be set before this sub.
		
eeread:
	BCF		EECON1,EEPGD	
	BCF		EECON1,CFGS		
	BSF		EECON1,RD
	NOP	
	NOP								; needed for K series PICs
	MOVF		EEDATA,W			; returns with data in W
	RETURN

;*****************************************************************
; write to EEPROM, EEADR must be set before this sub.
; data to write in W

eewrite:
	MOVWF	EEDATA		
	BCF		EECON1,EEPGD	
	BCF		EECON1,CFGS
	BSF		EECON1,WREN
	MOVFF	INTCON,TempINTCON
		
	CLRF		INTCON				; disable interrupts
	MOVLW	0x55
	MOVWF	EECON2
	MOVLW	0xAA
	MOVWF	EECON2
	BSF		EECON1,WR
; Test Eeprom was sucessfully written to.
eetest:
	BTFSC	EECON1,WR			; check it has written
	BRA		eetest
	BCF		PIR2,EEIF
	BCF		EECON1,WREN
	MOVFF	TempINTCON,INTCON		; reenable interrupts
	RETURN	

getcan:
	MOVF		CANCON,W			; look for a CAN frame. Used in main outine
	ANDLW	B'00001111'
	MOVWF	TempCANCON
	MOVF		ECANCON,W			; uses the ECAN in mode 2
	ANDLW	B'11110000'
	IORWF	TempCANCON,W
	MOVWF	ECANCON
	BTFSC	RXB0SIDL,EXID		; ignore extended frames here
	BRA		no_can
	BTFSS	RXB0DLC,RXRTR	; is it RTR?
	BRA		get_3
	CALL		isRTR
	BRA		no_can
		
get_3:
	MOVF		RXB0DLC,F
	BNZ		get_2					; ignore zero length frames
	BRA		no_can 
get_2:
	BSF		Datmode,0			; valid message frame. Datmode bit 0 indicates a CAN frame waiting.
	CALL		copyev				; save to buffer. Saves OpCode and 4 bytes.	
	RETURN

no_can:
	BCF		RXB0CON,RXFUL	; no valid CAN frame so clear for next.
	RETURN

;***********************************************************************

;********************************************************************
; main packet handling is here
; add more commands for incoming frames as needed
; test OPC and branch if match to 'opc_x'
; at opc_x, GOTO opc routine which is a sub and returns to 
; code that called 'packet'.
		
packet:
	MOVLW	OPC_ASRQ			; request state of block
	SUBWF	ev_opc,W
	BZ			asrq_x
	
;	now all other OPCs
		
	MOVLW	OPC_BOOT			; reboot
	SUBWF	ev_opc,W
	BZ			boot_x
	MOVLW	OPC_ENUM			; re-enumerate
	SUBWF	ev_opc,W
	BZ			enum_x
	MOVLW	OPC_RQNPN
	SUBWF	ev_opc,W
	BZ			rqnpn_x				; read individual parameters

	MOVLW	OPC_CANID			; force new CAN_ID
	SUBWF	ev_opc,W
	BZ			canid_x
	MOVLW	OPC_SNN				; set NN 
	SUBWF	ev_opc,W
	BZ			snn_x
	MOVLW	OPC_QNN			; QNN
	SUBWF	ev_opc,w
	BZ			qnn_x
	MOVLW	OPC_RQNP					
	SUBWF	ev_opc,W
	BZ			rqnp_x				; read node parameters
	MOVLW	OPC_RQMN		
	SUBWF	ev_opc,w
	BZ			rqmn_x				; read module name		
	MOVLW	OPC_NNLRN			; set to learn mode 
	SUBWF	ev_opc,W
	BZ			nnlrn_x		
	MOVLW	OPC_NNULN			; clear learn mode 
	SUBWF	ev_opc,W
	BZ			nnuln_x
	MOVLW	OPC_NNCLR			; clear all events on 0x55
	SUBWF	ev_opc,W
	BZ			nnclr_x
	MOVLW	OPC_NNEVN			; read number of events left
	SUBWF	ev_opc,W
	BZ			nnevn_x
	MOVLW	OPC_EVLRN			; is it set event?
	SUBWF	ev_opc,W
	BZ			evlrn_x				; do learn
	MOVLW	OPC_REVAL
	SUBWF	ev_opc,W
	BZ			reval_x
	MOVLW	OPC_EVULN			; is it unset event
	SUBWF	ev_opc,W			
	BZ			evuln_x
	MOVLW	OPC_REQEV			; read event variables
	SUBWF	ev_opc,W
	BZ			reqev_x
	MOVLW	OPC_NVSET			; set NV
	SUBWF	ev_opc,W
	BZ			nvset_x
	MOVLW	OPC_NVRD			; read NVs
	SUBWF	ev_opc,W
	BZ			nvrd_x
	
	MOVLW	OPC_NERD			; is it read events
	SUBWF	ev_opc,W
	BZ			nerd_x

	MOVLW	OPC_RQEVN
	SUBWF	ev_opc,W
	BZ			rqevn_x

	MOVLW	OPC_RQDDS			; request DDES data
	SUBWF	ev_opc,W
	BZ			rqdds_x
	RETURN							; no match of OPC

; Too long for a branch so use gotos

asrq_x:
	GOTO 	asrq
snn_x:
	GOTO		snn
boot_x:
	GOTO 	boot
enum_x:
	GOTO 	enum

rqnpn_x:
	GOTO		rqnpn
canid_x:
	GOTO		canid
rqnp_x:
	GOTO		rqnp
rqmn_x:
	GOTO		rqmn
nnlrn_x:
 	GOTO		nnlrn
nnuln_x:
	GOTO		nnuln
evlrn_x:
	GOTO		evlrn

reval_x:
	GOTO		reval
rqevn_x:
	GOTO		rqevn
nnclr_x:
	GOTO		nnclr
nvset_x:
	GOTO 	nvset
nvrd_x:
	GOTO		nvrd
nnevn_x:
	GOTO		nnevn
rqdds_x:
	GOTO		rqdds

qnn_x:
	GOTO		qnn

evuln_x:
	GOTO		evuln
reqev_x:
	GOTO		reqev

nerd_x:
	GOTO		nerd

;**********Read 4/8 table from EEPROM to ram**************************

read_LUT:
	MOVLW	0						; number in Table
	MOVWF	Count
	LFSR		FSR0,Lut				; Table in ram
	CLRF		EEADRH				; after boot test
	BSF		EEADRH,0			; Table is in page 1 of EEPROM
	CLRF		EEADR
	DECF		EEADR				; one less to allow for the inc.
table_loop:
	INCF		EEADR	
	CALL		eeread
	MOVWF	POSTINC0
	DECFSZ	Count					; end?
	BRA		table_loop
	CLRF		EEADRH				; back to page 0
	RETURN

;******Create DDES data*****************************************

set_DDES:
	MOVF		DNhi,F				; is DNhi = 0
	BNZ		set_DDES1			; send
	MOVF		DNlo,F				; is DNlo = 0
	BNZ		set_DDES1			; send
	RETLW	1						; invalid DN - don't send
	
set_DDES1:	
	BTFSC	Sflag,0				; Test send RailCom once flag (RailCom)
	 RETURN						; already sent - so RETURN
	MOVLW	OPC_DDES			; Set up CBUS frame to send
	MOVWF	Tx1d0
	MOVFF	DNhi,Tx1d1			; DN high
	MOVFF	DNlo,Tx1d2			; DN low
	MOVFF	reada,Tx1d3			; hi nibble = msg definition (0) lo nibble = 1 (RailCom)
	MOVFF	Adr1,Tx1d4			; loco address hi byte
	MOVFF	Adr2,Tx1d5			; loco address lo byte
	MOVFF	Lspd,Tx1d6			; Loco speed
	MOVFF	Loc_flgs,Tx1d7		; Loco info. 7 Sp, 6 Sp, 5-4 Drive status, 
										; 			   3-2 Direction Travel 1-0 Ori
	MOVLW	d'8'
	MOVWF	Dlc
	BTFSS	Sflag,0				; Test send RailCom once flag (RailCom)
	 CALL		sendTXb
	BSF		Sflag,0				; Set RailCom send once flag 
	CLRF		DgmFlags				; clear datagram flags
	RETURN

;********Utility routines used by various subroutines******************
;		Part of event learning.

learn2:
;	BTFSS		Mode,1				; FLiM?
	BTFSS	Datmode,3
	 RETURN	
	BTFSC	Datmode,5
	 BRA		unlearn
	MOVF		RXB0D6,W			; get which detector
	DECF		WREG					; starts at 0. Needs check here?
	RLNCF	WREG					; two bytes per detector
	MOVWF	EEADR				; set to write
	MOVLW	LOW DevNo
	ADDWF	EEADR
	MOVF		RXB0D3,W			; upper byte of DN
	MOVWF	DNhi					; save it
	CALL		eewrite				; put in
	INCF		EEADR
	MOVF		RXB0D4,W			; next byte
	MOVWF	DNlo
	CALL		eewrite

l_out1:
	BCF		Datmode,6
l_out2:
	BCF		RXB0CON,RXFUL
	MOVLW	OPC_WRACK
	CALL		nnrel					; send WRACK
	BCF		Datmode,0
	RETURN

;**********Unlearn an event  (there is only one)*********************

unlearn:
	MOVLW	LOW DevNo
	MOVWF	EEADR
	MOVLW	0
	MOVWF	DNhi					; save it
	CALL		eewrite				; put in
	INCF		EEADR
	MOVLW	0						; next byte
	MOVWF	DNlo
	CALL		eewrite
	BCF		Datmode,5   		; clear unlearn
	BRA		l_out2					; back

; *********Set node to learn mode*********************************

setlrn:
	CALL		thisNN				; set to learn mode
	SUBLW	0
	BNZ		setl_1
	BSF		Datmode,4
setl_1:
	RETURN							; not this NN

;********Check if command is for this node**************************

thisNN:
	MOVF		NN_temph,W
	SUBWF	RXB0D1,W
	BNZ		not_NN
	MOVF		NN_templ,W
	SUBWF	RXB0D2,W
	BNZ		not_NN
	RETLW 	0						; returns 0 if match
not_NN:
	RETLW	1
		
;*****Request frame for new NN or ack if not virgin********************
		
nnreq:
	MOVLW	OPC_RQNN		
nnrel:
	MOVWF	Tx1d0
	MOVFF	NN_temph,Tx1d1
	MOVFF	NN_templ,Tx1d2
	MOVLW	d'3'
	MOVWF	Dlc
	CALL		sendTX
	RETURN

;******Send contents of Tx1 buffer via CAN TXB1*********************

sendTX1:
	MOVFF	FSR1L,Fsr_temp1L	; save FSRs
	MOVFF	FSR1H,Fsr_temp1H
	MOVFF	FSR0L,Fsr_temp0L
	MOVFF	FSR0H,Fsr_temp0H

	LFSR		FSR0,Tx1con			; shift buffer to TXB1 registers
	LFSR		FSR1,TXB1CON
		
	MOVLB	d'15'					; check for buffer access
ldTX2:
	BTFSC	TXB1CON,TXREQ	; Tx buffer available...?
	BRA		ldTX2					; not yet  so loop
	MOVLB	0
		
ldTX1:
	MOVF		POSTINC0,W
	MOVWF	POSTINC1			; load TXB1 registers
	MOVLW	Tx1d7+1
	CPFSEQ	FSR0L
	BRA		ldTX1
		
	MOVLB	d'15'					; bank 15
tx1test:
	BTFSC	TXB1CON,TXREQ	; test if clear to send
	BRA		tx1test
	BSF		TXB1CON,TXREQ	; OK so send
		
tx1done:
	MOVLB	0						; bank 0
	MOVFF	Fsr_temp1L,FSR1L	; recover FSRs
	MOVFF	Fsr_temp1H,FSR1H
	MOVFF	Fsr_temp0L,FSR0L
	MOVFF	Fsr_temp0H,FSR0H
	RETURN							; successful send

;*******As sendTX1 bur wait till sent before returning*****************

sendTX2:
	MOVFF	FSR1L,Fsr_temp1L	; save FSRs
	MOVFF	FSR1H,Fsr_temp1H
	MOVFF	FSR0L,Fsr_temp0L
	MOVFF	FSR0H,Fsr_temp0H

	LFSR		FSR0,Tx1con			; shift buffer to TXB1 registers
	LFSR		FSR1,TXB1CON
		
	MOVLB	d'15'					; check for buffer access
ldTX2a:
	BTFSC	TXB1CON,TXREQ	; Tx buffer available...?
	BRA		ldTX2a				;... not yet  so loop
	MOVLB	0
		
ldTX2b:
	MOVF		POSTINC0,W
	MOVWF	POSTINC1			; load TXB1 registers
	MOVLW	Tx1d7+1
	CPFSEQ	FSR0L
	BRA		ldTX2b
		
	MOVLB	d'15'					; bank 15
tx2test:
	BTFSC	TXB1CON,TXREQ	; test if clear to send
	BRA		tx2test
	BSF		TXB1CON,TXREQ	; OK so send

tx2c:
	BTFSS	TXB1CON,TXBIF		; has it sent?
	BRA		tx2c					; no so wait
	BCF		TXB1CON,TXBIF
		
tx2done:
	MOVLB	0						; bank 0
	MOVFF	Fsr_temp1L,FSR1L	; recover FSRs
	MOVFF	Fsr_temp1H,FSR1H
	MOVFF	Fsr_temp0L,FSR0L
	MOVFF	Fsr_temp0H,FSR0H
	RETURN							; successful send

;********Send a CAN frame***************************************
; entry at sendTX puts the current NN in the frame - for producer events
; entry at sendTXa needs Tx1d1 and Tx1d2 setting first
; Latcount is the number of CAN send retries before priority is increased
; the CAN_ID is pre-loaded in the Tx1 buffer 
; Dlc must be loaded by calling source to the data length value
		
sendTX:
	MOVFF	NN_temph,Tx1d1	; get NN
	MOVFF	NN_templ,Tx1d2

sendTXa:
	MOVF		Dlc,W					; get data length
	MOVWF	Tx1dlc
	MOVLW	B'00001111'			; clear old priority
	ANDWF	Tx1sidh,F
	MOVLW	B'10110000'
	IORWF	Tx1sidh				; low priority
	MOVLW	d'10'
	MOVWF	Latcount
	CALL		sendTX1				; send frame
	RETURN			

sendTXb:
	MOVF		Dlc,W					; get data length
	MOVWF	Tx1dlc
	MOVLW	B'00001111'			; clear old priority
	ANDWF	Tx1sidh,F
	MOVLW	B'10110000'
	IORWF	Tx1sidh				; low priority
	MOVLW	d'10'
	MOVWF	Latcount
	CALL		sendTX2				; send frame and wait
	RETURN			

;********Put in NN from command*********************************

putNN:
	MOVFF	RXB0D1,NN_temph	; get new Node Number
	MOVFF	RXB0D2,NN_templ
	MOVLW	LOW NodeID			; put in EEPROM
	MOVWF	EEADR
	MOVF		RXB0D1,W
	CALL		eewrite
	INCF		EEADR
	MOVF		RXB0D2,W
	CALL		eewrite
	MOVLW	Modstat				; set module status to running mode and store it.
	MOVWF	EEADR
	MOVLW	B'00001000'			; Module status has NN set  (bit 3)
	MOVWF	Datmode				; running mode
	CALL		eewrite				; save mode
	RETURN

;***************************************************************************	

newid:
	MOVLW	LOW CANid			; put new CAN_ID etc in EEPROM
	MOVWF	EEADR
	CALL		eeread
	MOVWF	CanID_tmp			
	CALL		shuffle				; rearrange bits so there is a single CAN_ID byte
	MOVLW	B'11110000'
	ANDWF	Tx1sidh
	MOVF		IDtemph,W			; set current ID into CAN buffer
	IORWF	Tx1sidh				; leave priority bits alone
	MOVF		IDtempl,W
	MOVWF	Tx1sidl				; only top three bits used
	MOVLW 	Modstat
	MOVWF	EEADR
	CALL		eeread
	MOVWF	Datmode
	MOVLW	LOW DevNo
	MOVWF	EEADR
	CALL		eeread
	MOVWF	DNhi
	INCF		EEADR
	CALL		eeread
	MOVWF	DNlo
	MOVLW	LOW NodeID
	MOVWF	EEADR
	CALL		eeread
	MOVWF	NN_temph			; get stored NN
	INCF		EEADR
	CALL		eeread
	MOVWF	NN_templ
	MOVLW	LOW NVstart
	MOVWF	EEADR
	CALL		eeread
	MOVWF	Nvtemp
		
	MOVLB	d'15'	
; *********Put CAN_ID into TXB2 for enumeration response to RTR******
new_1:
	BTFSC	TXB2CON,TXREQ
	BRA		new_1
	CLRF		TXB2SIDH
	MOVF		IDtemph,W
	MOVWF	TXB2SIDH
	MOVF		IDtempl,W
	MOVWF	TXB2SIDL
	MOVLW	0xB0
	IORWF	TXB2SIDH			; set priority
	CLRF		TXB2DLC				; no data, no RTR
	RETURN

;**shuffle for standard ID. Puts 7 bit ID into IDtemph and IDtempl for CAN frame**	

shuffle:
	MOVFF	CanID_tmp,IDtempl; get 7 bit ID
	SWAPF	IDtempl,F
	RLNCF	IDtempl,W
	ANDLW	B'11100000'
	MOVWF	IDtempl				; has sidl
	MOVFF	CanID_tmp,IDtemph
	RRNCF	IDtemph,F
	RRNCF	IDtemph,F
	RRNCF	IDtemph,W
	ANDLW	B'00001111'
	MOVWF	IDtemph				; has sidh
	RETURN

;*******Reverse shuffle for incoming ID. sidh and sidl into one byte******

shuffin:
	MOVFF	RXB0SIDL,IDtempl
	SWAPF	IDtempl,F
	RRNCF	IDtempl,W
	ANDLW	B'00000111'
	MOVWF	IDtempl
	MOVFF	RXB0SIDH,IDtemph
	RLNCF	IDtemph,F
	RLNCF	IDtemph,F
	RLNCF	IDtemph,W
	ANDLW	B'01111000'
	IORWF	IDtempl,W			; returns with ID in W
	RETURN

;***********Self enumeration as separate subroutine*****************

self_en:
	MOVFF	FSR1L,Fsr_tmp1Le	; save FSR1 just in case
	MOVFF	FSR1H,Fsr_tmp1He 
	BSF		Datmode,1			; set to 'setup' mode
	MOVLW	d'14'
	MOVWF	Count
	LFSR		FSR0, Enum0
clr_en:
	CLRF		POSTINC0
	DECFSZ	Count
	BRA		clr_en

	MOVLW	0x00					; set T0 to 128 mSec (may need more?)
	MOVWF	TMR0H				; this waits till all other nodes have answered with their CAN_ID
	MOVLW	0x00
	MOVWF	TMR0L
	MOVLW	B'00000100'			; clock div  32 (0.5 uSec clock)									
	MOVWF	T0CON				; enable timer 0
	BSF		T0CON,TMR0ON
	BCF		INTCON,TMR0IF
; now send an RTR frame so all other nodes will send their CAN_IDs
	MOVLB	d'15'
	MOVLW	B'10111111'			; fixed node, default ID  
	MOVWF	TXB1SIDH
	MOVLW	B'11100000'
	MOVWF	TXB1SIDL
	MOVLW	B'01000000'			; RTR frame
	MOVWF	TXB1DLC
rtr_snd:
	BTFSC	TXB1CON,TXREQ
	BRA		rtr_snd
	BSF		TXB1CON,TXREQ
rtr_go:
	BTFSC	TXB1CON,TXREQ	; wait till sent
	BRA		rtr_go
	CLRF		TXB1DLC				; no more RTR frames
	MOVLB	0
				
;*********Wait for answers***************************************	

self_en1:
	BTFSC	INTCON,TMR0IF		; setup timer out?
	BRA		en_done
	BTFSC	COMSTAT,7			; look for CAN input. 
	BRA		getcan1
	BRA		self_en1				; no CAN
	
getcan1:
	MOVF		CANCON,W			; process answers
	ANDLW	B'00001111'
	MOVWF	TempCANCON
	MOVF		ECANCON,W
	ANDLW	B'11110000'
	IORWF	TempCANCON,W
	MOVWF	ECANCON
	BTFSC	RXB0SIDL,EXID		; ignore extended frames here
	BRA		no_can1
		
en_1:
	BTFSS	Datmode,1			; setup mode?
	BRA		no_can1				; must be in setup mode
	MOVF		RXB0DLC,F
	BNZ		no_can1				; only zero length frames
	CALL		setmode
	BRA		no_can1	

no_can1:
	BCF		RXB0CON,RXFUL
	BRA		self_en1				; loop till timer out 

en_done:
	BCF		T0CON,TMR0ON		; timer off
	BCF		INTCON,TMR0IF		; clear flag

;*******Now sort out the new CAN_ID******************************

	CLRF		IDcount
	INCF		IDcount,F				; ID starts at 1
	CLRF		Roll
	BSF		Roll,0
	LFSR		FSR1,Enum0			; set FSR to start
here1:
	INCF		INDF1,W				; find a space
	BNZ		here
	MOVLW	8
	ADDWF	IDcount,F
	INCF		FSR1L
	BRA		here1
here:
	MOVF		Roll,W
	ANDWF	INDF1,W
	BZ			here2
	RLCF		Roll,F
	INCF		IDcount,F
	BRA		here
here2:
	MOVLW	d'100'					; limit to ID
	CPFSLT	IDcount
	BRA		segful					; segment full
		
here3:
	MOVLW	LOW CANid			; put new ID in EEPROM
	MOVWF	EEADR
	MOVF		IDcount,W
	CALL		eewrite
	MOVF		IDcount,W
	CALL		newid					; put new ID in various buffers

			
	MOVFF	Fsr_tmp1Le,FSR1L
	MOVFF	Fsr_tmp1He,FSR1H 
	RETURN							; finished	

segful:
	MOVLW	d'7'					; segment full, no CAN_ID allocated
	CALL		errsub	;error
	SETF		IDcount
	BCF		IDcount,7
	BRA		here3

;********************************************************

isRTR:
	BTFSC	Datmode,1			; setup mode?
	RETURN							; back

	BTFSS	Datmode,3			; FLiM?
	RETURN
	MOVLB	d'15'
isRTR1:
	BTFSC	TXB2CON,TXREQ	
	BRA		isRTR1		
	BSF		TXB2CON,TXREQ	; send ID frame - preloaded in TXB2

	MOVLB	0
	RETURN

;****************************************************************
;
setmode:
	TSTFSZ	RXB0DLC
	RETURN							; only zero length frames for setup
		
	SWAPF	RXB0SIDH,W			; get ID into one byte
	RRCF		WREG
	ANDLW	B'01111000'			; mask
	MOVWF	Temp
	SWAPF	RXB0SIDL,W
	RRNCF	WREG
	ANDLW	B'00000111'
	IORWF	Temp,W
	MOVWF	IDcount				; has current incoming CAN_ID

	LFSR		FSR1,Enum0			; set enum to table
enum_st:
	CLRF		Roll					; start of enum sequence
	BSF		Roll,0
	MOVLW	d'8'
enum_1:
	CPFSGT	IDcount
	BRA		enum_2
	SUBWF	IDcount,F				; subtract 8
	INCF		FSR1L					; next table byte
	BRA		enum_1
enum_2:
	DCFSNZ	IDcount,F
	BRA		enum_3
	RLNCF	Roll,F
	BRA		enum_2
enum_3
	MOVF		Roll,W
	IORWF	INDF1,F
	BCF		RXB0CON,RXFUL	; clear read
	RETURN

;********Force self enumeration***********************************

enum:
	CALL		thisNN
	SUBLW	0
	BNZ		notNN1				; not there
	CALL		self_en				; do self enum
	MOVLW	OPC_NNACK
	CALL		nnrel					; send confirm frame
	BCF		RXB0CON,RXFUL
	MOVLW	B'00001000'			; back to normal running
	MOVWF	Datmode
notNN1:
	RETURN

;**************************************************************

snn:
	BTFSS	Datmode,2			; in NN set mode?
	RETURN							; no
	CALL		putNN					; put in NN
	MOVLW	Modstat
	MOVWF	EEADR
	MOVLW	B'00001000'
	CALL		eewrite				; set to normal status
	BCF		Datmode,1			; out of setup
	BCF		Datmode,2
	BSF		Datmode,3			; run mode
	
	MOVLW	d'10'
	MOVWF	Keepcnt				; for keep alive
	MOVLW	OPC_NNACK
	CALL		nnrel 					; confirm NN set
startNN:
	BSF		LED_PORT,YELLOW	; LED ON
	BCF		LED_PORT,GREEN	; LED off
	RETURN

;******Start bootloader******************************************	

boot:	
	CALL		thisNN
	SUBLW	0
	BNZ		retboot				; not there
		
reboot1:
	MOVLW	0xFF
	MOVWF	EEADR
	MOVLW	0x3F
	MOVWF	EEADRH
	MOVLW	0xFF
	CALL		eewrite				; set last EEPROM byte to 0xFF
	RESET							; software RESET to bootloader
										; should clear RETURN stack

retboot:
	RETURN	

;****************************************************************

rqnpn:	
	CALL		thisNN				; read parameter by index
	SUBLW	0
	BNZ		npnret
	CALL		para1rd
npnret:
	RETURN

;********Send module name - 7 bytes******************************

rqmn:
	BTFSS	Datmode,2			; setup mode only
	RETURN
	MOVLW	OPC_NAME
	MOVWF	Tx1d0
	MOVLW	LOW myName
	MOVWF	TBLPTRL
	MOVLW	HIGH myName
	MOVWF	TBLPTRH				; relocated code
	LFSR		FSR0,Tx1d1
	MOVLW	d'7'
	MOVWF	Count
	BSF		EECON1,EEPGD
		
name1:
	TBLRD*+
	MOVFF	TABLAT,POSTINC0
	DECFSZ	Count
	BRA		name1
	BCF		EECON1,EEPGD	
	MOVLW	d'8'
	MOVWF	Dlc
	CALL		sendTXa
	RETURN

;********Set a CAN_ID*******************************************

canid:
	CALL		thisNN
	SUBLW	0
	BNZ		canret					; abort
	MOVFF	RXB0D3,IDcount
	CALL		here2					; put in as if it was enumerated
	MOVLW	OPC_NNACK
	CALL		nnrel					; acknowledge new CAN_ID
canret:
	RETURN

;**************************************************************

rqnp:
	BTFSS	Datmode,2			; only in setup mode
	RETURN
	CALL		parasend
	RETURN

;******Learn a NN***********************************************

nnlrn:
	CALL		thisNN
	SUBLW	0
	BNZ		nnlret					; abort
	BSF		Datmode,4

nnlret:
	RETURN

;******Unlearn a NN*********************************************

nnuln:
	CALL		thisNN
	SUBLW	0
	BNZ		notret
	BCF		Datmode,4
	BCF		Datmode,7			; OK to scan
		
notln1:								; leave in learn mode
	BCF		Datmode,5
	
notret:
	RETURN
	
;*****Clear a node.  Must be in learn mode for safety******************

nnclr:
	CALL		thisNN
	SUBLW	0
	BNZ		clrret
	BTFSS	Datmode,4
	BRA		clrerr
	CALL		clr_sub
		
	MOVLW	OPC_WRACK
	CALL		nnrel					; send WRACK
	BRA		notln1
clrret:
	RETURN

clrerr:
	MOVLW	d'2'					; not in learn mode
	CALL		errmsg
	RETURN

;*******Set a NV************************************************

nvset:
	CALL		thisNN
	SUBLW	0
	BNZ		notnv					; not this node
	CALL		putNV
	MOVLW	OPC_WRACK
	CALL		nnrel					; send WRACK
notnv:
	RETURN

;*******************Learn an event*******************************

evlrn:
	BTFSS	Datmode,4			; is in learn mode?
	RETURN							; RETURN if not
	MOVF		EVidx,w				; check EV index
	BZ			noEV1
	MOVLW	EV_NUM+1
	CPFSLT	EVidx
	BRA		noEV1
	BRA		learn2

noEV1:
	MOVLW	d'6'
	CALL		errmsg
	RETURN

;*****************Unlearn an event*******************************

evuln:	
	BTFSS	Datmode,4
	RETURN							; prevent error message
	BSF		Datmode,5
	BRA		learn2	

;****************Error message send******************************

errmsg:
	CALL		errsub
	BRA		errret
errmsg1:
	CALL		errsub
	BCF		Datmode,6
	BRA		errret
errmsg2:
	CALL		errsub
		
errret:
	CLRF		PCLATH
	RETURN		

errsub:
	MOVWF	Tx1d3					; main error message send. Error no. in WREG
	MOVLW	OPC_CMDERR
	MOVWF	Tx1d0
	MOVLW	d'4'
	MOVWF	Dlc
	CALL		sendTX
	RETURN

;**************************************************************

rqevn:
	CALL		thisNN				; read event numbers
	SUBLW	0
	BNZ		rqevret
	CALL		rqevn1				; send event number
rqevret:
	RETURN

;****************Sends number of events**************************

rqevn1:
	MOVLW	1						; always 1 event
	MOVWF	Tx1d3
	MOVLW	OPC_NUMEV			; number of events
	MOVWF	Tx1d0
	MOVLW	d'4'
	MOVWF	Dlc
	CALL		sendTX	
	RETURN

;**********Called from REVAL. Sends EV by index********************
 
evsend:
	MOVLW	OPC_NEVAL			; 0xB5
	MOVWF	Tx1d0
	MOVFF	ENidx,Tx1d3			; event index (1 to 4)
	MOVFF	EVidx,Tx1d4			; event variable index. Always 1
	MOVFF	ENidx,Tx1d5			; EV value is always the event index
	MOVLW	d'6'
	MOVWF	Dlc
	CALL		sendTX
	RETURN

notEV:
	MOVLW	d'6'					; invalid EN#
	CALL		errsub
	RETURN

;*********Clears detector DN and restores defaults*******************

clr_sub:
	MOVLW	LOW DevNo
	MOVWF	EEADR
	MOVLW	1						; 2 bytes
	MOVWF	Count					; 1st byte
clrsub1:
	MOVLW	0
	CALL		eewrite			
	INCF		EEADR,F
	DECFSZ	Count
	BRA		clrsub1				; loop till done
	MOVLW	LOW DevNo			; restore defaults
	MOVF		EEADR
	INCF		EEADR				; two byte values
	MOVLW	1						; default
	CALL		eewrite
	RETURN

;***********Request an EV***************************************

reqev:
	BTFSS	Datmode,4
	RETURN							; prevent error message
	MOVF		EVidx,w				; check EV index
	BZ			rdev1
	MOVLW	EV_NUM+1
	CPFSLT	EVidx

rdev1:
	BRA		noEV1
	BSF		Datmode,6
	BRA		learn2

;*********Query NN*********************************************

qnn:
	MOVF		NN_temph,W		; respond if NN is not zero
	ADDWF	NN_templ,W
	BTFSS	STATUS,Z
	CALL		whoami
	RETURN	

;***********************************************************************
reval:
	CALL		thisNN
	SUBLW	0
	BNZ		revret					; abort
	MOVFF	RXB0D3, ENidx
	MOVFF	RXB0D4, EVidx
	CALL		evsend
revret:
	RETURN

;**************************************************************
; Not really needed

nnevn:
	CALL		thisNN
	SUBLW	0
	BNZ		nnnret
nnnret:
	RETURN

;**************************************************************

nerd:
	CALL		thisNN
	SUBLW	0
	BNZ		nrdret					; abort
	CALL		readen
nrdret:
	RETURN

;************Read back the stored event (detector number)************

readen:
	MOVLW	OPC_ENRSP			; ENRSP
	MOVWF	Tx1d0
	MOVLW	d'0'
	MOVWF	Tx1d3
	MOVWF	Tx1d4
	MOVLW	1
	MOVWF	Tx1d7					; to atart
	MOVLW	d'8'
	MOVWF	Dlc
	MOVLW	LOW DevNo
	MOVWF	EEADR
rden1:
	CALL		eeread
	MOVWF	Tx1d5
	INCF		EEADR
	CALL		eeread
	MOVWF	Tx1d6
	CALL		sendTX				; send event 1
	RETURN

;***********Request for read of node variable***********************

nvrd:
	CALL 		thisNN				; is it here?
	SUBLW	0
	BNZ		nvrd1					; no
	MOVLW	LOW	NVstart
	ADDWF	ev2,W					; add index
	MOVWF	EEADR
	DECF		EEADR,F				; index starts at 1, buffer at 0
	CALL		eeread
	MOVWF	Tx1d4					; NV val to transmit buffer
	MOVFF	ev2,Tx1d3			; transfer index
	MOVLW	OPC_NVANS			; NVANS
	MOVWF	Tx1d0
	MOVLW	d'5'
	MOVWF	Dlc
	CALL		sendTX				; send answer
nvrd1:
	RETURN

;*************A new NV so put it in EEPROM************************

putNV:
	CALL		thisNN  				; is it here?
	SUBLW	0
	BNZ		no_NV
	MOVLW	NV_NUM + 1			; put new NV in EEPROM 
	CPFSLT	ev2
	RETURN
	MOVF		ev2,W
	BZ			no_NV
	DECF		WREG					; NVI starts at 1
	ADDLW	LOW NVstart
	MOVWF	EEADR
	MOVF		ev3,W
	CALL		eewrite	
no_NV:
	RETURN
		
;*************Send individual parameter***************************
; Index 0 sends no of parameters

para1rd:
	MOVF		RXB0D3,W
	SUBLW	0
	BZ			numParams
	MOVLW	PRMCOUNT
	MOVFF	RXB0D3, Temp
	DECF		Temp
	CPFSLT	Temp
	BRA		pidxerr
	MOVLW	OPC_PARAN
	MOVWF	Tx1d0
	MOVLW	d'7'					; FLAGS index in nodeprm
	CPFSEQ	Temp
	BRA		notFlags			
	CALL		getflags
	MOVWF	Tx1d4
	BRA		addflags
notFlags:		
	MOVLW	LOW nodeprm
	MOVWF	TBLPTRL
	MOVLW	HIGH nodeprm
	MOVWF	TBLPTRH				; relocated code
	CLRF		TBLPTRU
	DECF		RXB0D3,W
	ADDWF	TBLPTRL
	BSF		EECON1,EEPGD
	TBLRD*
	MOVFF	TABLAT,Tx1d4
addflags:						
	MOVFF	RXB0D3,Tx1d3
	MOVLW	d'5'
	MOVWF	Dlc
	CALL		sendTX
	RETURN	
		
numParams:
	MOVLW	OPC_PARAN
	MOVWF	Tx1d0
	MOVLW	PRMCOUNT
	MOVWF	Tx1d4
	MOVFF	RXB0D3,Tx1d3
	MOVLW	d'5'
	MOVWF	Dlc
	CALL		sendTX
	RETURN
		
pidxerr:
	MOVLW	d'10'					; error
	CALL		errsub
	RETURN
		
getflags:								; create flags byte
	MOVLW	PF_CONSUMER
	BTFSC	Datmode,3
	IORLW	d'4'					; set bit 2
	MOVWF	Temp
	BSF		Temp,3				; set bit 3, we are bootable
	MOVF		Temp,W
	RETURN
		
;***********Send node parameter bytes (7 maximum)*****************

parasend:	
	MOVLW	OPC_PARAMS
	MOVWF	Tx1d0
	MOVLW	LOW nodeprm
	MOVWF	TBLPTRL
	MOVLW	d'8'
	MOVWF	TBLPTRH				; relocated code
	LFSR		FSR0,Tx1d1
	MOVLW	d'7'
	MOVWF	Count
	BSF		EECON1,EEPGD
		
para1:
	TBLRD*+
	MOVFF	TABLAT,POSTINC0
	DECFSZ	Count
	BRA		para1
	BCF		EECON1,EEPGD	
	MOVLW	D'8'
	MOVWF	Dlc
	CALL		sendTXa				; send CBUS message
	RETURN

;******Returns Node Number, Manufacturer Id, Module Id and Flags****** 

whoami:
	CALL		ldely					; wait (200mS) for other nodes
	MOVLW	OPC_PNN
	MOVWF	Tx1d0
	MOVLW	MAN_NO				; Manufacturer Id
	MOVWF	Tx1d3
	MOVLW	MODULE_ID			; Module Id
	MOVWF	Tx1d4
	CALL		getflags
	MOVWF	Tx1d5
	MOVLW	d'6'
	MOVWF	Dlc
	CALL		sendTX
	RETURN
	
; RQDDS  Request data as in DDES for Railcom detector. Response is DDRS (0xFB)

rqdds:
	MOVLW	OPC_DDRS			; load Tx1 with data
	MOVWF	Tx1d0
	MOVFF	DNhi,Tx1d1
	MOVFF	DNlo,Tx1d2
	MOVFF	reada,Tx1d3
	MOVFF	Adr1,Tx1d4
	MOVFF	Adr2,Tx1d5
	MOVFF	Lspd,Tx1d6
	MOVFF		Loc_flgs,Tx1d7	
	MOVLW	d'8'
	MOVWF	Dlc
	CALL		sendTX
	RETURN	

;************Request state of block (occ or unocc)*******************

asrq:
	BTFSS	Sflag,1				; (occupied)
	BRA		asrq1
	CALL		occ_sn1				; send occupied
	RETURN
asrq1:
	BTFSS	Sflag,2				; (unoccupied)
	RETURN
	CALL		un_occ1
	RETURN

;***************Time for PB debounce*****************************

pb_time:
	CALL		pb_deb				; PB debounce
	MOVF		WREG
	BNZ		bounce				; was bounce
	MOVLW	d'3'					; loops for delay
	MOVWF	Count
	CLRF		TMR0H
	CLRF		TMR0L
	MOVLW	B'00000111'			; set timer 0
	MOVWF	T0CON
	BSF		T0CON,TMR0ON		; start timer	1 second
pb_1a:
	BTFSC	S_PORT,S_BIT		; PB released?
	BRA		pb_2					; yes. Is too short 
	BTFSS	INTCON,TMR0IF		; timer out?
	BRA		pb_1a
	BCF		INTCON,TMR0IF
	DECFSZ	Count
	BRA		pb_1a					 ; keep looking
	RETLW	0
pb_2:
	RETLW 	1
bounce:
	RETLW	2

;**************Pushbutton debounce******************************
	
pb_deb:
	BTFSS	PIR2,TMR3IF			; uses timer 3
	BRA		pb_deb				; loop
	BCF		PIR2,TMR3IF
	BTFSS	S_PORT,S_BIT		; test PB again
	BRA		pb_ok					; still on
	RETLW 	1						; bounce
pb_ok:
	RETLW 	0						; is OK

;*********Flashes yellow LED in setup******************************
; 

flash:
	BTFSS	PIR2,TMR3IF			; Timer 3 out?
	RETURN							; no
	BCF		PIR2,TMR3IF			; RESET timer
	DECFSZ	T1count				; loop counter
	BRA		flash
	MOVLW	d'16'
	MOVWF	T1count				; RESET counter
		
flash1:
	BTG		LED_PORT,YELLOW	; flash yellow
	RETURN

;**************************************************************
unflash:
	BSF		LED_PORT,YELLOW	; yellow steady
	RETURN

;***********Copy event data to safe buffer*************************

copyev:
	MOVFF	RXB0D0, ev_opc
	MOVFF	RXB0D1, ev0
	MOVFF	RXB0D2, ev1
	MOVFF	RXB0D3, ev2
	MOVFF	RXB0D4, ev3
	MOVFF	RXB0D5, EVidx      	; only used by learn and some read cmds
	MOVFF	RXB0D6, EVdata	; only used by learn cmd

	MOVLW	OPC_ASON
	SUBWF	RXB0D0,W
	BZ			short					; is a short event
	MOVLW	OPC_ASOF
	SUBWF	RXB0D0,W
	BZ			short
	MOVLW	OPC_ASRQ
	SUBWF	RXB0D0,W
	BZ			short
	RETURN

short:
	CLRF		ev0					; here if a short event
	CLRF		ev1
	RETURN

;***********Send an 'occupied' frame******************************

occ_snd:
	BTFSC	Sflag,1				; has sent an occupied?
	BRA		no_occ
occ_sn1:
	MOVF		DNhi,F				; is DNhi = 0
	BNZ		occ_sn2
	MOVF		DNlo,F				; Is DNlo = 0
	BNZ		occ_sn2
	BRA		no_occ				; invalid DN - Don't send
occ_sn2:
	BSF		Sflag,1				; do once (occupied)
	MOVLB	d'15'					; bank 15
;*****Put info into TXB0 for occupied frame**************************
	CLRF		TXB0SIDH
	MOVF		IDtemph,W
	MOVWF	TXB0SIDH
	MOVF		IDtempl,W
	MOVWF	TXB0SIDL
	MOVLW	0xB0
	IORWF	TXB0SIDH			; set priority
	MOVLW	d'5'
	MOVWF	TXB0DLC	
	MOVLW	OPC_ASON
	MOVWF	TXB0D0
	MOVFF	NN_temph,TXB0D1
	MOVFF	NN_templ,TXB0D2
	MOVFF	DNhi,TXB0D3
	MOVFF	DNlo,TXB0D4

	BCF		TXB0D0,0				; ON event
tx0test:
	BTFSC	TXB0CON,TXREQ	; test if clear to send
	BRA		tx0test
	BSF		TXB0CON,TXREQ	; OK so send
tx0sent:
	BTFSS	TXB0CON,TXBIF
	BRA		tx0sent				; has it sent
	BCF		TXB0CON,TXBIF

	MOVLB	0							; bank 0

no_occ:
	BCF		Sflag,2				; clear unoccupied flag
	BCF		T1CON,TMR1ON		; stop timer
	MOVLW	0xF6					; RESET timer for short pulses
	MOVWF	TMR1H
	CLRF		TMR1L
	
	BSF		T1GCON,TMR1GE	; gated
	BSF		T1GCON,T1GGO		; gate set
	
	BSF		T1CON,TMR1ON		; start timer		
	BSF		PIE1,TMR1IE			; re-enable interrupts
	RETURN

;**********Test for valid DN  *************************************
un_occ:
;	BTFSC	Sflag,1				; has sent an occupied? 1 set = yes
;	BRA		no_occ
	MOVF		DNhi,F				; is DNhi = 0
	BNZ		un_occ1				; if DNhi > 0
	MOVF		DNlo,F				; is DNlo = 0
	BNZ		un_occ1				; if DNlo > 0
	BRA		un_occ2				; DN = 0 - don't send
un_occ1:
;****************** DN > 0 *************************************
	BCF		PIE1,TMR1IE
	BSF		Sflag,2				; do once - 2 = unoccupied
	MOVLB	d'15'					; bank 15
;********Put info into TXB0 for occupied frame***********************

	CLRF		TXB0SIDH
	MOVF		IDtemph,W
	MOVWF	TXB0SIDH
	MOVF		IDtempl,W
	MOVWF	TXB0SIDL
	MOVLW	0xB0
	IORWF	TXB0SIDH			; set priority
	MOVLW	d'5'
	MOVWF	TXB0DLC	
	MOVLW	OPC_ASON
	MOVWF	TXB0D0
	MOVFF	NN_temph,TXB0D1
	MOVFF	NN_templ,TXB0D2
	MOVFF	DNhi,TXB0D3
	MOVFF	DNlo,TXB0D4
	BSF		TXB0D0,0				; OFF event
tx0tes1:
	BTFSC	TXB0CON,TXREQ	; test if clear to send
	BRA		tx0tes1
	BSF		TXB0CON,TXREQ	; OK so send
tx0sen1:
	BTFSS	TXB0CON,TXBIF
	BRA		tx0sen1				; has it sent
	BCF		TXB0CON,TXBIF
	MOVLB	0						; bank 0

un_occ2:
;***************Reset Unoccupied********************************
	BCF		Sflag,1				; clear occupied flag
	BCF		Sflag,0				; Clear RailCom sent flag  - allow loco frame when next occupied

	CLRF		Ori						; Clear Orientation flags
	CLRF		Loc_flgs				; Clear Loc_flgs bits from last loco 

	CLRF		Bflag					; for next loco	

	MOVF		NV2_tmp,W
	MOVWF	NV2	

	BCF		T1CON,TMR1ON		; stop timer
	MOVLW	0xF6					; RESET timer for short pulses
	MOVWF	TMR1H
	CLRF		TMR1L
	
	BSF		T1GCON,TMR1GE	; gated
	BSF		T1GCON,T1GGO		; gate set

	BSF		T1CON,TMR1ON		; start timer
	RETURN

;********Longer delay (200mS)************************************

ldely:
	MOVLW	d'100'
	MOVWF	Count2
ldely1:
	CALL		dely 					; 2mS delay
	DECFSZ	Count2
	BRA		ldely1
	RETURN
			
;********A delay routine (2mS)************************************
			
dely:
	MOVLW	d'10'
	MOVWF	Count1
dely2:
	CLRF		Count
dely1:
	DECFSZ	Count,F
	GOTO		dely1
	DECFSZ	Count1
	BRA		dely2
	RETURN	

;***Initial set to running mode. Gets NN etc and sets to run************

slimset:
	CLRF		NN_temph
	CLRF		NN_templ
	CALL		clr_sub				; set defaults for events 
	BCF		LED_PORT,YELLOW	; yellow LED off
	BSF		LED_PORT,GREEN	; green LED on. Green for SLiM
		
set1:
	BTFSC	S_PORT,S_BIT		; look for PB on 
	BRA		set1					; loop till on
	CALL		pb_time				; time the on state
	BTFSC	Datmode,3
	GOTO		set2					; is pb in main loop
	SUBLW	0						; is it held in long enough?
	BNZ		abort					; bounce or too short
	BCF		LED_PORT,GREEN	; green off
set1a:
	BTFSS	S_PORT,S_BIT		; button released?
	BRA		set1a					; loop
	CALL		self_en				; do an enumerate
	BCF		Datmode,1
	BSF		Datmode,2
	MOVLW	Modstat
	MOVWF	EEADR
	MOVLW	B'00000100'
	CALL		eewrite	
	CALL		nnreq					; request node number RQNN
set1b:
	CALL		flash					; flash yellow
	BTFSC	S_PORT,S_BIT		; PB again?
	BRA		set1g					; go on
	BCF		LED_PORT,YELLOW	; in case it was on.
	CALL		pb_time				; how long is it in?
	SUBLW	0
	BNZ		set1g					; too short
	BRA		slimset				; try again
set1g:
	BTFSC	COMSTAT,7			; look for CAN input. 
	CALL		getcan				; wait for answer
set1c:
	BTFSS	Datmode,0			; got an answer
	BRA		set1b					; loop till		(maybe needs an abort mechanism?)
	MOVLW	OPC_RQNP			; request for parameters
	SUBWF	ev_opc,W
	BZ			set1e
	MOVLW	OPC_RQMN			; is it a name request?
	SUBWF	ev_opc,W
	BZ			set1h					; send name

	MOVLW	OPC_SNN				; set new NN
	SUBWF	ev_opc,W
	BZ			set1f
set1d:
	BCF		RXB0CON,RXFUL
	BCF		Datmode,0
	BRA		set1c					; look again

set1e:
	CALL		parasend				; send parameters to FCU
	BRA		set1d					; wait for SNN

set1h:
	CALL 		rqmn					; send name
	GOTO		set1d

set1f:
	CALL		putNN					; put in new NN. Sets Datmode to 8
		
	CALL		newid					; move all ID to EEPROM etc
	CALL		unflash				; yellow on 
	MOVLW	OPC_NNACK
	CALL		nnrel					; send NNACK
	BCF		RXB0CON,RXFUL	; clear CAN 
	BCF		Datmode,0
	BSF		Sflag,2				; (unoccupied)
	RETLW	0						; continue setup as if running mode
abort:
	BCF		LED_PORT,YELLOW
	RETLW	1						; too short or a bounce

set2:
	MOVWF	W_temp				; here if in running mode
		
	SUBLW	1
	BZ			set2a 
	MOVF		W_temp,W
	SUBLW	2
	BZ			set_bak2

	BRA		set_off				; is	held so  cancel run mode 
set2a:
	CALL		self_en				; do an enumerate
	CALL		nnreq					; request node number RQNN
set2b:
	CALL		flash					; flash yellow
	BTFSS	S_PORT,S_BIT		; pb in again?
	BRA		set_bk1				; set back to main
	BTFSC	COMSTAT,7			; look for CAN input. 
	CALL		getcan				; wait for answer
set2c:
	BTFSS	Datmode,0			; got an answer
	BRA		set2b					; loop till		(maybe needs an abort mechanism?)
	MOVLW	OPC_SNN				; set new NN
	SUBWF	ev_opc,W
	BZ			set2f
set2d:
	BCF		RXB0CON,RXFUL
	BCF		Datmode,0
	BRA		set2b					; look again

set2f:
	CALL		putNN					; put in new NN. Sets Datmode to 8
;	CALL		newid					; move all ID to EEPROM etc
set2g:
	CALL		unflash				; yellow on 
	BCF		Datmode,1			; out of NN waiting
	BCF		Datmode,2
	MOVLW	OPC_NNACK
	MOVWF	Tx1d0
	CALL		nnrel					; send NNACK
	GOTO		main					; back to main

set_bk1:
	BTFSS	S_PORT,S_BIT		; released 
	BRA		set_bk1
	BRA		set2g					; back

set_bak2:
	GOTO 	main					; bounce so do nothing 		

set_off:
	BCF		LED_PORT,YELLOW
	BSF		LED_PORT,GREEN
	BTFSS	S_PORT,S_BIT		; released
	BRA		set_off
	CLRF		Datmode
	MOVLW	LOW Modstat
	MOVWF	EEADR
	MOVF		Datmode,W
	CALL		eewrite
	MOVLW	OPC_NNREL
	MOVWF	Tx1d0					; send release
	CALL		nnrel
	MOVLW	0
	INCF		EEADR
	CALL		eewrite				; clear old NN
	MOVLW	0
	INCF		EEADR
	CALL		eewrite
	RETLW	1
		
;*****************************************************************	
setup1:
	BCF		LED_PORT,YELLOW	; clear if on
	MOVLW	Modstat				; get setup status
	MOVWF	EEADR
	CALL		eeread
	MOVWF	Datmode
	SUBLW	d'8'					; is it in run mode?
	BZ			setup2				; yes	
	MOVLW	0						; else set it
	MOVWF	Datmode				; not set yet
	CALL		eewrite
	CALL		slimset				; wait for setup PB
	MOVF		WREG					; is it long enough
	BNZ		setup1				; no
setup2:
	RETURN

;**************************************************************	
setsub:

	MOVLB	d'15'

	BSF		OSCTUNE,PLLEN
	
	CLRF		ANCON0				; disable A/D
	CLRF		ANCON1
	CLRF		CM1CON				; disable comparator
	CLRF		CM2CON
	CLRF		INTCON2			
	BSF		INTCON2,7			; weak pullups off
	CLRF		WPUB					; pullups clear
	MOVLB	0	

	MOVLW	B'00000100'			; PORTA has the  FLiM PB on RA2
	MOVWF	TRISA

	MOVLW	B'00000000'			; used for LEDs
	MOVWF	TRISB

	BSF		RCON,IPEN

	MOVLW	B'10000111'			; serial in on portc,7. RC0 and RC1 are loco orientation.
	MOVWF	TRISC					; gate on RC2

	CALL		read_LUT				; move LUT to RAM

	LFSR		FSR0, 0				; clear page 1
		
nextram:
	CLRF		POSTINC0		
	TSTFSZ	FSR0L
	BRA		nextram
	
	CLRF		INTCON				 ;no interrupts yet
	
	MOVLW	B'00000111'			; set timer 0 for block occupied delay
	MOVWF	T0CON
	CLRF		TMR0H
	CLRF		TMR0L

	MOVLW	B'11011000'
	MOVWF	T1GCON				; set timet 1 for gated mode
	MOVLW	B'01000110'
	MOVWF	T1CON				; set timer 1

	MOVLW	0xF6
	MOVWF	TMR1H
	CLRF		TMR1L
	BSF		IPR1,TMR1IP
	BSF		PIE1,TMR1IE			; high pri for TMR1
	
	BCF		PIR1,TMR1IF			; clear flag
	BSF		T1CON,TMR1ON
	CLRF		INTCON				; no interrupts yet
	MOVLW	B'00000100'		
	MOVWF	TXSTA1				; Baud rate is High
	MOVLW	B'10000000'
	MOVWF	RCSTA1				; set EUSART1 receive
	MOVLW	B'00000000'
	MOVWF	BAUDCON1
	CLRF		SPBRGH1
	MOVLW	d'15'
	MOVWF	SPBRG1				; set baud rate for 250K
	BCF		RCSTA1,CREN		; EUSART1

;****Next segment is essential************************************
	
	CLRF		BSR					; set to bank 0
	CLRF		EECON1				; no accesses to program memory	
	CLRF		Datmode
	CLRF		Latcount
	BSF		CANCON,7			; CAN to config mode
	MOVLW	B'10110000'
	MOVWF	ECANCON	
	BSF		ECANCON,5			; CAN mode 2 
	MOVF		ECANCON,W
	MOVWF	TempECAN 

	MOVLB	d'14'
	CLRF		BSEL0					; 8 frame FIFO
	CLRF		RXB0CON
	CLRF		RXB1CON
	CLRF		B0CON
	CLRF		B1CON
	CLRF		B2CON
	CLRF		B3CON
	CLRF		B4CON
	CLRF		B5CON
				
	MOVLW	CANBIT_RATE		; set CAN bit rate at 125000 
	MOVWF	BRGCON1
	MOVLW	B'10011110'			; set phase 1 etc
	MOVWF	BRGCON2
	MOVLW	B'00000011'			; set phase 2 etc
	MOVWF	BRGCON3
	MOVLB	0

	MOVLW	B'00100000'
	MOVWF	CIOCON				; CAN to high when off
		
mskload:
	LFSR		FSR0,RXM0SIDH		; Clear masks, point to start
mskloop:
	CLRF		POSTINC0		

	CPFSEQ	FSR0L					; 0xEFF is last mask address
	BRA		mskloop
		
	CLRF		CANCON				; out of CAN setup mode
	CLRF		CCP1CON

	CLRF		IPR5					; low priority CAN RX and Tx error interrupts

	CLRF		T3GCON	
	MOVLW	B'00110010'			; Timer 3 set Timer 3 for LED flash
	MOVWF	T3CON
	MOVLW	0x00
	MOVWF	TMR3H				; Timer 3 is a 16 bit timer
	MOVWF	TMR3L
	BSF		T3CON,TMR3ON		; run timer
	MOVLW	d'16'
	MOVWF	T1count				; flash delay counter
	MOVLW	d'4'
	MOVWF	Debcnt	

	BSF		PIE5,ERRIE			; Tx error enable
	BSF		PIE5,FIFOWMIE		; FIFO full interrupt

	CLRF		Tx1con	

;****Next segment required***************************************
		
	MOVLW	B'00000001'
	MOVWF	IDcount				; set at lowest value for starters
		
	CLRF		INTCON3	
	CLRF		T3GCON
			
	CLRF		PIR1
	CLRF		PIR2					; for timer 3
	CLRF		PIR3
	CLRF		PIR4
	CLRF		PIR5

	MOVLW	d'39'
	MOVWF	PR4					; 10mSec timer x256
	MOVLW	B'00000111'
	MOVWF	T4CON				; set timer 4
		
	BCF		RXB0CON,RXFUL	; ready for next
	BCF		COMSTAT,RXB0OVFL; clear overflow flags if set
	BCF		COMSTAT,RXB1OVFL
	CLRF		PIR5					; clear all ECAN flags

	CLRF		EEADRH				; upper EEPROM page to 0

	MOVLB	d'15'					; set busy frame
	CLRF		TXB0SIDH
	CLRF		TXB0SIDL
	CLRF		TXB0DLC
	MOVLB 	0

	MOVLW	B'11000000'			; start interrupts
	MOVWF	INTCON
	RETURN

;**************************************************************
	ORG 0xF00000			;EEPROM data. Defaults
	
CANid		DE	B'01111111',0x00	; CAN id default and module status
NodeID	DE	0x00,0x00			; Node ID

	ORG	0xF00010

DevNo		DE	0x00,0x01			; two byte DN

	ORG	0xF00020

NVstart	DE	0x02,0x7F			; two NVs, initialised to 0x06 and 7F

							; NV1 is 0 = do nothing
							;           2 = Ch 1 only
							; ****Not part of this release****
							;   	   	   4 = Ch 2 only
							;           6 = Ch 1 and Ch 2 
							; ***************************

							; NV2 is unoccupied delay

;	Table for 4/8 decoding

	ORG	0xF00100

Lut_start			

		DE	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x33,0x80,0x80,0x80,0x34,0x80,0x35,0x36,0x80
		DE  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x3A,0x80,0x80,0x80,0x3B,0x80,0x3C,0x37,0x80
		DE	0x80,0x80,0x80,0x3F,0x80,0x3D,0x38,0x80,0x80,0x3E,0x39,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x24,0x80,0x80,0x80,0x23,0x80,0x22,0x21,0x80
		DE	0x80,0x80,0x80,0x1F,0x80,0x1E,0x20,0x80,0x80,0x1D,0x1C,0x80,0x1B,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x19,0x80,0x18,0x1A,0x80,0x80,0x17,0x16,0x80,0x15,0x80,0x80,0x80
		DE	0x80,0x25,0x14,0x80,0x13,0x80,0x80,0x80,0x32,0x80,0x80,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x0E,0x80,0x0D,0x0C,0x80
		DE	0x80,0x80,0x80,0x0A,0x80,0x09,0x0B,0x80,0x80,0x08,0x07,0x80,0x06,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x04,0x80,0x03,0x05,0x80,0x80,0x02,0x01,0x80,0x00,0x80,0x80,0x80
		DE	0x80,0x0F,0x10,0x80,0x11,0x80,0x80,0x80,0x12,0x80,0x80,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x80,0x80,0x2B,0x30,0x80,0x80,0x80,0x2F,0x80,0x31,0x80,0x80,0x80
		DE	0x80,0x29,0x2E,0x80,0x2D,0x80,0x80,0x80,0x2C,0x2A,0x80,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x28,0x80,0x27,0x80,0x80,0x80,0x26,0x80,0x80,0x80,0x80,0x80,0x80,0x80
		DE	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80

; Note. ACK, NACK abd BUSY are stord as 0x80 for now. 


	ORG	0xF003FE
	DE		0,0		;for boot load
	END
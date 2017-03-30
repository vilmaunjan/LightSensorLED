;********************************************************
;   Project created by Vilma Un Jan
;	Class: Assembly language
;	XMEGA
//  Ports used
//	PORT R(0)	LED (yellow bottom)
//	PORT R(1)	LED (Yellow top)
//	
//	PORT A(3)	LCD Reset bar
//	PORT F(3)	LCD CS bar
//	PORT D(0)	LCD AO line
//	PORT D(1)	LCD XCK
//	PORT D(3)	LCD TX 
//	PORT E(4)	LCD back light (1 = on, 0 = off)
//	PORT ADCA channel 0		Light sensor
//
//
//  For external baord
//  LED on B-0	LED

;The aim of this project is to use the light sensor, 
;located at Port A pin 0, and turn on/off an external
;LED, located at Port B pin 0, according to the voltage
;that the light sensor receives.
; ******************************************************
.include	<atxmega256a3budef.inc>
; ============================================
; RESET AND INTVECTORS
; ============================================
.CSEG
.ORG		0x00
JMP			start
.ORG		TCD1_OVF_VECT	//	VECTOR DE INTERRUPCION
JMP			TCD1_ISR		//	GO TO INTERRUPT
.ORG		0xF6
; ============================================
; STACK SETUP
; ============================================
start:		
	LDI			R16,low(RAMEND)			;	initialize the stack
	STS			CPU_SPL,R16				;			*
	LDI			R16,high(RAMEND)		;			*
	STS			CPU_SPH,R16				;			*

	LDI			R16,0b00000111			;	Enable priorities LOW, MED, and HIGH
	STS			PMIC_CTRL,R16			;			*

	SEI									;	Enable global interrupts

; ============================================
; GPIO SETUP
; ============================================
	//		PUERTO R
	LDI			R16,0b00000011			;	PIN_R0,PIN_R1 SETUP AS output
	STS			PORTR_DIR,R16			;
	LDI			R16,0x00				;	turn off PIN_R0 and PIN_R1 
	STS			PORTR_OUT,R16			;			

	//		PUERTO B
	LDI			R16,0b00000001			;	PIN_B0 SETUP AS output
	STS			PORTB_DIR,R16			;			
	LDI			R16,0x00				;	turn off PINB0
	STS			PORTB_OUT,R16			;	

; ============================================
; TIMER SETUP
; ============================================
	LDI			R16,0b00010011			;	mode = single slope PWM
	STS			TCD1_CTRLB,R16			;			*

	LDI			R16,0					;	disable event actions on CCA
	STS			TCD1_CTRLD,R16			;			*

	LDI			R16,low(40000)			;	PER = 40,000
	STS			TCD1_PER,R16			;			*
	LDI			R16,high(40000)			;			*
	STS			TCD1_PER+1,R16			;			*

	LDI			R16,low(12000)			;	CCA = 12,000
	STS			TCD1_CCA,R16			;			*
	LDI			R16,high(12000)			;			*
	STS			TCD1_CCA+1,R16			;			*
		
	LDI			R16,0x01				;	count up
	STS			TCD1_CTRLFCLR,R16		;			*

	LDI			R16,0x02				;	OVF INT med priority
	STS			TCD1_INTCTRLA,R16		;			*

	LDI			R16,0b00000100			;	CONNECT Fper/8 To TCD1
	STS			TCD1_CTRLA,R16			;			*


; ============================================
; ADC SETUP
; ============================================
	LDI			R16,0b00000001					;	no DMA, do not start yet, enable ADCA
	STS			ADCA_CTRLA,R16					;			

	LDI			R16,0b00000100					;	unsigned, single conv, 
	STS			ADCA_CTRLB,R16					;	8-bit right just.

	LDI			R16,0b00000000					;	default prescale of 4
	STS			ADCA_PRESCALER,R16				;			

	LDI			R16,0b00000111					;	Pos input = ADC0 (A0) -> Ambient light sensor
	STS			ADCA_CH0_MUXCTRL,R16			;	Neg inpt  = Internal ground

	LDI			R16,0b00010000					;	ref voltage = Vcc / 1.6
	STS			ADCA_REFCTRL,R16				;			

	LDI			R16,0b00000001					;	single ended pos input
	STS			ADCA_CH0_CTRL,R16				;	no gain, do not start yet

; ============================================
; Init LCD
; ============================================
	CALL		SetCpuClockTo32MInt
	CALL		LCDSetupUsartSpiPins
	CALL		LCDSetupSPIOnUARTD0
	CALL		LCDReset
	CALL		LCDInit
	CALL		LCDBackLightOff
	CALL		LCDClearScreen


; ============================================
; MAIN LOOP
; ============================================
done:			JMP			done				

; ============================================
; TIMER INTERRUPT
; ============================================
TCD1_ISR:
loop:			LDS			R16,ADCA_CH0_CTRL		;	Start conversion
				ORI			R16,0b10000000			;	OR with flag START
				STS			ADCA_CH0_CTRL,R16		;	ADCA_CH0_CTRL = R16 

READ_ADC:		LDS			R16,ADCA_CH0_INTFLAGS	;	while conversion is not complete
				ANDI		R16,0x01				;	check again
				BREQ		READ_ADC				;			

				LDS			R16,ADCA_CH0_RES		;	r16 = ADCA_CH0_RES
				LDI			R17,1					;	r17 = 1
				MUL			R16,R17					;	r16 = R16*R17

				LDI			R17,60					;	r17 = LIMIT_SENSOR_DE_LUZ
				CP			R16,R17					;	COMPARE r16	with r17
				BRSH		ALARM					;	Branch if r16 >= r17 

				LDI			R18,0b00000011			;	r18 = 0b00000011
				STS			PORTR_OUT,R18			;	PIN_R0 AND PIN R1 OUTPUT HIGH

				LDI			R18,0b00000001			;	r18 = 0b00000001
				STS			PORTB_OUT,R18			;	PIN_B0 OUTPUT HIGH

; ============================================
; SHOW ADVC AND STATUS BAR IN LCD
; ============================================			
LCD:			
				STS			TCD1_CCA,R0				;	CCA = R1:R0
				STS			TCD1_CCA+1,R1			;					

if:				DEC			R4						;	if (--R4 < 0)
				BRNE		endif

				LDI			R17,100
				MOV			R4,R17
				
				CALL		LCDClearScreen			

; ============================================
; SHOW ADC(HEXADECIMAL) IN LCD
; ============================================
				MOV			R3,R16					; R3 = ADC result
				CLR			R0
				CLR			R1
				CALL		LCDSetCurser

				MOV			R16,R3
				LSR			R16
				LSR			R16
				LSR			R16
				LSR			R16
				MOV			R0,R16
				CALL		LCDWriteHexDigit

				MOV			R16,R3
				ANDI		R16,0x0F
				MOV			R0,R16
				CALL		LCDWriteHexDigit

; ============================================
; STATUS BAR
; ============================================
				CLR			R0
				LDI			R16,2
				MOV			R1,R16
				CALL		LCDSetCurser	
				LDS			R0,ADCA_CH0_RES		;  SHOW value ADC in status bar
				LSR			R0
				CALL		LCDWriteStatusBar

endif:			RETI

ALARM:
				LDI			R18,0b00000000				;	r18 = 0b00000000 
				STS			PORTR_OUT,R18				;	PIN_R0 AND PIN_R1 OUTPUT LOW
				LDI			R18,0b00000000				;	r18 = 0b00000000 
				STS			PORTB_OUT,R18				;	PIN_B0 OUTPUT LOW
				RJMP		LCD



// Writes the byte in R16 to the current colm of the current row of the LCD.
// The next colomn is incremented automatically.
// Algo: make A0 line high then send the byte to the LCD.
// Input: R16 = byte with bit pattern to display.

LCDWriteData:
				PUSH		R17					
				LDI			R17,0b00000001			; LCD_AO high (D0 <- 1)
				STS			PORTD_OUTSET,R17		;			*
				CALL		LCDSendByte				; Send the byte
				POP			R17
				RET

// Writes the byte in R16 to the LCD as a command.
// Algo: make A0 line low then send the byte to the LCD.
// Input: R16 = the byte command.

LCDWriteCmd:
													
				PUSH		R17
				LDI			R17,0b00000001			; LCD_AO low (D0 <- 0)
				STS			PORTD_OUTCLR,R17		;			*
				CALL		LCDSendByte				; Send the byte
				POP			R17
				RET

// Send the byte in R16 to the LCD. 

LCDSendByte:
				PUSH		R17						
				LDI			R17,0b00001000			; Make CS low (F3 <- 0)
				STS			PORTF_OUTCLR,R17		;			*
wcmd1:
				LDS			R17, USARTD0_STATUS		; loop until the data buffer is clear
				SBRS		R17,5
				RJMP		wcmd1
				STS			USARTD0_DATA,R16		; Send the byte to the LCD
wcmd2:
				LDS			R17, USARTD0_STATUS		; loop until the transmit is complete
				SBRS		R17,6
				RJMP		wcmd2
				CBR			R17,6
				STS			USARTD0_STATUS,R17		; CLEAR TRANSMIT COMPLETE
				POP			R17
				RET

;   wait a little, short delay used in RESET of the ST7565r
;	for lcd5 added an outer loop to increase the delay by a factor of 4.
;
wlittle:
		PUSH r17
		PUSH r18
		ldi r18,4
agab:
		ldi r17,85
agaa:
		nop
		nop
		nop
		dec r17
		brne agaa
		dec r18
		brne agab
		POP r18
		POP r17
		ret

// This routine tests the LCD by writting the numbers 0 through F on the top row of the LCD.

LCDTest:
				PUSH		R16						; ddd
				PUSH		R0

				LDI			R16,0					; row. Set the row and colm to 0 each
				MOV			R1,R16
				LDI			R16,0					; colm
				MOV			R0,R16
				RCALL		LCDSetCurser

				LDI			R16,15					; R0 = 15
				MOV			R0,R16
loop2:												; while R0 >= 0

				RCALL		LCDWriteHexDigit		;    Write the hex digit in R0
				DEC			R0						;	 R0 = R0 - 1
				BRNE		loop2					; end while
				RCALL		LCDWriteHexDigit		; Write the last hex digit R0 == 0
	
				POP			R0
				POP			R16
				RET

// Initialize the LCD
LCDInit:
				LDI			R16, 0xA0				; cmd = A0 (adc normal)
				CALL		LCDWriteCmd				;			*
				LDI			R16, 0xA6				; cmd = A6 (display in normal mode)
				CALL		LCDWriteCmd
				LDI			R16, 0xC8				; cmd = C8 (reverse scan)
				CALL		LCDWriteCmd
				LDI			R16, 0xA2				; cmd = A2 (lcd bias)
				CALL		LCDWriteCmd
				CALL		wlittle					; wants a small delay here
				LDI			R16, 0x2F				; cmd = 2F (power control)		
				CALL		LCDWriteCmd
				LDI			R16, 0xF8				; cmd = F8 (set  booster ratio)
				CALL		LCDWriteCmd
				LDI			R16, 0x00				; cmd = 00 (booster ratio 2x ... 4x)
				CALL		LCDWriteCmd
				LDI			R16, 0x21				; cmd = 21 (resister ratio)
				CALL		LCDWriteCmd
													; SHOULD CHECK 30 <-< 40 for contrast, called volume
													; in ST7565
				LDI			R16, 0x1F				; cmd = 1F (set contrast ???)
				CALL		LCDWriteCmd
				LDI			R16,0xAF				; cmd = AF (LCD on)
				CALL		LCDWriteCmd
				RET

// clears the whole LCD screen.
// Algo: Traverses each block of the screen writting a 00 bit pattern.
LCDClearScreen:
				PUSH		R16
				PUSH		R17
				PUSH		R18
				PUSH		R0
				PUSH		R1

				CLR			R0						; colm
				CLR			R1						; page
				CALL		LCDSetCurser

				CLR			R16
				LDI			R17,132
LCDCS_while1:	CALL		LCDWriteData
				DEC			R17
				BRNE		LCDCS_while1

				INC			R1
				CALL		LCDSetCurser
				CLR			R16
				LDI			R17,132
LCDCS_while2:	CALL		LCDWriteData
				DEC			R17
				BRNE		LCDCS_while2

				INC			R1
				CALL		LCDSetCurser
				CLR			R16
				LDI			R17,132
LCDCS_while3:	CALL		LCDWriteData
				DEC			R17
				BRNE		LCDCS_while3			
	
				INC			R1
				CALL		LCDSetCurser
				CLR			R16
				LDI			R17,132
LCDCS_while4:	CALL		LCDWriteData
				DEC			R17
				BRNE		LCDCS_while4
										
				POP			R1
				POP			R0
				POP			R18
				POP			R17
				POP			R16
				RET

LCDBackLightOn:
				PUSH		R16
				LDI			R16,0b00010000			; E4 <- 1 (LCD back light on)
				STS			PORTE_OUTSET,R16		
				POP			R16
				RET

LCDBackLightOff:
				PUSH		R16
				LDI			R16,0b00010000			; E4 <- 0 (LCD back light off)
				STS			PORTE_OUTCLR,R16		
				POP			R16
				RET
LCDReverseOn:
				PUSH		R16
				LDI			R16,0xA7				; cmd = A7 (Reverse on)
				RCALL		LCDWriteCmd
				POP			R16
				RET
LCDReverseOff:
				PUSH		R16				
				LDI			R16,0xA6				; cmd = A6 (Reverse off)
				RCALL		LCDWriteCmd
				POP			R16
				RET

LCDOn:
				PUSH		R16				
				LDI			R16,0xAF				; cmd = AF (LCD on)
				RCALL		LCDWriteCmd
				POP			R16
				RET

LCDOff:
				PUSH		R16				
				LDI			R16,0xAE				; cmd = AE (LCD off)
				RCALL		LCDWriteCmd
				POP			R16
				RET

; Setup the Pins used for the SPI on USART D0

; A3 = Reset/
; F3 = CS/
; D0 = AO of the LCD
; D1 = XCK
; D3 = TX
; E4 = back light (1 = on, 0 = off)

LCDSetupUsartSpiPins:
				PUSH		R16

				LDI			R16,0b00001000			;set usart-spi ports
				STS			PORTA_DIRSET,R16		;A3 out for Reset
				STS			PORTA_OUTSET,R16		;   high
				STS			PORTF_DIRSET,R16		;F3 out for CS
				STS			PORTF_OUTSET,R16		;   high
				LDI			R16,0b00001011
				STS			PORTD_DIRSET,R16		;D0,1,3 out for  D0=A0,D1=xck,D3=TX
				STS			PORTD_OUTSET,R16		;   high
				LDI			R16,0b00010000			;set usart-spi ports
				STS			PORTE_DIRSET,R16		;E4 out  for backlite
				STS			PORTE_OUTSET,R16		;   on

				POP			R16
				RET

; Reset the LCD.  
; Algo: Make CS/ low then Reset/ low then wait 1 ms then Reset/ high.
LCDReset:
				PUSH		R16
				LDI			R16,0b00001000
				STS			PORTF_OUTCLR,R16		; F3 = 0 (cs_bar low = active)
				STS			PORTA_OUTCLR,R16		; A3 = 0 (reset_bar low = start reset)
				CALL		wlittle					; delay 1 ms
				STS			PORTA_OUTSET,R16		; A3 = 1 (reset_bar high).
				POP			R16
				RET

; Set up master spi on UARTD0
; USART initialization should use the following sequence: 
; 1.    Set the TxD pin value high, and optionally set the XCK pin low.
; 2.    Set the TxD and optionally the XCK pin as output. DONE ABOVE
; 3.    Set the baud rate and frame format. 
; 4.    Set the mode of operation (enables XCK pin output in synchronous mode). 
; 5.    Enable the transmitter or the receiver, depending on the usage. 

LCDSetupSPIOnUARTD0:
				PUSH		R16
				
				LDI			R16, 0b01000000			; Step 1&2. invert xck
				STS			PORTD_PIN1CTRL,R16		; This is part of "SPI MODE 3"
				
				LDI			R16,0b00000010			; xck
				STS			PORTD_OUTCLR,R16	

				LDI			R16, 0b00001111			; Step 3. set BSEL USART xck to 0x0F
				STS			USARTD0_BAUDCTRLA,R16

				LDI			R16, 0b11000011			; Step 4.
				STS			USARTD0_CTRLC,R16		; MASTER,MSB FIRST, hafl of MODE 3, BIT0 ???, 

				LDI			R16, 0b00011000			; Step 5.
				STS			USARTD0_CTRLB,R16		; TX & RX ENABLE

				POP			R16
				RET

// Possitions the curser in the LCD. 
// input:	R1 = row (0 ... 131)
//			R0 = colm (page) (0 ... 7)
// each page is a block of 8 rows.
// each letter is a 8 X 6 matrix with the image in the top left 7 X 5 corner.
// Leave the bottom row and the rightmost column blank for spacing. 

LCDSetCurser:	MOV			R16,R0					; set the MSB of the colm address
				ANDI		R16,0xF0				; Code is 0001xxxx
				SWAP		R16
				ORI			R16,0x10
				RCALL		LCDWriteCmd

				MOV			R16,R0					; set the LSB of the colm address
				ANDI		R16,0x0F				; code is 0000xxxx
				RCALL		LCDWriteCmd

				MOV			R16,R1					; set the row (page)
				ANDI		R16,0x0F				; code is 1011xxxx
				ORI			R16,0xB0
				RCALL		LCDWriteCmd
				RET

// Displays the number in the least significant 4 bits of R0.
// The bit patterns for each digit is stored in the table LCDData in program memory.
// Each digit has 6 columns for the table has 6 bytes per digit. 
// Algo:	The digit value is multiplied by 6 to produce a byte offset from the start 
// of the table. Then the offset is added to the start of the table and that address points 
// to the first of the 6 bytes for that digit. Each of the 6 byte is sent to the LCD to display. 
// Input:	R0 = digit to display (00 ... 0F)

LCDWriteHexDigit:
				PUSH		R16
				PUSH		R17
				PUSH		R1
				PUSH		ZL	
				PUSH		ZH	
				PUSH		R0

				MOV			R16,R0					; Clear the MSB of R0 just in case 
				ANDI		R16,0x0F				;			*
				LDI			R17,6					; Z = LCDData + (digit * 6)
				MUL			R16,R17					;			* 
				MOV			R16,R0					;			*
				LDI			ZL,low(LCDData << 1)	;			*
				LDI			ZH,high(LCDData << 1)	;			*
				ADD			ZL,R16					;			*
				CLR			R16						;			*
				ADC			ZH,R16					;			*
				
				LPM			R16,Z+					; write( LCDData[Z++] )
				RCALL		LCDWriteData			;			*
				
				LPM			R16,Z+					; write( LCDData[Z++] )
				RCALL		LCDWriteData			;			*
				
				LPM			R16,Z+					; write( LCDData[Z++] )
				RCALL		LCDWriteData			;			*
				
				LPM			R16,Z+					; write( LCDData[Z++] )
				RCALL		LCDWriteData			;			*				
				
				LPM			R16,Z+					; write( LCDData[Z++] )
				RCALL		LCDWriteData			;			*
				
				LPM			R16,Z					; write( LCDData[Z] )
				RCALL		LCDWriteData			;			*		

				POP			R0
				POP			ZH
				POP			ZL
				POP			R1
				POP			R17
				POP			R16		

				RET

// Displays a bar starting at the current culumn and extending to the right for up to 130 columns more.
// It is assumed that the curser is set to the far left (column 0).
// Input:	R0 = number of solud bars to display (0 ... 131)

LCDWriteStatusBar:
				PUSH		R16
				PUSH		R17
				PUSH		R18
				

				LDI			R16,0xFF

				CLR			R18
LCDWSB_if:		CPI			R18,131
				BREQ		LCDWSB_endif
LCDWSB_if2:		CP			R18,R0
				BRNE		LCDWSB_endif2
				LDI			R16,0x00
LCDWSB_endif2:
				RCALL		LCDWriteData
				INC			R18
				JMP			LCDWSB_if
LCDWSB_endif:					
				POP			R18			
				POP			R17
				POP			R16		

				RET

LCDData:
LCDData0:		.DB			0x3E,0x41,0x41,0x41,0x3E,0x00 ; digit 0
LCDData1:		.DB			0x00,0x00,0x7F,0x00,0x00,0x00 ; digit 1
LCDData2:		.DB			0x32,0x49,0x49,0x49,0x06,0x00 ; digit 2
LCDData3:		.DB			0x41,0x49,0x49,0x49,0x36,0x00 ; digit 3
LCDData4:		.DB			0x0F,0x08,0x7E,0x08,0x08,0x00 ; digit 4
LCDData5:		.DB			0x2F,0x49,0x49,0x49,0x31,0x00 ; digit 5
LCDData6:		.DB			0x3E,0x49,0x49,0x49,0x32,0x00 ; digit 6
LCDData7:		.DB			0x41,0x21,0x11,0x09,0x07,0x00 ; digit 7
LCDData8:		.DB			0x36,0x49,0x49,0x49,0x36,0x00 ; digit 8
LCDData9:		.DB			0x06,0x09,0x09,0x09,0x7E,0x00 ; digit 9
LCDDataA:		.DB			0x7C,0x12,0x11,0x12,0x7C,0x00 ; digit A
LCDDataB:		.DB			0x7F,0x49,0x49,0x49,0x36,0x00 ; digit B
LCDDataC:		.DB			0x3E,0x41,0x41,0x41,0x41,0x00 ; digit C
LCDDataD:		.DB			0x7F,0x41,0x41,0x41,0x3E,0x00 ; digit D
LCDDataE:		.DB			0x7F,0x49,0x49,0x41,0x41,0x00 ; digit E
LCDDataF:		.DB			0x7F,0x09,0x09,0x01,0x00,0x00 ; digit F

SetCpuClockTo32MInt:
				LDS			R16,OSC_CTRL			; Enable the 32M Hz oscilator
				ORI			R16,0b00000010			;			*
				STS			OSC_CTRL,R16			;			*
while1:
				LDS			R16,OSC_STATUS			; Wait until its stable
				ANDI		R16,0x02				;			*
				BREQ		while1					;			*

				LDI			R16,0xD8				; Connect the 23 MHz OSC to the system clock
				OUT			CPU_CCP,R16				;			*
				LDI			R16,0x01				;			*
				STS			CLK_CTRL,R16			;			*

				LDI			R16,0xD8				; Reset the prescale stages A,B,C back to 1
				OUT			CPU_CCP,R16				;			*
				LDI			R16,0x00				;			*
				STS			CLK_PSCTRL,R16			;			*

				LDI			R16,0xD8				; Select the internal 32.768 KHz source
				OUT			CPU_CCP,R16				; for the RC32M DFLL
				LDS			R16,OSC_DFLLCTRL		;			*
				ANDI		R16,0b11111101			;			*
				STS			OSC_DFLLCTRL,R16		;			*

				LDS			R16,DFLLRC32M_CTRL		; Enable the DFLL for the RC32MHz
				ORI			R16,0x01				;			*
				STS			DFLLRC32M_CTRL,R16		;			*

				RET

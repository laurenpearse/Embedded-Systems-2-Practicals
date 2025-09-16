/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

LDR R5, GPIOA_BASE @ sets register 5 to the address of the start of the GPIOA base, this just makes it easier to get to

main_loop:

@ Checks if buttons are pressed, if not then default mode
@ Need to refer to this later when doing the button modes

LDR R4, [R5, #0x10] @ the IDR is at 0x48000010, it holds the push button states this makes R4 equal the value stored at R5 +0x10
@ NOTE the push buttons are active low
MOVS R6, #0x0F @ load mask for SW0-3 (0b1111) into a register
ANDS R4, R4, R6  @ just considers values of inputs for SW0-3

@ First want to check if button 2 or 3 pressed bcos they only pressed one at a time and dont want it mixed up with the later 0&1 pressing stuff

MOVS R6, #0x04 @ 0b0100 -> SW2 mask
TST R4, R6 @ Button 2, the TST instruction is an AND that wont change the R4 value, just changes flag
BEQ mode2 @ If result of above is 0, the Z flag =1 and it will go to mode2

MOVS R6, #0x08 @ 0b1000 -> SW3 mask
TST R4, R6 @ checks if button 3 pressed
BEQ mode3 @ same as above

MOVS R6, #0x03 @ 0b0011 -> SW0 & SW1 together
TST R4, R6 @ both button 0 and 1
BEQ mode01

MOVS R6, #0x01 @ 0b0001 -> SW0
TST R4, R6 @ button 0 pressed
BEQ mode0

MOVS R6, #0x02  @ 0b0010 -> SW1
TST R4, R6 @ button 1 pressed
BEQ mode1

B default_mode

mode01:
STR  R2, [R1, #0x14]
ADDS  R2, R2, #2
BL   shortdelay
@ keep this at end of this fucntion:
B    main_loop

mode0:
STR  R2, [R1, #0x14]
ADDS  R2, R2, #2
BL   longdelay

@ keep this at end of this fucntion:
B    main_loop

mode1:
STR R2, [R1, #0x14]
ADDS R2, R2, #1
BL shortdelay
@ keep this at end of this fucntion:
B main_loop

mode2:

MOVS R2, #0xAA     @ Load pattern into R2
STR R2, [R1, #0x14] @ Output to GPIO (LED pattern = 0xAA)

@ keep this at end of this function:
B main_loop @returns to main loop

mode3:
@ keep this at end of this function:
LDR R4, [R5, #0x10] @ Read GPIO input register
MOVS R6, #0x08 @ Mask for SW3 (bit 3)
TST R4, R6 @ Check if SW3 is pressed
BEQ mode3 @ If still pressed, loop here
B main_loop

default_mode:
STR R2, [R1, #0x14] @ display current count on LEDs (R2) sends to address for LED ODR
ADDS R2, R2, #1 @ increments LED counter
BL longdelay @ branch to 0.7s delay
B main_loop @ takes back to main loop


@ 0.7 seconds delay function
longdelay:
LDR R3, = LONG_DELAY_CNT @ set address of long delay to R3
LDR R3, [R3] @ loads the value of the long delay at its address and sets it to R3
longloop:
SUBS R3, R3, #1 @ R3 = R3-1
BNE longloop @ if R3 =0 then will exit loop, Z flag =1 then
BX LR @ going to call this fn later and this makes it jump back to where it was before in the main loop

shortdelay:
LDR R3, = SHORT_DELAY_CNT
LDR R3, [R3]
shortloop:
SUBS R3, R3, #1 @ R3 = R3-1
BNE shortloop @ if R3 =0 then will exit loop, Z flag =1 then
BX LR @ going to call this fn later and this makes it jump back to where it was before in the main loop

write_leds:
	STR R2, [R1, #0x14]
	B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays

@ Choosing long delay: 8 Mhz clock (8 million cycles per sec) and want a 0.7s delay.
@ 8 Mhz * 0.7 = 5600000 cycles
@ Did some research and apparently an ARM loop takes about 3.95 cycles to execute
@ Therefore delay value: 5600000 divided by 3.95 = 1417721 (initially tried 3 but was slow)
@ will have to double check this is correct when testing

LONG_DELAY_CNT: 	.word 1417721
SHORT_DELAY_CNT: 	.word 623376 @uses same logic as long delay but for 0.3s

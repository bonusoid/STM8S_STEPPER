                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
                                      4 ; This file was generated Sun May 15 22:26:54 2022
                                      5 ;--------------------------------------------------------
                                      6 	.module main
                                      7 	.optsdcc -mstm8
                                      8 	
                                      9 ;--------------------------------------------------------
                                     10 ; Public variables in this module
                                     11 ;--------------------------------------------------------
                                     12 	.globl _main
                                     13 	.globl _stepccw
                                     14 	.globl _stepcw
                                     15 	.globl _readreg
                                     16 	.globl _delay_init
                                     17 	.globl _delay_us
                                     18 	.globl _delay_ms
                                     19 	.globl _delay_timer
                                     20 	.globl _clock_init
                                     21 	.globl _i2c_init
                                     22 	.globl _i2c_set_start
                                     23 	.globl _i2c_set_address
                                     24 	.globl _i2c_set_stop
                                     25 	.globl _i2c_clear_ack
                                     26 	.globl _i2c_set_ack
                                     27 	.globl _i2c_ack_pos_current
                                     28 	.globl _i2c_ack_pos_next
                                     29 	.globl _i2c_poll_SB
                                     30 	.globl _i2c_poll_ADDR
                                     31 	.globl _i2c_poll_BTF
                                     32 	.globl _i2c_poll_TXE
                                     33 	.globl _i2c_poll_RXNE
                                     34 	.globl _i2c_clear_bits
                                     35 	.globl _i2c_clear_ADDR
                                     36 	.globl _i2c_enable_interrupts
                                     37 	.globl _i2c_disable_interrupts
                                     38 	.globl _adc_init
                                     39 	.globl _read_adc
                                     40 	.globl _uart1_init
                                     41 	.globl _uart1_send
                                     42 	.globl _uart1_recv
                                     43 	.globl _uart1_recv_i
                                     44 	.globl _pwm1_init
                                     45 	.globl _pwm2_init
                                     46 	.globl _pwm1ch1_enable
                                     47 	.globl _pwm1ch1_disable
                                     48 	.globl _pwm2ch1_enable
                                     49 	.globl _pwm2ch1_disable
                                     50 	.globl _pwm1_update
                                     51 	.globl _pwm2_update
                                     52 	.globl _loop
                                     53 	.globl _gpio_init
                                     54 	.globl _setstep
                                     55 ;--------------------------------------------------------
                                     56 ; ram data
                                     57 ;--------------------------------------------------------
                                     58 	.area DATA
      000001                         59 _readreg::
      000001                         60 	.ds 1
                                     61 ;--------------------------------------------------------
                                     62 ; ram data
                                     63 ;--------------------------------------------------------
                                     64 	.area INITIALIZED
      000002                         65 _stepcw::
      000002                         66 	.ds 4
      000006                         67 _stepccw::
      000006                         68 	.ds 4
                                     69 ;--------------------------------------------------------
                                     70 ; Stack segment in internal ram 
                                     71 ;--------------------------------------------------------
                                     72 	.area	SSEG
      00000A                         73 __start__stack:
      00000A                         74 	.ds	1
                                     75 
                                     76 ;--------------------------------------------------------
                                     77 ; absolute external ram data
                                     78 ;--------------------------------------------------------
                                     79 	.area DABS (ABS)
                                     80 ;--------------------------------------------------------
                                     81 ; interrupt vector 
                                     82 ;--------------------------------------------------------
                                     83 	.area HOME
      008000                         84 __interrupt_vect:
      008000 82 00 80 83             85 	int s_GSINIT ;reset
      008004 82 00 00 00             86 	int 0x0000 ;trap
      008008 82 00 00 00             87 	int 0x0000 ;int0
      00800C 82 00 00 00             88 	int 0x0000 ;int1
      008010 82 00 00 00             89 	int 0x0000 ;int2
      008014 82 00 00 00             90 	int 0x0000 ;int3
      008018 82 00 00 00             91 	int 0x0000 ;int4
      00801C 82 00 00 00             92 	int 0x0000 ;int5
      008020 82 00 00 00             93 	int 0x0000 ;int6
      008024 82 00 00 00             94 	int 0x0000 ;int7
      008028 82 00 00 00             95 	int 0x0000 ;int8
      00802C 82 00 00 00             96 	int 0x0000 ;int9
      008030 82 00 00 00             97 	int 0x0000 ;int10
      008034 82 00 00 00             98 	int 0x0000 ;int11
      008038 82 00 00 00             99 	int 0x0000 ;int12
      00803C 82 00 00 00            100 	int 0x0000 ;int13
      008040 82 00 00 00            101 	int 0x0000 ;int14
      008044 82 00 00 00            102 	int 0x0000 ;int15
      008048 82 00 00 00            103 	int 0x0000 ;int16
      00804C 82 00 00 00            104 	int 0x0000 ;int17
      008050 82 00 00 00            105 	int 0x0000 ;int18
      008054 82 00 00 00            106 	int 0x0000 ;int19
      008058 82 00 00 00            107 	int 0x0000 ;int20
      00805C 82 00 00 00            108 	int 0x0000 ;int21
      008060 82 00 00 00            109 	int 0x0000 ;int22
      008064 82 00 00 00            110 	int 0x0000 ;int23
      008068 82 00 00 00            111 	int 0x0000 ;int24
      00806C 82 00 00 00            112 	int 0x0000 ;int25
      008070 82 00 00 00            113 	int 0x0000 ;int26
      008074 82 00 00 00            114 	int 0x0000 ;int27
      008078 82 00 00 00            115 	int 0x0000 ;int28
      00807C 82 00 00 00            116 	int 0x0000 ;int29
                                    117 ;--------------------------------------------------------
                                    118 ; global & static initialisations
                                    119 ;--------------------------------------------------------
                                    120 	.area HOME
                                    121 	.area GSINIT
                                    122 	.area GSFINAL
                                    123 	.area GSINIT
      008083                        124 __sdcc_gs_init_startup:
      008083                        125 __sdcc_init_data:
                                    126 ; stm8_genXINIT() start
      008083 AE 00 01         [ 2]  127 	ldw x, #l_DATA
      008086 27 07            [ 1]  128 	jreq	00002$
      008088                        129 00001$:
      008088 72 4F 00 00      [ 1]  130 	clr (s_DATA - 1, x)
      00808C 5A               [ 2]  131 	decw x
      00808D 26 F9            [ 1]  132 	jrne	00001$
      00808F                        133 00002$:
      00808F AE 00 08         [ 2]  134 	ldw	x, #l_INITIALIZER
      008092 27 09            [ 1]  135 	jreq	00004$
      008094                        136 00003$:
      008094 D6 85 76         [ 1]  137 	ld	a, (s_INITIALIZER - 1, x)
      008097 D7 00 01         [ 1]  138 	ld	(s_INITIALIZED - 1, x), a
      00809A 5A               [ 2]  139 	decw	x
      00809B 26 F7            [ 1]  140 	jrne	00003$
      00809D                        141 00004$:
                                    142 ; stm8_genXINIT() end
                                    143 	.area GSFINAL
      00809D CC 80 80         [ 2]  144 	jp	__sdcc_program_startup
                                    145 ;--------------------------------------------------------
                                    146 ; Home
                                    147 ;--------------------------------------------------------
                                    148 	.area HOME
                                    149 	.area HOME
      008080                        150 __sdcc_program_startup:
      008080 CC 83 7A         [ 2]  151 	jp	_main
                                    152 ;	return from main will return to caller
                                    153 ;--------------------------------------------------------
                                    154 ; code
                                    155 ;--------------------------------------------------------
                                    156 	.area CODE
                                    157 ;	delay.c: 7: void delay_init()
                                    158 ;	-----------------------------------------
                                    159 ;	 function delay_init
                                    160 ;	-----------------------------------------
      0080A0                        161 _delay_init:
                                    162 ;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
      0080A0 35 04 53 47      [ 1]  163 	mov	0x5347+0, #0x04
      0080A4 81               [ 4]  164 	ret
                                    165 ;	delay.c: 12: void delay_us(unsigned long delus)
                                    166 ;	-----------------------------------------
                                    167 ;	 function delay_us
                                    168 ;	-----------------------------------------
      0080A5                        169 _delay_us:
      0080A5 52 06            [ 2]  170 	sub	sp, #6
                                    171 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080A7 4B 0A            [ 1]  172 	push	#0x0a
      0080A9 5F               [ 1]  173 	clrw	x
      0080AA 89               [ 2]  174 	pushw	x
      0080AB 4B 00            [ 1]  175 	push	#0x00
      0080AD 1E 0F            [ 2]  176 	ldw	x, (0x0f, sp)
      0080AF 89               [ 2]  177 	pushw	x
      0080B0 1E 0F            [ 2]  178 	ldw	x, (0x0f, sp)
      0080B2 89               [ 2]  179 	pushw	x
      0080B3 CD 84 A1         [ 4]  180 	call	__divulong
      0080B6 5B 08            [ 2]  181 	addw	sp, #8
      0080B8 1F 05            [ 2]  182 	ldw	(0x05, sp), x
      0080BA 17 03            [ 2]  183 	ldw	(0x03, sp), y
      0080BC 5F               [ 1]  184 	clrw	x
      0080BD 1F 01            [ 2]  185 	ldw	(0x01, sp), x
      0080BF                        186 00103$:
      0080BF 1E 01            [ 2]  187 	ldw	x, (0x01, sp)
      0080C1 90 5F            [ 1]  188 	clrw	y
      0080C3 13 05            [ 2]  189 	cpw	x, (0x05, sp)
      0080C5 90 9F            [ 1]  190 	ld	a, yl
      0080C7 12 04            [ 1]  191 	sbc	a, (0x04, sp)
      0080C9 90 9E            [ 1]  192 	ld	a, yh
      0080CB 12 03            [ 1]  193 	sbc	a, (0x03, sp)
      0080CD 24 0D            [ 1]  194 	jrnc	00101$
                                    195 ;	delay.c: 18: delay_timer(100);
      0080CF 4B 64            [ 1]  196 	push	#0x64
      0080D1 CD 81 3A         [ 4]  197 	call	_delay_timer
      0080D4 84               [ 1]  198 	pop	a
                                    199 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080D5 1E 01            [ 2]  200 	ldw	x, (0x01, sp)
      0080D7 5C               [ 2]  201 	incw	x
      0080D8 1F 01            [ 2]  202 	ldw	(0x01, sp), x
      0080DA 20 E3            [ 2]  203 	jra	00103$
      0080DC                        204 00101$:
                                    205 ;	delay.c: 20: delay_timer(delus%10);
      0080DC 4B 0A            [ 1]  206 	push	#0x0a
      0080DE 5F               [ 1]  207 	clrw	x
      0080DF 89               [ 2]  208 	pushw	x
      0080E0 4B 00            [ 1]  209 	push	#0x00
      0080E2 1E 0F            [ 2]  210 	ldw	x, (0x0f, sp)
      0080E4 89               [ 2]  211 	pushw	x
      0080E5 1E 0F            [ 2]  212 	ldw	x, (0x0f, sp)
      0080E7 89               [ 2]  213 	pushw	x
      0080E8 CD 84 31         [ 4]  214 	call	__modulong
      0080EB 5B 08            [ 2]  215 	addw	sp, #8
      0080ED 9F               [ 1]  216 	ld	a, xl
      0080EE 88               [ 1]  217 	push	a
      0080EF CD 81 3A         [ 4]  218 	call	_delay_timer
      0080F2 5B 07            [ 2]  219 	addw	sp, #7
      0080F4 81               [ 4]  220 	ret
                                    221 ;	delay.c: 23: void delay_ms(unsigned long delms)
                                    222 ;	-----------------------------------------
                                    223 ;	 function delay_ms
                                    224 ;	-----------------------------------------
      0080F5                        225 _delay_ms:
      0080F5 52 08            [ 2]  226 	sub	sp, #8
                                    227 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      0080F7 1E 0D            [ 2]  228 	ldw	x, (0x0d, sp)
      0080F9 89               [ 2]  229 	pushw	x
      0080FA 1E 0D            [ 2]  230 	ldw	x, (0x0d, sp)
      0080FC 89               [ 2]  231 	pushw	x
      0080FD 4B 64            [ 1]  232 	push	#0x64
      0080FF 5F               [ 1]  233 	clrw	x
      008100 89               [ 2]  234 	pushw	x
      008101 4B 00            [ 1]  235 	push	#0x00
      008103 CD 84 FB         [ 4]  236 	call	__mullong
      008106 5B 08            [ 2]  237 	addw	sp, #8
      008108 1F 07            [ 2]  238 	ldw	(0x07, sp), x
      00810A 17 05            [ 2]  239 	ldw	(0x05, sp), y
      00810C 5F               [ 1]  240 	clrw	x
      00810D 4F               [ 1]  241 	clr	a
      00810E 0F 01            [ 1]  242 	clr	(0x01, sp)
      008110                        243 00103$:
      008110 88               [ 1]  244 	push	a
      008111 13 08            [ 2]  245 	cpw	x, (0x08, sp)
      008113 7B 01            [ 1]  246 	ld	a, (1, sp)
      008115 12 07            [ 1]  247 	sbc	a, (0x07, sp)
      008117 7B 02            [ 1]  248 	ld	a, (0x02, sp)
      008119 12 06            [ 1]  249 	sbc	a, (0x06, sp)
      00811B 84               [ 1]  250 	pop	a
      00811C 24 19            [ 1]  251 	jrnc	00105$
                                    252 ;	delay.c: 29: delay_timer(100);
      00811E 88               [ 1]  253 	push	a
      00811F 89               [ 2]  254 	pushw	x
      008120 4B 64            [ 1]  255 	push	#0x64
      008122 CD 81 3A         [ 4]  256 	call	_delay_timer
      008125 84               [ 1]  257 	pop	a
      008126 85               [ 2]  258 	popw	x
      008127 84               [ 1]  259 	pop	a
                                    260 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      008128 1C 00 01         [ 2]  261 	addw	x, #0x0001
      00812B A9 00            [ 1]  262 	adc	a, #0x00
      00812D 88               [ 1]  263 	push	a
      00812E 7B 02            [ 1]  264 	ld	a, (0x02, sp)
      008130 A9 00            [ 1]  265 	adc	a, #0x00
      008132 6B 02            [ 1]  266 	ld	(0x02, sp), a
      008134 84               [ 1]  267 	pop	a
      008135 20 D9            [ 2]  268 	jra	00103$
      008137                        269 00105$:
      008137 5B 08            [ 2]  270 	addw	sp, #8
      008139 81               [ 4]  271 	ret
                                    272 ;	delay.c: 33: void delay_timer(unsigned char deltim)
                                    273 ;	-----------------------------------------
                                    274 ;	 function delay_timer
                                    275 ;	-----------------------------------------
      00813A                        276 _delay_timer:
                                    277 ;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
      00813A 35 01 53 40      [ 1]  278 	mov	0x5340+0, #0x01
                                    279 ;	delay.c: 36: while(TIM4_CNTR<deltim);
      00813E                        280 00101$:
      00813E AE 53 46         [ 2]  281 	ldw	x, #0x5346
      008141 F6               [ 1]  282 	ld	a, (x)
      008142 11 03            [ 1]  283 	cp	a, (0x03, sp)
      008144 25 F8            [ 1]  284 	jrc	00101$
                                    285 ;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
      008146 35 00 53 40      [ 1]  286 	mov	0x5340+0, #0x00
                                    287 ;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
      00814A 35 00 53 46      [ 1]  288 	mov	0x5346+0, #0x00
      00814E 81               [ 4]  289 	ret
                                    290 ;	periph_stm8s.c: 16: void clock_init()
                                    291 ;	-----------------------------------------
                                    292 ;	 function clock_init
                                    293 ;	-----------------------------------------
      00814F                        294 _clock_init:
                                    295 ;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
      00814F 35 00 50 C6      [ 1]  296 	mov	0x50c6+0, #0x00
                                    297 ;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
      008153 35 01 50 C0      [ 1]  298 	mov	0x50c0+0, #0x01
      008157 81               [ 4]  299 	ret
                                    300 ;	periph_stm8s.c: 24: void i2c_init()
                                    301 ;	-----------------------------------------
                                    302 ;	 function i2c_init
                                    303 ;	-----------------------------------------
      008158                        304 _i2c_init:
                                    305 ;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
      008158 35 00 52 10      [ 1]  306 	mov	0x5210+0, #0x00
                                    307 ;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
      00815C 35 10 52 12      [ 1]  308 	mov	0x5212+0, #0x10
                                    309 ;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
      008160 35 00 52 1C      [ 1]  310 	mov	0x521c+0, #0x00
                                    311 ;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
      008164 35 80 52 1B      [ 1]  312 	mov	0x521b+0, #0x80
                                    313 ;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
      008168 35 40 52 14      [ 1]  314 	mov	0x5214+0, #0x40
                                    315 ;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
      00816C 35 11 52 1D      [ 1]  316 	mov	0x521d+0, #0x11
                                    317 ;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
      008170 35 01 52 10      [ 1]  318 	mov	0x5210+0, #0x01
      008174 81               [ 4]  319 	ret
                                    320 ;	periph_stm8s.c: 40: void i2c_set_start()
                                    321 ;	-----------------------------------------
                                    322 ;	 function i2c_set_start
                                    323 ;	-----------------------------------------
      008175                        324 _i2c_set_start:
                                    325 ;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
      008175 72 10 52 11      [ 1]  326 	bset	0x5211, #0
      008179 81               [ 4]  327 	ret
                                    328 ;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
                                    329 ;	-----------------------------------------
                                    330 ;	 function i2c_set_address
                                    331 ;	-----------------------------------------
      00817A                        332 _i2c_set_address:
                                    333 ;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
      00817A 7B 03            [ 1]  334 	ld	a, (0x03, sp)
      00817C 97               [ 1]  335 	ld	xl, a
      00817D 58               [ 2]  336 	sllw	x
      00817E 7B 04            [ 1]  337 	ld	a, (0x04, sp)
      008180 A1 01            [ 1]  338 	cp	a, #0x01
      008182 26 09            [ 1]  339 	jrne	00104$
      008184 9F               [ 1]  340 	ld	a, xl
      008185 1A 04            [ 1]  341 	or	a, (0x04, sp)
      008187 AE 52 16         [ 2]  342 	ldw	x, #0x5216
      00818A F7               [ 1]  343 	ld	(x), a
      00818B 20 0D            [ 2]  344 	jra	00106$
      00818D                        345 00104$:
                                    346 ;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
      00818D 7B 04            [ 1]  347 	ld	a, (0x04, sp)
      00818F A1 FE            [ 1]  348 	cp	a, #0xfe
      008191 26 07            [ 1]  349 	jrne	00106$
      008193 9F               [ 1]  350 	ld	a, xl
      008194 14 04            [ 1]  351 	and	a, (0x04, sp)
      008196 AE 52 16         [ 2]  352 	ldw	x, #0x5216
      008199 F7               [ 1]  353 	ld	(x), a
      00819A                        354 00106$:
      00819A 81               [ 4]  355 	ret
                                    356 ;	periph_stm8s.c: 52: void i2c_set_stop()
                                    357 ;	-----------------------------------------
                                    358 ;	 function i2c_set_stop
                                    359 ;	-----------------------------------------
      00819B                        360 _i2c_set_stop:
                                    361 ;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
      00819B AE 52 11         [ 2]  362 	ldw	x, #0x5211
      00819E F6               [ 1]  363 	ld	a, (x)
      00819F AA 02            [ 1]  364 	or	a, #0x02
      0081A1 F7               [ 1]  365 	ld	(x), a
      0081A2 81               [ 4]  366 	ret
                                    367 ;	periph_stm8s.c: 57: void i2c_clear_ack()
                                    368 ;	-----------------------------------------
                                    369 ;	 function i2c_clear_ack
                                    370 ;	-----------------------------------------
      0081A3                        371 _i2c_clear_ack:
                                    372 ;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
      0081A3 AE 52 11         [ 2]  373 	ldw	x, #0x5211
      0081A6 F6               [ 1]  374 	ld	a, (x)
      0081A7 A4 FB            [ 1]  375 	and	a, #0xfb
      0081A9 F7               [ 1]  376 	ld	(x), a
      0081AA 81               [ 4]  377 	ret
                                    378 ;	periph_stm8s.c: 62: void i2c_set_ack()
                                    379 ;	-----------------------------------------
                                    380 ;	 function i2c_set_ack
                                    381 ;	-----------------------------------------
      0081AB                        382 _i2c_set_ack:
                                    383 ;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
      0081AB AE 52 11         [ 2]  384 	ldw	x, #0x5211
      0081AE F6               [ 1]  385 	ld	a, (x)
      0081AF AA 04            [ 1]  386 	or	a, #0x04
      0081B1 F7               [ 1]  387 	ld	(x), a
      0081B2 81               [ 4]  388 	ret
                                    389 ;	periph_stm8s.c: 67: void i2c_ack_pos_current()
                                    390 ;	-----------------------------------------
                                    391 ;	 function i2c_ack_pos_current
                                    392 ;	-----------------------------------------
      0081B3                        393 _i2c_ack_pos_current:
                                    394 ;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
      0081B3 AE 52 11         [ 2]  395 	ldw	x, #0x5211
      0081B6 F6               [ 1]  396 	ld	a, (x)
      0081B7 A4 F7            [ 1]  397 	and	a, #0xf7
      0081B9 F7               [ 1]  398 	ld	(x), a
      0081BA 81               [ 4]  399 	ret
                                    400 ;	periph_stm8s.c: 72: void i2c_ack_pos_next()
                                    401 ;	-----------------------------------------
                                    402 ;	 function i2c_ack_pos_next
                                    403 ;	-----------------------------------------
      0081BB                        404 _i2c_ack_pos_next:
                                    405 ;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
      0081BB AE 52 11         [ 2]  406 	ldw	x, #0x5211
      0081BE F6               [ 1]  407 	ld	a, (x)
      0081BF AA 08            [ 1]  408 	or	a, #0x08
      0081C1 F7               [ 1]  409 	ld	(x), a
      0081C2 81               [ 4]  410 	ret
                                    411 ;	periph_stm8s.c: 77: void i2c_poll_SB()
                                    412 ;	-----------------------------------------
                                    413 ;	 function i2c_poll_SB
                                    414 ;	-----------------------------------------
      0081C3                        415 _i2c_poll_SB:
                                    416 ;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
      0081C3                        417 00101$:
      0081C3 AE 52 17         [ 2]  418 	ldw	x, #0x5217
      0081C6 F6               [ 1]  419 	ld	a, (x)
      0081C7 A4 01            [ 1]  420 	and	a, #0x01
      0081C9 A1 01            [ 1]  421 	cp	a, #0x01
      0081CB 26 F6            [ 1]  422 	jrne	00101$
      0081CD 81               [ 4]  423 	ret
                                    424 ;	periph_stm8s.c: 82: void i2c_poll_ADDR()
                                    425 ;	-----------------------------------------
                                    426 ;	 function i2c_poll_ADDR
                                    427 ;	-----------------------------------------
      0081CE                        428 _i2c_poll_ADDR:
                                    429 ;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
      0081CE                        430 00101$:
      0081CE AE 52 17         [ 2]  431 	ldw	x, #0x5217
      0081D1 F6               [ 1]  432 	ld	a, (x)
      0081D2 A4 02            [ 1]  433 	and	a, #0x02
      0081D4 A1 02            [ 1]  434 	cp	a, #0x02
      0081D6 26 F6            [ 1]  435 	jrne	00101$
      0081D8 81               [ 4]  436 	ret
                                    437 ;	periph_stm8s.c: 87: void i2c_poll_BTF()
                                    438 ;	-----------------------------------------
                                    439 ;	 function i2c_poll_BTF
                                    440 ;	-----------------------------------------
      0081D9                        441 _i2c_poll_BTF:
                                    442 ;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
      0081D9                        443 00101$:
      0081D9 AE 52 17         [ 2]  444 	ldw	x, #0x5217
      0081DC F6               [ 1]  445 	ld	a, (x)
      0081DD A4 04            [ 1]  446 	and	a, #0x04
      0081DF A1 04            [ 1]  447 	cp	a, #0x04
      0081E1 26 F6            [ 1]  448 	jrne	00101$
      0081E3 81               [ 4]  449 	ret
                                    450 ;	periph_stm8s.c: 92: void i2c_poll_TXE()
                                    451 ;	-----------------------------------------
                                    452 ;	 function i2c_poll_TXE
                                    453 ;	-----------------------------------------
      0081E4                        454 _i2c_poll_TXE:
                                    455 ;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
      0081E4                        456 00101$:
      0081E4 AE 52 17         [ 2]  457 	ldw	x, #0x5217
      0081E7 F6               [ 1]  458 	ld	a, (x)
      0081E8 A4 80            [ 1]  459 	and	a, #0x80
      0081EA A1 80            [ 1]  460 	cp	a, #0x80
      0081EC 26 F6            [ 1]  461 	jrne	00101$
      0081EE 81               [ 4]  462 	ret
                                    463 ;	periph_stm8s.c: 97: void i2c_poll_RXNE()
                                    464 ;	-----------------------------------------
                                    465 ;	 function i2c_poll_RXNE
                                    466 ;	-----------------------------------------
      0081EF                        467 _i2c_poll_RXNE:
                                    468 ;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
      0081EF                        469 00101$:
      0081EF AE 52 17         [ 2]  470 	ldw	x, #0x5217
      0081F2 F6               [ 1]  471 	ld	a, (x)
      0081F3 A4 40            [ 1]  472 	and	a, #0x40
      0081F5 A1 40            [ 1]  473 	cp	a, #0x40
      0081F7 26 F6            [ 1]  474 	jrne	00101$
      0081F9 81               [ 4]  475 	ret
                                    476 ;	periph_stm8s.c: 102: void i2c_clear_bits()
                                    477 ;	-----------------------------------------
                                    478 ;	 function i2c_clear_bits
                                    479 ;	-----------------------------------------
      0081FA                        480 _i2c_clear_bits:
                                    481 ;	periph_stm8s.c: 104: readreg = I2C_SR1;
      0081FA AE 52 17         [ 2]  482 	ldw	x, #0x5217
      0081FD F6               [ 1]  483 	ld	a, (x)
      0081FE C7 00 01         [ 1]  484 	ld	_readreg+0, a
      008201 81               [ 4]  485 	ret
                                    486 ;	periph_stm8s.c: 107: void i2c_clear_ADDR()
                                    487 ;	-----------------------------------------
                                    488 ;	 function i2c_clear_ADDR
                                    489 ;	-----------------------------------------
      008202                        490 _i2c_clear_ADDR:
                                    491 ;	periph_stm8s.c: 109: readreg = I2C_SR1;
      008202 AE 52 17         [ 2]  492 	ldw	x, #0x5217
      008205 F6               [ 1]  493 	ld	a, (x)
                                    494 ;	periph_stm8s.c: 110: readreg = I2C_SR3;
      008206 AE 52 19         [ 2]  495 	ldw	x, #0x5219
      008209 F6               [ 1]  496 	ld	a, (x)
      00820A C7 00 01         [ 1]  497 	ld	_readreg+0, a
      00820D 81               [ 4]  498 	ret
                                    499 ;	periph_stm8s.c: 113: void i2c_enable_interrupts()
                                    500 ;	-----------------------------------------
                                    501 ;	 function i2c_enable_interrupts
                                    502 ;	-----------------------------------------
      00820E                        503 _i2c_enable_interrupts:
                                    504 ;	periph_stm8s.c: 115: I2C_ITR = 0x07;
      00820E 35 07 52 1A      [ 1]  505 	mov	0x521a+0, #0x07
      008212 81               [ 4]  506 	ret
                                    507 ;	periph_stm8s.c: 117: void i2c_disable_interrupts()
                                    508 ;	-----------------------------------------
                                    509 ;	 function i2c_disable_interrupts
                                    510 ;	-----------------------------------------
      008213                        511 _i2c_disable_interrupts:
                                    512 ;	periph_stm8s.c: 119: I2C_ITR = 0x00;
      008213 35 00 52 1A      [ 1]  513 	mov	0x521a+0, #0x00
      008217 81               [ 4]  514 	ret
                                    515 ;	periph_stm8s.c: 124: void adc_init()
                                    516 ;	-----------------------------------------
                                    517 ;	 function adc_init
                                    518 ;	-----------------------------------------
      008218                        519 _adc_init:
                                    520 ;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
      008218 35 40 54 01      [ 1]  521 	mov	0x5401+0, #0x40
                                    522 ;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
      00821C 35 08 54 02      [ 1]  523 	mov	0x5402+0, #0x08
                                    524 ;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
      008220 72 10 54 01      [ 1]  525 	bset	0x5401, #0
      008224 81               [ 4]  526 	ret
                                    527 ;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
                                    528 ;	-----------------------------------------
                                    529 ;	 function read_adc
                                    530 ;	-----------------------------------------
      008225                        531 _read_adc:
      008225 52 04            [ 2]  532 	sub	sp, #4
                                    533 ;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
      008227 AE 54 00         [ 2]  534 	ldw	x, #0x5400
      00822A F6               [ 1]  535 	ld	a, (x)
      00822B A4 F0            [ 1]  536 	and	a, #0xf0
      00822D F7               [ 1]  537 	ld	(x), a
                                    538 ;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
      00822E AE 54 00         [ 2]  539 	ldw	x, #0x5400
      008231 F6               [ 1]  540 	ld	a, (x)
      008232 1A 07            [ 1]  541 	or	a, (0x07, sp)
      008234 AE 54 00         [ 2]  542 	ldw	x, #0x5400
      008237 F7               [ 1]  543 	ld	(x), a
                                    544 ;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
      008238 72 10 54 01      [ 1]  545 	bset	0x5401, #0
                                    546 ;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
      00823C                        547 00101$:
      00823C AE 54 00         [ 2]  548 	ldw	x, #0x5400
      00823F F6               [ 1]  549 	ld	a, (x)
      008240 4D               [ 1]  550 	tnz	a
      008241 2A F9            [ 1]  551 	jrpl	00101$
                                    552 ;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
      008243 AE 54 04         [ 2]  553 	ldw	x, #0x5404
      008246 F6               [ 1]  554 	ld	a, (x)
      008247 0F 03            [ 1]  555 	clr	(0x03, sp)
      008249 6B 01            [ 1]  556 	ld	(0x01, sp), a
      00824B 0F 02            [ 1]  557 	clr	(0x02, sp)
      00824D AE 54 05         [ 2]  558 	ldw	x, #0x5405
      008250 F6               [ 1]  559 	ld	a, (x)
      008251 5F               [ 1]  560 	clrw	x
      008252 97               [ 1]  561 	ld	xl, a
      008253 72 FB 01         [ 2]  562 	addw	x, (0x01, sp)
                                    563 ;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
      008256 90 AE 54 00      [ 2]  564 	ldw	y, #0x5400
      00825A 90 F6            [ 1]  565 	ld	a, (y)
      00825C 90 AE 54 00      [ 2]  566 	ldw	y, #0x5400
      008260 90 F7            [ 1]  567 	ld	(y), a
                                    568 ;	periph_stm8s.c: 146: return adcval;
      008262 5B 04            [ 2]  569 	addw	sp, #4
      008264 81               [ 4]  570 	ret
                                    571 ;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
                                    572 ;	-----------------------------------------
                                    573 ;	 function uart1_init
                                    574 ;	-----------------------------------------
      008265                        575 _uart1_init:
                                    576 ;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
      008265 35 68 52 32      [ 1]  577 	mov	0x5232+0, #0x68
                                    578 ;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
      008269 35 03 52 33      [ 1]  579 	mov	0x5233+0, #0x03
                                    580 ;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
      00826D AE 52 34         [ 2]  581 	ldw	x, #0x5234
      008270 F6               [ 1]  582 	ld	a, (x)
      008271 AE 52 34         [ 2]  583 	ldw	x, #0x5234
      008274 F7               [ 1]  584 	ld	(x), a
                                    585 ;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
      008275 AE 52 36         [ 2]  586 	ldw	x, #0x5236
      008278 F6               [ 1]  587 	ld	a, (x)
      008279 AE 52 36         [ 2]  588 	ldw	x, #0x5236
      00827C F7               [ 1]  589 	ld	(x), a
                                    590 ;	periph_stm8s.c: 161: if(rxien==1) 
      00827D 7B 03            [ 1]  591 	ld	a, (0x03, sp)
      00827F A1 01            [ 1]  592 	cp	a, #0x01
      008281 26 0B            [ 1]  593 	jrne	00102$
                                    594 ;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
      008283 AE 52 35         [ 2]  595 	ldw	x, #0x5235
      008286 F6               [ 1]  596 	ld	a, (x)
      008287 AA 20            [ 1]  597 	or	a, #0x20
      008289 F7               [ 1]  598 	ld	(x), a
                                    599 ;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
      00828A 35 00 7F 74      [ 1]  600 	mov	0x7f74+0, #0x00
      00828E                        601 00102$:
                                    602 ;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
      00828E AE 52 35         [ 2]  603 	ldw	x, #0x5235
      008291 F6               [ 1]  604 	ld	a, (x)
      008292 AA 08            [ 1]  605 	or	a, #0x08
      008294 F7               [ 1]  606 	ld	(x), a
                                    607 ;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
      008295 AE 52 35         [ 2]  608 	ldw	x, #0x5235
      008298 F6               [ 1]  609 	ld	a, (x)
      008299 AA 04            [ 1]  610 	or	a, #0x04
      00829B F7               [ 1]  611 	ld	(x), a
      00829C 81               [ 4]  612 	ret
                                    613 ;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
                                    614 ;	-----------------------------------------
                                    615 ;	 function uart1_send
                                    616 ;	-----------------------------------------
      00829D                        617 _uart1_send:
                                    618 ;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
      00829D AE 52 31         [ 2]  619 	ldw	x, #0x5231
      0082A0 7B 03            [ 1]  620 	ld	a, (0x03, sp)
      0082A2 F7               [ 1]  621 	ld	(x), a
                                    622 ;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
      0082A3                        623 00101$:
      0082A3 AE 52 30         [ 2]  624 	ldw	x, #0x5230
      0082A6 F6               [ 1]  625 	ld	a, (x)
      0082A7 A4 80            [ 1]  626 	and	a, #0x80
      0082A9 A1 80            [ 1]  627 	cp	a, #0x80
      0082AB 26 F6            [ 1]  628 	jrne	00101$
      0082AD 81               [ 4]  629 	ret
                                    630 ;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
                                    631 ;	-----------------------------------------
                                    632 ;	 function uart1_recv
                                    633 ;	-----------------------------------------
      0082AE                        634 _uart1_recv:
                                    635 ;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
      0082AE AE 52 30         [ 2]  636 	ldw	x, #0x5230
      0082B1 F6               [ 1]  637 	ld	a, (x)
      0082B2 A4 20            [ 1]  638 	and	a, #0x20
      0082B4 A1 20            [ 1]  639 	cp	a, #0x20
      0082B6 26 05            [ 1]  640 	jrne	00102$
                                    641 ;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082B8 AE 52 31         [ 2]  642 	ldw	x, #0x5231
      0082BB F6               [ 1]  643 	ld	a, (x)
                                    644 ;	periph_stm8s.c: 184: else urecv=0;
      0082BC 21                     645 	.byte 0x21
      0082BD                        646 00102$:
      0082BD 4F               [ 1]  647 	clr	a
      0082BE                        648 00103$:
                                    649 ;	periph_stm8s.c: 185: return urecv;
      0082BE 81               [ 4]  650 	ret
                                    651 ;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
                                    652 ;	-----------------------------------------
                                    653 ;	 function uart1_recv_i
                                    654 ;	-----------------------------------------
      0082BF                        655 _uart1_recv_i:
                                    656 ;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082BF AE 52 31         [ 2]  657 	ldw	x, #0x5231
      0082C2 F6               [ 1]  658 	ld	a, (x)
                                    659 ;	periph_stm8s.c: 192: return urecv;
      0082C3 81               [ 4]  660 	ret
                                    661 ;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
                                    662 ;	-----------------------------------------
                                    663 ;	 function pwm1_init
                                    664 ;	-----------------------------------------
      0082C4                        665 _pwm1_init:
      0082C4 52 02            [ 2]  666 	sub	sp, #2
                                    667 ;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
      0082C6 35 00 52 60      [ 1]  668 	mov	0x5260+0, #0x00
                                    669 ;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
      0082CA 35 00 52 61      [ 1]  670 	mov	0x5261+0, #0x00
                                    671 ;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
      0082CE 7B 05            [ 1]  672 	ld	a, (0x05, sp)
      0082D0 0F 01            [ 1]  673 	clr	(0x01, sp)
      0082D2 AE 52 62         [ 2]  674 	ldw	x, #0x5262
      0082D5 F7               [ 1]  675 	ld	(x), a
                                    676 ;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
      0082D6 7B 06            [ 1]  677 	ld	a, (0x06, sp)
      0082D8 95               [ 1]  678 	ld	xh, a
      0082D9 4F               [ 1]  679 	clr	a
      0082DA 9E               [ 1]  680 	ld	a, xh
      0082DB AE 52 63         [ 2]  681 	ldw	x, #0x5263
      0082DE F7               [ 1]  682 	ld	(x), a
                                    683 ;	periph_stm8s.c: 204: pwm1ch1_enable();
      0082DF CD 83 3A         [ 4]  684 	call	_pwm1ch1_enable
                                    685 ;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
      0082E2 AE 52 5C         [ 2]  686 	ldw	x, #0x525c
      0082E5 F6               [ 1]  687 	ld	a, (x)
      0082E6 AE 52 5C         [ 2]  688 	ldw	x, #0x525c
      0082E9 F7               [ 1]  689 	ld	(x), a
                                    690 ;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
      0082EA 35 60 52 58      [ 1]  691 	mov	0x5258+0, #0x60
                                    692 ;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
      0082EE 5F               [ 1]  693 	clrw	x
      0082EF 89               [ 2]  694 	pushw	x
      0082F0 CD 83 4E         [ 4]  695 	call	_pwm1_update
      0082F3 5B 02            [ 2]  696 	addw	sp, #2
                                    697 ;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
      0082F5 35 80 52 6D      [ 1]  698 	mov	0x526d+0, #0x80
                                    699 ;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
      0082F9 AE 52 50         [ 2]  700 	ldw	x, #0x5250
      0082FC F6               [ 1]  701 	ld	a, (x)
      0082FD AA 01            [ 1]  702 	or	a, #0x01
      0082FF F7               [ 1]  703 	ld	(x), a
      008300 5B 02            [ 2]  704 	addw	sp, #2
      008302 81               [ 4]  705 	ret
                                    706 ;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
                                    707 ;	-----------------------------------------
                                    708 ;	 function pwm2_init
                                    709 ;	-----------------------------------------
      008303                        710 _pwm2_init:
      008303 52 02            [ 2]  711 	sub	sp, #2
                                    712 ;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
      008305 35 00 53 0E      [ 1]  713 	mov	0x530e+0, #0x00
                                    714 ;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
      008309 7B 05            [ 1]  715 	ld	a, (0x05, sp)
      00830B 0F 01            [ 1]  716 	clr	(0x01, sp)
      00830D AE 53 0F         [ 2]  717 	ldw	x, #0x530f
      008310 F7               [ 1]  718 	ld	(x), a
                                    719 ;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
      008311 7B 06            [ 1]  720 	ld	a, (0x06, sp)
      008313 95               [ 1]  721 	ld	xh, a
      008314 4F               [ 1]  722 	clr	a
      008315 9E               [ 1]  723 	ld	a, xh
      008316 AE 53 10         [ 2]  724 	ldw	x, #0x5310
      008319 F7               [ 1]  725 	ld	(x), a
                                    726 ;	periph_stm8s.c: 217: pwm2ch1_enable();
      00831A CD 83 44         [ 4]  727 	call	_pwm2ch1_enable
                                    728 ;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
      00831D AE 53 0A         [ 2]  729 	ldw	x, #0x530a
      008320 F6               [ 1]  730 	ld	a, (x)
      008321 AE 53 0A         [ 2]  731 	ldw	x, #0x530a
      008324 F7               [ 1]  732 	ld	(x), a
                                    733 ;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
      008325 35 60 53 07      [ 1]  734 	mov	0x5307+0, #0x60
                                    735 ;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
      008329 5F               [ 1]  736 	clrw	x
      00832A 89               [ 2]  737 	pushw	x
      00832B CD 83 64         [ 4]  738 	call	_pwm2_update
      00832E 5B 02            [ 2]  739 	addw	sp, #2
                                    740 ;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
      008330 AE 53 00         [ 2]  741 	ldw	x, #0x5300
      008333 F6               [ 1]  742 	ld	a, (x)
      008334 AA 01            [ 1]  743 	or	a, #0x01
      008336 F7               [ 1]  744 	ld	(x), a
      008337 5B 02            [ 2]  745 	addw	sp, #2
      008339 81               [ 4]  746 	ret
                                    747 ;	periph_stm8s.c: 224: void pwm1ch1_enable()
                                    748 ;	-----------------------------------------
                                    749 ;	 function pwm1ch1_enable
                                    750 ;	-----------------------------------------
      00833A                        751 _pwm1ch1_enable:
                                    752 ;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
      00833A 72 10 52 5C      [ 1]  753 	bset	0x525c, #0
      00833E 81               [ 4]  754 	ret
                                    755 ;	periph_stm8s.c: 229: void pwm1ch1_disable()
                                    756 ;	-----------------------------------------
                                    757 ;	 function pwm1ch1_disable
                                    758 ;	-----------------------------------------
      00833F                        759 _pwm1ch1_disable:
                                    760 ;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
      00833F 72 11 52 5C      [ 1]  761 	bres	0x525c, #0
      008343 81               [ 4]  762 	ret
                                    763 ;	periph_stm8s.c: 234: void pwm2ch1_enable()
                                    764 ;	-----------------------------------------
                                    765 ;	 function pwm2ch1_enable
                                    766 ;	-----------------------------------------
      008344                        767 _pwm2ch1_enable:
                                    768 ;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
      008344 72 10 53 0A      [ 1]  769 	bset	0x530a, #0
      008348 81               [ 4]  770 	ret
                                    771 ;	periph_stm8s.c: 239: void pwm2ch1_disable()
                                    772 ;	-----------------------------------------
                                    773 ;	 function pwm2ch1_disable
                                    774 ;	-----------------------------------------
      008349                        775 _pwm2ch1_disable:
                                    776 ;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
      008349 72 11 53 0A      [ 1]  777 	bres	0x530a, #0
      00834D 81               [ 4]  778 	ret
                                    779 ;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
                                    780 ;	-----------------------------------------
                                    781 ;	 function pwm1_update
                                    782 ;	-----------------------------------------
      00834E                        783 _pwm1_update:
      00834E 52 02            [ 2]  784 	sub	sp, #2
                                    785 ;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
      008350 7B 06            [ 1]  786 	ld	a, (0x06, sp)
      008352 95               [ 1]  787 	ld	xh, a
      008353 4F               [ 1]  788 	clr	a
      008354 9E               [ 1]  789 	ld	a, xh
      008355 AE 52 66         [ 2]  790 	ldw	x, #0x5266
      008358 F7               [ 1]  791 	ld	(x), a
                                    792 ;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
      008359 7B 05            [ 1]  793 	ld	a, (0x05, sp)
      00835B 0F 01            [ 1]  794 	clr	(0x01, sp)
      00835D AE 52 65         [ 2]  795 	ldw	x, #0x5265
      008360 F7               [ 1]  796 	ld	(x), a
      008361 5B 02            [ 2]  797 	addw	sp, #2
      008363 81               [ 4]  798 	ret
                                    799 ;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
                                    800 ;	-----------------------------------------
                                    801 ;	 function pwm2_update
                                    802 ;	-----------------------------------------
      008364                        803 _pwm2_update:
      008364 52 02            [ 2]  804 	sub	sp, #2
                                    805 ;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
      008366 7B 06            [ 1]  806 	ld	a, (0x06, sp)
      008368 95               [ 1]  807 	ld	xh, a
      008369 4F               [ 1]  808 	clr	a
      00836A 9E               [ 1]  809 	ld	a, xh
      00836B AE 53 12         [ 2]  810 	ldw	x, #0x5312
      00836E F7               [ 1]  811 	ld	(x), a
                                    812 ;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
      00836F 7B 05            [ 1]  813 	ld	a, (0x05, sp)
      008371 0F 01            [ 1]  814 	clr	(0x01, sp)
      008373 AE 53 11         [ 2]  815 	ldw	x, #0x5311
      008376 F7               [ 1]  816 	ld	(x), a
      008377 5B 02            [ 2]  817 	addw	sp, #2
      008379 81               [ 4]  818 	ret
                                    819 ;	main.c: 40: int main()
                                    820 ;	-----------------------------------------
                                    821 ;	 function main
                                    822 ;	-----------------------------------------
      00837A                        823 _main:
                                    824 ;	main.c: 42: clock_init();
      00837A CD 81 4F         [ 4]  825 	call	_clock_init
                                    826 ;	main.c: 43: delay_init();
      00837D CD 80 A0         [ 4]  827 	call	_delay_init
                                    828 ;	main.c: 44: gpio_init();
      008380 CD 83 F9         [ 4]  829 	call	_gpio_init
                                    830 ;	main.c: 46: loop();
      008383 CD 83 88         [ 4]  831 	call	_loop
                                    832 ;	main.c: 47: return 0;
      008386 5F               [ 1]  833 	clrw	x
      008387 81               [ 4]  834 	ret
                                    835 ;	main.c: 53: void loop()
                                    836 ;	-----------------------------------------
                                    837 ;	 function loop
                                    838 ;	-----------------------------------------
      008388                        839 _loop:
      008388 52 05            [ 2]  840 	sub	sp, #5
                                    841 ;	main.c: 57: while(1)
      00838A                        842 00110$:
                                    843 ;	main.c: 59: if((BTNIDR|BTNCW_MASKL)==BTNCW_MASKL) //If CW Button is pressed (Active Low)
      00838A AE 50 01         [ 2]  844 	ldw	x, #0x5001
      00838D F6               [ 1]  845 	ld	a, (x)
      00838E AA FD            [ 1]  846 	or	a, #0xfd
      008390 A1 FD            [ 1]  847 	cp	a, #0xfd
      008392 26 29            [ 1]  848 	jrne	00107$
                                    849 ;	main.c: 61: for(sti=0;sti<4;sti++)
      008394 AE 00 02         [ 2]  850 	ldw	x, #_stepcw+0
      008397 1F 02            [ 2]  851 	ldw	(0x02, sp), x
      008399 0F 01            [ 1]  852 	clr	(0x01, sp)
      00839B                        853 00112$:
                                    854 ;	main.c: 63: setstep(stepcw[sti]); //Run CW sequence
      00839B 5F               [ 1]  855 	clrw	x
      00839C 7B 01            [ 1]  856 	ld	a, (0x01, sp)
      00839E 97               [ 1]  857 	ld	xl, a
      00839F 72 FB 02         [ 2]  858 	addw	x, (0x02, sp)
      0083A2 F6               [ 1]  859 	ld	a, (x)
      0083A3 88               [ 1]  860 	push	a
      0083A4 CD 84 27         [ 4]  861 	call	_setstep
      0083A7 84               [ 1]  862 	pop	a
                                    863 ;	main.c: 64: delay_ms(10);
      0083A8 4B 0A            [ 1]  864 	push	#0x0a
      0083AA 5F               [ 1]  865 	clrw	x
      0083AB 89               [ 2]  866 	pushw	x
      0083AC 4B 00            [ 1]  867 	push	#0x00
      0083AE CD 80 F5         [ 4]  868 	call	_delay_ms
      0083B1 5B 04            [ 2]  869 	addw	sp, #4
                                    870 ;	main.c: 61: for(sti=0;sti<4;sti++)
      0083B3 0C 01            [ 1]  871 	inc	(0x01, sp)
      0083B5 7B 01            [ 1]  872 	ld	a, (0x01, sp)
      0083B7 A1 04            [ 1]  873 	cp	a, #0x04
      0083B9 25 E0            [ 1]  874 	jrc	00112$
      0083BB 20 CD            [ 2]  875 	jra	00110$
      0083BD                        876 00107$:
                                    877 ;	main.c: 67: else if((BTNIDR|BTNCCW_MASKL)==BTNCCW_MASKL) //If CCW Button is pressed (Active Low)
      0083BD AE 50 01         [ 2]  878 	ldw	x, #0x5001
      0083C0 F6               [ 1]  879 	ld	a, (x)
      0083C1 AA FB            [ 1]  880 	or	a, #0xfb
      0083C3 A1 FB            [ 1]  881 	cp	a, #0xfb
      0083C5 26 29            [ 1]  882 	jrne	00104$
                                    883 ;	main.c: 69: for(sti=0;sti<4;sti++)
      0083C7 AE 00 06         [ 2]  884 	ldw	x, #_stepccw+0
      0083CA 1F 04            [ 2]  885 	ldw	(0x04, sp), x
      0083CC 0F 01            [ 1]  886 	clr	(0x01, sp)
      0083CE                        887 00114$:
                                    888 ;	main.c: 71: setstep(stepccw[sti]); //Run CCW sequence
      0083CE 5F               [ 1]  889 	clrw	x
      0083CF 7B 01            [ 1]  890 	ld	a, (0x01, sp)
      0083D1 97               [ 1]  891 	ld	xl, a
      0083D2 72 FB 04         [ 2]  892 	addw	x, (0x04, sp)
      0083D5 F6               [ 1]  893 	ld	a, (x)
      0083D6 88               [ 1]  894 	push	a
      0083D7 CD 84 27         [ 4]  895 	call	_setstep
      0083DA 84               [ 1]  896 	pop	a
                                    897 ;	main.c: 72: delay_ms(10);
      0083DB 4B 0A            [ 1]  898 	push	#0x0a
      0083DD 5F               [ 1]  899 	clrw	x
      0083DE 89               [ 2]  900 	pushw	x
      0083DF 4B 00            [ 1]  901 	push	#0x00
      0083E1 CD 80 F5         [ 4]  902 	call	_delay_ms
      0083E4 5B 04            [ 2]  903 	addw	sp, #4
                                    904 ;	main.c: 69: for(sti=0;sti<4;sti++)
      0083E6 0C 01            [ 1]  905 	inc	(0x01, sp)
      0083E8 7B 01            [ 1]  906 	ld	a, (0x01, sp)
      0083EA A1 04            [ 1]  907 	cp	a, #0x04
      0083EC 25 E0            [ 1]  908 	jrc	00114$
      0083EE 20 9A            [ 2]  909 	jra	00110$
      0083F0                        910 00104$:
                                    911 ;	main.c: 77: MOTODR = 0x00;
      0083F0 35 00 50 0A      [ 1]  912 	mov	0x500a+0, #0x00
      0083F4 20 94            [ 2]  913 	jra	00110$
      0083F6 5B 05            [ 2]  914 	addw	sp, #5
      0083F8 81               [ 4]  915 	ret
                                    916 ;	main.c: 83: void gpio_init()
                                    917 ;	-----------------------------------------
                                    918 ;	 function gpio_init
                                    919 ;	-----------------------------------------
      0083F9                        920 _gpio_init:
                                    921 ;	main.c: 85: MOTDDR |= (OUTPUT<<MOTP1)|(OUTPUT<<MOTP2)|(OUTPUT<<MOTP3)|(OUTPUT<<MOTP4);
      0083F9 AE 50 0C         [ 2]  922 	ldw	x, #0x500c
      0083FC F6               [ 1]  923 	ld	a, (x)
      0083FD AA 78            [ 1]  924 	or	a, #0x78
      0083FF F7               [ 1]  925 	ld	(x), a
                                    926 ;	main.c: 86: MOTCR1 |= (pushpull<<MOTP1)|(pushpull<<MOTP2)|(pushpull<<MOTP3)|(pushpull<<MOTP4);
      008400 AE 50 0D         [ 2]  927 	ldw	x, #0x500d
      008403 F6               [ 1]  928 	ld	a, (x)
      008404 AA 78            [ 1]  929 	or	a, #0x78
      008406 F7               [ 1]  930 	ld	(x), a
                                    931 ;	main.c: 87: MOTCR2 |= (speed_2MHz<<MOTP1)|(speed_2MHz<<MOTP2)|(speed_2MHz<<MOTP3)|(speed_2MHz<<MOTP4);
      008407 AE 50 0E         [ 2]  932 	ldw	x, #0x500e
      00840A F6               [ 1]  933 	ld	a, (x)
      00840B AE 50 0E         [ 2]  934 	ldw	x, #0x500e
      00840E F7               [ 1]  935 	ld	(x), a
                                    936 ;	main.c: 89: BTNDDR |= (INPUT<<BTNCW)|(INPUT<<BTNCCW);
      00840F AE 50 02         [ 2]  937 	ldw	x, #0x5002
      008412 F6               [ 1]  938 	ld	a, (x)
      008413 AE 50 02         [ 2]  939 	ldw	x, #0x5002
      008416 F7               [ 1]  940 	ld	(x), a
                                    941 ;	main.c: 90: BTNCR1 |= (pullup<<BTNCW)|(pullup<<BTNCCW); //Use internal pull-up
      008417 AE 50 03         [ 2]  942 	ldw	x, #0x5003
      00841A F6               [ 1]  943 	ld	a, (x)
      00841B AA 06            [ 1]  944 	or	a, #0x06
      00841D F7               [ 1]  945 	ld	(x), a
                                    946 ;	main.c: 91: BTNCR2 |= (exti_disabled<<BTNCW)|(exti_disabled<<BTNCCW);
      00841E AE 50 04         [ 2]  947 	ldw	x, #0x5004
      008421 F6               [ 1]  948 	ld	a, (x)
      008422 AE 50 04         [ 2]  949 	ldw	x, #0x5004
      008425 F7               [ 1]  950 	ld	(x), a
      008426 81               [ 4]  951 	ret
                                    952 ;	main.c: 94: void setstep(unsigned char st)
                                    953 ;	-----------------------------------------
                                    954 ;	 function setstep
                                    955 ;	-----------------------------------------
      008427                        956 _setstep:
                                    957 ;	main.c: 96: MOTODR = st<<MOTP1; //Update step
      008427 7B 03            [ 1]  958 	ld	a, (0x03, sp)
      008429 48               [ 1]  959 	sll	a
      00842A 48               [ 1]  960 	sll	a
      00842B 48               [ 1]  961 	sll	a
      00842C AE 50 0A         [ 2]  962 	ldw	x, #0x500a
      00842F F7               [ 1]  963 	ld	(x), a
      008430 81               [ 4]  964 	ret
                                    965 	.area CODE
                                    966 	.area INITIALIZER
      008577                        967 __xinit__stepcw:
      008577 0A                     968 	.db #0x0A	; 10
      008578 09                     969 	.db #0x09	; 9
      008579 05                     970 	.db #0x05	; 5
      00857A 06                     971 	.db #0x06	; 6
      00857B                        972 __xinit__stepccw:
      00857B 05                     973 	.db #0x05	; 5
      00857C 09                     974 	.db #0x09	; 9
      00857D 0A                     975 	.db #0x0A	; 10
      00857E 06                     976 	.db #0x06	; 6
                                    977 	.area CABS (ABS)

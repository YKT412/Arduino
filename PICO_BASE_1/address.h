#define point(hex) (*(volatile uint32_t*)(hex))

////////////////////SIO/////////////////////////////////////

#define out_toggle point(0xd000001c)
#define in  point(0xd0000004)             //GPIO_IN
#define out point(0xd0000010)             //GPIO_OUT
#define out_xor point(0xd000001c)         //GPIO_OUT_XOR
#define out_set point  (0xd0000014)         //GPIO_OUT_SEST
#define out_clear point(0xd0000018)
#define oe point(0xd0000020)              //GPIO_OE
#define oe_set point(0xd0000024)          //GPIO_OE_SET
#define oe_clear point(0xd0000028)
//r 2, 29 to 0

////////////////GPIO//////////////////////////////////


#define gpc(num) point(0x40014004+(8*num))    //
#define gpc_set(num) point(0x40016004+(8*num))
#define gpc_clear(num) point(0x40017004+(8*num))5
/*  num-> 0-29        r 2, irqover 2, r 10,inover 2,r 2,oeover 2,r 2,outover 2,r 3,funcsel 5
 *  irq,in,oe,out --> dont invert, invert, low, high
 *  funcsel --> spi,uart,i2c, pwm, sio, pio0, pio1
 */
//#define gp_control(num) point(0x40014004+(8*num))
#define intr2 point(0x400140f8)
#define proc0_inte2 point(0x40014108)
#define proc0_ints2 point(0x40014128)
/* GPIO17_EDGE_LOW ---> 6
*/

//////////////////PADS///////////////////////////////////

 #define pad(num) point(0x4001c004+(4*num))
#define pad_set(num) point(0x4001e004+(4*num))
#define pad_clear(num) point(0x4001f004+(4*num))
/*  num --> 0-29       r,24,od,ie,drive .2,pue,pde,schmit, slewfast
 *   drive--> 2,3,8,12 mA (4mA default)   ie,pd,schmit default 1
 */


 //////////////////TIMER////////////////////////////0x40054000

 #define timelr point(0x4005400c)   //time read
 #define timelw point(0x40054004)   // time write
 #define alarm1 point(0x40054014)
 #define alarm2 point(0x40054018)
 #define armed point(0x40054020)
 #define timerawl point(0x40054028)  // raw time
 #define inte point(0x40054038) 
 #define intr point(0x40054034) 
 #define ints point(0x40054040)

 
 ///////////////////PWM////////////////////////////0x40050000

 #define pwm_top(chan) point(0x40050010+(20*chan))
 #define pwm_csr(chan) point(0x40050000+(20*chan))
 #define pwm_div(chan) point(0x40050004+(20*chan))
 #define pwm_cc(chan) point(0x4005000c+(20*chan))
 #define pwm_ctr(chan) point(0x40050008+(20*chan))
 #define pwm_en point(0x400500a0) 
 

////////////////////UART//////////////////////////

 #define uart0_i point(0x40034024)   //integer
 #define uart0_f point(0x40034028)   //fraction
 #define uart0_lcr point(0x4003402c) //line control 6-5-length (11 for 8),4-fifoen
 #define uart0_dr point(0x40034000)  //data register  11-oe,10-b,9pe,8-fe,7-0-data
 #define uart0_fr point(0x40034018)  //flag register
 #define uart0_cr point(0x40034030)  //control register      9-rxe,8-txe,0-uarten
 #define clk_peri point(0x40008048)  //peri_clcok control    11->enable

 #define uart1_i point(0x40038024)   //integer
 #define uart1_f point(0x40038028)   //fraction
 #define uart1_lcr point(0x4003802c) //line control 6-5-length (11 for 8),4-fifoen
 #define uart1_dr point(0x40038000)  //data register  11-oe,10-b,9pe,8-fe,7-0-data
 #define uart1_fr point(0x40038018)  //flag register
 #define uart1_cr point(0x40038030)  //control register      9-rxe,8-txe,0-uarten

 
//----------------UART INTERRUPT--------------------------------------

 #define uartimsc point(0x40034038)  //Interrupt Mask Set/Clear Register 
 #define uartris point(0x4003403c)   //Raw Interrupt Status Register
 #define uartmis point(0x40034040)   //Masked Interrupt Status Register
 #define uarticr point(0x40034044)   //Interrupt Clear Register
 #define uartifls point(0x40034034)  //fifo level select



 
//////////////////I2C////////////////////////////0x40044000  0x40048000

#define IC_CON(num) point(0x40044000 + num*16384) 
#define IC_TAR(num) point(0x40044004 + num*16384)
#define IC_SAR(num) point(0x40044008 + num*16384)
#define IC_DATA_CMD(num) point(0x40044010 + num*16384)
#define IC_SS_SCL_HCNT(num) point(0x40044014 + num*16384)
#define IC_SS_SCL_LCNT(num) point(0x40044018 + num*16384)
#define IC_FS_SCL_HCNT(num) point(0x4004401c + num*16384)
#define IC_FS_SCL_LCNT(num) point(0x40044020 + num*16384)
#define IC_INTR_STAT(num) point(0x4004402c + num*16384)
#define IC_INTR_MASK(num) point(0x40044030 + num*16384)
#define IC_RAW_INTR_STAT(num) point(0x40044034 + num*16384)
#define IC_EN(num) point(0x4004406c + num*16384)
#define IC_STATUS(num) point(0x40044070 + num*16380)
#define IC_RX_TL(num) point(0x40044070 + num*16380)
#define IC_TX_TL(num) point(0x4004403c + num*16380)

///////////////////INTERRUPT///////////////////////

#define irq(intr) point(0x20000000 + (16 + intr)*4)
#define nvic_iser point(0xe000e100)



/* nth pin

     TIMER_IRQ_0: 0
     TIMER_IRQ_1: 1
     TIMER_IRQ_2: 2
     TIMER_IRQ_3: 3
    PWM_IRQ_WRAP: 4
     USBCTRL_IRQ: 5
         XIP_IRQ: 6
      PIO0_IRQ_0: 7
      PIO0_IRQ_1: 8
      PIO1_IRQ_0: 9
      PIO1_IRQ_1: 10
       DMA_IRQ_0: 11
       DMA_IRQ_1: 12
    IO_IRQ_BANK0: 13
     IO_IRQ_QSPI: 14
   SIO_IRQ_PROC0: 15
   SIO_IRQ_PROC1: 16
      CLOCKS_IRQ: 17
        SPI0_IRQ: 18
        SPI1_IRQ: 19
       UART0_IRQ: 20
       UART1_IRQ: 21
    ADC_IRQ_FIFO: 22
        I2C0_IRQ: 23
        I2C1_IRQ: 24
         RTC_IRQ: 25
*/

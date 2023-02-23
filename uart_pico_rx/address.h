#define point(hex) (*(uint32_t*)(hex))

////////////////////SIO/////////////////////////////////////

#define out_toggle point(0xd000001c)
#define in  point(0xd0000004)             //GPIO_IN
#define out point(0xd0000010)             //GPIO_OUT
#define out_xor point(0xd000001c)         //GPIO_OUT_XOR
#define out_set point(0xd0000014)         //GPIO_OUT_SEST
#define oe point(0xd0000020)              //GPIO_OE
#define oe_set point(0xd0000024)          //GPIO_OE_SET
#define oe_clear point(0xd0000028)
//r 2, 29 to 0

////////////////GPIO//////////////////////////////////

#define gpc(num) point(0x40014004+(8*num))    //
#define gpc_set(num) point(0x40016004+(8*num))
#define gpc_clear(num) point(0x40017004+(8*num))
/*  num-> 0-29        r 2, irqover 2, r 10,inover 2,r 2,oeover 2,r 2,outover 2,r 3,funcsel 5
 *  irq,in,oe,out --> dont invert, invert, low, high
 *  funcsel --> spi,uart,i2c, pwm, sio, pio0, pio1
 */
//#define gp_control(num) point(0x40014004+(8*num))

//////////////////PADS///////////////////////////////////

#define pad(num) point(0x4001c004+(4*num))
#define pad_set(num) point(0x4001e004+(4*num))
#define pad_clear(num) point(0x4001f004+(4*num))
/*  num --> 0-29       r,24,od,ie,drive .2,pue,pde,schmit, slewfast
 *   drive--> 2,3,8,12 mA (4mA default)   ie,pd,schmit default 1
 */


 //////////////////TIMER////////////////////////////0x40054000

 #define timelr point(0x4005400c)
 #define timelw point(0x40054004) 
 #define alarm1 point(0x40054014)
 #define alarm2 point(0x40054018)
 #define armed point(0x40054020)
 #define timerawl point(0x40054028)
 

 
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
 
  // desired Baud rate = 


 
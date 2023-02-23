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

#define pad(num) point(0x4001c004)+(4*num))
#define pad_set(num) point(0x4001e004+(4*num))
#define pad_clear(num) point(0x4001f004+(4*num))
/*  num --> 0-29       r,24,od,ie,drive .2,pue,pde,schmit, slewfast
 *   drive--> 2,3,8,12 mA (4mA default)   ie,pd,schmit default 1
 */


 //////////////////TIMER////////////////////////////














/*
 
 #define point(hex) (*(uint32_t*)(hex))

//////////////////SIO/////////////////////////
#define in  point(0xd0000004)
#define out point(0xd0000010)
#define out_xor point(0xd000001c)
#define out_set point(0xd0000014)
#define out_clear point(0xd0000018)
#define oe point(0xd0000020)
#define oe_set point(0xd0000024)
#define oe_clear point(0xd0000028)
//r 2,29 to 0

//////////////////GPIO//////////////////////
#define gpc(num) point(0x40014004+(8*num))
#define gpc_set(num) point(0x40016004+(8*num))
#define gpc_clear(num) point(0x40017004+(8*num))
//num->0-29//////r 2,irqover 2,r 10,inover 2,r 2,oeover 2,r 2,outover 2,r 3,funcsel 5
//irq,in,oe,out -> dont invert,invert,low,high
//funcsel -> spi,uart,i2c,pwm,sio,pio0,pio1  /// default 0x1f

/////////////////PADS//////////////////////
#define pad(num) point(0x4001c004+(4*num))
#define pad_set(num) point(0x4001e004+(4*num))
#define pad_clear(num) point(0x4001f004+(4*num))
//num ->0-29//////r 24,od,ie,drive 2,pue,pde,schmit,slewfast
//drive-> 2,4,8,12 mA (4mA default)  //ie,pd,schmit default 1



/////////////////PWM////////////////////////
#define pwm_cr(num) point(0x40050000+(20*num))  // default 0
//num -> chn 0-7//r 24,phase+,phase-,div_mode 2,comp_b,comp_a,phase_cor,en
//div mode -> free run, b gate,b rising ++,b falling ++
#define pwm_div(num) point(0x40050004+(20*num))
//num -> ch 0-7//11:4 int , 3:0 fractional
#define pwm_set(num) point(0x4005000c+(20*num))
#define pwm_top(num) point(0x40050010+(20*num))
#define pwm_en point(0x400500a0)

///////////////TIMER//////////////////////
#define timerl point(0x40054028)
#define alarm1 point(0x40054014)
#define alarm2 point(0x40054018)
#define alarm3 point(0x4005401c)
#define alarm0 point(0x40054010)
#define armed point(0x40054020)
//r28 ,a3,a2,a1,a0  /////1 armed,0 disarmed






//#define  point(0x)
//#define  point(0x)
//#define  point(0x)
//#define  point(0x)
//#define  point(0x)
//#define  point(0x)*/

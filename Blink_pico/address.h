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
/*  num --> 0-29       r,24,od,ie,drive .2,pue,schmit, slewfast
 *   drive--> 2,3,8,12 mA (4mA default)   ie,pd,schmit default 1
 */


 //////////////////TIMER////////////////////////////
 

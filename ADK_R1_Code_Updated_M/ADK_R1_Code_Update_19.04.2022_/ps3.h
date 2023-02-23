#include <PS3BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
//PS3BT PS3(&Btd, 0x33, 0x03, 0x30, 0x0C, 0x48, 0x69);
PS3BT PS3(&Btd, 0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13); // new
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57);

class PS3_class
{
    //all buttons of ps3 included
  public:

    //digital buttons
    bool up;
    bool down;
    bool left;
    bool right;
    bool L1;
    bool L3;
    bool R1;
    bool R3;
    bool square;
    bool circle;
    bool cross;
    bool tri;
    bool start;
    bool select;
    bool ps;

    //analog buttons
    int16_t L2;
    int16_t R2;
    int16_t LeftHatX;
    int16_t LeftHatY;
    int16_t RightHatX;
    int16_t RightHatY;
};

PS3_class ps3;

void ps3_setup()
{
#if !defined(_MIPSEL_)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  while (!PS3.PS3Connected)
  {
    Serial.println(PS3.PS3Connected);
    Usb.Task();
  }

}

void ps3_data()
{
  //Call this function once in the void loop() to update the existing status of the buttons
  //Use ps.button_name to check the corresponding buttons

  Usb.Task();

  //digital buttons
  ps3.up    = PS3.getButtonPress(UP);
  ps3.right = PS3.getButtonPress(RIGHT);
  ps3.down  = PS3.getButtonPress(DOWN);
  ps3.left  = PS3.getButtonPress(LEFT);
  ps3.select  = PS3.getButtonPress(SELECT);
  ps3.L3    = PS3.getButtonPress(L3);
  ps3.R3    = PS3.getButtonPress(R3);
  ps3.start = PS3.getButtonPress(START);
  ps3.L1    = PS3.getButtonPress(L1);
  ps3.R1    = PS3.getButtonPress(R1);
  ps3.tri   = PS3.getButtonPress(TRIANGLE);
  ps3.circle  = PS3.getButtonPress(CIRCLE);
  ps3.cross  = PS3.getButtonPress(CROSS);
  ps3.square = PS3.getButtonPress(SQUARE);
  ps3.ps = PS3.getButtonClick(PS);


  //analog buttons
  ps3.L2    = PS3.getAnalogButton(L2);
  ps3.R2    = PS3.getAnalogButton(R2);
  ps3.LeftHatX = PS3.getAnalogHat(LeftHatX);
  ps3.LeftHatY = PS3.getAnalogHat(LeftHatY);
  ps3.RightHatX = PS3.getAnalogHat(RightHatX);
  ps3.RightHatY = PS3.getAnalogHat(RightHatY);
}

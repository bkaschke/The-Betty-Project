//
//    FILE: AS5600_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//    DATE: 2022-05-28


#include "AS5600.h"
#include "Wire.h"

AS5600 as5600;   //  use default Wire
int startupVal;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  //  ESP32
  //  as5600.begin(14,15);
  //  AVR
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  startupVal = as5600.getCumulativePosition();
  delay(1000);
}


void loop()
{
  //Serial.print(as5600.getCumulativePosition());
  Serial.print("Variable_1:");
  Serial.print((as5600.getCumulativePosition() - startupVal) * AS5600_RAW_TO_DEGREES * 0.338345865);
  Serial.println(",");
  //Serial.print("Variable_2:");
  //Serial.println(as5600.getCumulativePosition());
  delay(100);
}

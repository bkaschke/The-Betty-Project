#include "AS5600.h"
#include "Wire.h"
AS5600 as5600;   //  use default Wire
int startupVal;

String j5_data = "J5,189.00,500.00,-200.00,1,";
String angle;
float acceleration = 500.00;
float speed = -200.00;
float mode = 1;
const char* readyMessage = "READY";
const char* ackMessage = "ACK";
bool ackReceived = false;
bool ReadyToReceive = false;
char temp[1];
String J5_ID = "J5";
String J6_ID = "J6";
int ForeArm_Response_Code = 0;
bool j5updated = false;
bool j6updated = false;



void setup() {
  Serial1.begin(57600);
     while (!Serial1) {
   ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.begin(57600);
     while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Enter Value To Start:");


  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  startupVal = as5600.getCumulativePosition();

}

void SendForeArmData(String ID, String angle, float speed, float acceleration, int mode)
{
String data;
data = ID + "," + angle + "," + String(speed) + "," + String(acceleration) + "," + String(mode) + ",";
Serial1.print(data);
}
void ManualSendForeArmData(String ID, String angle, float speed, float acceleration, int mode)
{
String data;
data = "J5," + angle + "," + String(speed) + "," + String(acceleration) + "," + String(mode) + ",";
data = data + "J6," + angle + "," + String(speed) + "," + String(acceleration) + "," + String(mode) + ",";
Serial1.print(data);
}

void loop() {

  //////////////////////////////////////////////////////////Manual Entry Joint Control////////////////////////////////////////////
  ////////////         User can enter Motor ID and step value into the serial monitor to move selected joint       //////////////
  if(Serial.available()){
        angle = Serial.readStringUntil('\n');
        Serial.print("You typed: " );
        Serial.println(angle);
        j5updated = true;
    }

  if(j5updated == true){
  ManualSendForeArmData(J5_ID, angle, speed, acceleration, mode); //Input J5,2000 into serial monitor to move motors, use J6 if moving J6
  Serial.println("Sent Motor Data to Slave!");
  j5updated = false;
  }
  Serial.print("Angle: ");
  Serial.print((as5600.getCumulativePosition() - startupVal) * AS5600_RAW_TO_DEGREES * 0.338345865);
  Serial.println(",");
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////Program Test ///////////////////////////////////////////////////////////////////
  /*SendForeArmData(J5_ID, 0, speed, acceleration, mode);
  delay(500);
  SendForeArmData(J5_ID,3000, speed, acceleration, mode);
  delay(500);
  Serial.print("J4 Angle: ");
  Serial.print((as5600.getCumulativePosition() - startupVal) * AS5600_RAW_TO_DEGREES * 0.338345865);
  Serial.println(",");
*/





  //Serial.println("Testing..." );
  /*while(ForeArm_Response_Code != 55){
  if (Serial1.available() > 0) {
  ForeArm_Response_Code = Serial1.read();}
  }*/
 
  //SendForeArmData(J6_ID, angle + 2, speed, acceleration, mode);
  //angle++;
  //if(angle > 360){angle = 0;}
  //delay(1000);
}

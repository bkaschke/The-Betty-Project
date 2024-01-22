#include <TMCStepper.h>
#define EN_PIN           4 // Enable
#define DIR_PIN          12 // Direction
#define STEP_PIN         10 // Step
///#define CS_PIN           42 // Chip select
//#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
//#define SW_MISO          44 // Software Master In Slave Out (MISO)
#define SW_SCK           17 // Software Slave Clock (SCK)
//#define SW_RX            20 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial5 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
const int BUFFER_SIZE = 100;
char Received_Data[BUFFER_SIZE];
String Temp_Message_Data = "";
String Message_ID = "";
int Message_Read_Count = 0;
bool Reading_Message = false;
bool Message_Stored = false;
float j5[5];
float j6[5];
int Response_Code = 0; //This code will be returned back to the master to tell if data was successfully received. 
int value;
bool shaft = false;
int j5position = 0;
int j5max = 2300;

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  Serial.begin(57600);
  Serial2.begin(57600);
  Serial5.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
   while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Starting...");
  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(1200);        // Set motor RMS current
  driver.microsteps(16);          // Set microsteps to 1/16th
  driver.pwm_autoscale(true);
  j5position = 0;
  digitalWrite(DIR_PIN,LOW);
    for (uint16_t i = 2300; i>0; i--) {
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(150);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(150);
    }
}


void AckMessage(String ID, int response_code)
{
Serial1.write(response_code);
Serial.print("ACK: ");
Serial.println(response_code);
}
void loop() {
  if (Serial2.available() > 0) {
    value = Serial2.available();
    Reading_Message = true;
    Temp_Message_Data = "";
    int rlen = Serial2.readBytesUntil(',', Received_Data, BUFFER_SIZE);
    for(int i = 0; i < rlen; i++)
    {
      Temp_Message_Data = Temp_Message_Data + Received_Data[i];
    }
  if(Temp_Message_Data == "J5" || Temp_Message_Data == "J6"|| Temp_Message_Data == "EOAT"){
    Message_ID = Temp_Message_Data;
  }
  if(Temp_Message_Data != Message_ID) ///To Add new communication add in new message ID statement in this IF statement
  {
    if(Message_ID == "J5")
    {
      j5[Message_Read_Count] = Temp_Message_Data.toFloat();
      Message_Read_Count++;
    }
    if(Message_ID == "J6")
    {
      j6[Message_Read_Count] = Temp_Message_Data.toFloat();
      Message_Read_Count++;
    }
  }
  
  if(Message_Read_Count == 4)
  {
    Message_Read_Count = 0;
    Reading_Message = false;
    Message_Stored = true;
  }
}
  if(Message_Stored == true && Reading_Message == false)
  {
  if(Message_ID == "J5"){
  ////////////Joint 5 Serial Monitor View//////////////////////////
  Serial.print("Updated J5 Values: ");
  Serial.print("Angle = ");
  Serial.println(j5[0]);

  //Response_Code = 55;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////Stepper Driving Instructions//////////////////////////////////////////////////////
   
  if(j5[0] >= 0)
  {
    digitalWrite(DIR_PIN,HIGH);
    for (uint16_t i = j5[0]; i>0; i--) {
    //if(j5position < j5max)
    //{
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(150);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(150);
    j5position++;
    //}
  }}
  else
  {
    digitalWrite(DIR_PIN,LOW);
    for (uint16_t i = abs(j5[0]); i>0; i--) {
    //if(j5position > 0)
    //{
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(150);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(150);
    j5position--;
    //}
  }
  }
  shaft = !shaft;
  driver.shaft(shaft);
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////
}
  }
   ////////////Joint 6 Serial Monitor View//////////////////////////
 /* if(Message_ID == "J6"){
  Serial.print("Updated J6 Values: ");
  Serial.print("Angle = ");
  Serial.print(j6[0]);
  Serial.print(", Speed = ");
  Serial.print(j6[1]);
  Serial.print(", Accel = ");
  Serial.print(j6[2]);
  Serial.print(", Mode = ");
  Serial.print(j6[3]);
  Serial.println("");
  //Serial.println("");
  //Response_Code = 66;
  }*/
  //AckMessage(Message_ID, Response_Code);
  Response_Code = 0;
  Message_Stored = false;
  Serial.println(j5position);
  }




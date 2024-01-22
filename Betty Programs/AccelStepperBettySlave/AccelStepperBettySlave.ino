#include <TMCStepper.h>
#include <AccelStepper.h>

//////////// Joint 5 Motor Information////////////////////////////////
#define J5_EN_PIN           4 // Enable
#define J5_DIR_PIN          12 // Direction
#define J5_STEP_PIN         10 // Step
#define J5_SW_SCK           17 // Software Slave Clock (SCK)
#define J5_SERIAL_PORT Serial5 // TMC2208/TMC2224 HardwareSerial port
#define J5_DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define J5_R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075

//////////// Joint 6 Motor Information ////////////////////////////////
#define J6_EN_PIN           3 // Enable
#define J6_DIR_PIN          11 // Direction
#define J6_STEP_PIN         9 // Step
#define J6_SW_SCK           13 // Software Slave Clock (SCK)
#define J6_SERIAL_PORT Serial3 // TMC2208/TMC2224 HardwareSerial port
#define J6_DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define J6_R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075
///////////////////////////////////////////////////////////////////////
TMC2209Stepper J5_driver(&J5_SERIAL_PORT, J5_R_SENSE, J5_DRIVER_ADDRESS);
TMC2209Stepper J6_driver(&J6_SERIAL_PORT, J6_R_SENSE, J6_DRIVER_ADDRESS);
constexpr uint32_t steps_per_mm = 80;

AccelStepper J5_stepper(1, J5_STEP_PIN, J5_DIR_PIN);
AccelStepper J6_stepper(1, J6_STEP_PIN, J6_DIR_PIN);

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
  ////////////////////////Joint 5 Setup ///////
  pinMode(J5_EN_PIN, OUTPUT);
  pinMode(J5_STEP_PIN, OUTPUT);
  pinMode(J5_DIR_PIN, OUTPUT);
  digitalWrite(J5_EN_PIN, LOW);      // Enable driver in hardware

  //////////// Joint 6 Setup /////////////////
  pinMode(J6_EN_PIN, OUTPUT);
  pinMode(J6_STEP_PIN, OUTPUT);
  pinMode(J6_DIR_PIN, OUTPUT);
  digitalWrite(J6_EN_PIN, LOW);      // Enable driver in hardware

  Serial.begin(57600);
  Serial2.begin(57600);
  Serial3.begin(115200);
  Serial5.begin(115200);

  J5_stepper.setMaxSpeed(20000); //Max Speed 20000
  J5_stepper.setAcceleration(90000); ///Max for J6 and J5 are 150000.
  J6_stepper.setMaxSpeed(20000);
  J6_stepper.setAcceleration(90000);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
   while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting...");
  J5_driver.begin(); 
  J6_driver.begin();                //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  J5_driver.toff(5);                 // Enables driver in software
  J5_driver.rms_current(1200);        // Set motor RMS current
  J5_driver.microsteps(16);          // Set microsteps to 1/16th
  J5_driver.pwm_autoscale(true);

  J6_driver.toff(5);                 // Enables driver in software
  J6_driver.rms_current(1200);        // Set motor RMS current
  J6_driver.microsteps(16);          // Set microsteps to 1/16th
  J6_driver.pwm_autoscale(true);
  j5position = 0;

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
  Message_ID = "";}
  if(Message_ID == "J6"){
  ////////////Joint 5 Serial Monitor View//////////////////////////
  Serial.print("Updated J6 Values: ");
  Serial.print("Angle = ");
  Serial.println(j6[0]);
  Message_ID = "";}
  //Response_Code = 55;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////Stepper Driving Instructions//////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////
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
  //Serial.println(j5position);
        if (J6_stepper.distanceToGo() == 0 && J5_stepper.distanceToGo() == 0)
        {
      J6_stepper.moveTo(j6[0]);
      J5_stepper.moveTo(j5[0]);
        }
    J5_stepper.run();
    J6_stepper.run();
    Serial.print("J5 Position: ");
    Serial.print(J5_stepper.currentPosition());
    Serial.print(",  J6 Position: ");
    Serial.println(J6_stepper.currentPosition());
  }




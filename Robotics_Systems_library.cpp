
// TODO
/*
- Add MATLAB handshake to EstablishUSBConnection() or triage to Arduino serial port
- Add manual control mode in Arduino using Serial -> Ask which motor, then what angle ("x" to cancel)
- Make q_STS & q_dot_STS private!
- Use getters & setters
- Add switch statements to loop

*/



#include "Robotics_Systems_Library.h"

UOM_RS_Robot::UOM_RS_Robot() {
  num_ID = 0;

  // Initialise arrays based on max connected motors
  for (int i = 0; i <= MAX_ID; i++) {
    ID[i] = i+1;
    connected_ID[i] = 255;
    q_FB_STS[i] = 0;
    q_STS[i] = 0;
    q_dot_STS[i] = 0;
    control_mode[i] = POSITION;
  }
}


int UOM_RS_Robot::EstablishUSBConnection() {
  Serial.begin(USB_BAUD_RATE); // Set serial baud rate for USB

  int start_timer = millis();
  while (!Serial) // Wait For Serial To Connect
  {
    if (millis() - start_timer > CONNECTION_TIMEOUT) {
      return -1;
    }
  }
  connection_status = connectToMATLAB();


  if (connection_status == ERROR) {
    Serial.println("Failed to connect to MATLAB. Connecting to Arduino serial monitor instead.");          
  }

  // Serial.println("Serial Connected To Arduino Successfully.");             
  return 1;
}


int UOM_RS_Robot::EstablishMotorSerialConnection() {
  Serial1.begin(MOTOR_BAUD_RATE); // Set serial baud rate for motors

  int start_timer = millis();
  while (!Serial1) // Wait For Serial To Connect
  {
    if (millis() - start_timer > CONNECTION_TIMEOUT) {
      return -1;
    }
  }             
  motors.pSerial = &Serial1;        // Assign serial port to motor
  return 1;
}



int UOM_RS_Robot::getID() {

  // Get the number if motors connected and their IDs.

  // External Variables
  // @ ID       - Array of connected motor IDs.

  // Internal Variables
  // @ numID    - Number of connected IDs.
  // @ tempID   - Temporary value for storing pre-checked ID value.
  // @ SCSCL_ID - EPROM reference for accessing motor ID.

  // Internal Functions
  // readByte() - Read a byte of data returned by the motors.

  num_ID = 0;

  for (int i = 1; i <= MAX_ID; i++) {
    // Read ID value from motor
    int tempID = motors.readByte(i, SMS_STS_ID);

    // If returned ID is valid, store.
    if (tempID > 0 && tempID <= MAX_ID) {
      connected_ID[num_ID] = tempID;
      num_ID++;
    }

  }

  return 1;
}

void UOM_RS_Robot::UpdateMotorControlMode() {


  for (int i=0; i<MAX_ID; i++) {
    if (control_mode[i]) {
      motors.WheelMode(ID[i]);
    }
    else {
      motors.ServoMode(ID[i]);
    }
  }

}

void UOM_RS_Robot::SetMotorControlMode(bool *new_control_mode) {
  for (int i=0; i<MAX_ID; i++) {

    control_mode[i] = new_control_mode[i];
  }

  UpdateMotorControlMode();

}



void UOM_RS_Robot::getControlMode() {
  double control_mode_tmp[MAX_ID];
  bool control_mode_tmp_bool[MAX_ID];

  
  for (int i = 0; i < MAX_ID; i++) {

    if (Serial.parseInt() != 0) {
      control_mode_tmp_bool[i] = VELOCITY;
    }
    else {
      control_mode_tmp_bool[i] = POSITION;
    }
  }

  // Send acknowledgement that data has been received
  Serial.print(ACK_DATA_RECEIVED);

  SetMotorControlMode(control_mode_tmp_bool);

}


void UOM_RS_Robot::DriveMotors() {


  for (int i=0; i<num_ID; i++) {
    if (control_mode[connected_ID[i]-1]) {
      motors.WriteSpe(connected_ID[i], q_dot_STS[i]);
    }
    else {
      u16 Speed = 0;
      u8 ACC = 50;
      motors.WritePosEx(connected_ID[i], q_STS[i], Speed, ACC);
    }
    
  }

}



void UOM_RS_Robot::ReadFeedback() {
  for (int i=0; i<num_ID; i++) {

    q_FB_STS[ID[connected_ID[i]-1]-1]=motors.ReadPos(ID[connected_ID[i]-1]);

    // q_FB_STS[i]=motors.ReadPos(ID[i]);
  }
}


void UOM_RS_Robot::SerialMonitorMotorControl() {
  Serial.print("Enter Motor ID: ");
  while (Serial.available() == 0) {}     //wait for data available
  String teststr = Serial.readString();  //read until timeout
  teststr.trim();                        // remove any \r \n whitespace at the end of the String
  int motorID = teststr.toInt();
  Serial.println(motorID);

  Serial.print("Enter Value: ");
  while (Serial.available() == 0) {}     //wait for data available
  teststr = Serial.readString();  //read until timeout
  teststr.trim();                        // remove any \r \n whitespace at the end of the String
  int motorValue = teststr.toInt();
  Serial.println(motorValue);

  if (control_mode[ID[motorID-1]-1]) {
    q_dot_STS[ID[motorID-1]-1] = motorValue;
  }
  else {
    q_STS[ID[motorID-1]-1] = motorValue;
  }

  DriveMotors();
  // TO DO
  // - Drive Motor

}

void UOM_RS_Robot::InitMotorFeedback() {
  ReadFeedback();
  for (int i=0; i<MAX_ID; i++) {
    q_STS[i] = q_FB_STS[i];
  }
}


int UOM_RS_Robot::connectToMATLAB() {

  // Establish serial connection with MATLAB by sending a single char,
  // waiting for a single char response, and then sending a final char
  // to acknoledge one was received.

  char c = 'b';
  Serial.println("a");

  int start_timer = millis();
  while ( c != 'a' ) {
    // if (millis() - start_timer > CONNECTION_TIMEOUT) {
    //   return -1;
    // }
    c = Serial.read();
  }
  Serial.println("b");

  return 1;
}



void UOM_RS_Robot::sendDataSerial(int *data, int dataSize) {

  // Send integer data to MATLAB via serial.

  // External Variables
  // @ data         - Array of int data (e.g. motor feedback or joystick positions).
  // @ dataSize     - Integer of how large the expected data should be

  for (int i = 0; i < dataSize; i++) {

    // Notify MATLAB of how many digits to expect prior to sending data.
    if (data[i] < 10 && data[i] >= 0) {
      Serial.print("1");
      Serial.print(data[i]);
    }
    else if ((data[i] >= 10 && data[i] < 100) || (data[i] < 0 && data[i] > -10)) {
      Serial.print("2");
      Serial.print(data[i]);
    }
    else if ((data[i] >= 100 && data[i] < 1000) || (data[i] <= -10 && data[i] > -100)) {
      Serial.print("3");
      Serial.print(data[i]);
    }
    else if ((data[i] > 1000) || (data[i] <= -100 && data[i] > -1000)) {
      Serial.print("4");
      Serial.print(data[i]);
    }
    else if ((data[i] > 10000) || (data[i] <= -1000 && data[i] > -10000)) {
      Serial.print("5");
      Serial.print(data[i]);
    }
    else {
      Serial.print("e");
    }
  }

}

void UOM_RS_Robot::getDataSerial(double *serialData) {

  // Read data such as motor positions/velocities sent from MATLAB in rad/s.

  // External Variables
  // @ serialData           - Double array to store read data.
  

  for (int i = 0; i < MAX_ID; i++) {
    // serialData[i] = Serial.parseFloat(SKIP_NONE);
    serialData[i] = Serial.parseFloat();
  }

  // while (Serial.available()) {  // Absorb any left over serial data before next command
  //   Serial.read();
  // }

  // Send acknowledgement that data has been received
  Serial.print(ACK_DATA_RECEIVED);
}

int UOM_RS_Robot::getState() {
  // Read desired state to be performed from serial

  // Internal Variables
  // @ input         - Null terminated string read from MATLAB. This
  //          is sent prior to reading or writing data over serial.

  if (Serial.available() > 3) {
    

    // Read state command from MATLAB
    String input = Serial.readStringUntil('\n');
   

    // // Absorb any left over serial data before next command
    // while (Serial.available()) {  
    //   Serial.read();
    // }

    if (input == "FBK") {
      return READ_FB;
    }
    else if (input == "DRV") {
      return DRIVE_MOTOR;
    }
    else if (input == "UDM") {
      return UPDATE_DRIVE_MODE;
    }
    else if (input == "UPD") {
      return UPDATE_MOTOR;
    }
    else {
      return ERROR;
    }
  }
  
  return IDLE;
  
}

void UOM_RS_Robot::sendAck(char ack) {
  Serial.print(ack);          // Send acknowledgement to MATLAB
}

void UOM_RS_Robot::SendFB2MATLAB() {
  sendDataSerial(q_FB_STS, MAX_ID);
}

void UOM_RS_Robot::getReference() {
  double ref_tmp[MAX_ID];
  getDataSerial(ref_tmp);

  for (int i = 0; i < MAX_ID; i++) {

    if (control_mode[i]) {
      q_dot_STS[i] = ref_tmp[i] * RAD_2_SMS_VEL;
      
    }
    else {
      q_STS[i] = ref_tmp[i] * RAD_2_SMS;
    }

  }

}

void UOM_RS_Robot::sendMotorIDs() {

  // Send ID of each connected motor and total number of connected
  // motors to MATLAB.

  // Internal Variables
  // @ num_ID          - Number of connected IDs.
  // @ ID             - Array of connected motor IDs.

  // Internal Functions
  // sendDataSerial() - Sends desired data over serial to MATLAB.

  // Convert num_ID into single element array to send over serial.
  int numIDTemp[1] = {UOM_RS_Robot::num_ID};
  sendDataSerial(numIDTemp, 1);

  // Convert ID from type u8 to int for sending over serial.
  int idTemp[UOM_RS_Robot::num_ID];
  for (int i = 0; i < UOM_RS_Robot::num_ID; i++) {
    idTemp[i] = (int)  UOM_RS_Robot::connected_ID[i];
  }

  sendDataSerial(idTemp, UOM_RS_Robot::num_ID);
}



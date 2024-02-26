/*
Robotics_Systems_Library.h
--------------------------
Licenting Information: You are free to use or extend this project
for educational purposes provided that (1) you do not distribute or
publish solutions, (2) you retain this notice, and (3) you provide
clear attribution to the University of Melbourne, Department of
Mechanical Engineering and Product Design Lab.

Attribution Information: The ChessBot project was developed in collaboration
between Product Design Lab & the University of Melbourne. The core project was
primarily developed by Professor Denny Oetomo (doetomo@unimelb.edu.au). All
Starter Code and libraries were developed by Nathan Batham of Product
Design Lab
(nathan@productdesignlab.com.au)

*/


#ifndef ROBOTICS_SYSTEMS_LIBRARY_H
#define ROBOTICS_SYSTEMS_LIBRARY_H

#include <Arduino.h>
#include <SMS_STS.h>
#include <math.h>



#define MOTOR_BAUD_RATE 1000000                         // BaudRate used between Arduino and Motors
#define USB_BAUD_RATE 115200                            // BaudRate used between MATLAB and Arduino
#define CONNECTION_TIMEOUT 10000                        // Timeout for connecting to MATLAB
#define MAX_ID 8                                        // MAX Number of motors that can be connected
#define POSITION 0                                      // Set positiion control
#define VELOCITY 1                                      // Set velocity control
#define ERROR -1
#define RAD_2_SMS 4096/(2*PI)                           // Convert radians into motor encoder counts
#define RAD_2_SMS_VEL 4096/(2*PI)                       // Convert rad/s into motor encoder counts/sec
#define ACK_DATA_RECEIVED "d"                           // Acknowledgement sent to MATLAB to indicate data received


// State variable
static enum {
    IDLE,
    READ_FB,
    DRIVE_MOTOR,
    UPDATE_DRIVE_MODE
} state = IDLE;



class UOM_RS_Robot
{
public:
  SMS_STS motors;                                       // Motor Object used to send & receive motor data
  u8 num_ID;                                            // Number of connected motors
  int connection_status;                                // Connection to MATLAB status
  u8 ID[MAX_ID];                                        // Array of sequential ID numbers up to MAX_ID
  u8 connected_ID[MAX_ID];                              // ID numbers of connected motors
  s16 q_FB_STS[MAX_ID];                                 // Array of stored feedback
  s16 q_STS[MAX_ID];                                    // Reference position of each motor stored in encoder counts
  s16 q_dot_STS[MAX_ID];                                // Reference velocity of each motor stored in encoder counts/sec
  bool control_mode[MAX_ID];                            // Control mode (pos or vel) of each motor
  
  

  UOM_RS_Robot();
  int EstablishUSBConnection();                         // Initialise Serial communication & connect to MATLAB
  int EstablishMotorSerialConnection();                 // Initialise Serial1 communication & connect to motors
  int getID();                                          // Read the ID of each motor
                                  
  void SerialMonitorMotorControl();                     // Control the motors using Arduino's Serial Monitor
  
  void DriveMotors();                                   // Drive the motors based on the stored reference positions & velocities
  void ReadFeedback();                                  // Read position feedback from the motors
  void InitMotorFeedback();                             // Read initial feedback & set internal position variable
  int getState();                                       // Check for state change from MATLAB
  void SendFB2MATLAB();                                 // Send feedback to MATLAB
  void getReference();                                  // Read reference positions/velocities from MATLAB
  void sendMotorIDs();                                  // Send the ID of all connected motors to MATLAB
  void getControlMode();                                // Read the new control mode values for each motor
  

private:
  
  void UpdateMotorControlMode();                        // Update the control mode of each motor
  void sendDataSerial(int *data, int dataSize);         // Send data over Serial to MATLAB
  void getDataSerial(double *serialData);               // Read data over Serial from MATLAB
  void sendAck(char ack);                               // Send acknowledgement that data was received
  int connectToMATLAB();                                // Establish connection with MATLAB using handshake
  void SetMotorControlMode(bool *new_control_mode);     // Set the control control mode of the motors
  
};


#endif

/* 
Use an ultrasonic sensor to find objects around the robot
Alex Crotts and Luis Umana - 4/5/2022
*/

#include <Servo.h>
#include <SimpleRSLK.h>

Servo servo;

const int trigPin = 32;//This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad 

int MotorSpeed = 10;
float WheelDiameter = 6.985;     // In centimeters
float PulsePerRev = 360;          // Number of encoder pulses the microcontroller reads per 1 wheel rotation
float WheelBase = 13.335;       // In centimeters

// Number of encoder pulses per 1 degree of rotation
double PulsePerDegree = WheelBase/WheelDiameter;

void setup() {
  // Initialization
  pinMode(trigPin, OUTPUT);   // Set trigPin as an output
  pinMode(echoPin, INPUT);    // Set echoPin as an input
  servo.attach(38);
  servo.write(0);
  setupRSLK();
  resetLeftEncoderCnt();      // Reset encoder counts
  resetRightEncoderCnt();
  Serial.begin(9600);
  Serial.println("Beginning Scan");
  delay(1000);      // Delay to allow the serial monitor to settle
}

void Drive_Straight(int y) {
  // Integer y allows for this function to be called for any distance
  // Function for driving straight for X centimeters
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  // Set both motors to drive forward
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); 
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);  // Set both motors to the same speed
  int L_Pulse_Count = 0;      // Zero the left encoder pulse count
  int R_Pulse_Count = 0;      // Zero the right encoder pulse count

  while((L_Pulse_Count < y) || (R_Pulse_Count < y)) {
    // Run this loop until the number of pulses reaches the specified value 
    L_Pulse_Count = getEncoderLeftCnt();      // Read the left encoder value
    R_Pulse_Count = getEncoderRightCnt();     // Read the right encoder value

    // Drive the robot forward until it runs into something
    if(digitalRead(BP_SW_PIN_0) == 0)
      break;

    if(digitalRead(BP_SW_PIN_1) == 0)
      break;

    if(digitalRead(BP_SW_PIN_2) == 0)
      break;

    if(digitalRead(BP_SW_PIN_3) == 0)
      break;

    if(digitalRead(BP_SW_PIN_4) == 0)
      break;

    if(digitalRead(BP_SW_PIN_5) == 0)
      break;

    if((L_Pulse_Count + 1 < R_Pulse_Count)){
      // If the left is slower than the right, speed up left and slow down right
      setMotorSpeed(LEFT_MOTOR, ++MotorSpeed);      // Speed up the left motor
      setMotorSpeed(RIGHT_MOTOR, --MotorSpeed);     // Slow down the right motor
    }

    if((R_Pulse_Count + 1 < L_Pulse_Count)){
      // If the right is slower than the left, speed up right and slow down left
      setMotorSpeed(RIGHT_MOTOR, ++MotorSpeed);     // Speed up the right motor 
      setMotorSpeed(LEFT_MOTOR, --MotorSpeed);      // Slow down the left motor 
    }
    
    if(L_Pulse_Count >= y){
      // If the number of pulses reaches aspecified value, turn off motors
      disableMotor(LEFT_MOTOR);     // Turn off the left motor
      disableMotor(RIGHT_MOTOR);    // Turn off the right motor
      }

      // Print encoder counts to the serial monitor for debugging
      Serial.print("Driving Straight Now");
      Serial.print("\t");
      Serial.print("Left Encoder: ");
      Serial.print(L_Pulse_Count);
      Serial.print("\t");
      Serial.print("Right Encoder: ");
      Serial.println(R_Pulse_Count);
      delay(100);
  }
}

void Rotate(int z, int L_Motor_Dir, int R_Motor_Dir) {    
  // Integers allow for any rotation direction and degree
  // Function for rotating the RSLK robot in place
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  enableMotor(BOTH_MOTORS);
  // Set the left and right motors to drive in the specified directions
  setMotorDirection(LEFT_MOTOR, L_Motor_Dir);
  setMotorDirection(RIGHT_MOTOR, R_Motor_Dir);      
  setMotorSpeed(BOTH_MOTORS, MotorSpeed);    // Set the motors to the same speed
  int L_CCW_Pulse_Count = 0;      // Zero the encoder count
  int R_CCW_Pulse_Count = 0;      // Zero the encoder count

    while(R_CCW_Pulse_Count < z) {
    // Run this loop until the number of pulses reaches the specified value
    L_CCW_Pulse_Count = getEncoderLeftCnt();      // Read left encoder value
    R_CCW_Pulse_Count = getEncoderRightCnt();     // Read right encoder value

    if(R_CCW_Pulse_Count >= z) {
      // If the number of pulses reaches the specified value, turn off motors 
      disableMotor(LEFT_MOTOR);       // Turn off the left motor
      disableMotor(RIGHT_MOTOR);      // Turn off the right motor
      delay(1000);
    }

    //Print encoder counts to the serial monitor for debugging
    Serial.print("Turning CCW Now");
    Serial.print("\t");
    Serial.print("Left Encoder CCW Turn: ");
    Serial.print(L_CCW_Pulse_Count);
    Serial.print("\t");
    Serial.print("Right Encoder CCW Turn: ");
    Serial.println(R_CCW_Pulse_Count);
    delay(100);
  }
}

long Read_Distance() {
  // This function reads the distance from the ultrasonic sensor
  byte Readings[7];   // Declare an array of readings
  int x = 0;          // Array indexed at zero
  long pulseLength;   // Length of the ultrasonic pulse
  long centimeters;   // Calculated distance
  long total = 0;     // Initially zero the total for averaging the array
  long average;       // Calculated average of the array

  // Sending the pulse to the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Calculating the distance from the pulse length
  pulseLength = pulseIn(echoPin, HIGH);
  centimeters = pulseLength / 58;

  // Set up the loop to store values in an array
  for(Readings[x]; x < 7; x++) {
    // Read from the sensor:
    Readings[x] = centimeters;
    // Add the reading to the total:
    total = total + Readings[x];
  
    // If we're at the end of the array...
    if (x >= 7) {
      // ...wrap around to the beginning:
      x = 0;
      }
  }
  
  // Calculate the average of the array:
  average = total / 7;
  // send it to the computer as ASCII digits
  Serial.print("Average Distance: ");
  Serial.println(average);
  delay(100);        // delay in between reads for stability
  return(average);
    }

void loop() {
  int pos = 0;    // variable to store the servo position
  byte Sweep_1_Array[18];   // Declare the first array
  digitalWrite(77, HIGH);
  long Min1_value = Read_Distance();  // Store a reading as the min value
  int Min1_position = pos;    // Store the angle at which the min was found   
  
  for(pos = 0; pos < 180; pos += 10) { // Moves the servo from 0 to 180 degrees 
    servo.write(pos);     // tell servo to go to position in variable 'pos' 
    delay(15);     // waits 15ms for the servo to reach the position 
    Sweep_1_Array[pos/10] = Read_Distance();  // Read distance at this position
    delay(500);
    
    if(Sweep_1_Array[pos/10] < Min1_value) {
      // If the new distance is less than the min value, store the new distance
      Min1_value = Sweep_1_Array[pos/10];
      Min1_position = pos;    // Store the position the new min was found
    }
  }

  // Print the minimum distance and angle from this array
  Serial.print("Min1 distance: ");
  Serial.println(Min1_value);
  Serial.print("Min1 position: ");
  Serial.println(Min1_position);
  delay(1000);

  // Rotate the robot 180 degrees
  Rotate(180*PulsePerDegree, MOTOR_DIR_FORWARD, MOTOR_DIR_BACKWARD);

  byte Sweep_2_Array[18];   // Declare the second array
  digitalWrite(77, HIGH);
  long Min2_value = Read_Distance();  // Store a reading as the min value
  int Min2_position = pos;    // Store the angle at which the min was found
  
  for(pos = 180; pos>=1; pos -= 10)  {   // goes from 180 to 0 degrees 
    servo.write(pos);    // tell servo to go to position in variable 'pos' 
    delay(15);     // waits 15ms for the servo to reach the position 
    Sweep_2_Array[pos/10] = Read_Distance();  // Read distance at this position
    delay(500);
    
    if(Sweep_2_Array[pos/10] < Min2_value) {
      // If the new distance is less than the min value, store the new distance
      Min2_value = Sweep_2_Array[pos/10];
      Min2_position = pos;    // Store the position the new min was found
    }
  } 

  // Print the minimum distance and angle from this array
  Serial.print("Min2 distance: ");
  Serial.println(Min1_value);
  Serial.print("Min2 position: ");
  Serial.println(Min1_position);
  delay(1000);

  // Compare the minimum distances from the two arrays
  if(Min1_value <= Min2_value){ // If first array contained the smaller distance
    // Turn 180 degrees minus the angle of the servo at that position
    Rotate(180*PulsePerDegree - Min1_position*PulsePerDegree, MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD);
    Serial.print("Min 1 value is less than Min 2 value");
  }

  else { //If the second array contained the smallest distance
   // Turn the robot the same number of degrees as that servo position
    Rotate(Min2_position*PulsePerDegree, MOTOR_DIR_BACKWARD, MOTOR_DIR_FORWARD);
  }

  servo.write(0);   // Bring the servo back to forward
  delay(1000);

  // Drive straight 80 cm or until a bumb sensor is pressed
  Drive_Straight(80/((WheelDiameter * PI)/(PulsePerRev)));

  // Disable the motors once the bump switch is pressed
  Serial.println("Collision detected");
  disableMotor(BOTH_MOTORS);
}

#include <NewPing.h>
#include <Wire.h>
//Declaring some global variables
const int echoPinR = 5; // Echo Pin of Ultrasonic Sensor 
const int trigPinR = 6; // Trigger Pin of Ultrasonic Sensor
const int echoPinL = 10; // Echo Pin of Ultrasonic Sensor 
const int trigPinL = 11; // Trigger Pin of Ultrasonic Sensor
int lastContactR=-1;
int lastContactL=-1;
int currentState=0;
int startTime=0;

int go=0;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles,doPingR=true,doPingL=true;

const int push=4;
const int push2=8;
int oneClick;
int doubleClick;

int lastState=0,lastState2=0;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;

#define SONAR_NUM     2 // Number or sensors.
#define MAX_DISTANCE 200 // Max distance in cm.
#define PING_INTERVAL 7 // Milliseconds between pings.
 
unsigned long pingTimer[SONAR_NUM]; // When each pings.
int cm[SONAR_NUM]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(6, 5, MAX_DISTANCE),
  NewPing(11, 10, MAX_DISTANCE)
};

void setup() {

  oneClick=0;
  doubleClick=0;  
  digitalWrite(push,HIGH);
  digitalWrite(push2,HIGH);
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;                                                 
  Serial.begin(115200);
  loop_timer = micros();                                               //Reset the loop timer
  
  pingTimer[0] = millis() + 75; // First ping start in ms.
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;  
}

void loop(){

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }  
  
  float x,y;
  read_mpu_6050_data();   
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles)
  {                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  //Serial.print(" | AnglePitch  = "); Serial.println(angle_pitch_output);
  //Serial.print(" | AngleRoll  = "); Serial.println(angle_roll_output);

  x = angle_pitch_output;
  y = angle_roll_output+31;

  if(abs(x/10)>2)
  {
    Serial.print(x/3);
    Serial.print(" ");
    Serial.println("00.00");
  }
  else if (abs(y/8)>2)
  {
    Serial.print("00.00");
    Serial.print(" ");
    Serial.println(y/4);
  }
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer   


  if(digitalRead(push)==LOW&&lastState==0)
  {
      Serial.println("Click");
      lastState=1;
      delay(200);
  }
  else if (digitalRead(push)==HIGH)
  {
    lastState=0;
  }

  if(digitalRead(push2)==LOW&&lastState2==0)
  {
      Serial.println("Click2");
      lastState2=1;
      delay(200);
  }
  else if (digitalRead(push2)==HIGH)
  {
    lastState2=0;
  }  
 
}




void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}
long microsecondsToInches(long microseconds) // method to covert microsec to inches  
{  
  return microseconds / 74 / 2; 
}  
long microsecondsToCentimeters(long microseconds) // method to covert microsec to centimeters 
{    
  return microseconds / 29 / 2; 
} 


void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
void oneSensorCycle() { // Do something with the results.
  //Serial.println(cm[0]);
  //Serial.println(cm[1]);
  if(cm[0]!=0)
  {
    if(cm[0]<40||lastContactR!=-1)   
    {
      if(lastContactR==-1) lastContactR=cm[0];
      if(abs(lastContactR-cm[0])>5)
      {
        if(cm[0]>90) lastContactR=-1;
        else if((lastContactR-cm[0])>0) Serial.println("scrollUp");
        else if ((lastContactR-cm[0])<0)Serial.println("scrollDown");
      }
    }
    else
    {
      lastContactR=-1;
    }
  }
  if(cm[1]!=0)
  {
    if(cm[1]<40||lastContactL!=-1)   
    {
      startTime++;
      if(lastContactL==-1) 
        {
          lastContactL=cm[1];
        }
      if(abs(lastContactL-cm[1])>10)
      {
        if(cm[1]>60) lastContactL=-1;
        else if((lastContactL-cm[1])>0) 
        {
          Serial.println("switchTab");
          startTime = 0;
          lastContactL=-1;
          delay(1000);
        }
        else if ((lastContactL-cm[1])<0)
        {
          Serial.println("pause");
          startTime = 0;
          lastContactL=-1;
          delay(1000);
        }
      }
      else if(startTime>60)
      {
        Serial.println("switchApp");
        startTime = 0;
        lastContactL=-1;
        delay(1000);
      }
    }
    else
    {
      lastContactL=-1;
      startTime = 0;
    }
  }
}





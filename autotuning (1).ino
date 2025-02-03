///////////////////////7.37/////////////////
#include <QTRSensors.h>
#include <AFMotor.h>
AF_DCMotor M4(4);
AF_DCMotor M2(2);

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  5 // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

//best for 0.9 80 90//70  80

//best for 3.2 110 120//70  80
//1-->
float Kp = 0.149;//1.2;//the best
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0.0005; //the best
              //change the value by trial-and-error (ex: 0.0008).
float Kd =1.15;//the best 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

#define M4_DEFAULT_SPEED 110
#define M2_DEFAULT_SPEED 110
#define M4_MAX_SPEED 170
#define M2_MAX_SPEED 170

int lastError = 0;
int last_proportional = 0;
int integral = 0;
int move =1; ///////////put one to make the  robot move ///////////READ MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/////
long sum =0;

#include <SoftwareSerial.h>
int i =0;
int c=0;
long bestsum=1000000000;
int n=1;
SoftwareSerial mySerial(18, 19); // where is the RX connected to , where is the TXconnected to 

int arr[8]={0,0,0,0,0,0,0,0};

QTRSensorsAnalog qtra((unsigned char[]) {A8,A5, A4, A3, A2, A1,A0,A9},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int inPin1 = A10;
int val1 = 0;       

int inPin2 = A11;
int val2 = 0; 

int inPin3 = A12;
int val3 = 0;          


void setup()
{
  Serial1.begin(9600);

  while(c !=10)
  {
       c = Serial1.parseInt();

  }

  pinMode(inPin1, INPUT);
  pinMode(inPin2, INPUT);        
  pinMode(inPin3, INPUT);

  delay(1500);
  delay(1000);

}

  
void loop()
{
    long startTime = millis();
    long loopDuration = 10000; // 10 seconds

    while (millis() - startTime < loopDuration) {
        sum += run();
 }

    Serial1.print(" ki is: ");
      Serial1.println(Ki,4);

      Serial1.print("with the sum of it is: ");
      Serial1.println(sum);

    if(sum<bestsum)
    {
      bestsum=sum;
      Serial1.println("AND ITS THE BEST UNTILL NOW !!!");

    }
     
      Serial1.println("--------------------------------");
    sum=0;
    Ki+=0.000;


}
int run()
{

  val1 = digitalRead(inPin1);
  val2 = digitalRead(inPin2);
  val3 = digitalRead(inPin3);

/*if(val3==1&val2==1)
{
  while(val1==0)
  {
  val1 = digitalRead(inPin1);
  val2 = digitalRead(inPin2);
  val3 = digitalRead(inPin3);
  M4.setSpeed(70);     // set motor speed
  M2.setSpeed(70);     // set motor speed
  M4.run(FORWARD);  
  M2.run(FORWARD);

  }
  stop();
  delay(5000);
}*/
  ///////////////////////////////////////////////////////////////////////READ AND PUT IN sensorValues/////////////////////////////////////////////////////////////////////////////
  qtra.read(sensorValues);

  //////SWITCH DATA FROM ANALOG TO DIGITAL AND PUT IT IN ARR//////

  for(int i=0;i<=8;i++)
  {
      if(sensorValues[i]>500)
      {
        arr[i]=1;
      }else {
        arr[i]==0;
      }
  }
     ///////////////////////////////////////////////////////////////////////PID/////////////////////////////////////////////////////////////////////////////
    int position = qtra.readLine(sensorValues);
    int error = position - 3500;      
    ////////////////////////TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO TO DO///////////////////////////////////////////////////////
    // find the perfrct pose ising this equation
    //    0*value0 + 1000*value1 + 2000*value2 + ...
    //   --------------------------------------------
    //         value0  +  value1  +  value2 + ...

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;

  lastError = error; 
  int leftMotorSpeed = M4_DEFAULT_SPEED + motorspeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorspeed;

  if(move)
  {
   set_motors(leftMotorSpeed, rightMotorSpeed);
  }

  return abs(error);
}

void set_motors(int motor4speed, int motor2speed)
{
  if (motor4speed > M4_MAX_SPEED ) motor4speed = M4_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor4speed < 0) motor4speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
  M4.setSpeed(motor4speed);     // set motor speed
  M2.setSpeed(motor2speed);     // set motor speed
  M4.run(FORWARD);  
  M2.run(FORWARD);
}

void stop()
{
  M4.setSpeed(0);     // set motor speed
  M2.setSpeed(0);     // set motor speed
  M4.run(RELEASE);  
  M2.run(RELEASE);
}


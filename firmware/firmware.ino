/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>

//encoder 
#define outputA 2 //GREEN
#define outputB 3 //WHITE


//Motor driver
#define PWM 9 //BROWN
#define INA 7 //YELLOW
#define INB 8 //GREEN

#define EN A0  //WHITE

ros::NodeHandle  nh;

#define MAX_COUNT = 32767;
#define MIN_COUNT = -32768; 
volatile int counter = 0; 

int aState, bState;
int aLastState; 
uint8_t pwm_speed = 0;
uint8_t mot_dir = 0;

std_msgs::Int16 enc_count;
ros::Publisher encoder_publisher("encoder_count", &enc_count);

void directionCallback(const std_msgs::UInt8& mot_dir)
{
  if(mot_dir.data == 1) //motor rotate cw
  {
       digitalWrite(EN,LOW);
       digitalWrite(INA,HIGH);
       digitalWrite(INB,LOW);
     
  }else if (mot_dir.data == 0)//motor rotate ccw
  {
      digitalWrite(EN,LOW);
      digitalWrite(INA,LOW);
      digitalWrite(INB,HIGH);
    
  }
  digitalWrite(EN,HIGH);
}

void pwmCallback(const std_msgs::UInt8& pwm_val)
{
  pwm_speed = pwm_val.data;
  if (pwm_speed >255)
  {
    pwm_speed = 255;
  }
  else if(pwm_speed<0)
  {
    pwm_speed=0;
  }
}

ros::Subscriber<std_msgs::UInt8> pwm_subscriber("pwm_value", pwmCallback);
ros::Subscriber<std_msgs::UInt8> direction_subscriber("motor_direction",directionCallback);

const int interval = 50;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;




void setup() { 
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);

  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz D9 D10
  
  pinMode(PWM,OUTPUT); 
  pinMode(INA,OUTPUT);
  pinMode(INB,OUTPUT);
  pinMode(EN,OUTPUT); 
  
  digitalWrite(INA,HIGH); //Motor Rotate Clockwise
  digitalWrite(INB,LOW);
  digitalWrite(EN,HIGH);
  
  
  attachInterrupt(digitalPinToInterrupt(outputA), isrA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(outputB), isrB, CHANGE); 
  
  nh.initNode();
  nh.advertise(encoder_publisher);
  nh.subscribe(pwm_subscriber);
  nh.subscribe(direction_subscriber);
} 

void loop() { 
  currentMillis = millis();
  analogWrite(PWM,pwm_speed);
  if (currentMillis - previousMillis > interval) //every 100ms
  {
  previousMillis = currentMillis;
  enc_count.data = counter;
  encoder_publisher.publish(&enc_count);
  nh.spinOnce();
  }

}

void isrA()
{
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);
  if (aState == 0)
  {
    if (bState == 0) //krece se ccw
    {
      counter--;
    }
    else  //krece se cw
    {
      counter++;
    }
  }else
  {
     if (bState == 0) //krece se cw
    {
      counter++;
    }
    else //krece se ccw
    {
      counter--;
    }
  }
}
void isrB()
{
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);
  if (bState == 0) 
  {
    if (aState == 0) //krece se cw
    {
      counter++;
    }
    else  //krece se ccw
    {
      counter--;
    }
  }else
  {
     if (aState == 0) //krece se ccw
    {
      counter--;
    }
    else  //krece se cw
    {
      counter++;
    }
  }
}

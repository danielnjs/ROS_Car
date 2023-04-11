#include <ros.h>
#include <std_msgs/String.h>
#include <NewPing.h>

/**********
 * Define *
 *********/
#define Delay    10

#define US_Total    8
#define US_Max_Dist 200
#define US_MUX_S0   2     //MUX Sel 0
#define US_MUX_S1   3     //MUX Sel 1
#define US_MUX_S2   4     //MUX Sel 2
#define US_trigger  5
#define US_echo     6

/************************
 * Variable Declaration *
 ***********************/
String US_readings;

ros::NodeHandle nh;

std_msgs::String ultrasound_msg;

ros::Publisher ultrasound_pub("US_distance", &ultrasound_msg);

/************************
 * Function Declaration *
 ***********************/
void UltraSound_Mux(int);
NewPing sonar(US_trigger, US_echo, US_Max_Dist);

void setup() {
    nh.initNode();
    nh.advertise(ultrasound_pub);
    pinMode(US_MUX_S0,OUTPUT); pinMode(US_MUX_S1,OUTPUT); pinMode(US_MUX_S2,OUTPUT);
}

void loop() {
    for(int i=0; i< US_Total; i++){
      UltraSound_Mux(i);
      US_readings += String(sonar.ping_cm());
      if(i != US_Total - 1) US_readings += " ";
    }
   
    ultrasound_msg.data = readings.c_str();
    ultrasound_pub.publish(&ultrasound_msg);
    nh.spinOnce();
    delay(Delay);
}

/******************************************************************************
 * BRIEF : To toggle between the Sonar Mux, catered to 8 US Sensors currently *
 *****************************************************************************/
void UltraSound_Mux(int Idx) {
  switch (Idx){
    case 0: digitalWrite(US_MUX_S0,LOW);  digitalWrite(US_MUX_S1,LOW);  digitalWrite(US_MUX_S2,LOW);  break;
    case 1: digitalWrite(US_MUX_S0,HIGH); digitalWrite(US_MUX_S1,LOW);  digitalWrite(US_MUX_S2,LOW);  break;
    case 2: digitalWrite(US_MUX_S0,LOW);  digitalWrite(US_MUX_S1,HIGH); digitalWrite(US_MUX_S2,LOW);  break;
    case 3: digitalWrite(US_MUX_S0,HIGH); digitalWrite(US_MUX_S1,HIGH); digitalWrite(US_MUX_S2,LOW);  break;
    case 4: digitalWrite(US_MUX_S0,LOW);  digitalWrite(US_MUX_S1,LOW);  digitalWrite(US_MUX_S2,HIGH); break;
    case 5: digitalWrite(US_MUX_S0,HIGH); digitalWrite(US_MUX_S1,LOW);  digitalWrite(US_MUX_S2,HIGH); break;
    case 6: digitalWrite(US_MUX_S0,LOW);  digitalWrite(US_MUX_S1,HIGH); digitalWrite(US_MUX_S2,HIGH); break;
    case 7: digitalWrite(US_MUX_S0,HIGH); digitalWrite(US_MUX_S1,HIGH); digitalWrite(US_MUX_S2,HIGH); break;
  }
}

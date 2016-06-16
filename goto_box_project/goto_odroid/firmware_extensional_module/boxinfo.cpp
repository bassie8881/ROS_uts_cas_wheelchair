/*
 * Sending two button signals through rosserial
 */
 #define USE_USBCON1
 #include <ros.h>
 #include <std_msgs/Bool.h>
 #include <std_msgs/Int16.h>
 #include <std_msgs/Int16MultiArray.h>
 #include <std_msgs/MultiArrayLayout.h>
 #include <std_msgs/MultiArrayDimension.h>
 #include <ros/time.h>
 #include <tf/transform_broadcaster.h>
 #include <std_msgs/Empty.h>
 #include <Arduino.h>

 ros::NodeHandle nh;

 bool switching = false;
 bool last_reading1;
 bool last_reading2;
 bool published1 = true;
 bool published2 = true;
 bool Buzzer = false;
 bool second = false;
 bool third = false;
 bool fourth = false;
 bool fifth = false;
 long last_debounce_time1=0;
 long last_debounce_time2=0;
 long debounce_delay1=50;
 long debounce_delay2=50;
 const long interval = 600;
 unsigned long previousMillis=0;
 unsigned long pMilb=0;
 unsigned long cMilb=0;
 int looping = 5;
 char buttons[] = "/buttons";
 const int red_led = 2;
 const int green_led = 3;
 const int button_pin1 = 6;
 const int button_pin2 = 7;
 const int buzzer_pin = 9;

 void messageLED(const std_msgs::Int16MultiArray& LED_msg){
     if((LED_msg.data[0]==1)&&(LED_msg.data[1]==1)){
         switching = true;
     }
     if((LED_msg.data[0]==1)&&(LED_msg.data[1]==0)){
        digitalWrite(green_led, HIGH);switching =false;}
     if((LED_msg.data[0]==0)&&(LED_msg.data[1]==0)){
        digitalWrite(green_led, LOW);switching=false;}}

 void messageLED2(const std_msgs::Int16MultiArray& LED2_msg){
     if(LED2_msg.data[0]==1){
         digitalWrite(red_led, HIGH);
     }
 }

 void messageBUZZ(const std_msgs::Int16MultiArray& Buzz_msg){
     if(Buzz_msg.data[0]==1){
         Buzzer = true;
         looping = 0;
         cMilb = millis();}
 }

ros::Subscriber<std_msgs::Int16MultiArray> sub1("/LED", messageLED);
ros::Subscriber<std_msgs::Int16MultiArray> sub3("/LED2", messageLED2);
ros::Subscriber<std_msgs::Int16MultiArray> sub4("/Buzzer", messageBUZZ);
std_msgs::Int16MultiArray pushed_msg;
ros::Publisher pub_buttons("pushed", &pushed_msg);

 void setup()
 {
  nh.initNode();
  pushed_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) *2);
  pushed_msg.layout.dim[0].label = buttons;
  pushed_msg.layout.dim[0].size = 2;
  pushed_msg.layout.dim[0].stride = 1*2;
  pushed_msg.data_length = 2;
  pushed_msg.data = (int *)malloc(sizeof(int)*8);
  nh.advertise(pub_buttons);

  pinMode(button_pin1, INPUT);
  pinMode(button_pin2, INPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);

  digitalWrite(button_pin1, HIGH);
  digitalWrite(button_pin2, HIGH);
  digitalWrite(green_led, HIGH);

  last_reading1 = ! digitalRead(button_pin1);
  last_reading2 = ! digitalRead(button_pin2);

  nh.initNode();
  nh.spinOnce();
  nh.subscribe(sub1);
  nh.spinOnce();
  nh.subscribe(sub3);
  nh.spinOnce();
  nh.subscribe(sub4);
  nh.spinOnce();
 }

 void loop()
 {
  bool reading1 = ! digitalRead(button_pin1);
  bool reading2 = ! digitalRead(button_pin2);

  if(last_reading1!= reading1){
    last_debounce_time1 = millis();
    published1 = false;
  }
  if(last_reading2!= reading2){
    last_debounce_time2 = millis();
    published2 = false;
  }
  if( !published1 && (millis() - last_debounce_time1) > debounce_delay1){
    pushed_msg.data[0] = reading1;
    pushed_msg.data[1] = last_reading2;
    pub_buttons.publish(&pushed_msg);
    published1 = true;
  }
  if( !published2 && (millis() - last_debounce_time2) > debounce_delay2){
    pushed_msg.data[0] = last_reading1;
    pushed_msg.data[1] = reading2;
    pub_buttons.publish(&pushed_msg);
    published2 = true;
  }

  last_reading1 = reading1;
  last_reading2 = reading2;

  unsigned long currentMillis = millis();
  if((switching==true)&&(currentMillis - previousMillis >= interval)){
      previousMillis = currentMillis;
      digitalWrite(green_led, HIGH-digitalRead(green_led));
  }

  cMilb = millis();
  if((Buzzer==true)&&(cMilb - pMilb >= 900)&&(looping<=4)){
      pMilb = cMilb;
      analogWrite(buzzer_pin, 3);
      Buzzer = false;
      second = true;}

  cMilb = millis();
  if((second==true)&&(cMilb - pMilb >= 800)&&(looping<=4)){
      pMilb = cMilb;
      analogWrite(buzzer_pin, 9);
      second = false;
      third = true;}

  cMilb = millis();
  if((third==true)&&(cMilb - pMilb >= 800)&&(looping<=4)){
      pMilb = cMilb;
      analogWrite(buzzer_pin, 15);
      third = false;
      fourth = true;}

  cMilb = millis();
  if((fourth==true)&&(cMilb - pMilb >= 800)&&(looping<=4)){
      pMilb = cMilb;
      analogWrite(buzzer_pin, 0);
      fourth = false;
      fifth = true;}

  cMilb = millis();
  if((fourth==true)&&(cMilb - pMilb >= 1500)&&(looping<=4)){
      pMilb = cMilb;
      fifth = false;
      looping = looping + 1;
      if(looping>=5){
          Buzzer = false;}
      else{
          Buzzer = true;}}

  nh.spinOnce();
 }

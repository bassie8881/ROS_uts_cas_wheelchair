/*
 * Sending and receiving joystick information
 */
#define USE_USBCON1
#include <ros.h>
#include <sensor_msgs/Joy.h>

//consts
float JV_MAX_ = 4.0, JV_MIN_ = 1.0,
  JV_S_ = JV_MAX_+JV_MIN_, JV_D_ = JV_MAX_-JV_MIN_;
float BRAKE_[2] = { 0.0,0.0 }, JOY_DZ_ = 0.05;

//IO related
int JOY_R_PINS[4] = { A3,A1,A2,A0 }, JOY_W_PINS[4] = { 2,4,3,5 };
float DAC_ = 5 / 1023.0, ADC_ = 255 / 5.0;
int SWITCH_PINS[2] = { A6,A7 }, SWITCH_THRESH_ = 512;

//ROS
ros::NodeHandle nh;
sensor_msgs::Joy joyoutmsg_; ros::Publisher jpub("/wc_joy_", &joyoutmsg_);
//other
float joyr_[4],joyw_[4], joyraxes_[2],joysaxes_[2] = { 0.0,0.0 };

void joy_cb(const sensor_msgs::Joy& msg)
{
  joysaxes_[0] = msg.axes[0]; joysaxes_[1] = msg.axes[1];
}
ros::Subscriber<sensor_msgs::Joy> jsub("/wc_joy", joy_cb);

int get_switch_state()
{
  if(analogRead(SWITCH_PINS[0]) > SWITCH_THRESH_) return 1;
  else if(analogRead(SWITCH_PINS[1]) > SWITCH_THRESH_) return 2;
  else return 0;
}

void read_joy()
{
  for(int i=0; i<4; i++)
  {
    joyr_[i] = analogRead(JOY_R_PINS[i]) * DAC_;
  }

  joyraxes_[1] = max(-1.0, min((joyr_[0]-joyr_[1]) / JV_D_, 1.0));
  joyraxes_[0] = max(-1.0, min((joyr_[2]-joyr_[3]) / JV_D_, 1.0));
  if(abs(joyraxes_[1]) < JOY_DZ_) joyraxes_[1] = 0.0;
  if(abs(joyraxes_[0]) < JOY_DZ_) joyraxes_[0] = 0.0;
}

void write_joy(float axes[])
{
  float vpos_ = (axes[1]+1.0)/2, wpos_ = (axes[0]+1.0)/2;
  joyw_[0] = JV_MIN_ + vpos_*JV_D_; joyw_[1] = JV_S_ - joyw_[0];
  joyw_[2] = JV_MIN_ + wpos_*JV_D_; joyw_[3] = JV_S_ - joyw_[2];
  for(int i=0; i<4; i++) analogWrite(JOY_W_PINS[i], int(joyw_[i]*ADC_));
}

void setup()
{
  //node related
  nh.initNode();
  nh.subscribe(jsub);
  joyoutmsg_.axes_length = 2; joyoutmsg_.axes = joyraxes_; nh.advertise(jpub);
  nh.spinOnce();
}

void loop()
{
  read_joy(); jpub.publish(&joyoutmsg_);

  switch(get_switch_state())
  {
    case 1:
      write_joy(joyraxes_);
      break;
    case 2:
      write_joy(joysaxes_);
      break;
    default:
      write_joy(BRAKE_);
      break;
  }
  nh.spinOnce();
}

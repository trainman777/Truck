#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
#include <Arduino.h>
#include <Servo.h>

ros::NodeHandle nh;

geometry_msgs::Twist cmd_vel;
ros::Publisher chatter("cmd_vel_msg", &cmd_vel);
int steer = 9;
int drive = 10;
Servo steering;
Servo driving;
double angle, drive_speed,straight =100, still = 90, max_speed = 0.5, max_turn = 0.3, forward, turn;

void messageMV(const geometry_msgs::Twist& move_msg)
{
  forward = move_msg.linear.x; 
  turn = move_msg.angular.z; 
  
  //angular determines percent devoted to each side

  angle = straight-(turn/max_turn)*90;
  drive_speed = (forward/max_speed)*90+still;
  driving.write(drive_speed);
  steering.write(angle);
  
  //chatter.publish(angle);
}

ros::Subscriber<geometry_msgs::Twist> input("/cmd_vel", &messageMV);

void setup()
{
  nh.initNode();
  nh.subscribe(input);
  nh.advertise(chatter);
  steering.attach(steer);
  driving.attach(drive);
}

void loop()
{
  nh.spinOnce();
  delay(4);
}

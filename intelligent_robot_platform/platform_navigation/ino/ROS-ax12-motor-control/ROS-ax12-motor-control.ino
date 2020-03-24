/*
 * Title: Dual DC Motor Control with Encoder and ax12 servo
 * Author: Khairul Izwan 22-03-2020
 * Description: Controlling Dual DC Motor with Encoder using
 * 10Amp 7V-30V DC Motor Driver Shield for Arduino (2 Channels)
 */

/* Parts ::
 * 1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor 
 * :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor?search=12%2038rpm&description=1&src=search.list
 * 2. Driver:: Shield L298P Motor Driver with GPIO 
 * :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list
 */

//include necessary library
#include <ros.h>
#include <std_msgs/Int8.h>

/*
 * Add changes here!
 * AX-12
 */
#include "Arduino.h"
#include "AX12A.h"

/*
 * Add changes here!
 * AX-12
 */
#define DirectionPin   (10u)
#define BaudRate      (1000000ul)
#define ID1       (10u)
#define ID2        (9u)
#define ID3        (8u)
#define ID4        (7u)

//Speed
int SPD = 300;

//AX-12 motorDirection helper function
void messageCb_ax12_control(const std_msgs::Int8 &msg)
{
  if (msg.data == 2)
  {
    ax12a.turn(ID1, LEFT, SPD);
    ax12a.turn(ID2, LEFT, SPD);
    ax12a.turn(ID3, RIGHT, SPD);
    ax12a.turn(ID4, RIGHT, SPD);
    
    ax12a.ledStatus(ID1, ON);
    ax12a.ledStatus(ID2, ON);
    ax12a.ledStatus(ID3, ON);
    ax12a.ledStatus(ID4, ON);
  }
  else if (msg.data == 1)
  {
    ax12a.turn(ID1, RIGHT, SPD);
    ax12a.turn(ID2, RIGHT, SPD);
    ax12a.turn(ID3, LEFT, SPD);
    ax12a.turn(ID4, LEFT, SPD);
    
    ax12a.ledStatus(ID1, ON);
    ax12a.ledStatus(ID2, ON);
    ax12a.ledStatus(ID3, ON);
    ax12a.ledStatus(ID4, ON);
  }
  else
  {
    ax12a.turn(ID1, LEFT, 0);
    ax12a.turn(ID2, LEFT, 0);
    ax12a.turn(ID3, RIGHT, 0);
    ax12a.turn(ID4, RIGHT, 0);
  
    ax12a.ledStatus(ID1, OFF);
    ax12a.ledStatus(ID2, OFF);
    ax12a.ledStatus(ID3, OFF);
    ax12a.ledStatus(ID4, OFF);
  }
}

//Set up the ros node (publisher and subscriber)
ros::Subscriber<std_msgs::Int8> sub_ax12_control("/cmd_servo_dir", messageCb_ax12_control);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
  ax12a.begin(BaudRate, DirectionPin, &Serial1);
  
  ax12a.setEndless(ID1, ON);
  ax12a.setEndless(ID2, ON);
  ax12a.setEndless(ID3, ON);
  ax12a.setEndless(ID4, ON);

//  Initiate ROS-node
  nh.initNode();

  nh.subscribe(sub_ax12_control);
}

//put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
}

/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;
void messageCb(const std_msgs::Int8 &toggle_msg)
{
    if (toggle_msg.data==1)
    {
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    }
    else
    {
        digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    }
}

ros::Subscriber<std_msgs::Int8> sub("led", &messageCb);

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}

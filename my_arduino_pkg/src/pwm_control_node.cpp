// pwm_control_node.cpp
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <bbox_det/order_to_pwm.h>  // 추가: bbox_det 패키지의 order_to_pwm 메시지

ros::Publisher pwm_pub;  // 추가: PWM 값을 아두이노로 전달하기 위한 publisher

bool shouldStopMotor = false;
void pwmCallback(const std_msgs::UInt16& msg)
{
  // Your PWM control logic here
  ROS_INFO("Received PWM value: %d", msg.data);

  // Check for a condition to stop the motor
  if (msg.data > 500) {
    shouldStopMotor = true;
  } else {
    shouldStopMotor = false;
  }

  // Publish the received PWM value to Arduino
  pwm_pub.publish(msg);
}

void orderToPwmCallback(const bbox_det::order_to_pwm& order_msg)
{
  float diff_cx=order_msg.diff_cx;
  float diff_cy=order_msg.diff_cy;
  std_msgs::UInt16 pwm_msg;
  if (diff_cy <=90400){
  if (diff_cx >= 120) {
    ROS_INFO("Box data is greater than or equal to 30. Performing right steering control.");

    std_msgs::UInt16 pwm_msg;
    pwm_msg.data = 3;  
    pwm_pub.publish(pwm_msg);

  } else if (diff_cx <= -120) {
    ROS_INFO("Box data is less than or equal to -30. Performing left steering control.");

    std_msgs::UInt16 pwm_msg;
    pwm_msg.data = 1;  
    pwm_pub.publish(pwm_msg);

  } else {
    ROS_INFO("Box data is between -30 and 30. Performing straight steering control.");

    std_msgs::UInt16 pwm_msg;
    pwm_msg.data = 2;  
    pwm_pub.publish(pwm_msg);
  }
  }
  else{
    pwm_msg.data=0;
    pwm_pub.publish(pwm_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pwm_control_node");
  ros::NodeHandle nh;

  pwm_pub = nh.advertise<std_msgs::UInt16>("pwm_to_arduino", 10);

  // Subscribe to the PWM topic
  ros::Subscriber pwm_sub = nh.subscribe("pwm", 10, pwmCallback);
  
  // Subscribe to the order_to_pwm topic
  ros::Subscriber order_to_pwm_sub = nh.subscribe("order_to_pwm", 10, orderToPwmCallback);
  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}



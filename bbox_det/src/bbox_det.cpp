#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"
#include "bbox_det/order_to_pwm.h"

class BboxDet{ 

  private:
    int count = 0;
    
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher order_to_pwm_pub;

  public:
    BboxDet();
    ~BboxDet();
    void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg);

};

BboxDet::BboxDet(){
  sub = nh.subscribe("detection_result", 100, &BboxDet::detectionCallback, this);
  order_to_pwm_pub = nh.advertise<bbox_det::order_to_pwm>("order_to_pwm", 10);
}

BboxDet::~BboxDet(){}

void BboxDet::detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg){
  for (const auto& detection : msg->detections) { 
    //detection_result 토픽을 subscribe함
    float cx = detection.bbox.center.x;
    float cy = detection.bbox.center.y;
    float w = detection.bbox.size_x;
    float h = detection.bbox.size_y;

    // Bounding box의 네 꼭지점 좌표 계산
    float xmin = cx - w / 2.0;
    float ymin = cy - h / 2.0;
    float xmax = cx + w / 2.0;
    float ymax = cy + h / 2.0;

    //전체 카메라 화각의 중점좌표(절댓값) - 바운딩 박스의 중점 좌표 계산
    float diff_cx = 320 - cx;
    float diff_cy = 240 - cy;

    // order_to_pwm 토픽으로 위에 계산한 값들을 메시지로 publish
    bbox_det::order_to_pwm order_msg;
    order_msg.diff_cx = diff_cx;
    order_msg.diff_cy = diff_cy;
    order_to_pwm_pub.publish(order_msg);
       
    ROS_INFO( //publish한 값들 확인차 출력
      "Published order_to_pwm message: diff_center - (%f, %f)",
      diff_cx, diff_cy
    );
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bbox_det");

  BboxDet bbox_det;

  ros::Rate rate(10); // 토픽 발사 주기가 10hz

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
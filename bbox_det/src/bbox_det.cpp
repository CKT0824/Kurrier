#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"
#include "bbox_det/order_to_pwm.h"

// int count = 0;
// float prev_cx = 0.0, prev_cy = 0.0;


class BboxDet{

  private:
    int count = 0;
    float prev_cx = 0.0;
    float prev_cy = 0.0;
    
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
      float cx = detection.bbox.center.x;
      float cy = detection.bbox.center.y;
      float w = detection.bbox.size_x;
      float h = detection.bbox.size_y;

      // Bounding box의 네 꼭지점 좌표 계산
      float xmin = cx - w / 2.0;
      float ymin = cy - h / 2.0;
      float xmax = cx + w / 2.0;
      float ymax = cy + h / 2.0;

      if(count == 0)
      {
      /*
        ROS_INFO(
          "BB_left_bottom: (%f, %f) , BB_right_top: (%f, %f)",
          xmin, ymin, xmax, ymax
        );
      */
      }
      else if(count != 0)
      {
        float diff_cx = 320 - cx;
        float diff_cy = w*h;

        // order_to_pwm 토픽으로 메시지를 발행
        bbox_det::order_to_pwm order_msg;
        order_msg.diff_cx = diff_cx;
        order_msg.diff_cy = diff_cy;
        order_to_pwm_pub.publish(order_msg);

        
        ROS_INFO(
          "Published order_to_pwm message: diff_center - (%f, %f)",
          diff_cx, diff_cy
        );
        
      }
        
      /*
        ROS_INFO(
          "BB_left_bottom: (%f, %f) , BB_right_top: (%f, %f), diff_center: (%f, %f)",
          xmin, ymin, xmax, ymax, diff_cx, diff_cy
        );
      */
      prev_cx = cx;
      prev_cy = cy;
        /*
        "ID: %ld, Center: (%f, %f), Size: (%f, %f), BB_leftbottom_p: (%f, %f)",
        detection.results[0].id,
        cx, cy,
        w, h,
        xmin, ymin, xmax, ymax
        */
    }
    count++;
}
// void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg)
// {
//   /*
//   float prev_xmin = 0.0, prev_ymin = 0.0, prev_xmax = 0.0, prev_ymax = 0.0;
//   */
//   // ros::NodeHandle nh;
//   // ros::Publisher order_to_pwm_pub = nh.advertise<bbox_det::order_to_pwm>("order_to_pwm", 10);
  
//   for (const auto& detection : msg->detections) {
//     float cx = detection.bbox.center.x;
//     float cy = detection.bbox.center.y;
//     float w = detection.bbox.size_x;
//     float h = detection.bbox.size_y;

//     // Bounding box의 네 꼭지점 좌표 계산
//     float xmin = cx - w / 2.0;
//     float ymin = cy - h / 2.0;
//     float xmax = cx + w / 2.0;
//     float ymax = cy + h / 2.0;

//     if(count == 0)
//     {
//     /*
//       ROS_INFO(
//         "BB_left_bottom: (%f, %f) , BB_right_top: (%f, %f)",
//         xmin, ymin, xmax, ymax
//       );
//     */
//     }
//     else if(count != 0)
//     {
//       float diff_cx = cx - prev_cx;
//       float diff_cy = cy - prev_cy;

//       // order_to_pwm 토픽으로 메시지를 발행
//       bbox_det::order_to_pwm order_msg;
//       order_msg.diff_cx = diff_cx;
//       order_msg.diff_cy = diff_cy;
//       order_to_pwm_pub.publish(order_msg);

//       ROS_INFO(
//         "Published order_to_pwm message: diff_center - (%f, %f)",
//         diff_cx, diff_cy
//       );
//     }
      
//     /*
//       ROS_INFO(
//         "BB_left_bottom: (%f, %f) , BB_right_top: (%f, %f), diff_center: (%f, %f)",
//         xmin, ymin, xmax, ymax, diff_cx, diff_cy
//       );
//     */
//     prev_cx = cx;
//     prev_cy = cy;
//       /*
//       "ID: %ld, Center: (%f, %f), Size: (%f, %f), BB_leftbottom_p: (%f, %f)",
//       detection.results[0].id,
//       cx, cy,
//       w, h,
//       xmin, ymin, xmax, ymax
//       */
//   }
//   count++;
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "bbox_det");
  // ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("detection_result", 100, detectionCallback);

  // Subscriber 예시
  BboxDet bbox_det;

  ros::Rate rate(10); // 10hz

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  // ros::spin();

  return 0;
}
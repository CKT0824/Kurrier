#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <std_msgs/Int32.h>

class PerceptionNode {
private:
    ros::NodeHandle nh;
    ros::Subscriber detection_sub;
    ros::Publisher box_data_pub;

public:
    PerceptionNode() {
        detection_sub = nh.subscribe("detection_result", 100, &PerceptionNode::detectionCallback, this);
        box_data_pub = nh.advertise<std_msgs::Int32>("box_data", 10);
    }

    void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
        // Your perception logic to process the detection results and calculate box_data
        int box_data = /* Your calculation logic based on detection results */;
        
        std_msgs::Int32 box_data_msg;
        box_data_msg.data = box_data;
        box_data_pub.publish(box_data_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_node");
    PerceptionNode perception_node;
    ros::spin();
    return 0;
}


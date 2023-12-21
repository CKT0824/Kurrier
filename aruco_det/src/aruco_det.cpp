#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/Int32.h>

class ArucoDet {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    ros::Publisher aruco_id_pub;

public:
    ArucoDet() : it(nh) {
        sub = it.subscribe("usb_cam/image_raw", 1, &ArucoDet::imageCallback, this);
        aruco_id_pub = nh.advertise<std_msgs::Int32>("order_to_pwm_marker", 10);
    }

    ~ArucoDet() {}

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {//image_raw를 통해 받은 ROS 이미지를 OpenCV image변수 frame으로 변환, bgr색으로 인코딩 
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            std::vector<std::vector<cv::Point2f>> corners; //Aruco Marker 꼭지점 좌표 저장할 벡터 정의
            std::vector<int> ids; //Aruco Marker ID 저장할 벡터 정의
            cv::aruco::detectMarkers(frame, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), corners, ids);
            //cv::aruco::DICT_4X4_50 형태의 마커를 frame 이미지에서 발견하는 함수 작동, 이때 두 벡터를 통해 검출된 값 저장
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            //검출된 값들을 frame이미지에 표시
            cv::imshow("ArUco Detection", frame);
            //검출된 값들이 표시된 frame 이미지를 Aruco Detection이라는 새로운 창을 통해 보여줌
            cv::waitKey(1); //이미지 업데이트하는데 대기시간 1ms

            publishArucoIDs(ids); //Aruco Marker의 ID값만 topic으로 publish

        } catch (cv_bridge::Exception& e) { // cv_bridge에서의 예외처리
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void publishArucoIDs(const std::vector<int>& ids) {
        for (size_t i = 0; i < ids.size(); ++i) { //ids 벡터가 1개라도 있다면
            std_msgs::Int32 aruco_msg; //ROS topic(aruco_msg) 객체 생성
            aruco_msg.data = ids[i]; 
            aruco_id_pub.publish(aruco_msg); //id값들을 모두 publish
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_det");
    ArucoDet aruco_det;
    ros::spin();

    return 0;
}
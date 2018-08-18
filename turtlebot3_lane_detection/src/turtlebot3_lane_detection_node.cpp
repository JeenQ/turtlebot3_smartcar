#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

ros::Publisher pub_lane;
ros::Subscriber sub_image;

void getImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    Mat src, hsv, yellow, white, morpho_yellow, morpho_white, canny_yellow, canny_white;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    src = cv_ptr->image;
    Rect rect = Rect(0, 200, src.size().width, src.size().height-200);
    Mat roi = src(rect);

    GaussianBlur(roi, roi, Size(3, 3), 0, 0);
    cvtColor(roi, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(0, 100, 100), Scalar(40, 255, 255), yellow);
    inRange(hsv, Scalar(0, 0, 200), Scalar(180, 40, 255), white);
    threshold(yellow, yellow, 100, 255, THRESH_BINARY);
    threshold(white, white, 100, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(yellow, morpho_yellow, CV_MOP_CLOSE, element);
    Canny(morpho_yellow, canny_yellow, 0, 255, 3);
    // bitwise_not(morpho_yellow, not_yellow);
    // bitwise_and(white, not_yellow, white);
    morphologyEx(white, morpho_white, CV_MOP_CLOSE, element);
    Canny(morpho_white, canny_white, 0, 255, 3);

    vector<Vec4i> lines_yellow, lines_white;
    HoughLinesP(canny_yellow, lines_yellow, 1, CV_PI / 180, 50, 0, 0);
    HoughLinesP(canny_white, lines_white, 1, CV_PI / 180, 25, 0, 0);

    Vec4i avg_line_yellow, avg_line_white;
    int cnt_yellow=0, cnt_white=0;
    for(int i=0; i<lines_yellow.size(); i++){
        if (lines_yellow[i][0] != lines_yellow[i][2]) {
            double angle = atan((double)(lines_yellow[i][1] - lines_yellow[i][3]) / (lines_yellow[i][0] - lines_yellow[i][2])) * 180 / M_PI;
            // cout<<angle<<endl;
            if(abs(angle) > 30){
                // line(roi, Point(lines_yellow[i][0], lines_yellow[i][1]), Point(lines_yellow[i][2], lines_yellow[i][3]), Scalar(0, 0, 255), 2);
                avg_line_yellow+=lines_yellow[i];
                cnt_yellow++;
            }
        }
    }
    avg_line_yellow/=cnt_yellow;

    for(int i=0; i<lines_white.size(); i++){
        if (lines_white[i][0] != lines_white[i][2]) {
            double angle = atan((double)(lines_white[i][1] - lines_white[i][3]) / (lines_white[i][0] - lines_white[i][2])) * 180 / M_PI;
            if(abs(angle) > 30){
                // line(roi, Point(lines_white[i][0], lines_white[i][1]), Point(lines_white[i][2], lines_white[i][3]), Scalar(255, 0, 0), 2);
                avg_line_white+=lines_white[i];
                cnt_white++;
            }
        }
    }
    avg_line_white/=cnt_white;

    double a_yellow, b_yellow, a_white, b_white;
    a_yellow = (double)(avg_line_yellow[3]-avg_line_yellow[1]) / (avg_line_yellow[2]-avg_line_yellow[0]);
    b_yellow = (double)(avg_line_yellow[2]*avg_line_yellow[1] - avg_line_yellow[0]*avg_line_yellow[3]) / (avg_line_yellow[2]-avg_line_yellow[0]);
    a_white = (double)(avg_line_white[3]-avg_line_white[1]) / (avg_line_white[2]-avg_line_white[0]);
    b_white = (double)(avg_line_white[2]*avg_line_white[1] - avg_line_white[0]*avg_line_white[3]) / (avg_line_white[2]-avg_line_white[0]);
    line(roi, Point(-1*b_yellow/a_yellow, 0), Point((roi.size().height-b_yellow)/a_yellow, roi.size().height), Scalar(0, 0, 255), 2);
    line(roi, Point(-1*b_white/a_white, 0), Point((roi.size().height-b_white)/a_white, roi.size().height), Scalar(255, 0, 0), 2);

    std_msgs::Float64 lane_center;
    lane_center.data = (b_white - b_yellow) / (a_yellow - a_white);
    pub_lane.publish(lane_center);
    // cout<<lane_center<<endl;

    imshow("src", src);
    imshow("roi", roi);
    imshow("white", white);
    waitKey(33);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot3_lane_detection");
    ros::NodeHandle nh;
    pub_lane = nh.advertise<std_msgs::Float64>("/detect/lane", 1);
    sub_image = nh.subscribe("/usb_cam/image_raw", 1, getImage);

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
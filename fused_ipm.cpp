#include "image_properties.h"
//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include "IPM.h"
#include "IPM.hpp"

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;
Mat roi,final_grid;
Mat src_left=Mat::zeros(Size(1280, 720), CV_8UC3);
Mat src_center=Mat::zeros(Size(1280, 720), CV_8UC3);
Mat src_right=Mat::zeros(Size(1280, 720), CV_8UC3);
Mat map1_left,map2_left;
Mat map1_center,map2_center;
Mat map1_right,map2_right;

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub_fused;
image_transport::Publisher pub_1;
image_transport::Publisher pub_2;
image_transport::Publisher pub_3;
void leftimage(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    Mat src = cv_ptr->image;
    copyMakeBorder(src,src,0,0,100,0,BORDER_CONSTANT,Scalar(0));	//makes border of 100 pixel to the left to get extra view in src_left
    remap(src, src_left, map1_left, map2_left, CV_INTER_LINEAR);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_left).toImageMsg();
    pub_1.publish(msg1);
}

void centerimage(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
            return;
    }
    Mat src = cv_ptr->image;
    remap(src, src_center, map1_center, map2_center, CV_INTER_LINEAR);
	sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_center).toImageMsg();
    pub_2.publish(msg2);

}

void rightimage(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    Mat src = cv_ptr->image;
    copyMakeBorder(src,src,0,0,0,100,BORDER_CONSTANT,Scalar(0));	//makes border of 100 pixel to the right to get extra view in src_right
    src=src(Rect(100, 0, 1280, 720)).clone();	//removes 100 pixels from left side of src_right
    remap(src, src_right, map1_right, map2_right, CV_INTER_LINEAR);
    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_right).toImageMsg();
    pub_3.publish(msg3);

    
}


void fuse(){
    int image_width=1280,image_height=720;
    int height=1000,width=1480;
    int center_x = 750,center_y = 1000;
    final_grid = Mat::zeros(Size(width/*width*/, height/*height*/), CV_8UC3);
    roi = Mat::zeros(Size(width/*width*/, height/*height*/), CV_8UC3);

	//ipm left
    int h=150;
    int v=250;
    Point2f p_left[4] = {Point(26, 428), Point(551, 393),Point(836, 478) ,Point(15,622)};
    Point2f q_left[4] = {Point(h+center_x+(-1000/2), v+center_y-(-300/2)), Point(h+center_x+(-1000/2), v+center_y-(300/2)), Point(h+center_x+(-500/2),v+center_y-(300/2)), Point(h+center_x+(-500/2),v+center_y-(-300/2))};
    
    Mat transform_left (3, 3, CV_32FC1);
    transform_left = getPerspectiveTransform(p_left, q_left);

    //crop image below horizon
    Mat result_left = Mat::zeros(Size(image_width, image_height), CV_8UC3);
    src_left(Rect(0, /*31*/0, image_width, image_height-/*31*/0)).copyTo(result_left(Rect(0, /*31*/0, image_width, image_height-/*31*/0)));
    //imshow("processed1",result_left);
    //imshow("original1",src_left);

    
    warpPerspective(result_left, roi, transform_left, Size(width, height));

    Mat LeftROI;
    roi.copyTo(LeftROI);

    cv::Point2f pc(center_x,center_y);
    cv::Mat r = cv::getRotationMatrix2D(pc, -7, 1.0);
    cv::warpAffine(LeftROI, LeftROI, r, LeftROI.size());

    Mat black_left = Mat::zeros(Size(width-460, height), CV_8UC3);
    black_left.copyTo(LeftROI(Rect(460, 0, black_left.cols, black_left.rows)));

    //final_grid = final_grid + LeftROI;
        
    
    //ipm right
    h=-50;
    v=150;
    Point2f p_right[4] = {Point(720, 387), Point(1186, 417), Point(1216, 589), Point(482, 456)};
    Point2f q_right[4] = {Point(h+center_x+(1000/2), v+center_y-(300/2)), Point(h+center_x+(1000/2), v+center_y-(-300/2)), Point(h+center_x+(500/2), v+center_y-(-300/2)), Point(h+center_x+(500/2), v+center_y-(300/2))};

    Mat transform_right (3, 3, CV_32FC1);
    transform_right = getPerspectiveTransform(p_right, q_right);

    //crop image below horizon
    Mat result_right = Mat::zeros(Size(image_width, image_height), CV_8UC3);
    src_right(Rect(0, 310, image_width, image_height-310)).copyTo(result_right(Rect(0, 310, image_width, image_height-310)));
    //imshow("processed1",result_right);
    //imshow("original1",src_right);

    warpPerspective(result_right, roi, transform_right, Size(width, height));

    Mat RightROI;   
    roi.copyTo(RightROI);

    Mat black_right = Mat::zeros(Size(2*center_x-460, height), CV_8UC3);
    black_right.copyTo(RightROI(Rect(0, 0, black_right.cols, black_right.rows)));

    //final_grid = final_grid+RightROI;

    //ipm center
    Point2f p_center[4] = {Point(502, 451), Point(821,457), Point(935, 520), Point(400, 508)};
    Point2f q_center[4] = {Point(center_x+(-300/2), center_y-(1000/2)), Point(center_x+(300/2), center_y-(1000/2)), Point(center_x+(300/2), center_y-(500/2)), Point(center_x+(-300/2), center_y-(500/2))};

    Mat transform_center (3, 3, CV_32FC1);
    transform_center = getPerspectiveTransform(p_center, q_center);

    //crop image below horizon
    Mat result_center = Mat::zeros(Size(image_width, image_height), CV_8UC3);
    src_center(Rect(0, 340, image_width, image_height-340)).copyTo(result_center(Rect(0, 340, image_width, image_height-340)));
    //imshow("processed1",result_left);
    //imshow("original1",src_left);


    warpPerspective(result_center, roi, transform_center, Size(width, height));

    Mat CenterROI;
    roi.copyTo(CenterROI);  
    Mat black_center_left = Mat::zeros(Size(460, height), CV_8UC3);
    black_center_left.copyTo(CenterROI(Rect(0, 0, black_center_left.cols, black_center_left.rows)));
    Mat black_center_right = Mat::zeros(Size(width-(2*center_x-460), height), CV_8UC3);
    black_center_right.copyTo(CenterROI(Rect(2*center_x-460, 0, black_center_right.cols, black_center_right.rows)));

    // int num_points = 6;
    // int num_polygons = 1;
    // int line_type = 8;

    // cv::Point corners[1][4];
    // corners[0][0] = Point( 460,0 );
    // corners[0][1] = Point( 1020,0 );
    // corners[0][2] = Point( 1020,591 );
    // corners[0][3] = Point( 892,715 );
    // corners[0][4] = Point( 592,715 );
    // corners[0][5] = Point( 460,573 );
    // const Point* corner_list[1] = { corners[0] };

    // cv::Mat mask(height,width,CV_8UC3, cv::Scalar(0,0,0));
    // cv::fillPoly( mask, corner_list, &num_points, num_polygons, cv::Scalar( 255, 255, 255 ),  line_type);
    
    // cv::Mat result;
    // cv::bitwise_and(CenterROI, mask, result);

    final_grid =final_grid+ CenterROI;
    
    //ipm completed
    //final_grid=final_grid(Rect(0, 0, width, center_x));
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
    pub_fused.publish(msg);
    //imshow("final_grid",final_grid);
    //waitKey(1);
    final_grid.release();
    roi.release();
//src_left.release();
//src_center.release();
//src_right.release();
}

 
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Lane_D");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_left = it.subscribe("/camera1/image_raw", 1, leftimage);
    image_transport::Subscriber sub_center = it.subscribe("/camera2/image_raw", 1, centerimage);
    image_transport::Subscriber sub_right = it.subscribe("/camera3/image_raw", 1, rightimage);
    pub_fused = it.advertise("/camera/fused_ipm", 1);
    pub_1 = it.advertise("/camera1/fisheye", 1);
    pub_2 = it.advertise("/camera2/fisheye", 1);
    pub_3 = it.advertise("/camera3/fisheye", 1);
    ros::Rate loop_rate(20);


//left cam fisheye parameters
    std::ifstream matleft("/home/sine/final_lane/src/vatsal/src/intrinsic_matrix_left.txt",std::ios::in);
    std::ifstream distleft("/home/sine/final_lane/src/vatsal/src/distCoeff_left.txt",std::ios::in);
    
    std::vector<double> a;
    std::vector<double> distCoeff_left;
    double num1 = 0.0;
    while (matleft >> num1) {
        a.push_back(num1);
    }
    double num2 = 0.0;
    while (distleft >> num2) {
        distCoeff_left.push_back(num2);
    }

    matleft.close();
    distleft.close();
    Mat intrinsic_matrix_left = (Mat_<double>(3,3) << a[0], a[1], a[2], a[3],a[4], a[5], a[6], a[7], a[8]);
    Mat R=Mat::eye(3, 3, CV_8U);
    initUndistortRectifyMap(intrinsic_matrix_left, distCoeff_left,intrinsic_matrix_left, R, Size (1280,720), CV_32FC1, map1_left, map2_left);
    a.clear();
    distCoeff_left.clear();
    intrinsic_matrix_left.release();


//center cam fisheye parameters
    std::ifstream matcenter("/home/sine/final_lane/src/vatsal/src/intrinsic_matrix_center.txt",std::ios::in);
    std::ifstream distcenter("/home/sine/final_lane/src/vatsal/src/distCoeff_center.txt",std::ios::in);
    
    std::vector<double> distCoeff_center;
    num1 = 0.0;
    while (matcenter >> num1) {
            a.push_back(num1);
    }
    num2 = 0.0;
    while (distcenter >> num2) {
        distCoeff_center.push_back(num2);
    }

    matcenter.close();
    distcenter.close();
    Mat intrinsic_matrix_center = (Mat_<double>(3,3) << a[0], a[1], a[2], a[3],a[4], a[5], a[6], a[7], a[8]);
    R=Mat::eye(3, 3, CV_8U);
    initUndistortRectifyMap(intrinsic_matrix_center, distCoeff_center, R, intrinsic_matrix_center, Size (1280,720), CV_32FC1, map1_center, map2_center);
    a.clear();
    distCoeff_center.clear();
    intrinsic_matrix_center.release();


//right cam fisheye parameters
    std::ifstream matright("/home/sine/final_lane/src/vatsal/src/intrinsic_matrix_right.txt",std::ios::in);
    std::ifstream distright("/home/sine/final_lane/src/vatsal/src/distCoeff_right.txt",std::ios::in);
    
    std::vector<double> distCoeff_right;
    num1 = 0.0;
    while (matright >> num1) {
        a.push_back(num1);
    }
    num2 = 0.0;
    while (distright >> num2) {
        distCoeff_right.push_back(num2);
    }

    matright.close();
    distright.close();
    Mat intrinsic_matrix_right = (Mat_<double>(3,3) << a[0], a[1], a[2], a[3],a[4], a[5], a[6], a[7], a[8]);
    R=Mat::eye(3, 3, CV_8U);
    initUndistortRectifyMap(intrinsic_matrix_right, distCoeff_right, R, intrinsic_matrix_right, Size (1280,720), CV_32FC1, map1_right, map2_right);
    a.clear();
    distCoeff_right.clear();
    intrinsic_matrix_right.release();



    while(ros::ok())
    {   
    	

        //final_grid = Mat::zeros(Size(width, height), CV_8UC3);
        ros::spinOnce();
        fuse();
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
        // pub_fused.publish(msg);
        //imshow("",src_right);
        //waitKey(1);
        loop_rate.sleep();
        
    }

    ROS_INFO("videofeed::occupancygrid.cpp::No error.");

}

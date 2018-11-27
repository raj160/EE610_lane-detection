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

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

using namespace cv;
using namespace std;

ros::Publisher pub;

//Use method of ImageTransport to create image publisher

void position(const std_msgs::Int64MultiArray::ConstPtr& input_array)
{
    std::vector<cv::Point2f> Arr;
    std::vector<cv::Point2f> transformedPoints;
    // print all the remaining numbers
    for(std::vector<long int>::const_iterator it = input_array->data.begin(); it != input_array->data.end(); ++it)
    {
	    if(it != input_array->data.begin()){
        	Arr.push_back(cv::Point2f((*it)/10000,(*it)%10000));
	    }
    }
    
    int center_x = 750,center_y = 1000;
/*
	//ipm left
    int h=150;
    int v=250;
    Point2f p_left[4] = {Point(26, 428), Point(551, 393),Point(836, 478) ,Point(15,622)};
    Point2f q_left[4] = {Point(h+center_x+(-1000/2), v+center_y-(-300/2)), Point(h+center_x+(-1000/2), v+center_y-(300/2)), Point(h+center_x+(-500/2),v+center_y-(300/2)), Point(h+center_x+(-500/2),v+center_y-(-300/2))};
    
    Mat transform_left (3, 3, CV_32FC1);
    transform_left = getPerspectiveTransform(p_left, q_left);

    warpPerspective(result_left, roi, transform_left, Size(width, height));

    cv::Point2f pc(center_x,center_y);
    cv::Mat r = cv::getRotationMatrix2D(pc, -9, 1.0);
    cv::warpAffine(LeftROI, LeftROI, r, LeftROI.size());
        
    
    //ipm right
    h=-50;
    v=150;
    Point2f p_right[4] = {Point(720, 387), Point(1186, 417), Point(1216, 589), Point(482, 456)};
    Point2f q_right[4] = {Point(h+center_x+(1000/2), v+center_y-(300/2)), Point(h+center_x+(1000/2), v+center_y-(-300/2)), Point(h+center_x+(500/2), v+center_y-(-300/2)), Point(h+center_x+(500/2), v+center_y-(300/2))};

    Mat transform_right (3, 3, CV_32FC1);
    transform_right = getPerspectiveTransform(p_right, q_right);

    warpPerspective(result_right, roi, transform_right, Size(width, height));
//*/
    //ipm center
    Point2f p_center[4] = {Point(502, 451), Point(821,457), Point(935, 520), Point(400, 508)};
    Point2f q_center[4] = {Point(center_x+(-300/2), center_y-(1000/2)), Point(center_x+(300/2), center_y-(1000/2)), Point(center_x+(300/2), center_y-(500/2)), Point(center_x+(-300/2), center_y-(500/2))};

    Mat transform_center (3, 3, CV_32FC1);
    transform_center = getPerspectiveTransform(p_center, q_center);

    if(Arr.size()>0)
        perspectiveTransform(Arr, transformedPoints, transform_center);
    //warpPerspective(result_center, roi, transform_center, Size(width, height));

    std_msgs::Float64MultiArray output_array;
    output_array.data.clear();
    //for loop, pushing data in the size of the array
    for (size_t i=0;i<transformedPoints.size();i++){
        output_array.data.push_back((transformedPoints[i].x-center_x)*2/100.0);
        output_array.data.push_back((center_y-transformedPoints[i].y)*2/100.0);
    }
    //output_array.data=transformedPoints;
    //Publish array
    pub.publish(output_array);
} 

 
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "vehicle_ipm");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cam_scan", 100, position);
    
    pub = n.advertise<std_msgs::Float64MultiArray>("/vehicle", 100);
    ros::Rate loop_rate(20);



    while(ros::ok())
    {   
    	

        //final_grid = Mat::zeros(Size(width, height), CV_8UC3);
        ros::spinOnce();
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
        // pub_fused.publish(msg);
        //imshow("",src_right);
        //waitKey(1);
        loop_rate.sleep();
        
    }

    ROS_INFO("videofeed::occupancygrid.cpp::No error.");

}

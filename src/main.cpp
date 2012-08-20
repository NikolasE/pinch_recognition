/*
 * main.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: engelhan
 */

#include "rgbd_utils/calibration_utils.h"
// #include "projector_calibration/projector_calibrator.h"

using namespace std;

#include "pinch_recognition/cloud_gmm.h"

#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace enc = sensor_msgs::image_encodings;



const int mean_max_cnt = 20;
int mean_cnt;

std::vector<Cloud> clouds;
Cloud reference, current, changed;
cv::Mat user_mask;
cv::Mat col_img;
ros::Publisher pub_reference, pub_changed;

vector<cv::Point> points;

Background_substraction subtractor;



void showMask(const std::vector<cv::Point2i>& mask, cv::Mat& img){
 img.setTo(0);

 for (uint i=0; i<mask.size(); ++i){
  img.at<uchar>(mask[i].y,mask[i].x) = 255;
 }

}

void applyPointMask(cv::Mat& img,const std::vector<cv::Point2i>& mask){

 img.setTo(0);

 for (uint i=0; i<mask.size(); ++i){
  img.at<cv::Vec3b>(mask[i].y,mask[i].x) = cv::Vec3b(255,255,255);
 }
}



bool detectGrasp(cv::Mat& masked, std::vector<cv::Point2f>& res, cv::Mat* col){


 res.clear();
 //cv::dilate(masked, masked, cv::Mat());

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( masked, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }



 cv::Mat drawing = cv::Mat::zeros( masked.size(), CV_8UC3 );
 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];

  // ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 150;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  if (has_parent)
    ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;


  // check if object crosses boundary ob
  if (!has_parent){



  }


  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );



   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );
    // ROS_INFO("child with area: %f", area);




    // hole in hand

    cv::Moments mom = cv::moments(contours[i]);


    // mark center
    res.push_back(cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00));
   }

  }
  cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );



  }

// cv::namedWindow("contours");
// cv::imshow("contours", drawing);
// cv::waitKey(10);

 return res.size()> 0;

}







/*
 * Find minimum z-value of points and mark all that are not further than x cm from this depth
 *
 */
void z_filter(const std::vector<cv::Point2i>& pts, const Cloud& cloud, float max_dist, std::vector<cv::Point2i>& result){

 result.clear();

 float z_min = 1e6;
 for (uint i=0; i<pts.size(); ++i){
  pcl_Point p = cloud.at(pts[i].x, pts[i].y); if (p.x!=p.x) continue;
  z_min = min(z_min, p.z);
 }

 for (uint i=0; i<pts.size(); ++i){
  pcl_Point p = cloud.at(pts[i].x, pts[i].y); if (p.x!=p.x) continue;

  if (p.z == p.z && p.z < z_min + max_dist)
   result.push_back(pts[i]);
 }

}



void imgCloudCB(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){


 pcl::fromROSMsg(*cloud_ptr, current);


 // ROS_INFO("%i", mean_cnt);
 mean_cnt++;

 if (mean_cnt < mean_max_cnt){
  ROS_INFO("%i", mean_cnt);
  clouds.push_back(current);

  subtractor.addTrainingFrame(current);

  return;
 }


 if (mean_cnt == mean_max_cnt){

  subtractor.computeBackground(0.007);

  reference = computeMean(clouds);

  Cloud::Ptr msg = reference.makeShared();
  msg->header.frame_id =  "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub_reference.publish(msg);

 }

 cv_bridge::CvImagePtr cv_ptr;


 cv_ptr = cv_bridge::toCvCopy(img_ptr , sensor_msgs::image_encodings::BGR8);

 col_img = cv_ptr->image;


// col_img = subtractor.applyMask(col_img);
// cv::namedWindow("gmm");
// cv::imshow("gmm", col_img);
// cv::waitKey(100);



 std::vector<cv::Point2i> mask, z_filtered;
 // changed = removeMean(reference, current,0.02, &mask);

 changed = subtractor.removeBackground(current,0.02, &mask);

 z_filter(mask, current, 10 /*0.10*/, z_filtered);

// ROS_INFO("z filter: from %zu to %zu", mask.size(), z_filtered.size());


 cv::Mat bw(col_img.rows, col_img.cols, CV_8UC1);

 showMask(z_filtered,bw);

 cv::erode(bw, bw, cv::Mat());
 cv::dilate(bw, bw, cv::Mat());


 cv::namedWindow("foo");
 cv::imshow("foo",bw);





 std::vector<cv::Point2f> grasps;

 if (user_mask.cols == bw.cols){
  cv::Mat masked;
  bw.copyTo(masked, user_mask);
  detectGrasp(masked, grasps, &col_img);
 }
 else
  detectGrasp(bw, grasps, &col_img);



 for (uint i=0; i<grasps.size(); ++i){
  cv::circle(col_img,grasps[i], 6, cv::Scalar(0,255,255),-1);
 }



 cv::imshow("col", col_img);
 cv::waitKey(10);


 Cloud::Ptr msg = changed.makeShared();
 msg->header.frame_id =  "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_changed.publish(msg);

 //ROS_INFO("Found %zu changed points", changed.size());

}

void onMouse(int event, int x, int y,int, void*)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        ROS_INFO("%i %i", x,y);
        points.push_back(cv::Point(x,y));
    }

    for (uint i=0; i<points.size(); ++i){
     cv::circle(col_img, points[i], 5,CV_RGB(0,255,0), 4);
    }

    if (points.size() == 4){
     user_mask = cv::Mat(col_img.rows,col_img.cols,CV_8UC1);
     user_mask.setTo(0);
     cv::fillConvexPoly(user_mask, points, cv::Scalar::all(255));

     cv::namedWindow("m");
     cv::imshow("m", user_mask );
     cv::waitKey(10);

     points.clear();
    }


    return;
}

int main(int argc, char ** argv){


 ros::init(argc, argv,"pinch_detection");

 ros::NodeHandle nh;

 pub_reference = nh.advertise<Cloud>("reference", 1);
 pub_changed = nh.advertise<Cloud>("changed", 1);

 mean_cnt = 0;

 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> policy;
 message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
 message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/rgb/points", 1);
 message_filters::Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
 sync.registerCallback(boost::bind(&imgCloudCB, _1, _2));

 cv::namedWindow("col");
 cv::setMouseCallback("col", onMouse, 0);


 ros::Rate loop_rate(10);
 while ( ros::ok() ) {
  ros::spinOnce();
  loop_rate.sleep();
 }
}























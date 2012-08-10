/*
 * main.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: engelhan
 */

#include "projector_calibration/calibration_utils.h"
#include "projector_calibration/projector_calibrator.h"

using namespace std;

#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;



const int mean_max_cnt = 10;
int mean_cnt;

std::vector<Cloud> clouds;
Cloud reference, current, changed;


ros::Publisher pub_reference, pub_changed;


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



cv::Point2f detectGrasp(cv::Mat& masked){

 cv::Point2f res;
 res.x = res.y = -1;



 cv::dilate(masked, masked, cv::Mat());

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( masked, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 cv::Mat drawing = cv::Mat::zeros( masked.size(), CV_8UC3 );
 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = cv::contourArea(contours[i]);

  ROS_INFO("%i %f", i, area);

  if (area < 200) continue;

  cv::Scalar color;

  if (area > 2000)
   color =  cv::Scalar( 255,0,0 );
  else{
   color =  cv::Scalar( 0,0,255 );

   // hole in hand

   cv::Moments mom = cv::moments(contours[i]);


   // mark center
   res.x =  mom.m10/mom.m00; res.y = mom.m01/mom.m00;



  }
  cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );


  }

 cv::namedWindow("contours");
 cv::imshow("contours", drawing);
 cv::waitKey(10);

 return res;

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
  clouds.push_back(current);
  return;
 }


 if (mean_cnt == mean_max_cnt){
  reference = computeMean(clouds);

  Cloud::Ptr msg = reference.makeShared();
  msg->header.frame_id =  "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub_reference.publish(msg);

 }

 cv_bridge::CvImagePtr cv_ptr;


 cv_ptr = cv_bridge::toCvCopy(img_ptr , sensor_msgs::image_encodings::BGR8);

 cv::Mat col_img = cv_ptr->image;





 std::vector<cv::Point2i> mask, z_filtered;
 changed = removeMean(reference, current,0.2, &mask);


 z_filter(mask, current, 0.05, z_filtered);

 ROS_INFO("z filter: from %zu to %zu", mask.size(), z_filtered.size());


 cv::Mat bw(col_img.rows, col_img.cols, CV_8UC1);

 showMask(z_filtered,bw);

 // applyPointMask(col_img, z_filtered);







 cv::Point2f pos = detectGrasp(bw);

 if (pos.x > 0){
  cv::circle(col_img,pos, 4, cv::Scalar(255,0,0),-1);

 }


 cv::namedWindow("col");
 cv::imshow("col", col_img);
 cv::waitKey(10);


 Cloud::Ptr msg = changed.makeShared();
 msg->header.frame_id =  "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_changed.publish(msg);

 //ROS_INFO("Found %zu changed points", changed.size());

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


 ros::Rate loop_rate(10);
 while ( ros::ok() ) {
  ros::spinOnce();
  loop_rate.sleep();
 }
}























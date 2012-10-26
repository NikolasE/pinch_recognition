/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "pinch_recognition/pinch_detection.h"

using namespace std;







void detectGrasp(cv::Mat& foreground, std::vector<cv::Point2f>& grasps, cv::Mat* col, bool verbose){

 if (foreground.cols == 0){
  ROS_ERROR("no foreground detected!");
  return;
 }

 grasps.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

// ROS_INFO("found %zu contours", contours.size());

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;
 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];
//  ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );



    cv::Moments mom = cv::moments(contours[i]);

    cv::Point2f center = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);
    grasps.push_back(center);

    if (col)
     cv::circle(*col, center, 30, CV_RGB(255,0,0),-1);

   }

  }

  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }


}





void Pinch_detector::applyMaskToForeground(const cv::Mat& mask){

 if(!(mask.cols == foreground.cols)){
  ROS_INFO("Mask: %i, foreground: %i",mask.cols,foreground.cols );
  assert(1==0);
 }
 cv::Mat masked;
 foreground.copyTo(masked, mask);
 foreground = masked;
}



void Grasp_detector::detectGrasps(std::vector<cv::Point2f>& res, float max_dist, cv::Mat* current, cv::Mat* col, bool verbose){


 res.clear();


 cv::Mat foreground = getForeground(max_dist, *current);

 // cv::namedWindow("foreorund");
 // cv::imshow("foreorund", foreground);
 // cv::waitKey(10);


 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours(foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;

 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];

  // ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );

    cv::Moments mom = cv::moments(contours[i]);
    res.push_back(cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00));
   }

  }
  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }

}



bool Pinch_detector::detectGrasp(std::vector<cv::Point2f>& res, cv::Mat* col, bool verbose){

 if (foreground.cols == 0){
  ROS_ERROR("no foreground detected!");
  return false;
 }
 res.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 cv::findContours( foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

 // compute area for each contour:
 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);
 }


 cv::Mat drawing;

 if (verbose) drawing = cv::Mat::zeros( foreground.size(), CV_8UC3 );

 for( uint i = 0; i< contours.size(); i++ )
  {
  float area = areas[i];

  // ROS_INFO("%i %f", i, area);

  float large_area_threshold = 1500;
  float small_area_threshold = 100;

  bool has_parent = hierarchy[i][3] >= 0;
  bool is_small = area < small_area_threshold;
  bool is_large = area > large_area_threshold;
  bool parent_is_large = has_parent && areas[hierarchy[i][3]] > large_area_threshold;

  //  if (has_parent)
  //   ROS_INFO("Grasp: large: %f, small: %f", areas[hierarchy[i][3]], area);

  if (has_parent && is_small) continue;
  if (!has_parent && !is_large) continue;

  cv::Scalar color;

  if (area > large_area_threshold){
   color =  cv::Scalar( 255,0,0 );
  }
  else{
   color =  cv::Scalar( 0,0,255 );

   if (has_parent && !is_small && parent_is_large){
    color =  cv::Scalar( 0,255,0 );

    cv::Moments mom = cv::moments(contours[i]);
    res.push_back(cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00));
   }

  }
  if (verbose)
   cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

 if (verbose){
  cv::namedWindow("contours");
  cv::imshow("contours", drawing);
  cv::waitKey(10);
 }

 return res.size()> 0;

}




Cloud Pinch_detector::removeBackground(const Cloud& current,  float min_dist, float max_dist){

 // foreground is now a member variable
 Cloud result = Background_substraction::removeBackground(current, min_dist, max_dist, foreground);




 return result;

}

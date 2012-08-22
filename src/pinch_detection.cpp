/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "pinch_recognition/pinch_detection.h"

using namespace std;

//uint Pinch_detector::addTrainingFrame(const Cloud& c){
// subtractor.addTrainingFrame(c);
//}


void Pinch_detector::applyMaskToForeground(const cv::Mat& mask){
 assert(mask.cols == foreground.cols);
 cv::Mat masked;
 foreground.copyTo(masked, mask);
 foreground = masked;
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

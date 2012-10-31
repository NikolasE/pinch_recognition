/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "pinch_recognition/pinch_detection.h"

using namespace std;


int Graps_Debouncer::next_id = 0;

void Graps_Debouncer::updateGrasps(const std::vector<cv::Point2f>& detections){

 // no special handling for very close detections (one grasp could get several detections)

 std::vector<Grasp> new_grasps;

 for (uint i=0; i < detections.size(); ++i){

  cv::Point2f detection = detections[i];

  float min_dist = max_dist_px;
  int best_match = -1;
  //  for (uint j=0; j< grasps.size(); ++j){
  for (Grasp_it it = grasps.begin(); it!=grasps.end(); ++it){

   float dist = it->second.dist_to(detection);

   //   ROS_INFO("detection %i vs grasp %i: %f px", i,j,dist);

   if (dist < min_dist){
    min_dist = dist;
    best_match = it->first;
   }
  }

  if (best_match > -1){
   Grasp_it best = grasps.find(best_match);
   best->second.update(detection);
//   grasps[best_match].update(detection);
  }else{
   // start new grasp
   Grasp new_grasp(detection);
   new_grasp.id = next_id++;
   new_grasps.push_back(new_grasp);
  }

 }


 // publish confirmed grasps:

 Grasp_it it = grasps.begin();

 while( it != grasps.end()){

  Grasp *g = &it->second;


  ROS_INFO("grasp %i: state %i", g->id,g->state);


  if (g->state == Grasp_Confirmed){
//   ROS_INFO("grasp %i: now active", g->id);

   g->state = Grasp_Active;
  }

  /// grasp was confirmed and is now an official grasp
  if (g->detections.size() == 3 && g->state == Grasp_Initialized){
   //   ROS_INFO("New grasp at %f %f", g->last_detection.x ,g->last_detection.y);
   g->state = Grasp_Confirmed;
//   ROS_INFO("grasp %i: now confirmed", g->id);

  }



  /// remove grasp if it was finished in the last update
  // http://stackoverflow.com/questions/263945/what-happens-if-you-call-erase-on-a-map-element-while-iterating-from-begin-to
  if (g->state == Grasp_Finished){
   ROS_INFO("removing grasp with id %i", g->id);
   grasps.erase(it++);
//   ROS_INFO("Finished grasp: pos: %i, id: %i", i,g->id);
  }else{
   ++it;
  }

 }

 // published ended grasps
 ros::Time now = ros::Time::now();
 //for (uint i=0; i<grasps.size(); ++i){
 for (Grasp_it it = grasps.begin(); it!=grasps.end(); ++it){

  Grasp *g = &it->second;

  ros::Duration time_since_last_update = (now-g->last_seen);

  if (time_since_last_update > ros::Duration(0.2)){
   //   ROS_INFO("Closed grasp at %f %f", grasps[i].last_detection.x ,grasps[i].last_detection.y);
   g->state = Grasp_Finished;
  }

 }

 // append new grasps:
 for (uint i=0; i<new_grasps.size(); ++i){
  assert(new_grasps[i].state == Grasp_Initialized);
  grasps[new_grasps[i].id] = new_grasps[i];
 }


}


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

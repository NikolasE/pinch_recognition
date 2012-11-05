/*
 * pinch_detection.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#include "pinch_recognition/pinch_detection.h"

using namespace std;


/*
template <class T>
int Object_tracker<T>::next_id = 0;
 */

/*
template <class T>
void Object_tracker<T>::update_tracks(const std::vector<cv::Point2f>& detections){

 // no special handling for very close detections (one grasp could get several detections)

 std::vector<T> new_tracks;

 for (uint i=0; i < detections.size(); ++i){

  cv::Point2f detection = detections[i];

  float min_dist = max_dist_px;
  int best_match = -1;
  //  for (uint j=0; j< grasps.size(); ++j){
  for (obj_it it = tracks.begin(); it!=tracks.end(); ++it){

   float dist = it->second.dist_to(detection);

   //   ROS_INFO("detection %i vs grasp %i: %f px", i,j,dist);

   if (dist < min_dist){
    min_dist = dist;
    best_match = it->first;
   }
  }

  if (best_match > -1){
   obj_it best = tracks.find(best_match);
   best->second.update(detection);
//   grasps[best_match].update(detection);
  }else{
   // start new grasp
   T new_track(detection);
   new_track.id = next_id++;
   new_tracks.push_back(new_track);
  }

 }


 // publish confirmed grasps:

 obj_it it = tracks.begin();

 while( it != tracks.end()){

  T *g = &it->second;


  ROS_INFO("grasp %i: state %i", g->id,g->state);


  if (g->state == Track_Confirmed){
//   ROS_INFO("track %i: now active", g->id);
   g->state = Track_Active;
  }

  /// grasp was confirmed and is now an official grasp
  if (g->detections.size() == 3 && g->state == Track_Initialized){
   //   ROS_INFO("New track at %f %f", g->last_detection.x ,g->last_detection.y);
   g->state = Track_Confirmed;
//   ROS_INFO("track %i: now confirmed", g->id);
  }

  /// remove grasp if it was finished in the last update
  // http://stackoverflow.com/questions/263945/what-happens-if-you-call-erase-on-a-map-element-while-iterating-from-begin-to
  if (g->state == Track_Finished){
   ROS_INFO("removing grasp with id %i", g->id);
   tracks.erase(it++);
//   ROS_INFO("Finished grasp: pos: %i, id: %i", i,g->id);
  }else{
   ++it;
  }

 }

 // published ended grasps
 ros::Time now = ros::Time::now();
 //for (uint i=0; i<grasps.size(); ++i){
 for (obj_it it = tracks.begin(); it!=tracks.end(); ++it){

  T *g = &it->second;

  ros::Duration time_since_last_update = (now-g->last_seen);

  if (time_since_last_update > ros::Duration(0.2)){
   g->state = Track_Finished;
  }

 }

 // append new grasps:
 for (uint i=0; i<new_tracks.size(); ++i){
  assert(new_tracks[i].state == Track_Initialized);
  tracks[new_tracks[i].id] = new_tracks[i];
 }


}
 */


void detectPlayingPieces(cv::Mat& foreground, const cv::Mat& areaMask, const Cloud& scene, std::vector<Playing_Piece>& objects, bool& hand_visible, bool& border_crossing,  cv::Mat* col){

 assert(foreground.cols == int(scene.width));
 objects.clear();

 vector<vector<cv::Point> > contours;
 vector<cv::Vec4i> hierarchy;

 ros::Time start = ros::Time::now();
 cv::findContours(foreground, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
 ROS_INFO("findContours: %f ms", (ros::Time::now()-start).toSec()*1000.0);

 // compute area for each contour:

 cv::Mat edge;
 // get boundary of modeled are within the image:
 cv::Canny(areaMask, edge, 1,10);

// cv::imwrite("canny.png", edge);

 cv::dilate(edge, edge, cv::Mat(), cv::Point(-1,-1),1);

// cv::imwrite("dilate.png", edge);


 hand_visible = false;

 float areas[contours.size()];
 for( uint i = 0; i< contours.size(); i++ ){
  areas[i] = cv::contourArea(contours[i]);

  if (areas[i] > C_MIN_HAND_AREA)
   hand_visible = true;

 }

 cv::Scalar color =  cv::Scalar( 255,0,0 );


 cv::Mat contour_image(foreground.size(), CV_8UC1);
 contour_image.setTo(0);

 for( uint i = 0; i< contours.size(); i++ ){

  if (areas[i] < C_MIN_OBJECT_AREA)
   continue;

  // ROS_INFO("contour %i has size %f",i,areas[i]);


  cv::Moments mom = cv::moments(contours[i]);

  cv::Point2f center = cv::Point2f(mom.m10/mom.m00, mom.m01/mom.m00);

  Playing_Piece pp(center);
  pp.area = areas[i];
  objects.push_back(pp);

  cv::drawContours( contour_image, contours, i, color, -1, 8, hierarchy, 0, cv::Point() );

  if (col)
   cv::drawContours( *col, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
 }

// cv::imwrite("contours.png", contour_image);

 cv::Mat intersection; intersection.setTo(0);
 contour_image.copyTo(intersection, edge);

 border_crossing = cv::countNonZero(intersection)>0;

 if (border_crossing){
  ROS_INFO("FOUND INTERSECTION WITH BORDER");
 }

}


void detectGrasp(cv::Mat& foreground, std::vector<Grasp>& grasps, cv::Mat* col, bool verbose){

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
    Grasp grasp(center);
    grasps.push_back(grasp);

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

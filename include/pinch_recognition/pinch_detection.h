/*
 * pinch_detection.h
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#ifndef PINCH_DETECTION_H_
#define PINCH_DETECTION_H_


#include "pinch_recognition/cloud_gmm.h"


#define C_MIN_HAND_AREA 1000
#define C_MIN_OBJECT_AREA 20

// Grasp_Confirmed and Grasp_Finished are intermedite states
enum Tracking_State {Track_Initialized  = 0, Track_Confirmed, Track_Active, Track_Finished};



struct Tracked_Object {
 ros::Time last_seen;

 std::vector<cv::Point2f> detections;

 cv::Point2f last_detection;

 int id;
 Tracking_State state;


 Tracked_Object(){}

 Tracked_Object(const cv::Point2f& detection){
  update(detection);
  state = Track_Initialized;
 }

 float dist_to(const cv::Point2f& detection){
  return sqrt(pow(detection.x-last_detection.x,2)+pow(detection.y-last_detection.y,2));
 }

 void update(const cv::Point2f& detection){
  detections.push_back(detection);
  last_detection = detection;
  last_seen = ros::Time::now();
 }

};

struct Playing_Piece : public Tracked_Object {

 Playing_Piece(const cv::Point2f detection): Tracked_Object(detection){}
 Playing_Piece(){}

 float area;
 float volume;
 cv::Mat image;

};

struct Grasp : public Tracked_Object {

 Grasp(const cv::Point2f detection): Tracked_Object(detection){}
 Grasp(){}

};



void detectGrasp(cv::Mat& foreground, std::vector<cv::Point2f>& res, cv::Mat* col = NULL, bool verbose = false);
void detectPlayingPieces(cv::Mat& foreground,const cv::Mat& areaMask, const Cloud& scene, std::vector<Playing_Piece>& objects, bool& hand_visible, bool &border_crossing,  cv::Mat* col);


typedef std::map<int,Grasp>::iterator Grasp_it;
typedef std::map<int,Playing_Piece>::iterator Piece_it;

typedef std::map<int,Tracked_Object>::iterator Track_it;


template <class T>
class Object_tracker {

 typedef typename std::map<int,T>::iterator obj_it;

private:
 float max_dist_px;
 ros::Duration time_threshold;
 uint detections_before_confirmation;

 static int next_id;

public:

// void update_tracks_best_paris(const std::vector<T>& detections, float max_dist = -1 ){
//
// }

 void update_tracks(const std::vector<T>& detections, float max_dist = -1 ){
  // no special handling for very close detections (one grasp could get several detections)


  if (max_dist <= 0)
   max_dist = max_dist_px;

  ROS_INFO("Updating with max_dist = %f", max_dist);


  std::vector<T> new_tracks;


  bool detection_used[detections.size()];
  for (uint i=0; i < detections.size(); ++i){detection_used[i] = false;}


  /// every detection can be used to update several tracks, no hungarian method or similar is used!

  // for every track find closest detection
  for (obj_it it = tracks.begin(); it!=tracks.end(); ++it){
   float min_dist = max_dist;
   int best_match = -1;

   for (uint i=0; i < detections.size(); ++i){

    cv::Point2f detection = detections[i].last_detection;

    float dist = it->second.dist_to(detection);

    ROS_INFO("detection %i vs grasp %i: %f px", i,it->first,dist);

    if (dist < min_dist){
     min_dist = dist;
     best_match = i;
    }
   }

   // track has found a close detection
   if (best_match > -1){
    it->second.update(detections[best_match].last_detection);
    detection_used[best_match] = true;
   }

   //    //    obj_it best = tracks.find(best_match);
   //    //    best->second.update(detection);
   //    //    ROS_INFO("track %i is updated with detection %i", best->first, )
   //   }else{
   //    // start new grasp
   //    T new_track(detection);
   //    new_track.id = next_id++;
   //    new_tracks.push_back(new_track);
   //   }

  }


  // publish confirmed grasps:

  obj_it it = tracks.begin();

  ros::Time now = ros::Time::now();

  while( it != tracks.end()){

   T *g = &it->second;
   //    ROS_INFO("track %i: state %i", g->id,g->state);

   if (g->state == Track_Confirmed){
    //   ROS_INFO("track %i: now active", g->id);
    g->state = Track_Active;
   }

   /// track was seen in several frames and is now an official track
   if (g->detections.size() == detections_before_confirmation && g->state == Track_Initialized){
    //   ROS_INFO("New track at %f %f", g->last_detection.x ,g->last_detection.y);
    g->state = Track_Confirmed;
    //   ROS_INFO("track %i: now confirmed", g->id);
   }


   /**
   * Object was not seen in the last frames and was not yet confirmed.
   * The track is immediately erased without setting it to Track_Finished first
   */
   bool track_too_old = (now-g->last_seen) > time_threshold;


   /// remove grasp if it was finished in the last update
   // http://stackoverflow.com/questions/263945/what-happens-if-you-call-erase-on-a-map-element-while-iterating-from-begin-to
   if (g->state == Track_Finished || (track_too_old && g->state == Track_Initialized)){
    tracks.erase(it++);
   }else{

    if (track_too_old)
     g->state = Track_Finished;

    ++it;
   }

  }

  // append new grasps:
  //  for (uint i=0; i<new_tracks.size(); ++i){
  //   assert(new_tracks[i].state == Track_Initialized);
  //   tracks[new_tracks[i].id] = new_tracks[i];
  //  }

  for (uint i=0; i<detections.size(); ++i){
   if (detection_used[i])
    continue;

   T new_track(detections[i].last_detection);
   new_track.id = next_id++;
   new_track.state = Track_Initialized;
   tracks[new_track.id] = new_track;
  }

 }

 /// list of all active tracks
 std::map<int,T> tracks;

 Object_tracker(){
  max_dist_px = 30;
  time_threshold = ros::Duration(0.2);
  detections_before_confirmation = 5;
 }

};

template <class T>
int Object_tracker<T>::next_id = 0;

typedef Object_tracker<Grasp> Grasp_Tracker;
typedef Object_tracker<Playing_Piece> Piece_Tracker;



class Grasp_detector : public PixelGaussian {


public:
 void detectGrasps(std::vector<cv::Point2f>& res, float max_dist, cv::Mat* current, cv::Mat* col = NULL, bool verbose = false);

 // Grasp_detector(){}

};



class Pinch_detector : public Background_substraction {


private:
 cv::Mat foreground;
 bool initiated;

public:

 bool isInitiated(){return initiated;}


 void reset(){
  Background_substraction::reset();
  initiated = false;
 }

 bool computeBackground(float max_std_dev = 0.005){
  bool res = Background_substraction::computeBackground(max_std_dev);
  if (res) initiated = true;

  return res;
 }


 Cloud removeBackground(const Cloud& current, float min_dist, float max_dist);
 bool detectGrasp(std::vector<cv::Point2f>& res, cv::Mat* col = NULL, bool verbose = false);

 void applyMaskToForeground(const cv::Mat& mask);

 cv::Mat* getForeground(){return &foreground;}

 Pinch_detector(){
  initiated = false;
 }

};



#endif /* PINCH_DETECTION_H_ */

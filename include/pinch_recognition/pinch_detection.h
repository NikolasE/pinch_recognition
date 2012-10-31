/*
 * pinch_detection.h
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#ifndef PINCH_DETECTION_H_
#define PINCH_DETECTION_H_


#include "pinch_recognition/cloud_gmm.h"


void detectGrasp(cv::Mat& foreground, std::vector<cv::Point2f>& res, cv::Mat* col = NULL, bool verbose = false);


// Grasp_Confirmed and Grasp_Finished are intermedite states
enum Grasp_State {Grasp_Initialized  = 0, Grasp_Confirmed, Grasp_Active, Grasp_Finished};

struct Grasp {
 ros::Time last_seen;



 std::vector<cv::Point2f> detections;

 cv::Point2f last_detection;

 int id;
 Grasp_State state;


 Grasp(){}

 Grasp(const cv::Point2f& detection){
  update(detection);
  state = Grasp_Initialized;
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

typedef std::map<int,Grasp>::iterator Grasp_it;


class Graps_Debouncer {

private:
 float max_dist_px;
 ros::Duration time_threshold;

 static int next_id;

public:
 void updateGrasps(const std::vector<cv::Point2f>& detections);

 /// list of all active grasps
 //std::vector<Grasp> grasps;
 std::map<int,Grasp> grasps;

 Graps_Debouncer(){
  max_dist_px = 30;
  time_threshold = ros::Duration(0.2);
 }

};




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

/*
 * pinch_detection.h
 *
 *  Created on: Aug 21, 2012
 *      Author: lengelhan
 */

#ifndef PINCH_DETECTION_H_
#define PINCH_DETECTION_H_


#include "pinch_recognition/cloud_gmm.h"

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

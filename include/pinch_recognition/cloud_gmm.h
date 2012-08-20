/*
 * cloud_gmm.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: lengelhan
 */

#ifndef CLOUD_GMM_CPP_
#define CLOUD_GMM_CPP_


#include "rgbd_utils/calibration_utils.h"


#include <sensor_msgs/image_encodings.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


struct Background_substraction {

 Background_substraction(){
//  training_frames_cnt = 0;
 }

 uint addTrainingFrame(const Cloud& c);

 /*
  * TODO: don't use fixed threshold but compute p(x| mu, std_dev)
  * for each pixel and threshold this probability!
  */

 bool computeBackground(float max_std_dev = 0.005);

 Cloud removeBackground(const Cloud& current, float max_dist, std::vector<cv::Point2i>* valids);

 void reset(){
  dists.clear();
  reset_helper_images();
 }


 void showMask(){
  cv::namedWindow("BG: mask");
  cv::imshow("BG: mask", mask);
  cv::waitKey(10);
 }

 Cloud applyMask(Cloud& current);
 cv::Mat applyMask(cv::Mat& img);



private:

 cv::Mat mask;

 void reset_helper_images(){
  means.setTo(-1);
  std_dev.setTo(-100);
 }


 std::vector<cv::Mat> dists;
 cv::Mat means;
 cv::Mat std_dev;


};


#endif /* CLOUD_GMM_CPP_ */

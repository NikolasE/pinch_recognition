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



/**
 * Computation of mean values at each position in constant time.
 */
class PixelGaussian {

 /// mean value (CV_32FC1)
 cv::Mat mean;

 /// variance (CV_32FC1)
 cv::Mat var;

 /// number of training points (CV_32FC1)
 cv::Mat counter;

 /// sum of squared training values (CV_32FC1)
 cv::Mat sq;

 int training_frame_cnt;
 bool var_computed;
 void computeVariance();

public:


 /**
  * @param max_var maximum variance
  * @return   8UC1-image where white corresponds to pixels that passed the variance-test
  */
 /// computation of all pixels with training points and a variance smaller than max_var
 cv::Mat varianceThreshold(float max_var);

 /**
  *
  * @param max_dist  if distance to model is larger than max_dist, a point is considered foreground
  * @param current
  * @return 8UC1-image FG with FG(x,y) == 255 iff abs(current(x,y) - mean(x,y)) >= max_dist
  */
 /// get mask of pixels that are closer than max_dist to the cam as their corresponding pixel in the model
 cv::Mat getForeground(float max_dist, const cv::Mat& current);


 /**
  *
  * @param max_p
  * @return
  * @todo: implement me
  */
 /// if p(c|mu,var) > max_p, point is considered foreground
// cv::Mat getForegroundProb(float max_p);



 void reset(){
  training_frame_cnt = 0;
  var_computed = false;
 }

 PixelGaussian(){
  reset();
 }


 int updateModel(const cv::Mat& current);

 cv::Mat getCounter(){return counter;}
 cv::Mat getMean(){return mean;}
 cv::Mat getVariance();

};


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

 Cloud removeBackground(const Cloud& current,float min_dist, float max_dist, std::vector<cv::Point2i>* valids);
 Cloud removeBackground(const Cloud& current,float min_dist, float max_dist, cv::Mat& foreground);


 Cloud showBackground(const Cloud& cloud);

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
 void applyMask(cv::Mat& img);

 cv::Mat mask;

private:



 void reset_helper_images(){
  means.setTo(-1);
  std_dev.setTo(-100);
 }

 void showMask(const std::vector<cv::Point2i>& mask, cv::Mat& img);


 std::vector<cv::Mat> dists;
 cv::Mat means;
 cv::Mat std_dev;


};


#endif /* CLOUD_GMM_CPP_ */

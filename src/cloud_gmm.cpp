/*
 * cloud_gmm.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: lengelhan
 */


#include "pinch_recognition/cloud_gmm.h"

using namespace std;


cv::Mat PixelGaussian::getForeground(float max_dist, const cv::Mat& current){
 cv::Mat fg(mean.size(), CV_8UC1);
 fg.setTo(0);

 float m,c,v;

 assert(mean.size() == current.size());

 if (!var_computed)
  computeVariance();

 uchar fg_value = 255;

 for (int x=0; x<mean.cols; ++x)
  for (int y=0; y<mean.rows; ++y){

   m = mean.at<float>(y,x);
   c = current.at<float>(y,x);
   v = var.at<float>(y,x);

   // no measurement at this position
   if (c != c) continue;

   // if there was no valid measurement in this pixel, every valid measurement is foreground
//   if (v<0){
//    if (c==c)
//     fg.at<uchar>(y,x) = fg_value;
//    else
//     continue;
//   }


   if (abs(m-c) >= max_dist)
    fg.at<uchar>(y,x) = fg_value;
  }

 return fg;

}



cv::Mat PixelGaussian::varianceThreshold(float max_var){

 cv::Mat fg(var.size(), CV_8UC1);
 fg.setTo(0);


 float v;

 for (int x=0; x<var.cols; ++x)
  for (int y=0; y<var.rows; ++y){

   v = var.at<float>(y,x);

   if (v >= 0 && v <= max_var)
    fg.at<uchar>(y,x) = 255;
  }

 return fg;

}



int PixelGaussian::updateModel(const cv::Mat& current){

 assert(current.type() == CV_32FC1);

 if (training_frame_cnt == 0){

  mean = cv::Mat(current.size(), CV_32FC1);
  mean.setTo(0);

  var = cv::Mat(current.size(), CV_32FC1);
  var.setTo(0);

  sq = cv::Mat(current.size(), CV_32FC1);
  sq.setTo(0);

  counter = cv::Mat(current.size(), CV_32FC1);
  counter.setTo(0);
 }else{
//  ROS_INFO("current: %i %i, mean: %i %i", current.cols, current.rows, mean.cols, mean.rows);
  assert(current.size() == mean.size());
 }



 float mu_old, mu_new, sq_old,sq_new, val;
 float n_old;

 for (int x=0; x<current.cols; ++x)
  for (int y=0; y<current.rows; ++y){

   val = current.at<float>(y,x);

   if (val != val){
    continue;
   }

   mu_old = mean.at<float>(y,x);
   n_old = counter.at<float>(y,x);
   sq_old = sq.at<float>(y,x);

   mu_new = (n_old*mu_old+val)/(n_old+1);
   sq_new = sq_old + val*val;

   mean.at<float>(y,x) = mu_new;
   sq.at<float>(y,x)   = sq_new;

   counter.at<float>(y,x) = n_old + 1;
  }


 // cv::GaussianBlur(mean, mean, cv::Size(3,3),2,2);

 var_computed = false;

 return training_frame_cnt++;
}


cv::Mat PixelGaussian::getVariance(){

 if (!var_computed)
  computeVariance();

 return var;
}



void PixelGaussian::computeVariance(){


 float n, s, mu;

 for (int x=0; x<mean.cols; ++x)
  for (int y=0; y<mean.rows; ++y){

   n = counter.at<float>(y,x);

   if (n < 1){
    var.at<float>(y,x) = -1;
    continue;
   }

   if (n < 2){
    var.at<float>(y,x) = 0;
    continue;
   }

   s = sq.at<float>(y,x);
   mu = mean.at<float>(y,x);

   float new_var = s/n-mu*mu;
   var.at<float>(y,x) = new_var;
//   if (new_var < 0)
//    ROS_INFO("s: %f, n: %f, mu: %f, new_var: %f", s,n,mu, new_var);
//   assert(new_var >= 0);
  }

 var_computed = true;
}




void Background_substraction::showMask(const std::vector<cv::Point2i>& mask, cv::Mat& img){
 img.setTo(0);

 for (uint i=0; i<mask.size(); ++i){
  img.at<uchar>(mask[i].y,mask[i].x) = 255;
 }

}


uint Background_substraction::addTrainingFrame(const Cloud& c){

 if (dists.size() == 0){
  means = cv::Mat(c.height, c.width, CV_32FC1);
  std_dev = cv::Mat(c.height, c.width, CV_32FC1);
  reset_helper_images();
 }

 cv::Mat new_dist = cv::Mat(c.height, c.width, CV_32FC1);
 new_dist.setTo(0);

 for (uint x=0; x<c.width; ++x)
  for (uint y=0; y<c.height; ++y){
   pcl_Point p = c.at(x,y);

   // store distance to object if available, -1 else
   new_dist.at<float>(y,x) = (p.x==p.x)?norm(p):-1;
  }

 dists.push_back(new_dist);

 return dists.size();

}


void Background_substraction::applyMask(cv::Mat& img){


 if (mask.cols != img.cols){
  ROS_WARN("no background computed!");
 }
 cv::Mat result; result.setTo(0);
 mask.copyTo(result, img);
 mask = result;

}


// untested
Cloud Background_substraction::applyMask(Cloud& current){

 if (uint(mask.cols) != current.width){
  ROS_WARN("no background computed!");
  return Cloud();
 }

 Cloud result;
 result.reserve(current.size());

 for (uint x=0; x<current.width; ++x)
  for (uint y=0; y<current.height; ++y){
   if (mask.at<uchar>(y,x) == 255)
    result.push_back(current.at(x,y));
  }

 return result;

}


Cloud Background_substraction::showBackground(const Cloud& cloud){

 Cloud result; result = cloud;

 for (uint x=0; x<cloud.width; ++x)
  for (uint y=0; y<cloud.height; ++y){
   float m = means.at<float>(y,x);
   if (m > 0){
    pcl_Point c = cloud.at(x,y);
    if (c.x != c.x) continue;
    setLength(c,m);
    result.at(x,y) = c;
   }
  }

 return result;

}

Cloud Background_substraction::removeBackground(const Cloud& current, float min_dist, float max_dist, cv::Mat& foreground){
 std::vector<cv::Point2i> valids;

 Cloud result = removeBackground(current,min_dist,  max_dist, &valids);

 foreground = cv::Mat(current.height, current.width, CV_8UC1);
 showMask(valids,foreground);

 cv::erode(foreground, foreground, cv::Mat());
 cv::dilate(foreground, foreground, cv::Mat());

 return result;
}



Cloud Background_substraction::removeBackground(const Cloud& current, float min_dist, float max_dist, std::vector<cv::Point2i>* valids){

 Cloud result;



// assert(cloud.size() == reference.size());

// for (uint i=0; i<cloud.size(); ++i){


 ROS_INFO("min: %f, max: %f", min_dist,max_dist);

 if (min_dist == max_dist){
  ROS_WARN("removeBackground called with min = %f and max = %f", min_dist, max_dist);
  return result;
 }

 uint invalid = 0;

 for (uint x=0; x<current.width; ++x)
  for (uint y=0; y<current.height; ++y){

  pcl_Point c = current.at(x,y);


  if (c.x != c.x) { invalid++; continue;}

  if (mask.at<uchar>(y,x) == 0) continue;

  float d = norm(c);
  float mean = means.at<float>(y,x);

//   ROS_INFO("Norm: %f, mean: %f", d, mean);

  if ( mean - min_dist > d && d > mean - max_dist){
   result.push_back(c);
   if (valids) valids->push_back(cv::Point2i(x,y));
  }

 }

// ROS_INFO("BG: %i invalid", invalid);

 return result;

}


bool Background_substraction::computeBackground(float max_std_dev){

 reset_helper_images();


 if (dists.size() == 0){
  ROS_FATAL("Background_substraction::computeBackground: no Training data!");
  return false;
 }

 for (int x=0; x<dists[0].cols; ++x)
  for (int y=0; y<dists[0].rows; ++y){

   // iterate over all distimages and copy all valid dists
   vector<float> d;
   for (uint i=0; i<dists.size(); ++i){
    float c_d = dists[i].at<float>(y,x);
    if (c_d > 0)
     d.push_back(c_d);
   }

   uint valid_dists = d.size();

   if (valid_dists == 0){
    // means.at<float>(y,x)stays -1
    // vars.at<float>(y,x) stays -1
    continue;
   }


   if (valid_dists == 1){
    means.at<float>(y,x) = d[0];
    // vars.at<float>(y,x) stays -1
    continue;
   }


   // compute mean
   float mu = 0;
   for (uint i=0; i<valid_dists; ++i)
    mu += d[i]/valid_dists;

   means.at<float>(y,x) = mu;

   // compute variance:
   float var = 0;
   for (uint i=0; i<valid_dists; ++i)
    var += pow(d[i]-mu,2)/valid_dists;

//   if (var > 0.3){
//    cout << var <<": ";
//    for (uint i=0; i<valid_dists; ++i)
//     cout << d[i] << " ";
//     cout << endl;
//   }


   std_dev.at<float>(y,x) = sqrt(var);

  }
// ROS_INFO("Background computed");


// double min_val, max_val;
//
// cv::minMaxLoc(means, &min_val, &max_val);
// ROS_INFO("means: %f %f", min_val, max_val);
// means -= min_val;
// means /= (max_val-min_val);
//
// cv::namedWindow("mu");
// cv::imshow("mu", means);



 mask = cv::Mat(dists[0].rows, dists[0].cols, CV_8UC1);
 mask.setTo(0);


 for (int x=0; x<dists[0].cols; ++x)
  for (int y=0; y<dists[0].rows; ++y){
   float std = std_dev.at<float>(y,x);

   mask.at<uchar>(y,x) = (std > max_std_dev || std < 0)?0:255;

  }

// cv::dilate(mask, mask, cv::Mat());
 cv::erode(mask, mask, cv::Mat());


cv::imwrite("mask.jpg", mask);

// //vars -= min_val;
// // vars /= 0.02*0.02;
//
// // show points without any measurement (they have var = -100
// cv::Mat nans;
// std_dev *= -1;
// cv::threshold(std_dev, nans, 50, 1,CV_THRESH_BINARY);
// std_dev *= -1;
//
//
// cv::minMaxLoc(std_dev, &min_val, &max_val);
//
// ROS_INFO("var: %f %f", min_val, max_val);
//
//
// cv::minMaxLoc(std_dev, &min_val, &max_val);
//
//
// cv::Mat m1;
// cv::threshold(std_dev, m1, max_error*max_error, 1, CV_THRESH_BINARY);
//// cv::erode(vars, vars, cv::Mat());
//// cv::dilate(vars, vars, cv::Mat());
//
// ROS_INFO("var2: %f %f", min_val, max_val);
//
//// cv::Mat mask;
//// m1.copyTo(mask, nans);
//
// cv::namedWindow("var");
// cv::imshow("var", std_dev);
//
// cv::namedWindow("mask");
// cv::imshow("mask", mask);
//
// cv::namedWindow("nans");
// cv::imshow("nans", nans);


// cv::namedWindow("mask");
// cv::imshow("mask", mask);
//
// cv::waitKey(10);




 return true;


}

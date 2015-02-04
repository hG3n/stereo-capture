#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <dirent.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#define LEFT 0
#define RIGHT 1
char key;

int getFiles (std::string const& dir, std::vector<std::string> &files) {
  DIR *dp;
  struct dirent *dirp;

  //Unable to open dir
  if((dp  = opendir(dir.c_str())) == NULL)  {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  //read files and push them to vector
  while ((dirp = readdir(dp)) != NULL) {
    std::string name = std::string(dirp->d_name);
    //discard . and .. from list
    if(name != "." && name != ".."){
      files.push_back(std::string(dirp->d_name));
    }
  }
  closedir(dp);
  std::sort(files.begin(), files.end());
  return 0;
}


int main(int argc, char const *argv[]) {
    
  std::string folder;
  int DISP;
  int alpha;

  std::cout << "Folder to search in : ";
  std::cin >> folder;
  std::cout << "alpha value         : ";
  std::cin >> alpha;
  std::cout << "disparity 1 / 0     : ";
  std::cin >> DISP;
  std::cout << "" << std::endl;

  std::string dirLeft = std::string("./"+folder+"/left");
  std::string dirRight = std::string("./"+folder+"/right");
  std::vector<std::string> filenamesLeft = std::vector<std::string>();
  std::vector<std::string> filenamesRight = std::vector<std::string>();

  getFiles(dirLeft,filenamesLeft);
  getFiles(dirRight,filenamesRight);

  for (unsigned int i = 0; i < filenamesLeft.size(); ++i) {
      std::cout << filenamesLeft[i] << "  &  " << filenamesRight[i] << std::endl;
  }

  int hCorners = 9;
  int vCorners = 6;
  
  cv::Size boardSize = cv::Size(hCorners, vCorners);

  // calibration settings
  std::vector<cv::Mat> imagesLeft;
  std::vector<cv::Mat> imagesRight;

  // define the empty calibration matrix
  cv::Mat intrinsicLeft;
  cv::Mat intrinsicRight;
  
  // more calibration vars
  cv::Mat distCoeffsLeft;
  cv::Mat distCoeffsRight;
  std::vector<cv::Mat> rvecs,tvecs;
  
  // containers for obj and img points
  std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<std::vector<cv::Point2f> > imagePointsLeft;
  std::vector<std::vector<cv::Point2f> > imagePointsRight;
  std::vector<cv::Point2f> cornersLeft;
  std::vector<cv::Point2f> cornersRight;
  std::vector<cv::Point3f> obj;

  //squarsize in mm
  float a = 27.5;

  for(int y=0; y<vCorners; ++y) {
    for(int x=0; x<hCorners; ++x) {
      obj.push_back(cv::Point3f((float(x)*a),(float(y)*a),0));
    }
  }
  unsigned int numberOfImages = filenamesLeft.size();
  for(unsigned int i = 0; i < numberOfImages; ++i) {
    cv::Mat imageLeft = cv::imread("./"+folder+"/left/"+filenamesLeft[i]);
    cv::Mat imageRight = cv::imread("./"+folder+"/right/"+filenamesRight[i]);
    cv::Mat grayImageLeft;
    cv::Mat grayImageRight;

    cv::cvtColor(imageLeft, grayImageLeft, CV_BGR2GRAY);
    cv::cvtColor(imageRight, grayImageRight, CV_BGR2GRAY);

    bool foundLeft = cv::findChessboardCorners( grayImageLeft, boardSize, cornersLeft, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    bool foundRight = cv::findChessboardCorners( grayImageRight, boardSize, cornersRight, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    
    
    if(foundLeft && foundRight) {
      cv::cornerSubPix(grayImageLeft, cornersLeft, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.1));
      cv::cornerSubPix(grayImageRight, cornersRight, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.1));
      // drawChessboardCorners(imageLeft, boardSize, cornersLeft, foundLeft);
      // drawChessboardCorners(imageRight, boardSize, cornersRight, foundRight);
      imagesLeft.push_back(imageLeft);
      imagePointsLeft.push_back(cornersLeft);
      imagesRight.push_back(imageRight);
      imagePointsRight.push_back(cornersRight);
      objectPoints.push_back(obj);
      std::cout<<"Found  "<<i<<std::endl;
    }
    else {
      std::cout<<"Unable to finde corners in image "<<i<<". Image is ignored!\n";
      continue;
    }
    imageLeft.release();
    imageRight.release();

    grayImageLeft.release();
    grayImageRight.release();
  }

  double rmsL = cv::calibrateCamera(objectPoints, imagePointsLeft, imagesLeft[0].size(),intrinsicLeft, distCoeffsLeft, rvecs, tvecs);
  double rmsR = cv::calibrateCamera(objectPoints, imagePointsRight, imagesRight[0].size(), intrinsicRight, distCoeffsRight, rvecs, tvecs);

  cv::Mat R,T,E,F;
  cv::Size imagesize(imagesLeft[0].size());

  double stereoRMS = cv::stereoCalibrate(objectPoints,imagePointsLeft,imagePointsRight,
                                         intrinsicLeft,distCoeffsLeft,intrinsicRight,distCoeffsRight,
                                         imagesize,R,T,E,F,
                                         cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 300, 1e-7));

  std::cout<<rmsL<<" "<<rmsR<<" "<<stereoRMS<<std::endl;

  cv::Mat R0, R1, P0, P1, Q, map1R, map2R, map1L, map2L;
  std::vector<cv::Mat> undistortedLeft;
  std::vector<cv::Mat> undistortedRight;
  cv::Rect validROI[2];
  
  stereoRectify(intrinsicLeft, distCoeffsLeft,
                intrinsicRight, distCoeffsRight,
                imagesize, R,T,R0,R1,P0,P1,Q,0,alpha,
                imagesize, &validROI[0], &validROI[1]);
  std::cout << "Rectification complete!" << std::endl;

  cv::initUndistortRectifyMap(intrinsicLeft, distCoeffsLeft, R0, P0, imagesize, CV_32FC1, map1L, map2L);
  cv::initUndistortRectifyMap(intrinsicRight, distCoeffsRight, R1, P1, imagesize, CV_32FC1, map1R, map2R);

  for(unsigned int i = 0; i < imagesLeft.size(); ++i) {
    cv::Mat undistortedL, undistortedR;
    cv::remap(imagesLeft[i], undistortedL, map1L, map2L, cv::INTER_CUBIC);
    cv::remap(imagesRight[i], undistortedR, map1R, map2R, cv::INTER_CUBIC);
    undistortedLeft.push_back(undistortedL);
    undistortedRight.push_back(undistortedR);
    undistortedL.release();
    undistortedR.release();
  }

  for (unsigned int i = 0; i < imagesLeft.size(); ++i) {
    cv::imshow("LEFT", undistortedLeft[i]);
    cv::imshow("RIGHT", undistortedRight[i]);
    key = cv::waitKey(0);
    if(key == 'q')
      break;
  }

#if 0
  for (unsigned int i = 0; i < imagesLeft.size(); ++i) {
    cv::rectangle(undistortedLeft[i], validROI[0]);
    cv::rectangle(undistortedRight[i], validROI[1]);
    cv::imshow("Right", undistortedRight[i]);
    cv::imshow("Left", undistortedLeft[i]);
    key = cv::waitKey(0);
    if(key == 'q')
      break;
  }
#endif

  cv::StereoSGBM disparity(10,160,5,2312,9248);
  cv::Mat disparityMap, disparityNorm;

  if(DISP == 1) {
    disparity(undistortedLeft[1], undistortedRight[1], disparityMap);
    disparityMap*=(1/16.0);
    cv::normalize(disparityMap, disparityNorm, 20,255, cv::NORM_MINMAX, CV_8U);
    //cv::cvtColor(disparityMap, disparityNorm, CV_BGR2GRAY);
    cv::imshow("disparityMap", disparityNorm);
    cv::waitKey(0);
  }


  cv::FileStorage intrinsicLeft_YML("left.yml",cv::FileStorage::WRITE);
  cv::FileStorage intrinsicRight_YML("right.yml",cv::FileStorage::WRITE);
  cv::FileStorage extrinsic_YML("extrinsic.yml", cv::FileStorage::WRITE);

  intrinsicLeft_YML << "cameraMatrix" << intrinsicLeft<<"distCoeff"<<distCoeffsLeft;
  intrinsicLeft_YML.release();

  intrinsicRight_YML << "cameraMatrix" << intrinsicRight<<"distCoeff"<<distCoeffsRight;
  intrinsicRight_YML.release();

  extrinsic_YML <<"R" << R <<"T"<< T <<"E" << E << "F" << F;
  extrinsic_YML.release();



   /* for (unsigned int i = 0 ; i< imagesLeft.size(); ++i)
    {
         cv::imwrite(filenamesLeft[i] , imagesLeft[i]);
         cv::imwrite(filenamesRight[i] , imagesRight[i]);
    }
*/
    return 0;
}


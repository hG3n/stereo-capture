#include <iostream>
#include <vector>
#include <string>
#include <glob.h>
#include <omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

void loadImages(std::string folder, int number, std::vector<cv::Mat> *store) {
  for(int i = 1; i < number+1; ++i){
    std::string num = std::to_string(i);
    cv::Mat img;
    if(i < 10) 
      img = cv::imread(folder+"000"+num+".jpg");
    else
      img = cv::imread(folder+"00"+num+".jpg");
    store->push_back(img);
    img.release();
  }
}

void saveCoefficients(std::string filename,
                      cv::Mat matrix,
                      cv::Mat distCoeffs,
                      std::vector<cv::Mat> rvecs,
                      std::vector<cv::Mat> tvecs) {

  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "cameraMatrix" << matrix;
  fs << "distCoeff" << distCoeffs;
  fs << "rvecs" << rvecs;
  fs << "tvecs" << tvecs;
  fs.release();
}

int main() {

  // settings
  int hCorners = 9;
  int vCorners = 6;

  int numSquares = hCorners * vCorners;
  cv::Size boardSize = cv::Size(hCorners, vCorners);

  // calibration settings
  std::vector<cv::Mat> colorImages;

  // define the empty calibration matrix
  cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
  intrinsic.ptr<float>(0)[0] = 1;
  intrinsic.ptr<float>(1)[1] = 1;

  // more calibration vars
  cv::Mat distCoeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  // containers for obj and img points
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<cv::Point2f> corners;
  std::vector<cv::Point3f> obj;

  // load images from the specified destination to the vector
  loadImages("./img/", 9, &colorImages);

  for(int i = 0; i < numSquares; ++i)
    obj.push_back(cv::Point3f(i/hCorners, i%hCorners, 0.0f));

  //#pragma omp parallel for
  for(unsigned int i = 0; i < colorImages.size(); ++i) {
    cv::Mat grayImage;
    cv::cvtColor(colorImages[i], grayImage, CV_BGR2GRAY);

    bool found = cv::findChessboardCorners(colorImages[i], boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found) {
      cv::cornerSubPix(grayImage, corners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      //drawChessboardCorners(grayImage, boardSize, corners, found);
    }

    imagePoints.push_back(corners);
    objectPoints.push_back(obj);

    // camera calibration
    cv::calibrateCamera(objectPoints, imagePoints, colorImages[i].size(), intrinsic, distCoeffs, rvecs, tvecs);

    grayImage.release();
  }

  // undistort the images
 #if 0 
  for(unsigned int i = 0; i < colorImages.size(); ++i) {
    cv::Mat undistorted;
    std::string imageNumber = std::to_string(i);

    // undistort the original images using the calculated cameraMatrix and the disortion Coefficients
    cv::undistort(colorImages[i], undistorted, intrinsic, distCoeffs);

    // write the undistorted Image to the out folder
    cv::imwrite("./out/"+ imageNumber + ".jpg", undistorted);
    undistorted.release();
  }
#endif
  
  saveCoefficients("coefficients.yml", intrinsic, distCoeffs, rvecs, tvecs);

#if 0
  for(unsigned int i = 0; i < colorImages.size(); ++i) {
    cv::imshow("foo", colorImages[i]);
    cv::waitKey(0);
  }
#endif


  return 0;
}

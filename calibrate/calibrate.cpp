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
int calibrate;
std::string folder;
int showRectified;
int DISP;
int alpha;
int minDISP;
int numDISP;
int SADWindowSize;
int disparitySmoothness1;
int disparitySmoothness2;

// define the empty calibration matrix
cv::Mat cameraMatrices[2];
cv::Mat distCoeffs[2];
cv::Mat R,T,E,F;

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

void loadSettings(std::string name) {
  cv::FileStorage fs(name, cv::FileStorage::READ);
  fs["folder"] >> folder;
  fs["alpha"] >> alpha;
  fs["showRectified"] >> showRectified;
  fs["enableDisparity"] >> DISP;
  fs["minDisparity"] >> minDISP;
//  fs["numDisparities"] >> numDISP;
  fs["SADWindowSize"] >> SADWindowSize;
  fs["disparitySmoothness1"] >> disparitySmoothness1;
  fs["disparitySmoothness2"] >> disparitySmoothness2;
  fs.release();
}

void loadIntrinsic(std::string name, int cam) {
  cv::FileStorage fs(name, cv::FileStorage::READ);
  fs["cameraMatrix"] >> cameraMatrices[cam];
  fs["distCoeff"] >> distCoeffs[cam];
  fs.release();
} 

void loadExtrinsic(std::string name) {
  cv::FileStorage fs(name, cv::FileStorage::READ);
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs.release();
}

int main(int argc, char const *argv[]) {
    
#if 0
  std::cout << "Folder to search in : ";
  std::cin >> folder;
  std::cout << "alpha value         : ";
  std::cin >> alpha;
  std::cout << "disparity 1 / 0     : ";
  std::cin >> DISP;
  std::cout << "min Disparity       : ";
  std::cin >> minDISP;
  std::cout << "" << std::endl;
#endif

  loadSettings("settings.yml");

  if(calibrate == 0) {
   loadIntrinsic("left.yml", LEFT);
   loadIntrinsic("right.yml", RIGHT); 
   loadExtrinsic("extrinsic.yml");
  } 

  // get files & filenames
  std::string dirLeft = std::string("./"+folder+"/left");
  std::string dirRight = std::string("./"+folder+"/right");
  std::vector<std::string> filenamesLeft = std::vector<std::string>();
  std::vector<std::string> filenamesRight = std::vector<std::string>();

  getFiles(dirLeft,filenamesLeft);
  getFiles(dirRight,filenamesRight);

  for (unsigned int i = 0; i < filenamesLeft.size(); ++i) {
      std::cout << filenamesLeft[i] << "  &  " << filenamesRight[i] << std::endl;
  }

  // calibration pattern size
  int hCorners = 9;
  int vCorners = 6;
  cv::Size boardSize = cv::Size(hCorners, vCorners);

  // calibration settings
  std::vector<cv::Mat> imagesLeft;
  std::vector<cv::Mat> imagesRight;
  std::vector<cv::Mat> rvecs,tvecs;
  
  // containers for obj and img points
  std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<std::vector<cv::Point2f> > imagePointsLeft;
  std::vector<std::vector<cv::Point2f> > imagePointsRight;
  std::vector<cv::Point2f> cornersLeft;
  std::vector<cv::Point2f> cornersRight;
  std::vector<cv::Point3f> obj;

  //squaresize in mm
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

    if(calibrate == 1) {
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
        imagesRight.push_back(imageRight);

        imagePointsLeft.push_back(cornersLeft);
        imagePointsRight.push_back(cornersRight);
        
        objectPoints.push_back(obj);
        std::cout<<"Found  "<<i<<std::endl;
        
        grayImageLeft.release();
        grayImageRight.release();
      } else {
        std::cout<<"Unable to finde corners in image "<<i<<". Image is ignored!\n";
        continue;
      } 
    } else {
      imagesLeft.push_back(imageLeft);
      imagesRight.push_back(imageRight);
    }

    imageLeft.release();
    imageRight.release();

  }

  cv::Size imagesize(imagesLeft[0].size());
  if(calibrate == 1) {
    double rmsL = cv::calibrateCamera(objectPoints, imagePointsLeft, imagesLeft[0].size(),cameraMatrices[LEFT], distCoeffs[LEFT], rvecs, tvecs);
    double rmsR = cv::calibrateCamera(objectPoints, imagePointsRight, imagesRight[0].size(), cameraMatrices[RIGHT], distCoeffs[RIGHT], rvecs, tvecs);

    double stereoRMS = cv::stereoCalibrate(objectPoints,imagePointsLeft,imagePointsRight,
                                           cameraMatrices[LEFT],distCoeffs[LEFT],cameraMatrices[RIGHT],distCoeffs[RIGHT],
                                           imagesize,R,T,E,F,
                                           cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 300, 1e-7));

    std::cout<<rmsL<<" "<<rmsR<<" "<<stereoRMS<<std::endl;
  }

  // rectification variables
  cv::Mat R0, R1, P0, P1, Q, map1R, map2R, map1L, map2L;
  std::vector<cv::Mat> undistortedLeft;
  std::vector<cv::Mat> undistortedRight;
  cv::Rect validROI[2];
  
  std::cout << cameraMatrices[LEFT] << std::endl;
  std::cout << cameraMatrices[RIGHT] << std::endl;

  // rectification process
  stereoRectify(cameraMatrices[LEFT], distCoeffs[LEFT],
                cameraMatrices[RIGHT], distCoeffs[RIGHT],
                imagesize, R,T,R0,R1,P0,P1,Q,0,alpha,
                imagesize, &validROI[0], &validROI[1]);
  std::cout << "Rectification complete!" << std::endl;

  cv::initUndistortRectifyMap(cameraMatrices[LEFT], distCoeffs[LEFT], R0, P0, imagesize, CV_32FC1, map1L, map2L);
  cv::initUndistortRectifyMap(cameraMatrices[RIGHT], distCoeffs[RIGHT], R1, P1, imagesize, CV_32FC1, map1R, map2R);

  for(unsigned int i = 0; i < imagesLeft.size(); ++i) {
    cv::Mat undistortedL, undistortedR;
    cv::remap(imagesLeft[i], undistortedL, map1L, map2L, cv::INTER_CUBIC);
    cv::remap(imagesRight[i], undistortedR, map1R, map2R, cv::INTER_CUBIC);
    undistortedLeft.push_back(undistortedL);
    undistortedRight.push_back(undistortedR);
    undistortedL.release();
    undistortedR.release();
  }

  if(showRectified == 1) {
    for (unsigned int i = 0; i < imagesLeft.size(); ++i) {
      cv::imshow("LEFT", undistortedLeft[i]);
      cv::imshow("RIGHT", undistortedRight[i]);
      key = cv::waitKey(0);
      if(key == 'q')
        break;
    }
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
  minDISP = 0;
  numDISP = 16;
  SADWindowSize=0;

  if(DISP == 1) {
  
    while(true) {

    cv::StereoSGBM disparity(minDISP,numDISP,SADWindowSize, disparitySmoothness1, disparitySmoothness2);
    cv::Mat disparityMap, disparityNorm;

    disparity(undistortedLeft[1], undistortedRight[1], disparityMap);
    disparityMap*=(1/16.0);
    cv::normalize(disparityMap, disparityNorm, 0,255, cv::NORM_MINMAX, CV_8U);
    //cv::cvtColor(disparityMap, disparityNorm, CV_BGR2GRAY);
    cv::imshow("disparityMap", disparityNorm);
    key = cv::waitKey(0);
    if(key == 'n') 
      numDISP+=16;
    if (key = 'm')
      minDISP+=1;
    if (key == 's')
      SADWindowSize+=1; 
    if (key == 'q')
      break;
    std::cout << "minDISP : " << minDISP << "\t"
              << "numDISP : " << numDISP << std::endl;
    }
  }

  if (calibrate == 1) {
    cv::FileStorage intrinsicleft_YML("left.yml",cv::FileStorage::WRITE);
    cv::FileStorage intrinsicRight_YML("right.yml",cv::FileStorage::WRITE);
    cv::FileStorage extrinsic_YML("extrinsic.yml", cv::FileStorage::WRITE);

    intrinsicleft_YML << "cameraMatrix" << cameraMatrices[LEFT]<<"distCoeff"<<distCoeffs[LEFT];
    intrinsicleft_YML.release();

    intrinsicRight_YML << "cameraMatrix" << cameraMatrices[RIGHT]<<"distCoeff"<<distCoeffs[RIGHT];
    intrinsicRight_YML.release();

    extrinsic_YML <<"R" << R <<"T"<< T <<"E" << E << "F" << F;
    extrinsic_YML.release();
  }



   /* for (unsigned int i = 0 ; i< imagesLeft.size(); ++i)
    {
         cv::imwrite(filenamesLeft[i] , imagesLeft[i]);
         cv::imwrite(filenamesRight[i] , imagesRight[i]);
    }
*/
    return 0;
}


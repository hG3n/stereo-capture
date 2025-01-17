#include <iostream>
#include <vector>
#include <string>
#include <omp.h>
// #include <mpi.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "bgapi2_genicam.hpp"

char key;

// camera image sizes
CvSize imagesizeLeft(cvSize(1392/2, 1040/2));
CvSize imagesizeRight(cvSize(1392/2, 1040/2));
std::vector<int> camWidth(2,0);
std::vector<int> camHeight(2,0);

// baumer system variables
BGAPI2::SystemList     *systemList(NULL);
BGAPI2::System         *systemMem(NULL);
BGAPI2::String          systemID("");

BGAPI2::InterfaceList  *interfaceList(NULL);
BGAPI2::Interface      *interface(NULL);
BGAPI2::String          interfaceID("");

BGAPI2::DeviceList     *deviceList(NULL);
BGAPI2::Device         *device(NULL);
BGAPI2::String          deviceID("");

BGAPI2::DataStreamList *datastreamList_L(NULL);
BGAPI2::DataStreamList *datastreamList_R(NULL);
std::vector<BGAPI2::DataStreamList*> datastreamListVector;

BGAPI2::DataStream     *datastream_L(NULL);
BGAPI2::DataStream     *datastream_R(NULL);
std::vector<BGAPI2::DataStream*> datastreamVector;

BGAPI2::String          datastreamID_L("");
BGAPI2::String          datastreamID_R("");
std::vector<BGAPI2::String> datastreamIDVector;

BGAPI2::BufferList     *bufferList_L(NULL);
BGAPI2::BufferList     *bufferList_R(NULL);
std::vector<BGAPI2::BufferList*> bufferListVector;

BGAPI2::Buffer         *buffer_L(NULL);
BGAPI2::Buffer         *buffer_R(NULL);
std::vector<BGAPI2::Buffer*> bufferVector;

// picture Numbers
unsigned int pictureNumberLeft = 0;
unsigned int pictureNumberRight = 0;

// camera parameters
cv::Mat intrinsicLeft = cv::Mat(3, 3, CV_32FC1);
cv::Mat intrinsicRight = cv::Mat(3, 3, CV_32FC1);
std::vector<cv::Mat> cameraMatrices;

cv::Mat distCoeffLeft, distCoeffsRight;
std::vector<cv::Mat> distCoeffs;

// extrinsic parameters
cv::Mat R,T,E,F;

// settings
double exposure, gain;
int rectifyalpha, enableDisparity;
int minDisparity, numDisparities, SADWindowSize;
int disparitySmoothness1, disparitySmoothness2;

// callbacks
void initCameras();
void initStereoContainers();
void initParameters();
void openStream(IplImage *ref, int cam);
void saveImage(IplImage* ref, int cam);
void loadSettings(std::string name);
void loadIntrinsic(std::string name, int cam);
void loadExtrinsic(std::string name);

int main(int argc, char const *argv[]) {
    
  initStereoContainers();
  initParameters();
  loadSettings("settings.yml");
  initCameras();
  loadIntrinsic("left.yml", 0);
  loadIntrinsic("right.yml", 1);
  loadExtrinsic("extrinsic.yml");
  std::cout << "parameters loaded" << std::endl;

  IplImage *imageLIpl = cvCreateImage(imagesizeLeft, IPL_DEPTH_8U, 1);
  IplImage *imageRIpl = cvCreateImage(imagesizeRight, IPL_DEPTH_8U, 1);

  cv::Mat undistortedLeft, undistortedRight, 
          map1Left, map1Right,
          map2Left, map2Right,
          disparityMap,
          disparityNorm;

  cv::Mat R0, R1, P0, P1, Q;
  cv::Rect validROI[2];

  cv::StereoSGBM disparity(minDisparity, numDisparities, SADWindowSize,
                           disparitySmoothness1, disparitySmoothness2);

  cv::stereoRectify(cameraMatrices[0], distCoeffs[0], cameraMatrices[1], distCoeffs[1],
                      imagesizeLeft, R, T, R0, R1, P0, P1, Q, 0, rectifyalpha, 
                      imagesizeLeft, &validROI[0], &validROI[1]);

  int frame = 0;
 
  cvNamedWindow("Left", CV_WINDOW_NORMAL);
  cvNamedWindow("Right", CV_WINDOW_NORMAL);
  if(enableDisparity == 1) {
    cvNamedWindow("DisparityMap", CV_WINDOW_NORMAL);
  } 

  while(true) {

    ++frame;
    if(key == 27)
      break;
    key = cvWaitKey(1);

    openStream(imageLIpl, 0);
    openStream(imageRIpl, 1);

    cv::Mat imageL(imageLIpl, true);
    cv::Mat imageR(imageRIpl, true);
#if 1

    std::cout << P0 << std::endl;

    cv::initUndistortRectifyMap(cameraMatrices[0], distCoeffs[0], R0, P0, imagesizeLeft, CV_32FC1, map1Left, map2Left);
    cv::initUndistortRectifyMap(cameraMatrices[1], distCoeffs[1], R1, P1, imagesizeRight, CV_32FC1, map1Right, map2Right);

    cv::remap(imageL, undistortedLeft, map1Left, map2Left, cv::INTER_CUBIC);
    cv::remap(imageR, undistortedRight, map1Right, map2Right, cv::INTER_CUBIC);

    if(enableDisparity == 1) {
      disparity(undistortedLeft, undistortedRight, disparityMap);
      disparityMap*=(1/16.0);
      cv::normalize(disparityMap, disparityNorm, 0, 255, cv::NORM_MINMAX, CV_8U);
      //cv::cvtColor(disparityMap, disparityNorm, CV_BGR2GRAY);
    }

    cv::imshow("Left", undistortedLeft);
    cv::imshow("Right", undistortedRight);

    if(enableDisparity == 1) {
      cv::imshow("DisparityMap", disparityNorm);
    } 
    
    imageL.release();
    imageR.release();

    undistortedLeft.release();
    undistortedRight.release();

    disparityNorm.release();
    disparityMap.release();
#endif
    //cv::imshow("Left", imageL);
    //cv::imshow("Right", imageR);
  }

  return 0;
}

void initCameras() {

  try {
    systemList = BGAPI2::SystemList::GetInstance();
    systemList->Refresh();
    std::cout << "Systems: " << systemList->size() << std::endl;
    std::cout << "" << std::endl;

    systemList->begin()->second->Open();
    systemID = systemList->begin()->first;
    if (systemID == "") {
      std::cout << "Error: no system found" << std::endl;
    } else {
      systemMem = (*systemList)[systemID];
      //std::cout << "SystemID: " << systemID << std::endl;
    }

    // --- INTERFACE --- //
    interfaceList = systemMem->GetInterfaces();
    interfaceList->Refresh(100);
    std::cout << "Interfaces: " << interfaceList->size() << std::endl;
    std::cout << "" << std::endl;

    for(BGAPI2::InterfaceList::iterator interfaceIter = interfaceList->begin(); interfaceIter != interfaceList->end(); ++interfaceIter){
      interfaceIter->second->Open();
      deviceList = interfaceIter->second->GetDevices();
      deviceList->Refresh(100);

      if (deviceList->size() > 0) {
        std::cout << "Devices: " << deviceList->size() << std::endl;
        std::cout << "" << std::endl;
        interfaceID = interfaceIter->first;
        interface = interfaceIter->second;
        break;
      } else {
        interfaceIter->second->Close();
      }
    }

    int cam = 0;
    // --- DEVICES --- //
    for(BGAPI2::DeviceList::iterator deviceIter = deviceList->begin(); deviceIter != deviceList->end(); ++deviceIter) {
      device = deviceIter->second;
      std::cout << "Device Name: " << deviceIter->second->GetDisplayName() << std::endl;
      device->Open();

      // --- SETUP STUFF --- //
      device->GetRemoteNode("PixelFormat")->SetString("Mono8");
      device->GetRemoteNode("HqMode")->SetString("Off");
      device->GetRemoteNode("BinningHorizontal")->SetInt(2);
      device->GetRemoteNode("BinningVertical")->SetInt(2);
      device->GetRemoteNode("ExposureTime")->SetDouble(exposure);
      device->GetRemoteNode("Gain")->SetDouble(gain);

      // --- CAM RESOLUTION --- //
      camWidth[cam] = device->GetRemoteNode("Width")->GetInt();
      camHeight[cam] = device->GetRemoteNode("Height")->GetInt();
      std::cout << "  Resolution: " << camWidth[cam] << " x " << camHeight[cam] << std::endl;

      // --- DATASTREAMS --- //
      datastreamListVector[cam] = device->GetDataStreams();
      datastreamListVector[cam]->Refresh();
      std::cout << "  DataStreams: " << datastreamListVector[cam]->size() << std::endl;

      datastreamListVector[cam]->begin()->second->Open();
      datastreamIDVector[cam] = datastreamListVector[cam]->begin()->first;
      if (datastreamIDVector[cam] == "") {
        std::cout << "Error: no datastream found" << std::endl;
      } else {
        datastreamVector[cam] = (*datastreamListVector[cam])[datastreamIDVector[cam]];
        std::cout << "  DataStreamID: " << datastreamIDVector[cam] << std::endl;
      }

      // --- BUFFER --- // 
      bufferListVector[cam] = datastreamVector[cam]->GetBufferList();
      for(int i = 0; i < 4; ++i) {
        bufferVector[cam] = new BGAPI2::Buffer();
        bufferListVector[cam]->Add(bufferVector[cam]);
      }
      std::cout << "  Announced Buffers: " << bufferListVector[cam]->size() << std::endl;
      for(BGAPI2::BufferList::iterator buf = bufferListVector[cam]->begin(); buf != bufferListVector[cam]->end(); ++buf) {
        buf->second->QueueBuffer();
      }
      std::cout << "  Queued buffers: " << bufferListVector[cam]->GetQueuedCount() << std::endl;

      // --- START DATASTREAM AND FILL BUFFER --- //
      datastreamVector[cam]->StartAcquisitionContinuous();
      device->GetRemoteNode("AcquisitionStart")->Execute();

      std::cout << "" << std::endl;

      ++cam;
    }

  } catch(BGAPI2::Exceptions::IException& ex) {
    std::cerr << ex.GetErrorDescription() << std::endl;
  }
}

void initStereoContainers() {
  datastreamListVector.push_back(datastreamList_L);
  datastreamListVector.push_back(datastreamList_R);
  
  datastreamVector.push_back(datastream_L);
  datastreamVector.push_back(datastream_R);
  
  datastreamIDVector.push_back(datastreamID_L);
  datastreamIDVector.push_back(datastreamID_R);
  
  bufferListVector.push_back(bufferList_L);
  bufferListVector.push_back(bufferList_R);

  bufferVector.push_back(buffer_L);
  bufferVector.push_back(buffer_R);
}

void initParameters() {
  // camera matrices
  cameraMatrices.push_back(intrinsicLeft);
  cameraMatrices.push_back(intrinsicRight);

  cameraMatrices[0].ptr<float>(0)[0] = 1;
  cameraMatrices[0].ptr<float>(1)[1] = 1;

  cameraMatrices[1].ptr<float>(0)[0] = 1;
  cameraMatrices[1].ptr<float>(1)[1] = 1;

  // distortion coefficiants
  distCoeffs.push_back(distCoeffLeft);
  distCoeffs.push_back(distCoeffsRight);
}

void openStream(IplImage *ref, int cam) {
  char *img = nullptr;
  try {
    BGAPI2::Buffer *bufferFilled = NULL;
    bufferFilled = datastreamVector[cam]->GetFilledBuffer(1000);
    if(bufferFilled == NULL) {
      std::cout << "Error: Buffer Timeout" << std::endl;
    }

    img = (char*)bufferFilled->GetMemPtr();
    bufferFilled->QueueBuffer();

    IplImage *tempframe = cvCreateImageHeader(cvSize(camWidth[cam], camHeight[cam]), IPL_DEPTH_8U, 1);
    cvSetData(tempframe, img, camWidth[cam]);

    cvCopy(tempframe, ref, NULL);
    cvReleaseImageHeader(&tempframe);

    cvFlip(ref, ref, 1);
    // pressing spacebar saves the current image
    if (char(key) == 32) {  
      saveImage(ref, cam);
    }
  } catch (BGAPI2::Exceptions::IException& ex) {
    std::cerr << ex.GetErrorDescription() << std::endl;
  }
}

void saveImage(IplImage* ref, int cam) {
  if(cam == 0) {
    ++pictureNumberRight;
    std::string rightName = "./out/right/right_0";
    if (pictureNumberRight <10)
      rightName +="00";
    else if(pictureNumberRight >= 10 && pictureNumberRight<100)
      rightName+="0";
    cvSaveImage((rightName+std::to_string(pictureNumberRight)+".png").c_str(), ref);
    std::cout << "Image right_0" << pictureNumberRight << ".png " << "saved!" << std::endl;

  } else {
    ++pictureNumberLeft;
    std::string leftName = "./out/left/left_0";
    if (pictureNumberLeft <10)
      leftName +="00";
    else if(pictureNumberLeft >= 10 && pictureNumberLeft<100)
        leftName+="0";
    cvSaveImage((leftName+std::to_string(pictureNumberLeft)+".png").c_str(), ref);
    std::cout << "Image left_0" << pictureNumberLeft << ".png " << "saved!" << std::endl;
  }
}

void loadSettings(std::string name) {
  cv::FileStorage fs(name, cv::FileStorage::READ);
  fs["exposure"] >> exposure;
  fs["gain"] >> gain;
  fs["rectifyalpha"] >> rectifyalpha;
  fs["enableDisparity"] >> enableDisparity;
  fs["minDisparity"] >> minDisparity;
  fs["numDisparities"] >> numDisparities;
  fs["SADWindowSize"] >> SADWindowSize;
  fs["disparitySmoothness1"] >> disparitySmoothness1;
  fs["disparitySmoothness2"] >> disparitySmoothness2;
  fs.release();
}

void loadIntrinsic(std::string name, int cam) {
  cv::FileStorage fs("./parameters/" + name, cv::FileStorage::READ);
  fs["cameraMatrix"] >> cameraMatrices[cam];
  fs["distCoeff"] >> distCoeffs[cam];
  fs.release();
} 

void loadExtrinsic(std::string name) {
  cv::FileStorage fs("./parameters/" + name, cv::FileStorage::READ);
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs.release();
}

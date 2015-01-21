#include <iostream>
#include <vector>
#include <string>
#include <omp.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "bgapi2_genicam.hpp"

char key;
int camWidth, camHeight;

CvSize imagesize(cvSize(1384, 1036));
IplImage* frame1(cvCreateImage(imagesize, IPL_DEPTH_8U, 1));

// baumer system variables
BGAPI2::SystemList*     systemList(NULL);
BGAPI2::System*         systemMem(NULL);
BGAPI2::String          systemID("");

BGAPI2::InterfaceList*  interfaceList(NULL);
BGAPI2::Interface*      interface(NULL);
BGAPI2::String          interfaceID("");

BGAPI2::DeviceList*     deviceList(NULL);
BGAPI2::Device*         device(NULL);
BGAPI2::String          deviceID("");

BGAPI2::DataStreamList* datastreamList(NULL);
BGAPI2::DataStream*     datastream(NULL);
BGAPI2::String          datastreamID("");

BGAPI2::BufferList*     bufferList(NULL);
BGAPI2::Buffer*         buffer(NULL);

// calibration variables
int hCorners = 9;
int vCorners = 6;

int numSquares = hCorners * vCorners;
CvSize boardsize(cvSize(hCorners, vCorners));

std::vector<cv::Point3f> obj;
std::vector<CvPoint2D32f> corners;

//callbacks
void initCameras();
void openStream(IplImage *ref, int frame);

int main(int argc, char const *argv[]) {
    
  initCameras();

  IplImage* image1 = cvCreateImage(imagesize, IPL_DEPTH_8U, 1);
//  IplImage* image1Gray = cvCreateImage(imagesize, IPL_DEPTH_8U, 1);

//  cv::Mat image1Mat(image1, )
  int frame = 0;

  // prepare obj-point vector for calibraiotn
  for(int i = 0; i < numSquares; ++i) {
    obj.push_back(cv::Point3f(i/hCorners, i%hCorners, 0.0f));
  }
 
  //cvNamedWindow("CaptureStream", CV_WINDOW_AUTOSIZE);

  while(true) {

    ++frame;
    if (key == 27)
      break;

    openStream(image1, frame);
    key = cvWaitKey(10);

    cv::Mat image1Mat(image1, true);

    bool found = cv::findChessboardCorners(image1Mat, boardsize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
#if 0
    if(found) {
      cv::drawChessboardCorners(image1Mat, boardsize, corners, found);
    }
#endif
    //cvFindChessboardCorners(image1, boardsize, corners);
    cv::imshow("CaptureStream", image1Mat);
    //cvShowImage("CaptureStream", image1);
  };


  std::cout << "end" << std::endl;
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

    // --- DEVICES --- //
    for(BGAPI2::DeviceList::iterator deviceIter = deviceList->begin(); deviceIter != deviceList->end(); ++deviceIter) {
      device = deviceIter->second;
      std::cout << "Device Name: " << deviceIter->second->GetDisplayName() << std::endl;
      device->Open();

      // --- SETUP STUFF --- //
      device->GetRemoteNode("PixelFormat")->SetString("Mono8");
      // device->GetRemoteNode("BinningHorizontal")->SetInt(2);
      // device->GetRemoteNode("BinningVertical")->SetInt(2);
      //device->GetRemoteNode("HqMode")->SetString("On");

      // --- CAM RESOLUTION --- //
      camWidth = device->GetRemoteNode("Width")->GetInt();
      camHeight = device->GetRemoteNode("Height")->GetInt();
      std::cout << "  Resolution: " << camWidth << " x " << camHeight << std::endl;      

      // --- DATASTREAMS --- //
      datastreamList = device->GetDataStreams();
      datastreamList->Refresh();
      std::cout << "  DataStreams " << datastreamList->size() << std::endl;

      datastreamList->begin()->second->Open();
      datastreamID = datastreamList->begin()->first;
      if (datastreamID == "") {
        std::cout << "Error: no datastream found" << std::endl;
      } else {
        datastream = (*datastreamList)[datastreamID];
        std::cout << "  DataStreamID: " << datastreamID << std::endl;
      }

      // --- BUFFER --- // 
      bufferList = datastream->GetBufferList();
      for(int i = 0; i < 4; ++i) {
        buffer = new BGAPI2::Buffer();
        bufferList->Add(buffer);
      }
      std::cout << "  Announced Buffers: " << bufferList->size() << std::endl;
      for(BGAPI2::BufferList::iterator buf = bufferList->begin(); buf != bufferList->end(); ++buf) {
        buf->second->QueueBuffer();
      }
      std::cout << "  Queued buffers: " << bufferList->GetQueuedCount() << std::endl;

      // --- START DATASTREAM AND FILL BUFFER --- //
      datastream->StartAcquisitionContinuous();
      device->GetRemoteNode("AcquisitionStart")->Execute();

      std::cout << "" << std::endl;

      break;
    }

  } catch(BGAPI2::Exceptions::IException& ex) {
    std::cerr << ex.GetErrorDescription() << std::endl;
  }
}

void openStream(IplImage *ref, int frame) {
  char *img = nullptr;
  try {
    BGAPI2::Buffer *bufferFilled = NULL;
    bufferFilled = datastream->GetFilledBuffer(1000);
    if(bufferFilled == NULL) {
      std::cout << "Error: Buffer Timeout" << std::endl;
    }

    img = (char*)bufferFilled->GetMemPtr();
    bufferFilled->QueueBuffer();

    IplImage *tempframe = cvCreateImageHeader(cvSize(camWidth, camHeight), IPL_DEPTH_8U, 1);
    cvSetData(tempframe, img, camWidth);

    cvCopy(tempframe, ref, NULL);
    cvReleaseImageHeader(&tempframe);

    cvFlip(ref, ref, 1);
    // pressing spacebar saves the current image
    if (char(key) == 32) {
      cvSaveImage("frame.png", ref);
    }
  } catch (BGAPI2::Exceptions::IException& ex) {
    std::cerr << ex.GetErrorDescription() << std::endl;
  }
}
#include <iostream>
#include <vector>
#include <string>
#include <omp.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "bgapi2_genicam.hpp"

char key;

// camera image sizes
CvSize imagesizeLeft(cvSize(1392, 1040));
CvSize imagesizeRight(cvSize(1392, 1040));
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

BGAPI2::DataStreamList *datastreamList_R(NULL);
BGAPI2::DataStreamList *datastreamList_L(NULL);
std::vector<BGAPI2::DataStreamList*> datastreamListVector;

BGAPI2::DataStream     *datastream_R(NULL);
BGAPI2::DataStream     *datastream_L(NULL);
std::vector<BGAPI2::DataStream*> datastreamVector;

BGAPI2::String          datastreamID_R("");
BGAPI2::String          datastreamID_L("");
std::vector<BGAPI2::String> datastreamIDVector;

BGAPI2::BufferList     *bufferList_R(NULL);
BGAPI2::BufferList     *bufferList_L(NULL);
std::vector<BGAPI2::BufferList*> bufferListVector;

BGAPI2::Buffer         *buffer_R(NULL);
BGAPI2::Buffer         *buffer_L(NULL);
std::vector<BGAPI2::Buffer*> bufferVector;

//
unsigned int pictureNumberLeft = 0;
unsigned int pictureNumberRight = 0;

//callbacks
void initCameras();
void openStream(IplImage *ref, int cam);
void saveImage(IplImage* ref, int cam);

int main(int argc, char const *argv[]) {
    
  datastreamListVector.push_back(datastreamList_R);
  datastreamListVector.push_back(datastreamList_L);
  
  datastreamVector.push_back(datastream_R);
  datastreamVector.push_back(datastream_L);
  
  datastreamIDVector.push_back(datastreamID_R);
  datastreamIDVector.push_back(datastreamID_L);
  
  bufferListVector.push_back(bufferList_R);
  bufferListVector.push_back(bufferList_L);

  bufferVector.push_back(buffer_R);
  bufferVector.push_back(buffer_L);

  initCameras();

  IplImage *imageL = cvCreateImage(imagesizeLeft, IPL_DEPTH_8U, 1);
  IplImage *imageR = cvCreateImage(imagesizeRight, IPL_DEPTH_8U, 1);

  int frame = 0;
 
  cvNamedWindow("Left", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Right", CV_WINDOW_AUTOSIZE);

  while(true) {

    ++frame;

    if(key == 27)
      break;

    openStream(imageR, 1);
    openStream(imageL, 0);

    key = cvWaitKey(10);

    cvShowImage("Left", imageL);
    cvShowImage("Right", imageR);
  };

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

      // --- CAM RESOLUTION --- //
      camWidth[cam] = device->GetRemoteNode("Width")->GetInt();
      camHeight[cam] = device->GetRemoteNode("Height")->GetInt();
      std::cout << "  Resolution: " << camWidth[cam] << " x " << camHeight[cam] << std::endl;
      
      // --- SETUP STUFF --- //
      device->GetRemoteNode("PixelFormat")->SetString("Mono8");
      //device->GetRemoteNode("HqMode")->SetString("On");

      // --- DATASTREAMS --- //
      datastreamListVector[cam] = device->GetDataStreams();
      datastreamListVector[cam]->Refresh();
      std::cout << "  DataStreams " << datastreamListVector[cam]->size() << std::endl;

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

  } else {
    ++pictureNumberLeft;
    std::string leftName = "./out/left/left_0";
    if (pictureNumberLeft <10)
      leftName +="00";
    else if(pictureNumberLeft >= 10 && pictureNumberLeft<100)
        leftName+="0";
    cvSaveImage((leftName+std::to_string(pictureNumberLeft)+".png").c_str(), ref);

  }
}
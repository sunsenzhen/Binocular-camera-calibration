#ifndef HVIDEOCAPTURE_H
#define HVIDEOCAPTURE_H

/*
 * capturing from UVC cam
 * requires: libjpeg-dev
 * build: gcc -std=c99 capture.c -ljpeg -o capture
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <jpeglib.h>
#include<opencv2/opencv.hpp>
#include <iostream>
#include <iostream>
#include <string>
#include <time.h>
using namespace cv;
using namespace std;
#define TRUE 1
#define FALSE 0
typedef struct {
  uint8_t* start;
  size_t length;
} buffer_t;

typedef struct {
  int fd;
  uint32_t width;
  uint32_t height;
  size_t buffer_count;
  buffer_t* buffers;
  buffer_t head;
} camera_t;


//#define FILE_VIDEO1 "/dev/video1"

class hvideoCapture
{
public:
    hvideoCapture();
    ~hvideoCapture();

   bool myinit(std::string usbnamecom, int width, int height);


   Mat getcameraFrame();
   void setCameraSize(int width,int height);
//设置相机的曝光模式及曝光值
   void setCamera_exPouse(int emode,int expousevalue);
//设置是否关闭图像防止卡死
   void stoporcontinuCamera(bool stoporcont);
   //void continueCamera();

private:

   // myClass thisclass;

   camera_t* camera_open(const char * device, uint32_t width, uint32_t height);

   void camera_init(camera_t* camera);



   void camera_start(camera_t* camera);

   void camera_stop(camera_t* camera);


   void camera_finish(camera_t* camera);


   void camera_close(camera_t* camera);



   int camera_capture(camera_t* camera);


   int camera_frame(camera_t* camera, struct timeval timeout);


   ///转换的比较快
   /// \brief getMatForyuyvCV
   /// \param yuyv
   /// \param width
   /// \param height
   /// \return
   ///
   cv::Mat  getMatForyuyvCV(uint8_t* yuyv, uint32_t width, uint32_t height);

   void quit(const char * msg);

   int xioctl(int fd, int request, void* arg);


private:


    bool initok;
    camera_t* mCamera;
    struct timeval timeout;
    std::string camerausbcom;
    int imageWidth;
    int imageHight;
    int expouse_value;
    std::string strVideo0;
    Mat matImage;
    bool camerastarting;
    int stopcount;

};

#endif // HVIDEOCAPTURE_H

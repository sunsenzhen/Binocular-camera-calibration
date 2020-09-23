#include "hvideocapture.h"



hvideoCapture::hvideoCapture()
{
  initok=false;
  stopcount=0;
}


hvideoCapture::~hvideoCapture()
{
 if(initok)
 {
     camera_stop(mCamera);
     camera_finish(mCamera);
     camera_close(mCamera);
 }

}
Mat  hvideoCapture::getcameraFrame()
{
    if(initok)
    {
        int rcontinue= camera_frame(mCamera, timeout);

         if(rcontinue==1)
         {
              matImage=getMatForyuyvCV(mCamera->head.start, mCamera->width, mCamera->height);
              stopcount--;
              if(stopcount<0)
              {
                  stopcount=0;
              }

         }else
         {
             stopcount++;
              matImage=Mat::zeros(imageHight,imageWidth,CV_8UC3);

              cv::imwrite("error.jpg",matImage);
             std::cout<<camerausbcom<<"相机运行异常!"<<std::endl;
         }



    }else
    {
        std::cout<<camerausbcom<<"相机启动失败!"<<std::endl;
        matImage=Mat::zeros(imageHight,imageWidth,CV_8UC3);
    }

    if(stopcount>3)
    {
        stopcount=-2;
        std::cout<<camerausbcom<<"重启相机!"<<std::endl;

        camera_stop(mCamera);
        camera_finish(mCamera);
        camera_close(mCamera);
        myinit(camerausbcom,imageWidth,imageHight);

        matImage=getMatForyuyvCV(mCamera->head.start, mCamera->width, mCamera->height);
        cv::imwrite("repaire.jpg",matImage);


    }



    return matImage;

}
///
/// \brief hvideoCapture::getcameraFrame
/// \return
///


bool hvideoCapture::myinit(std::string  usbnamecom,int width,int height)
{

    camerausbcom =usbnamecom;
    mCamera = camera_open(camerausbcom.c_str(), width, height);

    camera_init(mCamera);
    camera_start(mCamera);
    //timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    /* skip 5 frames for booting a cam */
    for (int i = 0; i < 2; i++) {
      camera_frame(mCamera, timeout);
    }
    std::cout<<usbnamecom<<"相机启动成功!"<<std::endl;
    initok=true;
    camerastarting=true;
    return true;
}
///
/// \brief hvideoCapture::stoporcontinuCamera
/// \param stoporcont
///
 void hvideoCapture::stoporcontinuCamera(bool stoporcont)
 {

     if(stoporcont&&!camerastarting)
     {
         camera_start(mCamera);
         camerastarting=stoporcont;
     }
     if(!stoporcont&&camerastarting)
     {
         camera_stop(mCamera);
         camerastarting=stoporcont;
     }

 }

//设置相机的曝光模式及曝光值
void hvideoCapture::setCamera_exPouse(int emode,int expousevalue)
{


    if(expousevalue<0){expousevalue=0;}
    if(expousevalue>1000){expousevalue=1000;}
   struct v4l2_control control_s;
    if(emode==0)
    {
        //
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_APERTURE_PRIORITY;
        ioctl(mCamera->fd, VIDIOC_S_CTRL, &control_s);

    }else
    {

        //手动曝光
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_MANUAL;
        ioctl(mCamera->fd, VIDIOC_S_CTRL, &control_s);

        expouse_value= expousevalue;
        control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control_s.value = expouse_value;
        ioctl(mCamera->fd, VIDIOC_S_CTRL, &control_s);
    }



}


void hvideoCapture::setCameraSize(int width,int height)
{
    imageWidth=width;
    imageHight=height;
}
camera_t* hvideoCapture::camera_open(const char * device, uint32_t width, uint32_t height)
{
  int fd = open(device, O_RDWR | O_NONBLOCK, 0);
  if (fd == -1) quit("open");
  camera_t* camera = (camera_t*)malloc(sizeof (camera_t));
  camera->fd = fd;
  camera->width = width;
  camera->height = height;
  camera->buffer_count = 0;
  camera->buffers = NULL;
  camera->head.length = 0;
  camera->head.start = NULL;
  return camera;
}


void hvideoCapture::camera_init(camera_t* camera) {
  struct v4l2_capability cap;
  if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1) quit("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) quit("no capture");
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) quit("no streaming");

  struct v4l2_cropcap cropcap;
  memset(&cropcap, 0, sizeof cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
    struct v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1) {
      // cropping not supported
    }
  }

  struct v4l2_format format;
  memset(&format, 0, sizeof format);
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = camera->width;
  format.fmt.pix.height = camera->height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1) quit("VIDIOC_S_FMT");

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1) quit("VIDIOC_REQBUFS");
  camera->buffer_count = req.count;
  camera->buffers =(buffer_t* ) calloc(req.count, sizeof (buffer_t));

  size_t buf_max = 0;
  for (size_t i = 0; i < camera->buffer_count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
      quit("VIDIOC_QUERYBUF");
    if (buf.length > buf_max) buf_max = buf.length;
    camera->buffers[i].length = buf.length;
    camera->buffers[i].start =static_cast<uint8_t*>(mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,camera->fd, buf.m.offset)) ;
    if (camera->buffers[i].start == MAP_FAILED) quit("mmap");
  }
  camera->head.start =(uint8_t*) malloc(buf_max);
}


void hvideoCapture::camera_start(camera_t* camera)
{
  for (size_t i = 0; i < camera->buffer_count; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) quit("VIDIOC_QBUF");
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1)
    quit("VIDIOC_STREAMON");
}

void hvideoCapture::camera_stop(camera_t* camera)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
    quit("VIDIOC_STREAMOFF");
}

void hvideoCapture::camera_finish(camera_t* camera)
{
  for (size_t i = 0; i < camera->buffer_count; i++) {
    munmap(camera->buffers[i].start, camera->buffers[i].length);
  }
  free(camera->buffers);
  camera->buffer_count = 0;
  camera->buffers = NULL;
  free(camera->head.start);
  camera->head.length = 0;
  camera->head.start = NULL;
}

void hvideoCapture::camera_close(camera_t* camera)
{
  if (close(camera->fd) == -1) quit("close");
  free(camera);
}


int hvideoCapture::camera_capture(camera_t* camera)
{
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1) return FALSE;
  memcpy(camera->head.start, camera->buffers[buf.index].start, buf.bytesused);
  camera->head.length = buf.bytesused;
  if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) return FALSE;
  return TRUE;
}

int hvideoCapture::camera_frame(camera_t* camera, struct timeval timeout) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(camera->fd, &fds);
  int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
  if (r == -1) quit("select");
  if (r == 0) return FALSE;
  return camera_capture(camera);
}

///转换的比较快
/// \brief getMatForyuyvCV
/// \param yuyv
/// \param width
/// \param height
/// \return
///
cv::Mat  hvideoCapture::getMatForyuyvCV(uint8_t* yuyv, uint32_t width, uint32_t height)
{

    cv::Mat yuvImg;
    cv::Mat rgbImg(height,width,CV_8UC3);
    yuvImg.create(height,width,CV_8UC2);
    memcpy(yuvImg.data,yuyv,width*height*2);
    cv::cvtColor(yuvImg,rgbImg,CV_YUV2BGRA_YUYV);
    return rgbImg;
}
void hvideoCapture::quit(const char * msg)
{
  fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

int hvideoCapture::xioctl(int fd, int request, void* arg)
{
  for (int i = 0; i < 100; i++) {
    int r = ioctl(fd, request, arg);
    if (r != -1 || errno != EINTR) return r;
  }
  return -1;
}

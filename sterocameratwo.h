#ifndef STEROCAMERATWO_H
#define STEROCAMERATWO_H
#include "hvideocapture.h"
typedef struct CameraPamera//相机参数
{

    int cmaeraIndex;//0:left 1:right
    ////左相机内参数矩阵
    float cmaeraIntrinsic[3][3];
    //左相机畸变系数
    float cmaeraDistortion[5];
    //左相机旋转参数
    float cmaeraRotation[3][3];
    //左相机平移向量
    float cmaeraTranslation[1][3] ;
    //右相机内参数矩阵



}CAMERAPAMERA;
struct CameraList//相机列表
{

    int camerasize;

    float offsetAx;
    float offsetAy;
    float offsetAz;

    float cmaeraIntrinsicP[3][3];
   //双目标定内参数
   // float leftRotationP[3][3] ;

    float cmaeraRotationP[3][3] ;
    //右相机相对左相机的旋转


    CAMERAPAMERA *pcameraList;
};


class steroCameraTwo
{
public:
    steroCameraTwo();
    ~steroCameraTwo();

   bool InitTwoCameras(string cameradev1,string cameradev2,int cwidth,int cheight);

   bool InitTwoCameraParames(string filepath);

    void getCamFramfirst();//获取硬件图像

    Mat getOrginalFrameLeft();
    Mat getOrginalFrameRight();




    Mat getRemapFrameLeft();
    Mat getRemapFrameRight();



private:


    CameraList ReadData( std::string filepath);

    void rereadCameralistfile( std::string filepath);


    void pCorrectDistortionRight(Mat image);
    void pCorrectDistortionLeft(Mat image);
private:

    hvideoCapture cameraLR;
    hvideoCapture cameraLeft;
    hvideoCapture cameraRight;
     bool tfcomsbcam ;//=false;//是否是单ID双目摄像头
     Mat frameLRimg;//单个ID 全图像
    Mat frameRight;
    Mat frameLeft;
    string strVideo0;
    string strVideo1;

    int cameraWidth;
    int cameraHeight;



    //左相机内参数矩阵
    float leftIntrinsic[3][3] ;
    //左相机畸变系数
    float leftDistortion[5] ;
    //左相机旋转参数
    float leftRotation[3][3];
    //左相机平移向量
    float leftTranslation[1][3] ;

    //右相机内参数矩阵
    float rightIntrinsic[3][3] ;
    //右相机畸变系数
    float rightDistortion[5];
    //右相机旋转矩阵
    float rightRotation[3][3];
    //右相机平移向量
    float rightTranslation[1][3];

    //公共内参数矩阵
    float IntrinsicPl[3][3] ;

    float leftRotationP[3][3] ;

    float rightRotationP[3][3] ;


    bool isgetstortRectifyMap;//是否已经经过的图像畸变纠正

    Mat rfmapxLeft ;//= Mat(image_size, CV_32FC1);
    Mat rfmapyLeft ;//= Mat(image_size, CV_32FC1);

    Mat rfmapxRight ;//= Mat(image_size, CV_32FC1);
    Mat rfmapyRight ;//= Mat(image_size, CV_32FC1);


    bool isreadmcameralist;

    CameraList mcameralist;//相机列表

    char  gEnergyFile[1000];//文件读取缓存


};

#endif // STEROCAMERATWO_H

#include "sterocameratwo.h"

steroCameraTwo::steroCameraTwo()
{
     strVideo0="/dev/video1";
     strVideo1="/dev/video0";

     cameraWidth=1920;
     cameraHeight=1080;
     isgetstortRectifyMap=false;
     tfcomsbcam=false;//是否是单ID双目摄像头
}

steroCameraTwo::~steroCameraTwo()
{

    if(isreadmcameralist)
    {
        free(mcameralist.pcameraList);
    }

    mcameralist.pcameraList=NULL;
}

void steroCameraTwo::getCamFramfirst()//获取硬件图像
{
     if(tfcomsbcam)//如果是单ID摄像头
     {
         frameLRimg=cameraLR.getcameraFrame();

         frameLeft=frameLRimg(Rect(0,0,cameraWidth,cameraHeight));
         frameRight=frameLRimg(Rect(cameraWidth,0,cameraWidth,cameraHeight));

     }else
     {

         frameLeft= cameraLeft.getcameraFrame();
         frameRight= cameraRight.getcameraFrame();
     }

}

Mat steroCameraTwo::getOrginalFrameLeft()
{
    return frameLeft;
}
Mat steroCameraTwo::getOrginalFrameRight()
{
    return frameRight;
}


Mat steroCameraTwo::getRemapFrameLeft()
{

    if(!isgetstortRectifyMap)
    {
        pCorrectDistortionLeft(frameLeft);
        pCorrectDistortionRight(frameLeft);
         isgetstortRectifyMap=true;
    }

    cv::remap(frameLeft,frameLeft,rfmapxLeft,rfmapyLeft,INTER_LINEAR);



    return frameLeft;
}
Mat steroCameraTwo::getRemapFrameRight()
{

   if(!isgetstortRectifyMap)
   {
       pCorrectDistortionLeft(frameRight);
       pCorrectDistortionRight(frameRight);
        isgetstortRectifyMap=true;
   }

   cv::remap(frameRight,frameRight,rfmapxRight,rfmapyRight,INTER_LINEAR);

   return frameRight;
}


void steroCameraTwo::pCorrectDistortionRight(Mat image)
{
    Size image_size = image.size();
    //float intrinsic[3][3] = { 589.2526583947847,0,321.8607532099886,0,585.7784771038199,251.0338528599469,0,0,1 };
    //float distortion[1][5] = { -0.5284205687061442, 0.3373615384253201, -0.002133029981628697, 0.001511983002864886, -0.1598661778309496 };
    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, rightIntrinsic);
    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, rightDistortion);
    Mat R = Mat::eye(3, 3, CV_32F);
    Mat Rotright = Mat(3, 3, CV_32FC1, rightRotation);
    Mat intrinsic_matrixP = Mat(3, 3, CV_32FC1, IntrinsicPl);

    rfmapxRight = Mat(image_size, CV_32FC1);
    rfmapyRight = Mat(image_size, CV_32FC1);
    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, rfmapxRight, rfmapyRight);


//    Size image_size = image.size();
//    //float intrinsic[3][3] = { 589.2526583947847,0,321.8607532099886,0,585.7784771038199,251.0338528599469,0,0,1 };
//    //float distortion[1][5] = { -0.5284205687061442, 0.3373615384253201, -0.002133029981628697, 0.001511983002864886, -0.1598661778309496 };
//     Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, rightIntrinsic);
//    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, rightDistortion);
//    Mat R = Mat::eye(3, 3, CV_32F);
//    Mat Rotright = Mat(3, 3, CV_32FC1, rightRotation);
//    Mat intrinsic_matrixP = Mat(3, 3, CV_32FC1, IntrinsicPl);

//    rfmapxRight = Mat(image_size, CV_32FC1);
//    rfmapyRight = Mat(image_size, CV_32FC1);
//    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, Rotright, intrinsic_matrixP, image_size, CV_32FC1, rfmapxRight, rfmapyRight);
}
//左图像畸变参数计算
///
/// \brief CameraLocationjs::pCorrectDistortionLeft
/// \param image
/// \param cameraK
/// \param cadistC
///
void steroCameraTwo::pCorrectDistortionLeft(Mat image)
{
    Size image_size = image.size();
    //float intrinsic[3][3] = { 589.2526583947847,0,321.8607532099886,0,585.7784771038199,251.0338528599469,0,0,1 };
    //float distortion[1][5] = { -0.5284205687061442, 0.3373615384253201, -0.002133029981628697, 0.001511983002864886, -0.1598661778309496 };
    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, leftIntrinsic);
    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, leftDistortion);
    Mat R = Mat::eye(3, 3, CV_32F);

    Mat Rotleft = Mat(3, 3, CV_32FC1, leftRotation);
    Mat intrinsic_matrixP = Mat(3, 3, CV_32FC1, IntrinsicPl);
    rfmapxLeft = Mat(image_size, CV_32FC1);
    rfmapyLeft = Mat(image_size, CV_32FC1);
    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, rfmapxLeft, rfmapyLeft);

//    Size image_size = image.size();
//    //float intrinsic[3][3] = { 589.2526583947847,0,321.8607532099886,0,585.7784771038199,251.0338528599469,0,0,1 };
//    //float distortion[1][5] = { -0.5284205687061442, 0.3373615384253201, -0.002133029981628697, 0.001511983002864886, -0.1598661778309496 };

//    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, leftIntrinsic);
//    Mat distortion_coeffs = Mat(1, 5, CV_32FC1, leftDistortion);

//    Mat R = Mat::eye(3, 3, CV_32F);

//    Mat Rotleft = Mat(3, 3, CV_32FC1, leftRotation);
//    Mat intrinsic_matrixP = Mat(3, 3, CV_32FC1, IntrinsicPl);
//    rfmapxLeft = Mat(image_size, CV_32FC1);
//    rfmapyLeft = Mat(image_size, CV_32FC1);
//    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, Rotleft, intrinsic_matrixP, image_size, CV_32FC1, rfmapxLeft, rfmapyLeft);

}

bool steroCameraTwo::InitTwoCameraParames(string filepath)
{

    rereadCameralistfile(filepath);



    return isreadmcameralist;
}




bool steroCameraTwo::InitTwoCameras(string cameradev1,string cameradev2,int cwidth,int cheight)
{
    strVideo0=cameradev1;
    strVideo1=cameradev2;
    cameraWidth=cwidth;
    cameraHeight=cheight;

    if(strVideo0==strVideo1)
    {
       tfcomsbcam=true;//如果两个驱动地址一样设置为单个ID 模式；以左为上
    }
    if(tfcomsbcam)//如果非单个ID
    {
         cout<<strVideo0<<"单ID双目相机启动"<<endl;
        if(!cameraLR.myinit(strVideo0,cameraWidth*2,cameraHeight))
        {
            cout<<strVideo0<<"相机初始失败"<<endl;

            return false;
        }
        cout<<"单ID双目相机启动成功"<<endl;

    }else{

        if(!cameraLeft.myinit(strVideo0,cameraWidth,cameraHeight))
        {
            cout<<strVideo0<<"相机初始失败"<<endl;

            return false;
        }
        if(!cameraRight.myinit(strVideo1,cameraWidth,cameraHeight))
        {

            cout<<strVideo1<<"相机初始失败"<<endl;

            return false;
        }

    }


    return true;
}

///
/// \brief steroCameraTwo::ReadData
/// \param filepath
/// \return
///
CameraList steroCameraTwo::ReadData( std::string filepath)
{
    using namespace std;
    // std::cout<<"------find error step 2-<"<<filepath<<std::endl;
    CameraList merof;
    int i;//j,Tcount;
    FILE    *fp;
    char    stemp[100];
    strcpy(gEnergyFile,filepath.c_str());
    //strcpy(gEnergyFile,"cinploys.txt");
    //cinploydouble
    fp = fopen( gEnergyFile, "r" );
    //  std::cout<<"------find error step 3-<"<<filepath<<std::endl;

    if ( fp == NULL )
    {
        printf( "\n Can not open energy data file:%s\n", gEnergyFile);
        isreadmcameralist=false;
        //return merof;
        exit(0);
    }
    printf("load cameraList: %s\n",gEnergyFile);
    fseek( fp, 0, SEEK_SET);
    fscanf( fp, "%s", stemp);
  std::cout<<"------find error step 4-<"<<filepath<<std::endl;
    //OffsetAngle:
    while( strcmp( stemp,"OffsetAngle:" ) != 0)  fscanf( fp, "%s", stemp);
    fscanf( fp, "%f%f%f", &(merof.offsetAx), &(merof.offsetAy), &(merof.offsetAz));

    while( strcmp( stemp,"cmaeraIntrinsicP:" ) != 0)  fscanf( fp, "%s", stemp);
    fscanf( fp, "%f%f%f%f%f%f%f%f%f", &(merof.cmaeraIntrinsicP[0][0]), &(merof.cmaeraIntrinsicP[0][1]), &(merof.cmaeraIntrinsicP[0][2]),
            &(merof.cmaeraIntrinsicP[1][0]), &(merof.cmaeraIntrinsicP[1][1]), &(merof.cmaeraIntrinsicP[1][2]),
            &(merof.cmaeraIntrinsicP[2][0]),&(merof.cmaeraIntrinsicP[2][1]), &(merof.cmaeraIntrinsicP[2][2]));

    while( strcmp( stemp,"cmaeraRotationP:" ) != 0)  fscanf( fp, "%s", stemp);
    fscanf( fp, "%f%f%f%f%f%f%f%f%f", &(merof.cmaeraRotationP[0][0]), &(merof.cmaeraRotationP[0][1]), &(merof.cmaeraRotationP[0][2]),
            &(merof.cmaeraRotationP[1][0]), &(merof.cmaeraRotationP[1][1]), &(merof.cmaeraRotationP[1][2]),
            &(merof.cmaeraRotationP[2][0]),&(merof.cmaeraRotationP[2][1]), &(merof.cmaeraRotationP[2][2]));

    while( strcmp( stemp,"CameraSize:" ) != 0)  fscanf( fp, "%s", stemp);
    fscanf( fp, "%d", &(merof.camerasize));
    merof.pcameraList=(CAMERAPAMERA*)malloc(sizeof(CAMERAPAMERA)*merof.camerasize);
    ////////////////////////////
      std::cout<<"------find error step 5-<"<<filepath<<std::endl;
    if(merof.camerasize>1)
    {
        for(i=0;i<merof.camerasize;i++)
        {
            fscanf( fp, "%s", stemp);
            while( strcmp( stemp,"Index:" ) != 0)  fscanf( fp, "%s", stemp);
            fscanf( fp, "%d", &( merof.pcameraList[i].cmaeraIndex));

            while( strcmp( stemp,"cmaeraIntrinsic:" ) != 0)  fscanf( fp, "%s", stemp);
            fscanf( fp, "%f%f%f%f%f%f%f%f%f", &(merof.pcameraList[i].cmaeraIntrinsic[0][0]), &(merof.pcameraList[i].cmaeraIntrinsic[0][1]), &(merof.pcameraList[i].cmaeraIntrinsic[0][2]),
                    &(merof.pcameraList[i].cmaeraIntrinsic[1][0]), &(merof.pcameraList[i].cmaeraIntrinsic[1][1]), &(merof.pcameraList[i].cmaeraIntrinsic[1][2]),
                    &(merof.pcameraList[i].cmaeraIntrinsic[2][0]), &(merof.pcameraList[i].cmaeraIntrinsic[2][1]), &(merof.pcameraList[i].cmaeraIntrinsic[2][2]));

            while( strcmp( stemp,"cmaeraDistortion:" ) != 0)  fscanf( fp, "%s", stemp);
            fscanf( fp, "%f%f%f%f%f", &(merof.pcameraList[i].cmaeraDistortion[0]), &(merof.pcameraList[i].cmaeraDistortion[1]), &(merof.pcameraList[i].cmaeraDistortion[2]),
                    &(merof.pcameraList[i].cmaeraDistortion[3]), &(merof.pcameraList[i].cmaeraDistortion[4]));

            while( strcmp( stemp,"cmaeraRotation:" ) != 0)  fscanf( fp, "%s", stemp);
            fscanf( fp, "%f%f%f%f%f%f%f%f%f", &(merof.pcameraList[i].cmaeraRotation[0][0]), &(merof.pcameraList[i].cmaeraRotation[0][1]), &(merof.pcameraList[i].cmaeraRotation[0][2]),
                    &(merof.pcameraList[i].cmaeraRotation[1][0]), &(merof.pcameraList[i].cmaeraRotation[1][1]), &(merof.pcameraList[i].cmaeraRotation[1][2]),
                    &(merof.pcameraList[i].cmaeraRotation[2][0]), &(merof.pcameraList[i].cmaeraRotation[2][1]), &(merof.pcameraList[i].cmaeraRotation[2][2]));

            while( strcmp( stemp,"cmaeraTranslation:" ) != 0)  fscanf( fp, "%s", stemp);
            fscanf( fp, "%f%f%f", &(merof.pcameraList[i].cmaeraTranslation[0][0]), &(merof.pcameraList[i].cmaeraTranslation[0][1]), &(merof.pcameraList[i].cmaeraTranslation[0][2]));


        }

    }
    fclose( fp );
    isreadmcameralist=true;
    return merof;

}

//读取相机配置文件
///
/// \brief steroCameraTwo::rereadCameralistfile
/// \param filepath
///
void steroCameraTwo::rereadCameralistfile( std::string filepath)
{

    mcameralist=ReadData(filepath);

    //左相机内参数矩阵

    leftIntrinsic[0][0]=mcameralist.pcameraList[0].cmaeraIntrinsic[0][0];
    leftIntrinsic[0][1]=mcameralist.pcameraList[0].cmaeraIntrinsic[0][1];
    leftIntrinsic[0][2]=mcameralist.pcameraList[0].cmaeraIntrinsic[0][2];

    leftIntrinsic[1][0]=mcameralist.pcameraList[0].cmaeraIntrinsic[1][0];
    leftIntrinsic[1][1]=mcameralist.pcameraList[0].cmaeraIntrinsic[1][1];
    leftIntrinsic[1][2]=mcameralist.pcameraList[0].cmaeraIntrinsic[1][2];

    leftIntrinsic[2][0]=mcameralist.pcameraList[0].cmaeraIntrinsic[2][0];
    leftIntrinsic[2][1]=mcameralist.pcameraList[0].cmaeraIntrinsic[2][1];
    leftIntrinsic[2][2]=mcameralist.pcameraList[0].cmaeraIntrinsic[2][2];


    //    //左相机畸变系数

 leftDistortion[0] =mcameralist.pcameraList[0].cmaeraDistortion[0];
    leftDistortion[1] =mcameralist.pcameraList[0].cmaeraDistortion[1];
    leftDistortion[2] =mcameralist.pcameraList[0].cmaeraDistortion[2];
    leftDistortion[3] =mcameralist.pcameraList[0].cmaeraDistortion[3];
    leftDistortion[4] =mcameralist.pcameraList[0].cmaeraDistortion[4];

    //    //左相机旋转参数

    leftRotation[0][0]=mcameralist.pcameraList[0].cmaeraRotation[0][0];
    leftRotation[0][1]=mcameralist.pcameraList[0].cmaeraRotation[0][1];
    leftRotation[0][2]=mcameralist.pcameraList[0].cmaeraRotation[0][2];

    leftRotation[1][0]=mcameralist.pcameraList[0].cmaeraRotation[1][0];
    leftRotation[1][1]=mcameralist.pcameraList[0].cmaeraRotation[1][1];
    leftRotation[1][2]=mcameralist.pcameraList[0].cmaeraRotation[1][2];

    leftRotation[2][0]=mcameralist.pcameraList[0].cmaeraRotation[2][0];
    leftRotation[2][1]=mcameralist.pcameraList[0].cmaeraRotation[2][1];
    leftRotation[2][2]=mcameralist.pcameraList[0].cmaeraRotation[2][2];
    //左相机平移向量




    leftTranslation[0][0]=mcameralist.pcameraList[0].cmaeraTranslation[0][0];
    leftTranslation[0][1]=mcameralist.pcameraList[0].cmaeraTranslation[0][1];
    leftTranslation[0][2]=mcameralist.pcameraList[0].cmaeraTranslation[0][2];
    //    //右相机内参数矩阵

    rightIntrinsic[0][0]=mcameralist.pcameraList[1].cmaeraIntrinsic[0][0];
    rightIntrinsic[0][1]=mcameralist.pcameraList[1].cmaeraIntrinsic[0][1];
    rightIntrinsic[0][2]=mcameralist.pcameraList[1].cmaeraIntrinsic[0][2];

    rightIntrinsic[1][0]=mcameralist.pcameraList[1].cmaeraIntrinsic[1][0];
    rightIntrinsic[1][1]=mcameralist.pcameraList[1].cmaeraIntrinsic[1][1];
    rightIntrinsic[1][2]=mcameralist.pcameraList[1].cmaeraIntrinsic[1][2];

    rightIntrinsic[2][0]=mcameralist.pcameraList[1].cmaeraIntrinsic[2][0];
    rightIntrinsic[2][1]=mcameralist.pcameraList[1].cmaeraIntrinsic[2][1];
    rightIntrinsic[2][2]=mcameralist.pcameraList[1].cmaeraIntrinsic[2][2];

    //    //右相机畸变系数

    rightDistortion[0]=mcameralist.pcameraList[1].cmaeraDistortion[0];
    rightDistortion[1]=mcameralist.pcameraList[1].cmaeraDistortion[1];
    rightDistortion[2]=mcameralist.pcameraList[1].cmaeraDistortion[2];
    rightDistortion[3]=mcameralist.pcameraList[1].cmaeraDistortion[3];
    rightDistortion[4]=mcameralist.pcameraList[1].cmaeraDistortion[4];
   //    //右相机旋转矩阵

    rightRotation[0][0] =mcameralist.pcameraList[1].cmaeraRotation[0][0];
    rightRotation[0][1] =mcameralist.pcameraList[1].cmaeraRotation[0][1];
    rightRotation[0][2] =mcameralist.pcameraList[1].cmaeraRotation[0][2];

    rightRotation[1][0] =mcameralist.pcameraList[1].cmaeraRotation[1][0];
    rightRotation[1][1] =mcameralist.pcameraList[1].cmaeraRotation[1][1];
    rightRotation[1][2] =mcameralist.pcameraList[1].cmaeraRotation[1][2];

    rightRotation[2][0] =mcameralist.pcameraList[1].cmaeraRotation[2][0];
    rightRotation[2][1] =mcameralist.pcameraList[1].cmaeraRotation[2][1];
    rightRotation[2][2] =mcameralist.pcameraList[1].cmaeraRotation[2][2];

    //    //右相机平移向量

    rightTranslation[0][0]= mcameralist.pcameraList[1].cmaeraTranslation[0][0];
    rightTranslation[0][1]= mcameralist.pcameraList[1].cmaeraTranslation[0][1];
    rightTranslation[0][2]= mcameralist.pcameraList[1].cmaeraTranslation[0][2];



    IntrinsicPl[0][0]=mcameralist.cmaeraIntrinsicP[0][0];
    IntrinsicPl[0][1]=mcameralist.cmaeraIntrinsicP[0][1];
    IntrinsicPl[0][2]=mcameralist.cmaeraIntrinsicP[0][2];

    IntrinsicPl[1][0]=mcameralist.cmaeraIntrinsicP[1][0];
    IntrinsicPl[1][1]=mcameralist.cmaeraIntrinsicP[1][1];
    IntrinsicPl[1][2]=mcameralist.cmaeraIntrinsicP[1][2];

    IntrinsicPl[2][0]=mcameralist.cmaeraIntrinsicP[2][0];
    IntrinsicPl[2][1]=mcameralist.cmaeraIntrinsicP[2][1];
    IntrinsicPl[2][2]=mcameralist.cmaeraIntrinsicP[2][2];



    rightRotationP[0][0] =mcameralist.cmaeraRotationP[0][0];
    rightRotationP[0][1] =mcameralist.cmaeraRotationP[0][1];
    rightRotationP[0][2] =mcameralist.cmaeraRotationP[0][2];

    rightRotationP[1][0] =mcameralist.cmaeraRotationP[1][0];
    rightRotationP[1][1] =mcameralist.cmaeraRotationP[1][1];
    rightRotationP[1][2] =mcameralist.cmaeraRotationP[1][2];

    rightRotationP[2][0] =mcameralist.cmaeraRotationP[2][0];
    rightRotationP[2][1] =mcameralist.cmaeraRotationP[2][1];
    rightRotationP[2][2] =mcameralist.cmaeraRotationP[2][2];

    //固定的
    leftRotationP[0][0]=1;
    leftRotationP[0][1]=0;
    leftRotationP[0][2]=0;

    leftRotationP[1][0]=0;
    leftRotationP[1][1]=1;
    leftRotationP[1][2]=0;

    leftRotationP[2][0]=0;
    leftRotationP[2][1]=0;
    leftRotationP[2][2]=1;

   // IntrinsicPl


    float mtmp= mcameralist.pcameraList[0].cmaeraIntrinsic[1][2]-mcameralist.pcameraList[1].cmaeraIntrinsic[1][2];

   // mabsLeftRightYdes=abs(mtmp);



    std::cout<<"-leftIntrinsic[][]:"<<leftIntrinsic[0][0]<<" , "<<leftIntrinsic[0][1]<<" , "<<leftIntrinsic[0][2]<<" , "
                                    <<leftIntrinsic[1][0]<<" , "<<leftIntrinsic[1][1]<<" , "<<leftIntrinsic[1][2]<<" , "
                                    <<leftIntrinsic[2][0]<<" , "<<leftIntrinsic[2][1]<<" , "<<leftIntrinsic[2][2]<<" , "
            <<std::endl;
    std::cout<<"-leftRotation[][]:"<<leftRotation[0][0]<<" , "<<leftRotation[0][1]<<" , "<<leftRotation[0][2]<<" , "
                                    <<leftRotation[1][0]<<" , "<<leftRotation[1][1]<<" , "<<leftRotation[1][2]<<" , "
                                    <<leftRotation[2][0]<<" , "<<leftRotation[2][1]<<" , "<<leftRotation[2][2]<<" , "
            <<std::endl;
     std::cout<<"-leftDistortion[]:"<<leftDistortion[0]<<" , "<<leftDistortion[1]<<" , "<<leftDistortion[2]<<" , "<<leftDistortion[1]<<" , "<<leftDistortion[2]<<" , "<<std::endl;


     std::cout<<"-leftTranslation[]:"<<leftTranslation[0][0]<<" , "<<leftTranslation[0][1]<<" , "<<leftTranslation[0][2]<<std::endl;



     std::cout<<"-rightIntrinsic[][]:"<<rightIntrinsic[0][0]<<" , "<<rightIntrinsic[0][1]<<" , "<<rightIntrinsic[0][2]<<" , "
                                    <<rightIntrinsic[1][0]<<" , "<<rightIntrinsic[1][1]<<" , "<<rightIntrinsic[1][2]<<" , "
                                    <<rightIntrinsic[2][0]<<" , "<<rightIntrinsic[2][1]<<" , "<<rightIntrinsic[2][2]<<" , "
            <<std::endl;
    std::cout<<"-rightRotation[][]:"<<rightRotation[0][0]<<" , "<<rightRotation[0][1]<<" , "<<rightRotation[0][2]<<" , "
                                    <<rightRotation[1][0]<<" , "<<rightRotation[1][1]<<" , "<<rightRotation[1][2]<<" , "
                                    <<rightRotation[2][0]<<" , "<<rightRotation[2][1]<<" , "<<rightRotation[2][2]<<" , "
            <<std::endl;
    std::cout<<"-rightDistortion[]:"<<rightDistortion[0]<<" , "<<rightDistortion[1]<<" , "<<rightDistortion[2]<<" , "<<rightDistortion[1]<<" , "<<rightDistortion[2]<<" , "<<std::endl;
    std::cout<<"-rightTranslation[]:"<<rightTranslation[0][0]<<" , "<<rightTranslation[0][1]<<" , "<<rightTranslation[0][2]<<std::endl;



}




/**
  雙目相機標定程序
  */

#include"hvideocapture.h"
#include "sterocameratwo.h"
#include <iostream>
#include <string>
#include <unistd.h>
//#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
typedef struct sbCameraPamera
{

    int cmaeraIndex;//0:left 1:right				////左相机内参数矩阵
    int imageWidth;                             //摄像头的分辨率
    int imageHeight;
    int boardWidth;                               //横向的角点数目
    int boardHeight;
    int frameNumber;                             //相机标定时需要采用的图像帧数  												  //const int squareSize = 15.47;
    float squareSize;  //标定板黑白格子的大小 单位mm
    int findsize;      //角点查找尺寸
    float cl1, cl2, cl3, cl4, cl5, cl6, cl7, cl8, cl9;
    float pl1, pl2, pl3, pl4, pl5;

    float cr1, cr2, cr3, cr4, cr5, cr6, cr7, cr8, cr9;
    float pr1, pr2, pr3, pr4, pr5;
                       //右相机内参数矩阵
    char caleftfpname[100];
    char carightpname[100];

}SBCAMERAPAMERA;
char  gEnergyFile[90];

SBCAMERAPAMERA ReadData(string filepath)  /////读取多边形数据
{
    SBCAMERAPAMERA merof;
    //int i, j, Tcount;
    FILE    *fp;
    char    stemp[100];
    strcpy(gEnergyFile, filepath.c_str());
    //strcpy(gEnergyFile,"cinploys.txt");
    fp = fopen(gEnergyFile, "r");
    if (fp == NULL)
    {
        printf("\n Can not open energy data file:%s\n", gEnergyFile);
        system("pause");

        exit(0);
    }
    fseek(fp, 0, SEEK_SET);
    fscanf(fp, "%s", stemp);
    while (strcmp(stemp, "CmaeraIndex:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.cmaeraIndex));

    ////////////////////////////

    while (strcmp(stemp, "imageWidth:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.imageWidth));

    while (strcmp(stemp, "imageHeight:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.imageHeight));

    while (strcmp(stemp, "boardWidth:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.boardWidth));

    while (strcmp(stemp, "boardHeight:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.boardHeight));

    while (strcmp(stemp, "frameNumber:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.frameNumber));

    while (strcmp(stemp, "squareSize:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%f", &(merof.squareSize));

    while (strcmp(stemp, "CameraLeftIntcs:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%f%f%f%f%f%f%f%f%f",
        &(merof.cl1), &(merof.cl2), &(merof.cl3),
        &(merof.cl4), &(merof.cl5), &(merof.cl6),
        &(merof.cl7), &(merof.cl8), &(merof.cl9));

    while (strcmp(stemp, "CameraLeftPcs:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%f%f%f%f%f",
        &(merof.pl1), &(merof.pl2), &(merof.pl3),
        &(merof.pl4), &(merof.pl5));

    while (strcmp(stemp, "CameraRightIntcs:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%f%f%f%f%f%f%f%f%f",
        &(merof.cr1), &(merof.cr2), &(merof.cr3),
        &(merof.cr4), &(merof.cr5), &(merof.cr6),
        &(merof.cr7), &(merof.cr8), &(merof.cr9));

    while (strcmp(stemp, "CameraRightPcs:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%f%f%f%f%f",
        &(merof.pr1), &(merof.pr2), &(merof.pr3),
        &(merof.pr4), &(merof.pr5));

    while (strcmp(stemp, "CameraLeftFileName:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%s", &(merof.caleftfpname));

    while (strcmp(stemp, "CameraRightFileName:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%s", &(merof.carightpname));

    while (strcmp(stemp, "CameraFindsize:") != 0)  fscanf(fp, "%s", stemp);
    fscanf(fp, "%d", &(merof.findsize));
    //CameraFindsize:

    //cout << merof.caleftfpname<< endl;
    fclose(fp);
    return merof;

}



std::string readCurrentPath()//读当前程序路径
{
    char path[100];
    getcwd(path,100);
    std::string filePath(path);
    return filePath;
}



Mat R, T, E, F;                                         //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
vector<Mat> rvecs;                                        //旋转向量
vector<Mat> tvecs;                                        //平移向量
vector<vector<Point2f>> imagePointL;                    //左边摄像机所有照片角点的坐标集合
vector<vector<Point2f>> imagePointR;                    //右边摄像机所有照片角点的坐标集合
vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合


vector<Point2f> cornerL;                              //左边摄像机某一照片角点坐标集合
vector<Point2f> cornerR;                              //右边摄像机某一照片角点坐标集合

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;

Mat Rl, Rr, Pl, Pr, Q;                                  //校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）
Mat mapLx, mapLy, mapRx, mapRy;                         //映射表
Rect validROIL, validROIR;                              //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Mat cameraMatrixL;                                      //单目标定的参数
Mat distCoeffL;                                         //单目标定的参数
Mat cameraMatrixR;                                      //单目标定的参数
Mat distCoeffR;                                         //单目标定的参数

int imageWidth ;//= mycamera.imageWidth;                             //摄像头的分辨率
int imageHeight;// = mycamera.imageHeight;


int boardWidth ;//= mycamera.boardWidth;                               //横向的角点数目
int boardHeight;// = mycamera.boardHeight;                              //纵向的角点数据
int boardCorner;// = boardWidth * boardHeight;       //总的角点数据
int frameNumber;//= mycamera.frameNumber;                             //相机标定时需要采用的图像帧数
                                                                    //const int squareSize = 15.47;
float squareSize;// = mycamera.squareSize;  //标定板黑白格子的大小 单位mm


int coorfindsize ;//= mycamera.findsize;


bool foundImageCorner(Size boardSize)
{

    bool isFindL, isFindR;

    isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL, 1);
    if(!isFindL)return false;
    isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR, 1);
    if(!isFindR)return false;

  if (isFindL == true && isFindR == true)
  {
      cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
      cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);

      cornerSubPix(grayImageL, cornerL, Size(coorfindsize, coorfindsize), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 40, 0.01));
      drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);

      cornerSubPix(grayImageR, cornerR, Size(coorfindsize, coorfindsize), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 40, 0.01));
      drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);

      return true;
  }

    return false;
}
////判断两组角点平均差异，如果差异不大就返回
float checkPointscornerdiff( vector<Point2f> cornerT,vector<Point2f> cornerK );
void outputCameraParam(void);
/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);
int main()
{
    ////////////读取标定参数///////////////
    bool isfixintrinsic = false;
    int camerafiletype = 0;
    SBCAMERAPAMERA mycamera;
    string camerafilepath = "/home/sunkit/文档/0udata/modules_test/grabDoubleImageyuyv/tcameraIndex.txt";
    cout << "请输入是否同意调整相机内参数，0否1是 ..." << endl;
    cin >> camerafiletype;

    if (camerafiletype == 0)
    {
        isfixintrinsic = false;
    }else
    {
        isfixintrinsic = true;
    }

     mycamera = ReadData(camerafilepath);
     ////读取单目标定成果
     cameraMatrixL = (Mat_<double>(3, 3) << mycamera.cl1, mycamera.cl2, mycamera.cl3,
         mycamera.cl4, mycamera.cl5, mycamera.cl6,
         mycamera.cl7, mycamera.cl8, mycamera.cl9);


     distCoeffL = (Mat_<double>(5, 1) << mycamera.pl1, mycamera.pl2, mycamera.pl3, -mycamera.pl4, mycamera.pl5);

      cameraMatrixR = (Mat_<double>(3, 3) << mycamera.cr1, mycamera.cr2, mycamera.cr3,
          mycamera.cr4, mycamera.cr5, mycamera.cr6,
          mycamera.cr7, mycamera.cr8, mycamera.cr9);
      distCoeffR = (Mat_<double>(5, 1) << mycamera.pr1, mycamera.pr2, mycamera.pr3, mycamera.pr4, mycamera.pr5);




     imageWidth = mycamera.imageWidth;                             //摄像头的分辨率
     imageHeight = mycamera.imageHeight;
     boardWidth = mycamera.boardWidth;                               //横向的角点数目
     boardHeight = mycamera.boardHeight;                              //纵向的角点数据
     boardCorner = boardWidth * boardHeight;       //总的角点数据
     frameNumber = mycamera.frameNumber;                             //相机标定时需要采用的图像帧数                                                                  //const int squareSize = 15.47;
     squareSize = mycamera.squareSize;  //标定板黑白格子的大小 单位mm
    Size boardSize = Size(boardWidth, boardHeight);   //
    Size imageSize = Size(imageWidth, imageHeight);
     coorfindsize = mycamera.findsize;
    double apha = -1;




    //////////初始化相机////////////
    steroCameraTwo mtwoCamera;


    string strVideo0=mycamera.caleftfpname;
    string strVideo1=mycamera.carightpname;

    int cameraWidth=mycamera.imageWidth;
    int cameraHeight=mycamera.imageHeight;
    bool isinitcameas=mtwoCamera.InitTwoCameras(strVideo0,strVideo1,cameraWidth,cameraHeight);


    if(isinitcameas)
    {

        cout <<"初始化相机成功"<<endl;


    }else
    {
        cout <<"初始化相机失败"<<endl;
        return 0;
    }
      cout <<"開始進行雙目標定，发现表顶板按p鍵拍照..."<<endl;
    int cpicindex=0;

    bool iscla=false;

    while (true)
    {
         mtwoCamera.getCamFramfirst();
        //grab and retrieve each frames of the video sequentially
         rgbImageR=mtwoCamera.getOrginalFrameRight();
         rgbImageL=mtwoCamera.getOrginalFrameLeft();

         Mat ksFramL,ksFramR;
         ksFramL=rgbImageL.clone();
         ksFramR=rgbImageR.clone();
        //wait for 40 milliseconds
        int c = cvWaitKey(40);
        //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
        if (27 == char(c)) break;
        if(char(c)=='c')//标定计算
        {

            if(imagePointL.size()>=frameNumber)//有足够的数量的标定数据
            {

                frameNumber=imagePointL.size();

                /*
                计算实际的校正点的三维坐标
                根据实际标定格子的大小来设置
                */
                calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
                cout << "cal real successful" << endl;
                /*
                标定摄像头
                由于左右摄像机分别都经过了单目标定
                所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
                */

                double rms = 0;
              //  if (isfixintrinsic)
                {
                    rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
                        cameraMatrixL, distCoeffL,
                        cameraMatrixR, distCoeffR,
                        Size(imageWidth, imageHeight), R, T, E, F,
                        CALIB_USE_INTRINSIC_GUESS,
                        TermCriteria(TermCriteria::EPS, 100, 1e-5));
                }
//                else
//                {
//                    rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
//                        cameraMatrixL, distCoeffL,
//                        cameraMatrixR, distCoeffR,
//                        Size(imageWidth, imageHeight), R, T, E, F,
//                        CV_CALIB_FIX_INTRINSIC,
//                        TermCriteria(TermCriteria::EPS, 100, 1e-5));

//                }



                cout << "Stereo Calibration done with RMS error = " << rms << endl;


                /*
                立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
                使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
                stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
                左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
                其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
                Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差
                */
                stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
                    CALIB_ZERO_DISPARITY, apha, imageSize, &validROIL, &validROIR);
                cout << "stereoRectify = "  << endl;
                /*
                根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
                mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
                ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
                所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
                */
                initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
                initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pl, imageSize, CV_32FC1, mapRx, mapRy);


                cout << "initUndistortRectifyMap = " << endl;
                iscla=true;
                 outputCameraParam();

             }




        }




       if(!iscla)//如果还没有标定就标定
       {

           if(foundImageCorner(boardSize))//如果找到
           {
               if(char(c)=='p')
               {
                   if(imagePointL.size()>0)
                   {
                     if(checkPointscornerdiff(imagePointL[imagePointL.size()-1],cornerL)>10)
                     {
                         imagePointL.push_back(cornerL);
                         imagePointR.push_back(cornerR);
                     }else
                     {
                         cout <<"图像角点移动过小，没有记录，<10pix"<<endl;
                         continue;
                     }

                   }else
                   {

                       imagePointL.push_back(cornerL);
                       imagePointR.push_back(cornerR);
                   }

                   cout <<"-->"<<char(c)<<endl;
                   cpicindex++;

                   stringstream ssL;
                   stringstream ssR;
                   string picleftpath;
                   string picrightpath;
                   if(cpicindex<10)
                   {
                       ssL<<"image-left-0"<<cpicindex<<".jpg";
                       ssR<<"image-right-0"<<cpicindex<<".jpg";
                   }else
                   {
                       ssL<<"image-left-"<<cpicindex<<".jpg";
                       ssR<<"image-right-"<<cpicindex<<".jpg";
                   }
                   ssL>>picleftpath;
                   ssR>>picrightpath;
                   ssL.clear();
                   ssR.clear();
                   imwrite(picleftpath,ksFramL);
                   imwrite(picrightpath,ksFramR);
                   cout << "Leftpic"<<picleftpath<<endl;
                   cout << "Rightpic"<<picrightpath<<endl;

                   if(frameNumber<cpicindex)
                   {
                       cout << "提示！当前已经满足标定条,请按C进行计算！"<<endl;

                   }

               }


           }



       }


       if(iscla)//如果完成标定
       {
        Mat rectifyImageL, rectifyImageR;

        remap(ksFramL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
        remap(ksFramR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

      //  imshow("rectifyImageL", rectifyImageL);
      //  imshow("rectifyImageR", rectifyImageR);

        Mat resizeLeft,resizeRight;
        Size m_size=rgbImageL.size();

        resize(rectifyImageL,resizeLeft,m_size/2);
        resize(rectifyImageR,resizeRight,m_size/2);


        Mat combinLRmat;
        hconcat(resizeLeft,resizeRight,combinLRmat);

        int combwidth=combinLRmat.size().width;
        int combheight=combinLRmat.size().height;

        for(int i=80;i<combheight;i=i+100)
        {
            line(combinLRmat,Point(0,i),Point(combwidth,i),Scalar(0,255,0),1,8);
        }

         cv::imshow("cameraLRremap", combinLRmat);

       }
       else
       {
           Mat resizeLeft,resizeRight;
           Size m_size=rgbImageL.size();

           resize(rgbImageL,resizeLeft,m_size/2);
           resize(rgbImageR,resizeRight,m_size/2);


           Mat combinLRmat;
           hconcat(resizeLeft,resizeRight,combinLRmat);


            cv::imshow("cameraLR", combinLRmat);
       }





    }





    return 0;
}

void outputCameraParam(void)
{
    /*保存数据*/

    /*输出数据*/
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
        fs.release();
        cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
    }
    else
    {
        cout << "Error: can not save the intrinsics!!!!!" << endl;
    }

    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs <<"E"<<E<<"F"<<F<< "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
        cout << "E" << E << "F" << F << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";
}


////判断两组角点平均差异，如果差异不大就返回
/// float checkPointscornerdiff( vector<Point2f> cornerT,vector<Point2f> cornerK );
float checkPointscornerdiff( vector<Point2f> cornerT,vector<Point2f> cornerK )
{

    if(cornerT.size()!=cornerK.size())
    {
        std::cout<<"异常，两组角点数量不一致！"<<std::endl;
        return -1;
    }

    if(cornerT.size()==0)
    {
         std::cout<<"异常，角点数量为0！"<<std::endl;
        return -2;
    }
    Point2f ptv;

    float tmap=0;

    for(int i=0;i<cornerT.size();i++)
    {
        ptv=cornerT[i]-cornerK[i];

        tmap+=(abs(ptv.x)+abs(ptv.y));
    }

    return tmap/cornerT.size();

}
/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
    //  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));
    vector<Point3f> imgpoint;
    for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
    {
        for (int colIndex = 0; colIndex < boardwidth; colIndex++)
        {
            //  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);
            imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
        }
    }
    for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
    {
        obj.push_back(imgpoint);
    }
}

/*
 *  calibration_wukong.cpp
 *  calibration
 *
 *  Created by Wukong   on 6/11/17.
 *  Copyright 2017 wukong Corp. All rights reserved.
 *
 */
// stereoCalibration.cpp : 定义控制台应用程序的入口点。
//
//在进行双目摄像头的标定之前，最好事先分别对两个摄像头进行单目视觉的标定
//分别确定两个摄像头的内参矩阵，然后再开始进行双目摄像头的标定
//在此例程中是先对两个摄像头进行单独标定(见上一篇单目标定文章)，然后在进行立体标定

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>


#define KEY_A       1048673
using namespace std;
using namespace cv;

const int imageWidth = 640;                             //摄像头的分辨率
const int imageHeight = 480;
const int boardWidth = 11;                               //横向的角点数目
const int boardHeight = 8;                              //纵向的角点数据
const int boardCorner = boardWidth * boardHeight;       //总的角点数据
const int frameNumber = 13;                             //相机标定时需要采用的图像帧数
const double squareSize = 0.025;                              //标定板黑白格子的大小 单位mm
const Size boardSize = Size(boardWidth, boardHeight);   //
Size imageSize = Size(imageWidth, imageHeight);
vector<vector<Point2f> > imagePointL;                    //左边摄像机所有照片角点的坐标集合
vector<vector<Point3f> > objRealPoint;                   //各副图像的角点的实际物理坐标集合
vector<Point2f> cornerL;                              //左边摄像机某一照片角点坐标集合
Mat cameraMatrix ;
Mat distCoeff ;



/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f> >& obj, int boardwidth, int boardheight, int imgNumber, double squaresize)
{
  //  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));
  vector<Point3f> imgpoint;
  for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
  {
    for (int colIndex = 0; colIndex < boardwidth; colIndex++)
    {
      //  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);
//      imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
      imgpoint.push_back(Point3f(colIndex * squaresize, rowIndex * squaresize, 0));

    }
  }

  for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
  {
    obj.push_back(imgpoint);
  }

}



void realsense_calibration(  vector<Mat>& rvecs,vector<Mat>& tvecs,vector<Mat>& pics_rgb)
{

  int goodFrameCount = 0;

  vector<Mat> pics_gray(frameNumber);
  for(int i=0;i<frameNumber;i++){
      cvtColor(pics_rgb[i],pics_gray[i],CV_BGR2GRAY);
  }

  int num=0;

  while (goodFrameCount < frameNumber)
  {
    bool isFind;
    isFind = findChessboardCorners(pics_rgb[num], boardSize, cornerL,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
    cout<<"isfindl"<<isFind<<endl;
    if (isFind == true )
    {
      /*
            Size(5,5) 搜索窗口的一半大小
            Size(-1,-1) 死区的一半尺寸
            TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
            */
      cornerSubPix(pics_gray[num], cornerL, Size(5, 5), Size(-1, -1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
      drawChessboardCorners(pics_rgb[num], boardSize, cornerL, isFind);
      imshow("chessboard", pics_rgb[num]);
      imagePointL.push_back(cornerL);
      num++;
      goodFrameCount++;
      cout << "The image is good" << endl;
    }
    else
    {
      cout << "The image "<<num+1<< " is bad please try again" << endl;
    }

  }

  /*
    计算实际的校正点的三维坐标
    根据实际标定格子的大小来设置
    */
  calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
  cout << "objRealPoint cal real successful" << endl;

  double rms;
  rms = calibrateCamera(objRealPoint, imagePointL, imageSize, cameraMatrix, distCoeff, rvecs, tvecs,CV_CALIB_FIX_K3);

//cvFindExtrinsicCameraParams2();


  vector<Point3f> objpoint;
  objpoint.push_back(Point3f(0,0,0));
  objpoint.push_back(Point3f(0,8*squareSize,0));
  objpoint.push_back(Point3f(5*squareSize,0,0));
  objpoint.push_back(Point3f(8*squareSize,5*squareSize,0));


  cout<<"objRealPoint"<<endl<<objRealPoint.size()<<endl;
//  double obj_corner[] ={
//  CvMat mat = cvMat(4,1,obj_corner)
//  cvFindExtrinsicCameraParams2();


  cout<<"Matrix"<<std::endl;
  cout<<cameraMatrix.at<double>(0,0)<<"\t"<<cameraMatrix.at<double>(0,1)<<"\t"<<cameraMatrix.at<double>(0,2)<<std::endl
      <<cameraMatrix.at<double>(1,0)<<"\t"<<cameraMatrix.at<double>(1,1)<<"\t"<<cameraMatrix.at<double>(1,2)<<std::endl
      <<cameraMatrix.at<double>(2,0)<<"\t"<<cameraMatrix.at<double>(2,1)<<"\t"<<cameraMatrix.at<double>(2,2)<<std::endl;
  cout<<"rms"<<rms<<endl;
//  cout<<"rvecs"<<rvecs.size()<<endl;
  cout<<"tvecs[0]"<<endl<<tvecs[0]<<endl;
//  cout<<"discoffes"<<endl<<distCoeff<<endl;
//  vector<Mat> ro33(13);
//  for(int i=0;i<13;++i){
//      Rodrigues(rvecs[i],ro33[i]);
//  }

//  typedef Eigen::Matrix<double,3,3> Matrix33d;

//  Matrix33d mm;
//    mm<<1,2,3,
//        4,5,6,
//        7,8,9;
////mat 2 matrix
//    vector<Matrix33d> ro33_m(13,mm);
//    for(int i = 0;i<13;++i){
//        cv2eigen(ro33[i],ro33_m[i]);

//    }

//    cout<<"ro33"<<endl<<ro33[0]<<endl;
//    cout<<"ro33_m"<<endl<<ro33_m[0]<<endl;

//    vector<Eigen::Vector3d> trans(13);
//    for(int i= 0;i<13;++i){
//        cv2eigen(tvecs[i],trans[i]);
//    }

//    cout<<"tvecs[0]"<<endl<<tvecs[0]<<endl;
//    cout<<"trans[0]"<<endl<<trans[0]<<endl;

//    vector<Eigen::Matrix4d> cam2board(13);

//    for(int i=0;i<13;++i){
//        cam2board[i] = Eigen::Matrix4d::Identity();
//        cam2board[i].block<3,3>(0,0) = ro33_m[i];
//        cam2board[i].block<3,1>(0,3) = trans[i];
//    }

//    cout<<"cam2board"<<endl<<cam2board[5]<<endl;
//    return cam2board;

}

/*
 *  calibration_wukong.cpp
 *  calibration
 *
 *  Created by Wukong   on 6/11/17.
 *  Copyright 2017  Corp. All rights reserved.
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/Transform.h>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace std;
using namespace cv;

class HandEyeCalibration{
public:

  HandEyeCalibration(){}
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

  void realsense_calibration(  vector<Mat>& rvecs,vector<Mat>& tvecs,vector<Mat>& pics_rgb,double frameNumber)
  {

    vector<vector<Point2f> > imagePointL;                    //摄像机所有照片角点的坐标集合
    vector<Point2f> cornerL;                              //摄像机某一照片角点坐标集合
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
      cout<<"isfind"<<isFind<<endl;
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
        cout << "The image "
             <<num+1<< " is bad please ensure the board in the middle of vision and try again"
             << endl;
      }

    }
    /*
      计算实际的校正点的三维坐标
      根据实际标定格子的大小来设置
      */
    vector<vector<Point3f> > objRealPoint;
    calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
    cout << "objRealPoint cal real successful" << endl;

    double rms;
    rms = calibrateCamera(objRealPoint, imagePointL, imageSize, cameraMatrix, distCoeff, rvecs, tvecs,CV_CALIB_FIX_K3);

    cout<<"objRealPoint"<<endl<<objRealPoint.size()<<endl;
    cout<<"imagePointL"<<endl<<imagePointL.size()<<endl;

    cout<<"Matrix"<<std::endl;
    cout<<cameraMatrix.at<double>(0,0)<<"\t"<<cameraMatrix.at<double>(0,1)<<"\t"<<cameraMatrix.at<double>(0,2)<<std::endl
        <<cameraMatrix.at<double>(1,0)<<"\t"<<cameraMatrix.at<double>(1,1)<<"\t"<<cameraMatrix.at<double>(1,2)<<std::endl
        <<cameraMatrix.at<double>(2,0)<<"\t"<<cameraMatrix.at<double>(2,1)<<"\t"<<cameraMatrix.at<double>(2,2)<<std::endl;
    cout<<"rms"<<rms<<endl;
    cout<<"tvecs[0]"<<endl<<tvecs[0]<<endl;


  }

  /*********************************
  函数名：  PrintMat(CvMat *matrix)
  函数输入：matrix指针 数据类型opencv规定的任意一个
  函数作用：在屏幕上打印矩阵
  **********************************/
  void PrintMat(CvMat *matrix, bool save_or_show =false,FILE *fp=NULL)
  {
      int i=0;
      int j=0;
      for(i=0;i < matrix->rows;i++)//行
      {
          if (save_or_show)
          {
              fprintf(fp,"\n");
          }
          else
          {
              printf("\n");
          }
          switch(matrix->type&0X07)
          {
          case CV_32F:
          case CV_64F:
              {
                  for(j=0;j<matrix->cols;j++)//列
                  {
                      if (save_or_show)
                      {
                          fprintf(fp,"%9.3f ",(float)cvGetReal2D(matrix,i,j));
                      }
                      else
                      {
                          printf("%9.3f ",(float)cvGetReal2D(matrix,i,j));
                      }
                  }
                  break;
              }
          case CV_8U:
          case CV_16U:
              {
                  for(j=0;j<matrix->cols;j++)
                  {
                      printf("%6d  ",(int)cvGetReal2D(matrix,i,j));
                      if (save_or_show)
                      {
                          fprintf(fp,"%6d  ",(int)cvGetReal2D(matrix,i,j));
                      }
                      else
                      {
                          printf("%6d  ",(int)cvGetReal2D(matrix,i,j));
                      }
                  }
                  break;
              }
          default:
              break;
          }
      }
  }
  //*****************************

  void calboardcenter(Mat pic_,geometry_msgs::Transform& center2cam){


      vector<Point2f> corner;

      Mat pic_gray;
      cvtColor(pic_,pic_gray,CV_BGR2GRAY);

      bool isFind;
      isFind = findChessboardCorners(pic_, boardSize, corner,
               CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
      cout<<"isfindl"<<isFind<<endl;
      if (isFind == true )
      {
           cornerSubPix(pic_gray, corner, Size(5, 5), Size(-1, -1),
                     TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
      }


  //    cout<<"corner"<<corner<<endl;


      int board_n = boardWidth*boardHeight;
      CvMat* image_points  = cvCreateMat(board_n,2,CV_32FC1);
      CvMat* object_points = cvCreateMat(board_n,3,CV_32FC1);
      CvMat* rotation_vector=cvCreateMat(3,1,CV_32FC1);
      CvMat* translation_vector=cvCreateMat(3,1,CV_32FC1);
      CvMat* cameraMatrix = cvCreateMat(3,3,CV_32FC1);
      CvMat* distCoeff = cvCreateMat(1,5,CV_32FC1);

      CV_MAT_ELEM(*cameraMatrix,float,0,0) =474.8207092285156;
      CV_MAT_ELEM(*cameraMatrix,float,0,1) =0.0;
      CV_MAT_ELEM(*cameraMatrix,float,0,2) =318.01226806640625;
      CV_MAT_ELEM(*cameraMatrix,float,1,0) =0.0;
      CV_MAT_ELEM(*cameraMatrix,float,1,1) =474.8207092285156;
      CV_MAT_ELEM(*cameraMatrix,float,1,2) =244.9693145751953;
      CV_MAT_ELEM(*cameraMatrix,float,2,0) =0.0;
      CV_MAT_ELEM(*cameraMatrix,float,2,1) =0.0;
      CV_MAT_ELEM(*cameraMatrix,float,2,2) =1.0;

      CV_MAT_ELEM(*distCoeff,float,0,0) =0.13163471221923828;
      CV_MAT_ELEM(*distCoeff,float,0,1) =0.09715849161148071;
      CV_MAT_ELEM(*distCoeff,float,0,2) =0.005144651047885418;
      CV_MAT_ELEM(*distCoeff,float,0,3) =0.004264745861291885;
      CV_MAT_ELEM(*distCoeff,float,0,4) =0.045015204697847366;

      for( int i=0; i<board_n; ++i )
        {
         CV_MAT_ELEM(*image_points, float,i,0) = corner[i].x;
         CV_MAT_ELEM(*image_points, float,i,1) = corner[i].y;
         CV_MAT_ELEM(*object_points,float,i,0) = (i%boardWidth)*squareSize;
         CV_MAT_ELEM(*object_points,float,i,1) = (i/boardWidth)*squareSize;
         CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
        }
      cvFindExtrinsicCameraParams2(object_points,image_points,cameraMatrix,distCoeff,
                                   rotation_vector,translation_vector);
  //cout translation_vector
      cout<<"left_up"<<endl;
      PrintMat(translation_vector);
      cout<<endl;
      CvMat* r33 = cvCreateMat(3,3,CV_32FC1);

      cvRodrigues2(rotation_vector,r33);

  //get the translation
      CvMat* center2cam_t=cvCreateMat(3,1,CV_32FC1);
      CvMat* center2cam_ =cvCreateMat(3,1,CV_32FC1);
      CV_MAT_ELEM(*center2cam_, float,0,0) = 0.125;
      CV_MAT_ELEM(*center2cam_, float,1,0) = (0.025*7)/2;
      CV_MAT_ELEM(*center2cam_, float,2,0) = 0;
      cvMatMul(r33,center2cam_,center2cam_t);
      cvAdd(center2cam_t,translation_vector,center2cam_t);

      //cout
      cout<<"center2cam_CVMAT"<<endl;
      PrintMat(center2cam_t);
      cout<<endl;

  //get the rotate
      Eigen::Matrix3d rotate_m;
      rotate_m<<CV_MAT_ELEM(*r33, float,0,0),CV_MAT_ELEM(*r33, float,0,1),CV_MAT_ELEM(*r33, float,0,2),
                CV_MAT_ELEM(*r33, float,1,0),CV_MAT_ELEM(*r33, float,1,1),CV_MAT_ELEM(*r33, float,1,2),
                CV_MAT_ELEM(*r33, float,2,0),CV_MAT_ELEM(*r33, float,2,1),CV_MAT_ELEM(*r33, float,2,2);

      Eigen::Quaterniond tt = Eigen::Quaterniond(rotate_m);
      typedef Eigen::Matrix<double,4,1> matrix41d;
      matrix41d coeffs;
      coeffs = tt.coeffs();

      center2cam.rotation.x = coeffs(0);
      center2cam.rotation.y = coeffs(1);
      center2cam.rotation.z = coeffs(2);
      center2cam.rotation.w = coeffs(3);
      center2cam.translation.x = CV_MAT_ELEM(*center2cam_t, float,0,0);
      center2cam.translation.y = CV_MAT_ELEM(*center2cam_t, float,1,0);
      center2cam.translation.z = CV_MAT_ELEM(*center2cam_t, float,2,0);

      cout<<"center2cam_geometry_msg"<<endl
                        <<center2cam.translation.x<<" "
                        <<center2cam.translation.y<<" "
                        <<center2cam.translation.z<<" "
                        <<endl;

  }

  void boardinpic(Mat pic_,double& ratio){
      vector<Point2f> corner;

      Mat pic_gray;
      cvtColor(pic_,pic_gray,CV_BGR2GRAY);

      bool isFind;
      isFind = findChessboardCorners(pic_, boardSize, corner,
               CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
      cout<<"isfindl"<<isFind<<endl;
      if (isFind == true )
      {
           cornerSubPix(pic_gray, corner, Size(5, 5), Size(-1, -1),
                     TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
      }
      double corner_area,a,b;
      a = sqrt(pow(fabs(corner[0].x-corner[10].x),2)+pow(fabs(corner[0].y-corner[10].y),2));
      b = sqrt(pow(fabs(corner[0].x-corner[77].x),2)+pow(fabs(corner[0].y-corner[77].y),2));
      corner_area = a*b;
      ratio = corner_area/(640*480);

  }

  void calibrationTsai(std::vector<vpHomogeneousMatrix>& cMo,
                       std::vector<vpHomogeneousMatrix>& rMe,
                       vpHomogeneousMatrix &eMc){

      vpColVector x ;
      unsigned int nbPose = (unsigned int)cMo.size();
      if(cMo.size()!=rMe.size()){
        cout<<"error:cMo and rMe have different sizes"<<endl;
      }
      {
          vpMatrix A ;
          vpColVector B ;
          unsigned int k = 0 ;
          for (unsigned int i=0 ; i < nbPose ; i++)
          {
              vpRotationMatrix rRei, ciRo ;
              rMe[i].extract(rRei) ;
              cMo[i].extract(ciRo) ;

              for (unsigned int j=0 ; j < nbPose ; j++)
              {
                  if (j>i)
                  {
                      vpRotationMatrix rRej, cjRo ;
                      rMe[j].extract(rRej) ;
                      cMo[j].extract(cjRo) ;

                      vpRotationMatrix rReij = rRej.t() * rRei;

                      vpRotationMatrix cijRo = cjRo * ciRo.t();

                      vpThetaUVector rPeij(rReij);

                      double theta = sqrt(rPeij[0]*rPeij[0] + rPeij[1]*rPeij[1]
                              + rPeij[2]*rPeij[2]);

                      for (unsigned int m=0;m<3;m++) rPeij[m] = rPeij[m] * vpMath::sinc(theta/2);

                      vpThetaUVector cijPo(cijRo) ;
                      theta = sqrt(cijPo[0]*cijPo[0] + cijPo[1]*cijPo[1]
                              + cijPo[2]*cijPo[2]);
                      for (unsigned int m=0;m<3;m++) cijPo[m] = cijPo[m] * vpMath::sinc(theta/2);

                      vpMatrix As;
                      vpColVector b(3) ;

                      As = vpColVector::skew(vpColVector(rPeij) + vpColVector(cijPo)) ;

                      b =  (vpColVector)cijPo - (vpColVector)rPeij ;           // A.40

                      if (k==0)
                      {
                          A = As ;
                          B = b ;
                      }
                      else
                      {
                          A = vpMatrix::stack(A,As) ;
                          B = vpColVector::stack(B,b) ;
                      }
                      k++ ;
                  }
              }
          }


          vpMatrix AtA = A.AtA() ;

          vpMatrix Ap ;
          AtA.pseudoInverse(Ap, 1e-6) ;
          x = Ap*A.t()*B ;
          double theta ;
          double   d=x.sumSquare() ;
          for (unsigned int i=0 ; i < 3 ; i++) x[i] = 2*x[i]/sqrt(1+d) ;
          theta = sqrt(x.sumSquare())/2 ;
          theta = 2*asin(theta) ;
          if (std::fabs(theta) > std::numeric_limits<double>::epsilon())
          {
              for (unsigned int i=0 ; i < 3 ; i++) x[i] *= theta/(2*sin(theta/2)) ;
          }
          else
              x = 0 ;
      }

      vpThetaUVector xP(x[0],x[1],x[2]);
      vpRotationMatrix eRc(xP);

      {
          vpMatrix A ;
          vpColVector B ;
          vpRotationMatrix I3 ;
          I3.eye() ;
          int k = 0 ;
          for (unsigned int i=0 ; i < nbPose ; i++)
          {
              vpRotationMatrix rRei, ciRo ;
              vpTranslationVector rTei, ciTo ;
              rMe[i].extract(rRei) ;
              cMo[i].extract(ciRo) ;
              rMe[i].extract(rTei) ;
              cMo[i].extract(ciTo) ;


              for (unsigned int j=0 ; j < nbPose ; j++)
              {
                  if (j>i)
                  {

                      vpRotationMatrix rRej, cjRo ;
                      rMe[j].extract(rRej) ;
                      cMo[j].extract(cjRo) ;

                      vpTranslationVector rTej, cjTo ;
                      rMe[j].extract(rTej) ;
                      cMo[j].extract(cjTo) ;

                      vpRotationMatrix rReij = rRej.t() * rRei ;

                      vpTranslationVector rTeij = rTej+ (-rTei);

                      rTeij = rRej.t()*rTeij ;

                      vpMatrix a = vpMatrix(rReij) - vpMatrix(I3);

                      vpTranslationVector b ;
                      b = eRc*cjTo - rReij*eRc*ciTo + rTeij ;

                      if (k==0)
                      {
                          A = a ;
                          B = b ;
                      }
                      else
                      {
                          A = vpMatrix::stack(A,a) ;
                          B = vpColVector::stack(B,b) ;
                      }
                      k++ ;
                  }
              }
          }
          vpMatrix AtA = A.AtA() ;
          vpMatrix Ap ;
          vpColVector AeTc ;
          AtA.pseudoInverse(Ap, 1e-6) ;
          AeTc = Ap*A.t()*B ;

          vpTranslationVector eTc(AeTc[0],AeTc[1],AeTc[2]);

          eMc.insert(eTc) ;
          eMc.insert(eRc) ;
      }
  }

  ~HandEyeCalibration(){}

private:

  const int imageWidth = 640;                             //摄像头的分辨率
  const int imageHeight = 480;
  const int boardWidth = 11;                               //横向的角点数目
  const int boardHeight = 8;                              //纵向的角点数据
  const int boardCorner = boardWidth * boardHeight;       //总的角点数据
  const double squareSize = 0.025;                              //标定板黑白格子的大小 单位mm
  const Size boardSize = Size(boardWidth, boardHeight);   //
  Size imageSize = Size(imageWidth, imageHeight);

  Mat cameraMatrix ;
  Mat distCoeff ;

};

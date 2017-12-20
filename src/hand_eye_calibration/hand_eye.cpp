#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <visp/vpHomogeneousMatrix.h>
#include "realsense_calibration.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//#include <librealsense/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
//#include <hand_eye_calibration/pose_element.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visp_bridge/3dpose.h>
#include <hand_eye_calibration/shooting_pose.h>
#include <hand_eye_calibration/shooting_poseRequest.h>
#include <hand_eye_calibration/shooting_poseResponse.h>

using namespace cv;
using namespace std;
using namespace visp_bridge;

#define KEY_ESC     1048603
#define KEY_A       1048673
#define KEY_S       115
#define KEY_D       1048676

int num = 0;
int frame_num =13;
bool capture_flag = false;
Eigen::Matrix4d temp_T;
cv_bridge::CvImagePtr cv_ptr;
boost::shared_ptr<pcl::PCLPointCloud2> pcloud(new pcl::PCLPointCloud2);
vector<geometry_msgs::Transform> shooting_pose;
vector<Mat> pics_rgb(13);
static int couter=0;

void CallBack_rgbImage_PointCoude_sensor(sensor_msgs::PointCloud2 pt2)
{

  pcl_conversions::toPCL(pt2, *pcloud);
  //  std::cout<<"width: "<<pcloud->width<<std::endl
  //            <<"height: "<<pcloud->height<<std::endl
  //            <<"stamp: "<<pcloud->header.stamp<<std::endl
  //            <<"dense: "<<pcloud->is_dense<<std::endl;

}

void CallBack_rgbImage_raw_sensor(sensor_msgs::Image img)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}
void ChatterCallBack_cmd(geometry_msgs::TransformStamped T )

{
    geometry_msgs::Quaternion q;
    q=T.transform.rotation;
    Eigen::Quaterniond qq(q.w,q.x,q.y,q.z);
    Eigen::Matrix3d data;
    data = qq.toRotationMatrix();
    temp_T.block<3,3>(0,0)<<data;
    temp_T.block<3,1>(0,3)<<T.transform.translation.x,T.transform.translation.y,T.transform.translation.z;
    capture_flag = true;

}


void calibrationTsai(std::vector<vpHomogeneousMatrix>& cMo,
                     std::vector<vpHomogeneousMatrix>& rMe,
                     vpHomogeneousMatrix &eMc);
bool capture_pose(hand_eye_calibration::shooting_pose::Request &req,
                  hand_eye_calibration::shooting_pose::Response &res){

    cout<<"capture_pose"<<endl;
    capture_flag =true;
    cout<<"capture_pose1"<<endl;

//    shooting_pose.push_back(req.T);

    if(cv_ptr!=NULL&&capture_flag == true){
        capture_flag = false;
        shooting_pose.push_back(req.T);

        cout<<"capture_pose3"<<endl;
        cout<<"req.T"<<req.T<<endl;
        //store image
        char filename[30];
        sprintf(filename, "/home/wsj/buffer/pic/%d.jpg", num);
        IplImage  tmp = IplImage(cv_ptr->image);
        cvSaveImage(filename, &tmp);
        cout<<"capture_pose4"<<endl;

        pics_rgb[couter] = cv_ptr->image;
        couter++;
        num++;

    }
    if(couter>=13){
        couter =13;
    }
    cout<<"capture_pose5"<<endl;
//    sleep(5);
    res.result =true;
    cout<<"capture_pose6"<<endl;

    return true;

}

int main(int argc, char *argv[])
{
    // initialise the node
    ros::init(argc, argv, "ros_capture_tt");
    ros::NodeHandle n;

//    tf::TransformListener listener(ros::Duration(10));
    tf::TransformBroadcaster br;
    ros::Subscriber chatter_sub_getPic = n.subscribe("/camera/rgb/image_raw",
                                                         20, CallBack_rgbImage_raw_sensor);
    ros::Subscriber chatter_sub_getPicCloud = n.subscribe("/camera/depth_registered/points",
                                                          20, CallBack_rgbImage_PointCoude_sensor);
//    ros::Subscriber chatter_sub_cmd = n.subscribe("/capture/cmd",100,ChatterCallBack_cmd);

//    ros::Publisher chatter_pub_cam2end = n.advertise<geometry_msgs::Transform>("/cam2end",100);
    ros::ServiceServer service = n.advertiseService("shooting_pose",capture_pose);
    ros::Rate r(100);

    vector<Mat> rvecs;
    vector<Mat> tvecs;
    bool calibrate_flag =true;
    bool br_flag =false;
    tf::Transform cam2end_tf;


    while(ros::ok()){

        if(couter == 13&&calibrate_flag == true){
            cout<<"in while ros::ok"<<endl;
            realsense_calibration(rvecs,tvecs,pics_rgb);

            vpHomogeneousMatrix cMo;//cam
            vpHomogeneousMatrix wMe;//end
            vector<vpHomogeneousMatrix> vcMo;
            vector<vpHomogeneousMatrix> vwMe;
            for (int i = 0; i < rvecs.size(); i++)
            {

                cMo.buildFrom(tvecs[i].at<double>(0,0),
                              tvecs[i].at<double>(1,0),
                              tvecs[i].at<double>(2.0),
                              rvecs[i].at<double>(0,0),
                              rvecs[i].at<double>(1,0),
                              rvecs[i].at<double>(2.0));//cam
        //        wMe.buildFrom(t[i]);//end
                wMe = visp_bridge::toVispHomogeneousMatrix(shooting_pose[i]);//end
                cout<<"wMe"<<endl<<wMe<<endl;
                vcMo.push_back(cMo);
                vwMe.push_back(wMe);
            }

            vpHomogeneousMatrix eMc;
            calibrationTsai(vcMo, vwMe, eMc);
            std::cout<<"eMc"<<endl<<eMc<<std::endl;

            // to transform
            geometry_msgs::Transform cam2end;
            cam2end = visp_bridge::toGeometryMsgsTransform(eMc);
            tf::transformMsgToTF(cam2end,cam2end_tf);
            br_flag =true;
            calibrate_flag =false;

        }

        if(br_flag == true){
            br.sendTransform(tf::StampedTransform(cam2end_tf,ros::Time::now(),"end_tcp","cam_end"));
//            ROS_INFO("boardcast the cam2end");
        }


        ros::spinOnce();

    }
    return 0;
}




void calibrationTsai(std::vector<vpHomogeneousMatrix>& cMo,
                     std::vector<vpHomogeneousMatrix>& rMe,
                     vpHomogeneousMatrix &eMc){

    vpColVector x ;
    unsigned int nbPose = (unsigned int)cMo.size();
    if(cMo.size()!=rMe.size());
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

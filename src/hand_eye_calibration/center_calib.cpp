#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <visp/vpHomogeneousMatrix.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
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
#include "center_calib.h"
#include <hand_eye_calibration/find_center.h>


using namespace cv;
using namespace std;
using namespace visp_bridge;

int ww =0;
int num_ = 0;
int counter_clibration = 0;
const int frame_num =13;
bool isfindcenter = false;
bool first_calibration = true;
bool sec_calibration = true;
bool height_flag_first = true;
bool height_flag_final =true;
bool height_flag_sec = true;
bool capture_flag = true;
bool calibrate_before_flag = false;
bool calibrate_flag =false;
bool br_flag =false;
bool cacu_plane = false;
cv_bridge::CvImagePtr cv_ptr;
boost::shared_ptr<pcl::PCLPointCloud2> pcloud(new pcl::PCLPointCloud2);
vector<geometry_msgs::Transform> shooting_pose_first;
vector<geometry_msgs::Transform> shooting_pose_;
vector<geometry_msgs::Transform> shooting_pose;
vector<Mat> pics_rgb_first;
vector<Mat> pics_rgb_;
vector<Mat> pics_rgb(frame_num);
static int couter=0;
vector<Mat> rvecs_first;
vector<Mat> tvecs_first;
vector<Mat> rvecs_;
vector<Mat> tvecs_;

vector<Mat> rvecs;
vector<Mat> tvecs;

tf::Transform cam2end_gloabl_first;//first
tf::Transform cam2end_tf_;//second

tf::Transform cam2end_tf;
Mat pic_last;
geometry_msgs::Transform center2cam_last;

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

bool capture_pose(hand_eye_calibration::shooting_pose::Request &req,
                  hand_eye_calibration::shooting_pose::Response &res){


    cout<<"capture_pose: "<<counter_clibration<<endl;
    capture_flag =true;

    if(cv_ptr!=NULL&&counter_clibration>=0&&counter_clibration<=frame_num-1){
        capture_flag = false;
        shooting_pose.push_back(req.T);

        //store image
        char filename[30];
        sprintf(filename, "/home/wukong/buffer/pic/%d.jpg", num_);
        IplImage  tmp = IplImage(cv_ptr->image);
        cvSaveImage(filename, &tmp);
        pics_rgb[counter_clibration] = cv_ptr->image;
        counter_clibration++;
        num_++;
        res.result =true;

    }

    if(counter_clibration == frame_num){
        cout<<"2222"<<endl;
        calibrate_flag = true;
        counter_clibration++;
        res.result =true;

    }
    if( cacu_plane == true){
        tf::Transform t1,t2,shooting_pose_last,center2cam_last_tf;
        tf::transformMsgToTF(shooting_pose[frame_num-1],shooting_pose_last);
        tf::transformMsgToTF(center2cam_last,center2cam_last_tf);
        t1.mult(shooting_pose_last,cam2end_tf);
        t2.mult(t1,center2cam_last_tf);
        tf::Vector3 n(0,0,1),n_board;

        tf::Matrix3x3 rotate;
        rotate =  t2.getBasis();
        n_board = n*rotate;
        double e1,e2,e3,e4;
        e1 = n_board.getX();
        e2 = n_board.getY();
        e3 = n_board.getZ();
        e4 = -n_board.getX()*(t2.getOrigin().getX())
                    -n_board.getY()*(t2.getOrigin().getY())
                    -n_board.getZ()*(t2.getOrigin().getZ());
        res.result =false;
        res.plane[0] = e1;
        res.plane[1] = e2;
        res.plane[2] = e3;
        res.plane[3] = e4;
        double d;
        d = fabs(e1*shooting_pose[frame_num-1].translation.x
            +e2*shooting_pose[frame_num-1].translation.y
            +e3*shooting_pose[frame_num-1].translation.z+e4)/sqrt(pow(e1,2)+pow(e2,2)+pow(e3,2));
        cout<<"height"<<d<<endl;
    }


    return true;

}

int main(int argc, char *argv[])
{
    // initialise the node
    ros::init(argc, argv, "ros_capture_tt");
    ros::NodeHandle n;

//    tf::TransformListener listener(ros::Duration(10));
    tf::TransformBroadcaster br;
    ros::Subscriber chatter_sub_getPic = n.subscribe("/camera/rgb/image_raw",20, CallBack_rgbImage_raw_sensor);
    ros::ServiceServer service = n.advertiseService("shooting_pose",capture_pose);
    ros::ServiceClient client = n.serviceClient<hand_eye_calibration::find_center>("find_center");
    hand_eye_calibration::find_center srv;
    HandEyeCalibration hecobject;
    ros::Rate r(100);

    cout<<"start to find center..."<<endl;
    while(ros::ok()&&isfindcenter == true){
        if(cv_ptr!=NULL){
            cout<<"got image"<<endl;
            //height adaption
            while(height_flag_first == true&&ros::ok()){
                cout<<"z-aix-model"<<endl;
                Mat pic_first;
                sleep(0.5);

                for(int i= 0;i<100;++i){
                    ros::spinOnce();
                }
                pic_first = cv_ptr->image;
                double ratio;
                hecobject.boardinpic(pic_first,ratio);
                cout<<"ratio:"<<ratio<<endl;
                //too high
                if(ratio<0.12){
                    srv.request.t_vary.translation.x= 0.01;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =1;
                    srv.request.is_center_over = true;
                    if(client.call(srv)){
                        ROS_INFO("srv sended down 1cm");
                    }
                }
                else if(ratio>0.16){
                    srv.request.t_vary.translation.x= -0.01;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =1;
                    srv.request.is_center_over = true;
                    if(client.call(srv)){
                        ROS_INFO("srv sended up 1cm");
                    }
                }
                else{
                    height_flag_first =false;
                    break;
                }
            }

            //first calibration
            while(first_calibration == true&&ros::ok()){
                if(ww==0){
                    cout<<"first_calibrarion_init"<<endl;
                    srv.request.t_vary.translation.x= -0.1;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =1;
                    srv.request.is_center_over = true;
                }
                if(ww==1){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;

                }
                if(ww==2){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= -0.2;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
                }
                if(ww==3){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
                }
                if(ww==4){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(ww==5){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0.2;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(ww==6){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                 }
                if(client.call(srv)){
                    for(int i= 0;i<100;++i){
                        ros::spinOnce();
                    }
                    shooting_pose_first.push_back(srv.response.t_end);
                    pics_rgb_first.push_back(cv_ptr->image);
                    cout<<"shooting_pic_first_size:"<<shooting_pose_first.size()
                       <<pics_rgb_first.size()<<endl;
                    ww++;
                }
                if(ww==7){
                    hecobject.realsense_calibration(rvecs_first,tvecs_first,pics_rgb_first,7);
                    vpHomogeneousMatrix cMo_;//cam
                    vpHomogeneousMatrix wMe_;//end
                    vector<vpHomogeneousMatrix> vcMo_;
                    vector<vpHomogeneousMatrix> vwMe_;
                    for (int i = 0; i < rvecs_first.size(); i++)
                    {

                        cMo_.buildFrom(tvecs_first[i].at<double>(0,0),
                                      tvecs_first[i].at<double>(1,0),
                                      tvecs_first[i].at<double>(2.0),
                                      rvecs_first[i].at<double>(0,0),
                                      rvecs_first[i].at<double>(1,0),
                                      rvecs_first[i].at<double>(2.0));//cam
                //        wMe.buildFrom(t[i]);//end
                        wMe_ = visp_bridge::toVispHomogeneousMatrix(shooting_pose_first[i]);//end
                        cout<<"wMe_first"<<endl<<wMe_<<endl;
                        vcMo_.push_back(cMo_);
                        vwMe_.push_back(wMe_);
                    }

                    vpHomogeneousMatrix eMc_;
                    hecobject.calibrationTsai(vcMo_, vwMe_, eMc_);
                    std::cout<<"eMc_first"<<endl<<eMc_<<std::endl;

                    // to transform
                    geometry_msgs::Transform cam2end_init;
                    geometry_msgs::Transform center2cam;
                    tf::Transform center2cam_tf;
                    tf::Transform cam2end_tf_;
                    cam2end_init = visp_bridge::toGeometryMsgsTransform(eMc_);

                    Mat pic_1;
                    for(int i= 0;i<100;++i){
                        ros::spinOnce();
                    }
                    pic_1= cv_ptr->image;

                    //caculate the center
                    hecobject.calboardcenter(pic_1,center2cam);
                    center2cam.translation.z = 0;
                    cout<<"center2cam"<<endl
                       <<center2cam.translation.x<<endl
                        <<center2cam.translation.y<<endl
                       <<center2cam.translation.z<<endl;

                    //caculate the transform
                    tf::transformMsgToTF(cam2end_init,cam2end_tf_);
                    cam2end_gloabl_first = cam2end_tf_;
                    tf::transformMsgToTF(center2cam,center2cam_tf);
                    tf::Transform t1,t2,t3;

                    //right mul
                    t1.mult(cam2end_tf_,center2cam_tf);
                    t2 = cam2end_tf_.inverse();
                    t3.mult(t1,t2);

                    geometry_msgs::Transform TT;
                    tf::transformTFToMsg(t3,TT);
                    cout<<"TT"<<endl;
                    cout<<TT.translation.x<<" "
                        <<TT.translation.y<<" "
                        <<TT.translation.z<<" "
                        <<endl;
                    srv.request.t_vary =TT;
                    srv.request.is_center_over = true;

                    if(client.call(srv)){
                        ROS_INFO("center found first");
                    }
                    first_calibration = false;

                }
            }

            //height adaption
            while(height_flag_sec == true&&ros::ok()){
                cout<<"z-aix-model..."<<endl;
                Mat pic_final;
                sleep(0.5);

                for(int i= 0;i<100;++i){
                    ros::spinOnce();
                }
                pic_final = cv_ptr->image;
                double ratio_final;
                hecobject.boardinpic(pic_final,ratio_final);
                cout<<"ratio:"<<ratio_final<<endl;
                //too high
                if(ratio_final<0.12){
                    srv.request.t_vary.translation.x= 0.01;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =1;
                    srv.request.is_center_over = true;
                    if(client.call(srv)){
                        ROS_INFO("srv sended");
                    }
                }
                else{
                    height_flag_sec =false;
                    break;
                }
            }

            //sec calibration
            while(sec_calibration == true&&ros::ok()){
                if(couter==0){
                    cout<<"calibrarion_init"<<endl;
                    srv.request.t_vary.translation.x= -0.1;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =1;
                    srv.request.is_center_over = true;
                }
                if(couter==1){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;

                }
                if(couter==2){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= -0.2;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==3){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==4){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==5){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0.2;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==6){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                 }
                if(couter==7){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =sin(3.14159/36);
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(3.14159/36);
                    srv.request.is_center_over = true;

                }
                if(couter==8){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;

                }
                if(couter==9){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= -0.2;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==10){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0;
                    srv.request.t_vary.translation.z= 0.1;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.z =0;
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==11){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==12){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= 0.2;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(-4*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(-4*3.14159/180);
                    srv.request.is_center_over = true;
    //                calibrate_before_flag =true;
                }
                if(couter==13){
                    srv.request.t_vary.translation.x= 0;
                    srv.request.t_vary.translation.y= -0.1;
                    srv.request.t_vary.translation.z= 0;
                    srv.request.t_vary.rotation.x =0;
                    srv.request.t_vary.rotation.y =0;
                    srv.request.t_vary.rotation.z =sin(2*3.14159/180);
                    srv.request.t_vary.rotation.w =cos(2*3.14159/180);
                    srv.request.is_center_over = true;
                    calibrate_before_flag =true;
                 }
                if(client.call(srv)){
                    for(int i= 0;i<100;++i){
                        ros::spinOnce();
                    }
                    shooting_pose_.push_back(srv.response.t_end);
                    pics_rgb_.push_back(cv_ptr->image);

                    //store image
                    char filename[30];
                    sprintf(filename, "/home/wukong/buffer/pic_/%d.jpg", num_);
                    //convert Mat to IplImage
                    IplImage  tmp = IplImage(cv_ptr->image);
                    cvSaveImage(filename, &tmp);
                    couter++;
                    num_++;
                }
                if(calibrate_before_flag == true){
                    calibrate_before_flag=false;
                    sec_calibration = false;
                    //get the cam parameters
                    hecobject.realsense_calibration(rvecs_,tvecs_,pics_rgb_,14);
                    vpHomogeneousMatrix cMo_;//cam
                    vpHomogeneousMatrix wMe_;//end
                    vector<vpHomogeneousMatrix> vcMo_;
                    vector<vpHomogeneousMatrix> vwMe_;
                    for (int i = 0; i < rvecs_.size(); i++)
                    {

                        cMo_.buildFrom(tvecs_[i].at<double>(0,0),
                                      tvecs_[i].at<double>(1,0),
                                      tvecs_[i].at<double>(2.0),
                                      rvecs_[i].at<double>(0,0),
                                      rvecs_[i].at<double>(1,0),
                                      rvecs_[i].at<double>(2.0));//cam
                //        wMe.buildFrom(t[i]);//end
                        wMe_ = visp_bridge::toVispHomogeneousMatrix(shooting_pose_[i]);//end
                        cout<<"wMe_"<<endl<<wMe_<<endl;
                        vcMo_.push_back(cMo_);
                        vwMe_.push_back(wMe_);
                    }

                    vpHomogeneousMatrix eMc_;
                    hecobject.calibrationTsai(vcMo_, vwMe_, eMc_);
                    std::cout<<"eMc_"<<endl<<eMc_<<std::endl;

                    // to transform
                    geometry_msgs::Transform cam2end_init;
                    geometry_msgs::Transform center2cam;
                    tf::Transform center2cam_tf;
                    cam2end_init = visp_bridge::toGeometryMsgsTransform(eMc_);

                    cout<<"pic_rgb_size"<<pics_rgb_.size()<<endl;
                    cout<<"shootingpose_size"<<shooting_pose_.size()<<endl;

                    Mat pic_now;
                    for(int i= 0;i<100;++i){
                        ros::spinOnce();
                    }
                    pic_now = cv_ptr->image;

                    //store image
                    char filename[30];
                    sprintf(filename, "/home/wukong/buffer/pic1/%d.jpg", num_);

                    //convert Mat to IplImage
                    IplImage  tmp = IplImage(pic_now);
                    cvSaveImage(filename, &tmp);

                    //caculate the center
                    hecobject.calboardcenter(pic_now,center2cam);
                    center2cam.translation.z = 0;
                    cout<<"center2cam"<<endl
                        <<center2cam.translation.x<<endl
                        <<center2cam.translation.y<<endl
                        <<center2cam.translation.z<<endl;

                    //caculate the transform
                    tf::transformMsgToTF(cam2end_init,cam2end_tf_);
    //                tf::transformMsgToTF(cam2end_real,cam2end_tf_);//debug
                    tf::transformMsgToTF(center2cam,center2cam_tf);
                    tf::Transform t1,t2,t3;

                    //right mul
                    t1.mult(cam2end_tf_,center2cam_tf);
                    t2 = cam2end_tf_.inverse();
                    t3.mult(t1,t2);

                    geometry_msgs::Transform TT;
                    tf::transformTFToMsg(t3,TT);
                    cout<<"TT"<<endl;
                    cout<<TT.translation.x<<" "
                        <<TT.translation.y<<" "
                        <<TT.translation.z<<" "
                        <<endl;
                    srv.request.t_vary =TT;
                    srv.request.is_center_over = true;

                    if(client.call(srv)){
                        ROS_INFO("center found twice");
                    }


                }

            }

            //height adaption
            while(height_flag_final == true&&ros::ok()){
                cout<<"z-aix-model-final"<<endl;
                Mat pic_final;
                sleep(0.5);
                for(int i= 0;i<100;++i){
                    ros::spinOnce();
                }
                pic_final = cv_ptr->image;
                double ratio_final;
                hecobject.boardinpic(pic_final,ratio_final);
                cout<<"ratio:"<<ratio_final<<endl;
                //too high
                if(ratio_final<0.12){
                    //need todebug***
                    tf::Transform t_cam_tf;
                    tf::Transform t_end_tf;
                    tf::Transform end2cam_inv,t1;

                    geometry_msgs::Transform t_cam;
                    geometry_msgs::Transform t_end;
                    t_cam.translation.x =0;
                    t_cam.translation.y = 0;
                    t_cam.translation.z = 0.01;
                    t_cam.rotation.x = 0;
                    t_cam.rotation.y = 0;
                    t_cam.rotation.z = 0;
                    t_cam.rotation.w = 1;
                    tf::transformMsgToTF(t_cam,t_cam_tf);
                    end2cam_inv = cam2end_tf_.inverse();
                    t1.mult(t_cam_tf,end2cam_inv);
                    t_end_tf.mult(cam2end_tf_,t1);
                    tf::transformTFToMsg(t_end_tf,t_end);
                    srv.request.t_vary = t_end;
                    srv.request.is_center_over = true;
                    if(client.call(srv)){
                        ROS_INFO("srv sended");
                    }
                }
                else{
                    height_flag_final =false;
                    srv.request.is_center_over =false;
                    if(client.call(srv)){
                        ROS_INFO("center found exit...");

                    }
                    isfindcenter = false;
                }
            }

        }
        ros::spinOnce();
    }

    cout<<"start to hand_eye_calibrate..."<<endl;

    while(ros::ok()){

        if(calibrate_flag == true){
            //caculate the center of the last pic
            for(int i= 0;i<100;++i){
                ros::spinOnce();
            }
            pic_last = cv_ptr->image;
            hecobject.calboardcenter(pic_last,center2cam_last);

            //waican
            cout<<"caculate the waican"<<endl;
            hecobject.realsense_calibration(rvecs,tvecs,pics_rgb,frame_num);
            cout<<"complete the waicai"<<endl;
            vpHomogeneousMatrix cMo;//cam
            vpHomogeneousMatrix wMe;//end
            vector<vpHomogeneousMatrix> vcMo;
            vector<vpHomogeneousMatrix> vwMe;
            for (int i = 0; i <rvecs.size(); i++)
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
                //save the shootingpose
                ofstream ff;
                ff.open("/home/wukong/buffer/shootingpose.txt",ios_base::app);
                ff<<wMe<<endl<<endl;
                ff.close();
                vcMo.push_back(cMo);
                vwMe.push_back(wMe);
            }

            vpHomogeneousMatrix eMc;
            hecobject.calibrationTsai(vcMo, vwMe, eMc);
            std::cout<<"eMc"<<endl<<eMc<<std::endl;

            // to transform
            geometry_msgs::Transform cam2end;
            cam2end = visp_bridge::toGeometryMsgsTransform(eMc);
            tf::transformMsgToTF(cam2end,cam2end_tf);
            br_flag =true;
            cacu_plane = true;
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



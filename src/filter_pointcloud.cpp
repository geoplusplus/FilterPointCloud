#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf/message_filter.h>
#include <tf2_msgs/TFMessage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "sensor_msgs/Imu.h"
#include "tracklets.h"

ros::Publisher pub_pc;
ros::Publisher imu_pc;
ros::Publisher tf_pc;
ros::Publisher tf_pc2;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
Tracklets * tracklets(new Tracklets());
Eigen::MatrixXd tmat;
Eigen::MatrixXd pmat;
Eigen::MatrixXd rmat;
Eigen::MatrixXd res;
cv::Mat rect_view;

cv::Vec3b Terrain(210,0,200);
cv::Vec3b Building(140,140,140);
cv::Vec3b TrafficSign(255,255,0);
cv::Vec3b TrafficLight(200,200,0);
cv::Vec3b Road(100, 60,100);
cv::Vec3b Vegetation(90, 240, 0);
cv::Vec3b Truck(160, 60, 60);
cv::Vec3b Misc(80, 80, 80);
cv::Vec3b Pole(255, 130, 0);

Eigen::MatrixXd point(4,1);
Eigen::MatrixXd point_t(4,1);


void tf_cb(tf::tfMessage tf)
{
  for(unsigned int i = 0; i < tf.transforms.size(); i++)
  {
    tf.transforms[i].header.stamp = ros::Time::now();
  }

  tf_pc.publish(tf);

}

void tf_cb2(tf::tfMessage tf)
{
  for(unsigned int i = 0; i < tf.transforms.size(); i++)
  {
    tf.transforms[i].header.stamp = ros::Time::now();
  }

  tf_pc2.publish(tf);

}


void 
callback (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::Imu::ConstPtr& imu)
{
  pcl::fromROSMsg(*input, *laserCloudIn);
  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(msg, "rgb8");
  cv::Mat rect_view = cv_cam->image;
  static tf::TransformBroadcaster br;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  for(unsigned int i = 0; i < laserCloudIn->size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = laserCloudIn->at(i).x;
    p.y = laserCloudIn->at(i).y;
    p.z = laserCloudIn->at(i).z;
    p.r = 0;
    p.g = 0;
    p.b = 0;

    point << p.x, p.y, p.z, 1;

    point_t = res * point;

    if (point_t(2,0) > 0) {
      int image_u = (int)(point_t(0,0)/(point_t(2,0)));
      int image_v = (int)(point_t(1,0)/(point_t(2,0)));
      if (0 <= image_u && image_u < rect_view.size().width &&
          0 <= image_v && image_v < rect_view.size().height) {
        cv::Vec3b color= rect_view.at<cv::Vec3b>(cv::Point(image_u, image_v));
        if(color == Building || color == Road || color == Terrain || color == Vegetation || color == TrafficLight || color == TrafficSign || color == Misc || color == Pole){
          filtered_cloud->push_back(p);
        }
      }

    }

    //filtered_cloud->push_back(p);
  }
              
  sensor_msgs::PointCloud2 scan_filter = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*filtered_cloud, scan_filter);
  scan_filter.header = input->header;
  pub_pc.publish (scan_filter);
  imu_pc.publish(*imu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  if (!tracklets->loadFromFile("/home/michelle/datasets/tracklet_labels.xml"))
  std::cout << "ERROR " << std::endl;


  ros::NodeHandle nh;

  tmat = Eigen::MatrixXd(4,4);
  tmat <<   7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
          1.480249e-02,7.280733e-04, -9.998902e-01, -7.631618e-02,
           9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
           0.0f, 0.0f, 0.0f, 1.0f;

  pmat = Eigen::MatrixXd(3,4);
  pmat << 7.215377e+02, 0.000000e+00, 6.095593e+02,
        0.000000e+00, 0.000000e+00, 7.215377e+02,
        1.728540e+02, 0.000000e+00, 0.000000e+00,
        0.000000e+00, 1.000000e+00, 0.000000e+00;

  rmat = Eigen::MatrixXd(4,4);
  rmat << 9.999239e-01, 9.837760e-03, -7.445048e-03,0,
          -9.869795e-03, 9.999421e-01, -4.278459e-03,0,
          7.402527e-03, 4.351614e-03, 9.999631e-01,0,
          0,0,0,1;

  res = pmat * rmat * tmat;

  message_filters::Subscriber<sensor_msgs::Image> sub_camera(nh, "/kitti/camera_color_left/image_raw", 5);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc(nh,"/kitti/velo/pointcloud", 10);
  message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh,"/kitti/oxts/imu", 5);

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub_pc, sub_camera, sub_imu);

  ros::Subscriber sub_tf = nh.subscribe ("/tf", 5, tf_cb);
  ros::Subscriber sub_tf2 = nh.subscribe ("/tf_static_in", 5, tf_cb2);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("points2", 10);

  imu_pc = nh.advertise<sensor_msgs::Imu> ("imu", 10);

  tf_pc = nh.advertise<tf::tfMessage> ("tf_in", 10);
  tf_pc2 = nh.advertise<tf::tfMessage> ("tf_static", 10);

  ros::spin();
}
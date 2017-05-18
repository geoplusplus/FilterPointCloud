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
#include <tf2_msgs/TFMessage.h>

#include "sensor_msgs/Imu.h"
#include "tracklets.h"

ros::Publisher pub_pc;
ros::Publisher imu_pc;
ros::Publisher tf_pc;
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
Tracklets * tracklets(new Tracklets());
Eigen::MatrixXd tmat;
Eigen::MatrixXd pmat;
Eigen::MatrixXd rmat;

int count_frames = 0;

// void tf_cb(const tf2_msgs::TFMessage& msg)
// {
//   std::cout  << "TESTE" << std::endl;
//   tf_pc.publish(msg);
// }

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_pc.publish(*msg);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("Got point cloud!");
  count_frames++;
  pcl::fromROSMsg(*input, *laserCloudIn);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *laserCloudIn;
  pcl::CropBox<pcl::PointXYZ> clipper;
  clipper.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*cloud));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour_laser_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud;
  std::vector< int >  total_indices; 

  for(unsigned int i = 0; i < laserCloudIn->size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = laserCloudIn->at(i).x;
    p.y = laserCloudIn->at(i).y;
    p.z = laserCloudIn->at(i).z;
    p.r = 0;
    p.g = 0;
    p.b = 0;
    colour_laser_cloud->push_back(p);
  }
  for(int i = 0; i < tracklets->numberOfTracklets(); i++){
    if(tracklets->isActive(i,count_frames)){
      Tracklets::tTracklet* tracklet = tracklets->getTracklet(i);
      
      Tracklets::tPose *pose;
      if(tracklets->getPose(i, count_frames, pose)){
        clipper.setTranslation(Eigen::Vector3f(pose->tx, pose->ty, pose->tz));
        clipper.setRotation(Eigen::Vector3f(pose->rx, pose->ry, pose->rz));
        clipper.setMin(-Eigen::Vector4f(tracklet->l/2, tracklet->w/2, 0, 0));
        clipper.setMax(Eigen::Vector4f(tracklet->l/2, tracklet->w/2, tracklet->h, 0));
        std::vector< int > indices ;
        clipper.filter(indices);
        for (int j=0; j < indices.size(); j++) {
          colour_laser_cloud->at(indices[j]).r = 255;
          total_indices.push_back(indices[j]);
        }
      }
    }
  }

  pcl::PointIndices point_indices;
  point_indices.indices = total_indices;
  pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
  extractIndices.setIndices(boost::make_shared<const pcl::PointIndices> (point_indices));
  extractIndices.setInputCloud(colour_laser_cloud);
  extractIndices.setNegative(true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  extractIndices.filter(*output);

  sensor_msgs::PointCloud2 scan_color = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*output, scan_color);
//  scan_color.header.frame_id = "velo_link";
  scan_color.header = input->header;
  pub_pc.publish (scan_color);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  if (!tracklets->loadFromFile("/home/michelle/datasets/tracklet_labels.xml"))
    std::cout << "ERROR " << std::endl;

  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_pc = nh.subscribe ("/kitti/velo/pointcloud", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("points2", 10);

  ros::Subscriber sub_imu = nh.subscribe("/kitti/oxts/imu", 1000, imu_cb);

  //ros::Subscriber sub_tf = nh.subscribe("/tf_bag", 1000, tf_cb);

  imu_pc = nh.advertise<sensor_msgs::Imu> ("imu", 1000);
  ///tf_pc = nh.advertise<tf2_msgs::TFMessage>("tf_in", 1000);

  ros::spin();

  delete tracklets;
}

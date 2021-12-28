#include "include/lidar_camera_calib.hpp"
#include "ceres/ceres.h"
#include "include/common.h"
#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <include/range_image.h>

livox_camera_calib::CalibConfig calib_config;
std::atomic_bool has_new_config{false};

void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg,
                   const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                   Calibration& calibra) {
  cv_bridge::CvImageConstPtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  auto &image = calibra.rgb_image_;
  cv_img_ptr->image.copyTo(image);

  auto& origin_cloud = calibra.raw_lidar_cloud_;
  origin_cloud->clear();
  pcl::PointCloud<RslidarPoint> cloud;
  fromROSMsg(*cloud_msg, cloud);
  RangeImage<> range_img(cloud);
  range_img.GenerateVizImg();
  cv::namedWindow("range", cv::WINDOW_NORMAL);
  cv::imshow("range", range_img.viz_img);
  cv::waitKey(100);
  cv::namedWindow("range raw", cv::WINDOW_NORMAL);
  cv::imshow("range raw", range_img.img);
  cv::waitKey(100);
  for (uint i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZI p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    p.intensity = static_cast<float>(cloud.points[i].intensity);
    origin_cloud->points.push_back(p);
  }

  if (has_new_config) {
    calibra.UpdateCalibConfig(calib_config);
    has_new_config = false;
  }
  calibra.extractImgAndPointcloudEdges();
  std::vector<VPnPData> pnp_list;
  int match_dis = 25;
  calibra.buildVPnp(calibra.calib_params(), match_dis, true,
                    calibra.rgb_egde_cloud_, calibra.plane_line_cloud_,
                    pnp_list);

  sensor_msgs::PointCloud2 pub_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  calibra.colorCloud(calibra.calib_params(), 1, image,
                     origin_cloud, rgb_cloud);
  pcl::toROSMsg(*rgb_cloud, pub_cloud);
  pub_cloud.header.frame_id = "rslidar";
  calibra.init_rgb_cloud_pub_.publish(pub_cloud);

  cv::Mat init_img = calibra.getProjectionImg(calibra.calib_params(), origin_cloud, image);
  cv::namedWindow("Initial extrinsic", cv::WINDOW_NORMAL);
  cv::imshow("Initial extrinsic", init_img);
  cv::waitKey(1);
}

void param_callback(livox_camera_calib::CalibConfig &config,
                    uint32_t level) {
  calib_config = config;
  has_new_config = true;
  ROS_INFO("New voxel_size: %f", config.voxel_size);
  ROS_INFO("New down_sample_size: %f", config.down_sample_size);
  ROS_INFO("New min_points_size: %f", config.plane_min_points_size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalibOnline");
  ros::NodeHandle nh;
  ros::Rate loop_rate(4);

  ros::NodeHandle nh_("~");  // LOCAL
  string image_topic, cloud_topic, frame_id;
  nh_.param<string>("image_topic", image_topic,
                    "/usb_cam/image_raw");
  nh_.param<string>("cloud_topic", cloud_topic,
                    "/rslidar_points");
  nh_.param<string>("frame_id", frame_id,
                    "rslidar");

  const std::string CameraConfigPath = std::string(argv[1]);
  const std::string CalibSettingPath = std::string(argv[2]);
  const std::string BagPath;
  const std::string ResultPath;
  Calibration calibra(CameraConfigPath, CalibSettingPath);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, image_topic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_, cloud_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&ImageCallback, _1, _2, calibra));

  // ROS param callback
  dynamic_reconfigure::Server<livox_camera_calib::CalibConfig> server;
  dynamic_reconfigure::Server<
      livox_camera_calib::CalibConfig>::CallbackType f;
  f = boost::bind(param_callback, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {
    auto& plane_line_cloud = calibra.pub_line_cloud_->Get();
    sensor_msgs::PointCloud2 pub_plane_line_cloud;
    pcl::toROSMsg(plane_line_cloud, pub_plane_line_cloud);
    pub_plane_line_cloud.header.frame_id = frame_id;
    calibra.line_cloud_pub_.publish(pub_plane_line_cloud);

    auto& plane_cloud = calibra.pub_plane_cloud_->Get();
    sensor_msgs::PointCloud2 pub_plane_cloud;
    pcl::toROSMsg(plane_cloud, pub_plane_cloud);
    pub_plane_cloud.header.frame_id = frame_id;
    calibra.planner_cloud_pub_.publish(pub_plane_cloud);

    ros::spinOnce();
    loop_rate.sleep();
  }

  cv::destroyAllWindows();
  return 0;
}
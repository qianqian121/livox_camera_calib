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
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

//#define add_error
// instrins matrix
Eigen::Matrix3d inner;
// Distortion coefficient
Eigen::Vector4d distor;
Eigen::Vector4d quaternion;
Eigen::Vector3d transation;
livox_camera_calib::CalibConfig calib_config;
std::atomic_bool has_new_config{false};

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RosCloudToCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[7].offset; // mag_snr

  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    pcl::PointXYZI point;
    point.x = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z = BytesTo<float>(msg->data, point_start_byte + z_offset);
    point.intensity = BytesTo<float>(msg->data, point_start_byte + ring_offset);
    cloud.push_back(point);
  }

  return boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);
}

void ImageCallback(const sensor_msgs::CompressedImageConstPtr &img_msg,
                   const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                   Calibration& calibra) {
  // Image to cv::Mat
  cv::Mat image;
  try
  {
    image = cv::imdecode(cv::Mat(img_msg->data),1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
    return;
  }

  cv::Mat imageCopy;
  image.copyTo(imageCopy);
  cv::namedWindow("out", cv::WINDOW_NORMAL);
  cv::imshow("out", imageCopy);
  cv::waitKey(1);

  auto cloud = RosCloudToCloud(cloud_msg);
  auto& origin_cloud = calibra.raw_lidar_cloud_;
  origin_cloud->clear();
  for (uint i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZI p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.intensity = cloud->points[i].intensity;
    origin_cloud->points.push_back(p);
  }

  if (has_new_config) {
    calibra.UpdateCalibConfig(calib_config);
    has_new_config = false;
  }

  if (origin_cloud->empty()) return;
  cv::Mat init_img = calibra.getProjectionImg(calibra.calib_params(), origin_cloud, imageCopy);
  cv::namedWindow("Front", cv::WINDOW_NORMAL);
  cv::imshow("Front", init_img);
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
                    "/cam_front/image_rect_color/compressed");
  nh_.param<string>("cloud_topic", cloud_topic,
                    "/pc2/radar/front/pointcloud/merged/dynamic/vp104_2_vp104_3_merge");
  nh_.param<string>("frame_id", frame_id,
                    "front");

  const std::string CameraConfigPath = std::string(argv[1]);
  const std::string CalibSettingPath = std::string(argv[2]);
  Calibration calibra(CameraConfigPath, CalibSettingPath);

  message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(nh_, image_topic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_, cloud_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> MySyncPolicy;
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
    ros::spinOnce();
    loop_rate.sleep();
  }

  cv::destroyAllWindows();
  return 0;
}
#include "include/lidar_camera_calib.hpp"
#include "ceres/ceres.h"
#include "include/common.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//#define add_error
// instrins matrix
Eigen::Matrix3d inner;
// Distortion coefficient
Eigen::Vector4d distor;
Eigen::Vector4d quaternion;
Eigen::Vector3d transation;

// Normal pnp solution
class pnp_calib {
public:
  pnp_calib(PnPData p) { pd = p; }
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    residuals[0] = ud - T(pd.u);
    residuals[1] = vd - T(pd.v);
    return true;
  }
  static ceres::CostFunction *Create(PnPData p) {
    return (
        new ceres::AutoDiffCostFunction<pnp_calib, 2, 4, 3>(new pnp_calib(p)));
  }

private:
  PnPData pd;
};

// pnp calib with direction vector
class vpnp_calib {
public:
  vpnp_calib(VPnPData p) { pd = p; }
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    if (T(pd.direction(0)) == T(0.0) && T(pd.direction(1)) == T(0.0)) {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
    } else {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
      Eigen::Matrix<T, 2, 2> I =
          Eigen::Matrix<float, 2, 2>::Identity().cast<T>();
      Eigen::Matrix<T, 2, 1> n = pd.direction.cast<T>();
      Eigen::Matrix<T, 1, 2> nt = pd.direction.transpose().cast<T>();
      Eigen::Matrix<T, 2, 2> V = n * nt;
      V = I - V;
      Eigen::Matrix<T, 2, 2> R = Eigen::Matrix<float, 2, 2>::Zero().cast<T>();
      R.coeffRef(0, 0) = residuals[0];
      R.coeffRef(1, 1) = residuals[1];
      R = V * R * V.transpose();
      residuals[0] = R.coeffRef(0, 0);
      residuals[1] = R.coeffRef(1, 1);
    }
    return true;
  }
  static ceres::CostFunction *Create(VPnPData p) {
    return (new ceres::AutoDiffCostFunction<vpnp_calib, 2, 4, 3>(
        new vpnp_calib(p)));
  }

private:
  VPnPData pd;
};

void roughCalib(Calibration &calibra, Vector6d &calib_params,
                double search_resolution, int max_iter) {
  float match_dis = 25;
  Eigen::Vector3d fix_adjust_euler(0, 0, 0);
  for (int n = 0; n < 2; n++) {
    for (int round = 0; round < 3; round++) {
      Eigen::Matrix3d rot;
      rot = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
      // std::cout << "init rot" << rot << std::endl;
      float min_cost = 1000;
      for (int iter = 0; iter < max_iter; iter++) {
        Eigen::Vector3d adjust_euler = fix_adjust_euler;
        adjust_euler[round] = fix_adjust_euler[round] +
                              pow(-1, iter) * int(iter / 2) * search_resolution;
        Eigen::Matrix3d adjust_rotation_matrix;
        adjust_rotation_matrix =
            Eigen::AngleAxisd(adjust_euler[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(adjust_euler[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(adjust_euler[2], Eigen::Vector3d::UnitX());
        Eigen::Matrix3d test_rot = rot * adjust_rotation_matrix;
        // std::cout << "adjust_rotation_matrix " << adjust_rotation_matrix
        //           << std::endl;
        Eigen::Vector3d test_euler = test_rot.eulerAngles(2, 1, 0);
        // std::cout << "test euler: " << test_euler << std::endl;
        Vector6d test_params;
        test_params << test_euler[0], test_euler[1], test_euler[2],
            calib_params[3], calib_params[4], calib_params[5];
        std::vector<VPnPData> pnp_list;
        calibra.buildVPnp(test_params, match_dis, false,
                          calibra.rgb_egde_cloud_, calibra.plane_line_cloud_,
                          pnp_list);
        // std::vector<float> residual_list;
        // calibra.calcResidual(test_params, pnp_list, residual_list);
        // float mean_residual = 0;
        // for (int j = 0; j < residual_list.size(); j++) {
        //   mean_residual += fabs(residual_list[j]);
        // }
        // mean_residual = mean_residual / residual_list.size();
        float cost = (calibra.plane_line_cloud_->size() - pnp_list.size()) *
                     1.0 / calibra.plane_line_cloud_->size();
        if (cost < min_cost) {
          std::cout << "Rough calibration min cost:" << cost << std::endl;
          min_cost = cost;
          calib_params[0] = test_params[0];
          calib_params[1] = test_params[1];
          calib_params[2] = test_params[2];
          calibra.buildVPnp(calib_params, match_dis, true,
                            calibra.rgb_egde_cloud_, calibra.plane_line_cloud_,
                            pnp_list);
          cv::Mat projection_img = calibra.getProjectionImg(calib_params);
          cv::imshow("Rough Optimization", projection_img);
          cv::waitKey(50);
        }
      }
    }
  }
}

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
//  cv::Mat imageCopy;
//  image.copyTo(imageCopy);
//  cv::namedWindow("out", cv::WINDOW_NORMAL);
//  cv::imshow("out", imageCopy);
//  cv::waitKey(1);

  auto& origin_cloud = calibra.raw_lidar_cloud_;
  origin_cloud->clear();
  pcl::PointCloud<RslidarPoint> cloud;
  fromROSMsg(*cloud_msg, cloud);
  for (uint i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZI p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    p.intensity = static_cast<float>(cloud.points[i].intensity);
    origin_cloud->points.push_back(p);
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

//  cv::Mat init_img = calibra.getProjectionImg(calibra.calib_params(), origin_cloud, imageCopy);
  cv::Mat init_img = calibra.getProjectionImg(calibra.calib_params(), origin_cloud, image);
  cv::namedWindow("Initial extrinsic", cv::WINDOW_NORMAL);
  cv::imshow("Initial extrinsic", init_img);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalibOnline");
  ros::NodeHandle nh;
  ros::Rate loop_rate(4);

  ros::NodeHandle nh_("~");  // LOCAL
  string image_topic, cloud_topic;
  nh_.param<string>("image_topic", image_topic,
                    "/usb_cam/image_raw");
  nh_.param<string>("cloud_topic", cloud_topic,
                    "/rslidar_points");

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

//  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2>
//      sync(image_sub, cloud_sub, 10); // queue size 10
//  sync.registerCallback(boost::bind(&ImageCallback, _1, _2));

  while (ros::ok()) {
    auto& plane_cloud = calibra.plane_cloud_->Get();
//    std::cout << plane_cloud.size() << std::endl;
    sensor_msgs::PointCloud2 pub_plane_cloud;
    pcl::toROSMsg(plane_cloud, pub_plane_cloud);
    pub_plane_cloud.header.frame_id = "livox";
    calibra.line_cloud_pub_.publish(pub_plane_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }

  cv::destroyAllWindows();
  return 0;


  Eigen::Vector3d init_euler_angle =
      calibra.init_rotation_matrix_.eulerAngles(2, 1, 0);
  Eigen::Vector3d init_transation = calibra.init_translation_vector_;

  Vector6d calib_params;
  calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
      init_transation(0), init_transation(1), init_transation(2);

  std::vector<PnPData> pnp_list;
  std::vector<VPnPData> vpnp_list;

  ROS_INFO_STREAM("Finish prepare!");
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  inner << calibra.fx_, 0.0, calibra.cx_, 0.0, calibra.fy_, calibra.cy_, 0.0,
      0.0, 1.0;
  distor << calibra.k1_, calibra.k2_, calibra.p1_, calibra.p2_;
  R = calibra.init_rotation_matrix_;
  T = calibra.init_translation_vector_;
  bool use_vpnp = true;
#ifdef add_error
  // add error to the init extrinsic, comment them in real calibration
  Eigen::Vector3d error_euler_angle;
  error_euler_angle << calibra.adjust_euler_angle_[0],
      calibra.adjust_euler_angle_[1], calibra.adjust_euler_angle_[2];
  Eigen::Matrix3d rotation_matrix_error;
  rotation_matrix_error =
      Eigen::AngleAxisd(error_euler_angle[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(error_euler_angle[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(error_euler_angle[2], Eigen::Vector3d::UnitX());
  R = R * rotation_matrix_error;
#endif
  Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
  calib_params[0] = euler[0];
  calib_params[1] = euler[1];
  calib_params[2] = euler[2];
  calib_params[3] = T[0];
  calib_params[4] = T[1];
  calib_params[5] = T[2];
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  calibra.colorCloud(calib_params, 1, calibra.rgb_image_,
                     calibra.raw_lidar_cloud_, rgb_cloud);
  pcl::toROSMsg(*rgb_cloud, pub_cloud);
  pub_cloud.header.frame_id = "livox";
  calibra.init_rgb_cloud_pub_.publish(pub_cloud);
  cv::Mat init_img = calibra.getProjectionImg(calib_params);
  cv::imshow("Initial extrinsic", init_img);
  cv::waitKey(1000);

  roughCalib(calibra, calib_params, DEG2RAD(0.1), 50);
  cv::Mat test_img = calibra.getProjectionImg(calib_params);
  cv::imshow("After rough extrinsic", test_img);
  cv::waitKey(1000);
  int iter = 0;
  // Maximum match distance threshold: 15 pixels
  // If initial extrinsic lead to error over 15 pixels, the algorithm will not
  // work
  int dis_threshold = 20;
  bool opt_flag = true;

  // Iteratively reducve the matching distance threshold
  for (dis_threshold = 20; dis_threshold > 6; dis_threshold -= 1) {
    // For each distance, do twice optimization
    for (int cnt = 0; cnt < 2; cnt++) {
      std::cout << "Iteration:" << iter++ << " Dis:" << dis_threshold
                << std::endl;
      if (use_vpnp) {
        calibra.buildVPnp(calib_params, dis_threshold, true,
                          calibra.rgb_egde_cloud_, calibra.plane_line_cloud_,
                          vpnp_list);
      } else {
        calibra.buildPnp(calib_params, dis_threshold, true,
                         calibra.rgb_egde_cloud_, calibra.plane_line_cloud_,
                         pnp_list);
      }
      cv::Mat projection_img = calibra.getProjectionImg(calib_params);
      cv::imshow("Optimization", projection_img);
      cv::waitKey(100);
      Eigen::Quaterniond q(R);
      Eigen::Vector3d ori_t = T;
      double ext[7];
      ext[0] = q.x();
      ext[1] = q.y();
      ext[2] = q.z();
      ext[3] = q.w();
      ext[4] = T[0];
      ext[5] = T[1];
      ext[6] = T[2];
      Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
      Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem problem;

      problem.AddParameterBlock(ext, 4, q_parameterization);
      problem.AddParameterBlock(ext + 4, 3);
      if (use_vpnp) {
        for (auto val : vpnp_list) {
          ceres::CostFunction *cost_function;
          cost_function = vpnp_calib::Create(val);
          problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
        }
      } else {
        for (auto val : pnp_list) {
          ceres::CostFunction *cost_function;
          cost_function = pnp_calib::Create(val);
          problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
        }
      }
      ceres::Solver::Options options;
      options.preconditioner_type = ceres::JACOBI;
      options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.minimizer_progress_to_stdout = true;
      options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << std::endl;
      Eigen::Matrix3d rot = m_q.toRotationMatrix();
      Eigen::Vector3d euler_angle = rot.eulerAngles(2, 1, 0);
      // std::cout << rot << std::endl;
      // std::cout << m_t << std::endl;
      calib_params[0] = euler_angle[0];
      calib_params[1] = euler_angle[1];
      calib_params[2] = euler_angle[2];
      calib_params[3] = m_t(0);
      calib_params[4] = m_t(1);
      calib_params[5] = m_t(2);
      R = rot;
      T[0] = m_t(0);
      T[1] = m_t(1);
      T[2] = m_t(2);
      Eigen::Quaterniond opt_q(R);
      std::cout << "q_dis:" << RAD2DEG(opt_q.angularDistance(q))
                << " ,t_dis:" << (T - ori_t).norm() << std::endl;
      // getchar();
      if (opt_q.angularDistance(q) < DEG2RAD(0.01) &&
          (T - ori_t).norm() < 0.005) {
        opt_flag = false;
      }
      if (!opt_flag) {
        break;
      }
    }
    if (!opt_flag) {
      break;
    }
  }

  ros::Rate loop(0.5);
  // roughCalib(calibra, calib_params, DEG2RAD(0.01), 20);

  R = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
  std::string result_file = ResultPath + "/extrinsic.txt";
  std::ofstream outfile(result_file);
  for (int i = 0; i < 3; i++) {
    outfile << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "," << T[i]
            << std::endl;
  }
  outfile << 0 << "," << 0 << "," << 0 << "," << 1 << std::endl;
  cv::Mat opt_img = calibra.getProjectionImg(calib_params);
  cv::imshow("Optimization result", opt_img);
  cv::waitKey(1000);
  Eigen::Matrix3d init_rotation;
  init_rotation << 0, -1.0, 0, 0, 0, -1.0, 1, 0, 0;
  Eigen::Matrix3d adjust_rotation;
  adjust_rotation = init_rotation.inverse() * R;
  Eigen::Vector3d adjust_euler = adjust_rotation.eulerAngles(2, 1, 0);
  outfile << RAD2DEG(adjust_euler[0]) << "," << RAD2DEG(adjust_euler[1]) << ","
          << RAD2DEG(adjust_euler[2]) << "," << 0 << "," << 0 << "," << 0
          << std::endl;
  while (ros::ok()) {
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    calibra.colorCloud(calib_params, 1, calibra.rgb_image_,
                       calibra.raw_lidar_cloud_, rgb_cloud);
    pcl::toROSMsg(*rgb_cloud, pub_cloud);
    pub_cloud.header.frame_id = "livox";
    calibra.rgb_cloud_pub_.publish(pub_cloud);
    sensor_msgs::ImagePtr img_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", calibra.rgb_image_)
            .toImageMsg();
    calibra.image_pub_.publish(img_msg);
    std::cout << "push enter to publish again" << std::endl;
    getchar();
    /* code */
  }
  return 0;
}
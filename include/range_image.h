//
// Created by autoware on 12/25/21.
//

#ifndef SRC_RANGE_IMAGE_H
#define SRC_RANGE_IMAGE_H

#include <cmath>

#include <opencv2/core/eigen.hpp>
#include <pcl/impl/point_types.hpp>

#include "common.h"
#include "rslidar_point.h"

template<int LINES = 128, int CHANNEL_WIDTH = 125>
class RangeImage {
public:

  explicit RangeImage(const pcl::PointCloud<RslidarPoint>& cloud, uint8_t min_intensity = 5, float min_depth = 2.5, float max_depth = 80.0) :
    channels(cloud.height),
    channel_width(CHANNEL_WIDTH),
    width(channels * channel_width),
    min_depth(min_depth),
    max_depth(max_depth),
    img(LINES, width, CV_16UC1, cv::Scalar(0)) {
    const auto cloud_size = cloud.size();
    for (int k = 0; k < channels; ++k) {
      for (int j = 0; j < channel_width; ++j) {
        for (int i = 0; i < LINES; ++i) {
          const int cloud_j = i * channel_width + j;
          if (cloud_j < cloud.width) {
            const int idx = channels * cloud_j + k;
            const auto &rs_point = cloud.points.at(idx);
            const auto &x = rs_point.x;
            const auto &y = rs_point.y;
            const auto &z = rs_point.z;
            const auto &intensity = rs_point.intensity;
            const auto &ring = rs_point.ring;
            if (intensity < min_intensity || std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            float depth = sqrt(x * x + y * y + z * z);
            int jj = i % 2 ? (channel_width - 1) - j : j;
            jj += k * channel_width;
            img.at<ushort>(i, jj) = depth / max_depth * 65536.0;
          }
        }
      }
    }
  }

  void GenerateVizImg() {
    viz_img = cv::Mat::zeros(LINES, width, CV_8UC3);
    for (int x = 0; x < viz_img.cols; x++) {
      for (int y = 0; y < viz_img.rows; y++) {
        uint8_t r, g, b;
        float norm = img.at<ushort>(y, x) / 65536.0;
        if (norm < min_depth / max_depth) continue;
        mapJet(norm, 0, 1, r, g, b);
        viz_img.at<cv::Vec3b>(y, x)[0] = b;
        viz_img.at<cv::Vec3b>(y, x)[1] = g;
        viz_img.at<cv::Vec3b>(y, x)[2] = r;
      }
    }
  }

  int channels{0};
  int channel_width{0};
  int width{0};
  float min_depth{0};
  float max_depth{0};
  cv::Mat img;
  cv::Mat viz_img;
};

#endif //SRC_RANGE_IMAGE_H

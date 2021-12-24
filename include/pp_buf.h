//
// Created by autoware on 12/23/21.
//

#ifndef SRC_PP_BUF_H
#define SRC_PP_BUF_H

#include <atomic>
#include <vector>
#include <pcl/impl/point_types.hpp>


template<typename T>
class PpBuf {
public:
//  PpBuf() = default;
//  PpBuf(const PpBuf& other) : idx_(0) {}
  void Update() {
    ++idx_;
  }
  T& Get() {
    return cloud_[idx_ % 2];
  }
  void Update(const T& other) {
    Update();
    Get() = other;
  }
  std::array<T, 2> cloud_;
  std::atomic_uint idx_{0};
};

#endif //SRC_PP_BUF_H

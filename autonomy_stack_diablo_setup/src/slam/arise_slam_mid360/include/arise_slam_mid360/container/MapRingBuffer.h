//
// Created by ubuntu on 2020/6/29.
//

#ifndef MAPRINGBUFFER_H
#define MAPRINGBUFFER_H

#include <iostream>
#include <map>

/* MapRingBuffer 是一个基于 std::map 的模板类，实现了一个环形缓冲区（Ring Buffer），用于管理按时间排序的测量数据。 */
template <typename Meas>
class MapRingBuffer {
public:
  std::map<double, Meas> measMap_; // 以时间戳为键存储测量数据
  typename std::map<double, Meas>::iterator itMeas_;

  int size; // 环形缓冲区的最大容量
  double maxWaitTime_;
  double minWaitTime_;

  MapRingBuffer() {
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  }

  virtual ~MapRingBuffer() {}
  // 分配缓冲区大小 
  bool allocate(const int sizeBuffer) {
    if (sizeBuffer <= 0) {
      return false;
    } else {
      size = sizeBuffer;
      return true;
    }
  }
  // 获取缓冲区当前大小
  int getSize() { return measMap_.size(); }

  // 添加测量数据到缓冲区
  void addMeas(const Meas& meas, const double& t) {
    measMap_.insert(std::make_pair(t, meas));

    // ensure the size of the map, and remove the last element
    if ((int) measMap_.size() > size) {
      measMap_.erase(measMap_.begin());
    }
  }

  // 清空缓冲区
  void clear() { measMap_.clear(); }

  // 清理时间戳小于等于t的数据
  void clean(double t) {
    while (measMap_.size() >= 1 && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }

  // 找到第一个大于 actualTime 的时间戳并返回， 没找到就返回false
  bool getNextTime(double actualTime, double& nextTime) {
    itMeas_ = measMap_.upper_bound(actualTime); // 查找键值严格大于指定键值的第一个元素的迭代器
    if (itMeas_ != measMap_.end()) {
      nextTime = itMeas_->first;
      return true;
    } else {
      return false;
    }
  }
  void waitTime(double actualTime, double& time) {
    double measurementTime = actualTime - maxWaitTime_;
    if (!measMap_.empty() &&
        measMap_.rbegin()->first + minWaitTime_ > measurementTime) {
      measurementTime = measMap_.rbegin()->first + minWaitTime_;
    }
    if (time > measurementTime) {
      time = measurementTime;
    }
  }

  // 获取环形缓冲区最近(新)的时间戳
  bool getLastTime(double& lastTime) {
    if (!measMap_.empty()) {
      lastTime = measMap_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }

  // 获取环形缓冲区最旧(老)的时间戳
  bool getFirstTime(double& firstTime) {
    if (!measMap_.empty()) {
      firstTime = measMap_.begin()->first;
      return true;
    } else {
      return false;
    }
  }

  // 获取环形缓冲区最近(新)的测量数据
  bool getLastMeas(Meas& lastMeas) {
    if (!measMap_.empty()) {
      lastMeas = measMap_.rbegin()->second;
      return true;
    } else {
      return false;
    }
  }

  // 获取环形缓冲区次新的测量数据
  bool getLastLastMeas(Meas& lastlastMeas) {
    if (measMap_.size() >= 2) {
      auto itr = measMap_.rbegin();
      itr++;
      lastlastMeas = itr->second;
      return true;
    } else {
      return false;
    }
  }

  // 获取环形缓冲区最老的测量数据
  bool getFirstMeas(Meas& firstMeas) {
    if (!measMap_.empty()) {
      firstMeas = measMap_.begin()->second;
      return true;
    } else {
      return false;
    }
  }

  // 查看环形缓冲区是否有保存在某个时间戳上的测量数据
  bool hasMeasurementAt(double t) { return measMap_.count(t) > 0; }

  bool empty() { return measMap_.empty(); }

  void printContainer() {
    itMeas_ = measMap_.begin();
    while (measMap_.size() >= 1 && itMeas_ != measMap_.end()) {
      std::cout << itMeas_->second << " ";
      itMeas_++;
    }
  }
};
#endif // MAPRINGBUFFER_H

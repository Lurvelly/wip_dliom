#pragma once

#include <mutex>
#include <boost/circular_buffer.hpp>

template <typename T, size_t N>
class SafeBuffer {
public:
  SafeBuffer() { buffer.set_capacity(N); }
  ~SafeBuffer() = default;
  
  void push_back(const T& element) {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    this->buffer.push_back(element);
  }

  void push_back(T&& element) {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    this->buffer.push_back(element);
  }

  void pop_back() {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    this->buffer.pop_back();
  }

  void pop_front() {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    this->buffer.pop_back();
  }

  T& front() {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    this->buffer.front();
  }

  bool empty() {
    std::unique_lock<decltype(this->mutex)> lock(this->mutex);
    return this->buffer.empty();
  }

private:
  std::mutex mutex {};
  boost::circular_buffer<T> buffer {};
};

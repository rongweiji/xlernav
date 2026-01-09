#pragma once

#include <opencv2/opencv.hpp>

#include <string>

namespace xlernav {

std::string BuildH264RtpPipeline(int port, const std::string & decoder);

class StreamReceiver {
public:
  StreamReceiver(int port, std::string decoder);

  bool Open();
  bool Read(cv::Mat & frame);
  bool IsOpened() const;

private:
  int port_;
  std::string decoder_;
  cv::VideoCapture cap_;
};

}  // namespace xlernav

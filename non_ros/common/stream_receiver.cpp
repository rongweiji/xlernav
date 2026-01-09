#include "stream_receiver.hpp"

#include <utility>

namespace xlernav {

std::string BuildH264RtpPipeline(int port, const std::string & decoder)
{
  return
    "udpsrc port=" + std::to_string(port) +
    " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" ! "
    "rtph264depay ! h264parse ! " + decoder + " ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false";
}

StreamReceiver::StreamReceiver(int port, std::string decoder)
  : port_(port),
    decoder_(std::move(decoder))
{
}

bool StreamReceiver::Open()
{
  cap_.open(BuildH264RtpPipeline(port_, decoder_), cv::CAP_GSTREAMER);
  return cap_.isOpened();
}

bool StreamReceiver::Read(cv::Mat & frame)
{
  return cap_.read(frame);
}

bool StreamReceiver::IsOpened() const
{
  return cap_.isOpened();
}

}  // namespace xlernav

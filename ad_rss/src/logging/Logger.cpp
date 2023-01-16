#include "ad/rss/logging/Logger.hpp"

namespace ad
{
namespace rss
{
namespace logging
{

RssLogger* RssLogger::instance_{nullptr};
std::mutex RssLogger::mtx_;

RssLogger * RssLogger::getInstance()
{
  if (instance_ == nullptr)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (instance_ == nullptr)
    {
      instance_ = new RssLogger();
    }
  }
  return instance_;
}

void RssLogger::logError(const std::string & msg)
{
  spdlog::error(msg);
}

}
}
}

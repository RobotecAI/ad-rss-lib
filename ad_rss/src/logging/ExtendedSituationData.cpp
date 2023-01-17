#include "ad/rss/logging/ExtendedSituationData.hpp"

namespace ad
{
namespace rss
{
namespace logging
{

ExtendedSituationData* ExtendedSituationData::instance_{nullptr};
std::mutex ExtendedSituationData::mtx_;

ExtendedSituationData & ExtendedSituationData::getInstance()
{
  if (instance_ == nullptr)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (instance_ == nullptr)
    {
      instance_ = new ExtendedSituationData();
    }
  }
  return *instance_;
}

void ExtendedSituationData::setSituationData(DataIntersection & data_variant)
{
  situation_data.data_variant = &data_variant;
}

void ExtendedSituationData::setSituationData(DataNonIntersection & data_variant)
{
  situation_data.data_variant = &data_variant;
}

void ExtendedSituationData::setSituationData(DataUnstructured & data_variant)
{
  situation_data.data_variant = &data_variant;
}

}
}
}

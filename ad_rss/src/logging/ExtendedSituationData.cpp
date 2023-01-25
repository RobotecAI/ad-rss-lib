#include "ad/rss/logging/ExtendedSituationData.hpp"

namespace ad {
namespace rss {
namespace logging {

ExtendedSituationData *ExtendedSituationData::instance_{nullptr};
std::mutex ExtendedSituationData::mtx_;

ExtendedSituationData &ExtendedSituationData::getInstance()
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

void ExtendedSituationData::clear()
{
  situation_data.clear();
}

void ExtendedSituationData::setSituationData(SituationData &situation)
{
  this->situation_data.push_back(situation);
}

// SituationData

void SituationData::setSituationData(DataIntersection &data_variant)
{
  data_variant_ = &data_variant;
}

void SituationData::setSituationData(DataNonIntersection &data_variant)
{
  data_variant_ = &data_variant;
}

void SituationData::setSituationData(DataUnstructured &data_variant)
{
  data_variant_ = &data_variant;
}

} // namespace logging
} // namespace rss
} // namespace ad

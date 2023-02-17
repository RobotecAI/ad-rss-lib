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

SituationData &ExtendedSituationData::safeGetLastSituationDataElement()
{
  if (situation_data.empty())
  {
    SituationData sd = SituationData();
    situation_data.push_back(sd);
  }
  return situation_data.back();
}

DataIntersection &SituationData::getDataIntersection()
{
  if (data_intersection_.has_value())
  {
    return data_intersection_.value();
  }
  else
  {
    data_intersection_ = DataIntersection();
    return data_intersection_.value();
  }
}

DataNonIntersection &SituationData::getDataNonIntersection()
{
  if (data_non_intersection_.has_value())
  {
    return data_non_intersection_.value();
  }
  else
  {
    data_non_intersection_ = DataNonIntersection();
    return data_non_intersection_.value();
  }
}

DataUnstructured &SituationData::getDataUnstructured()
{
  if (data_unstructured_.has_value())
  {
    return data_unstructured_.value();
  }
  else
  {
    data_unstructured_ = DataUnstructured();
    return data_unstructured_.value();
  }
}

void SituationData::setDataIntersection(DataIntersection data)
{
  data_intersection_ = data;
}

void SituationData::setDataNonIntersection(DataNonIntersection data)
{
  data_non_intersection_ = data;
}

void SituationData::setDataUnstructured(DataUnstructured data)
{
  data_unstructured_ = data;
}

} // namespace logging
} // namespace rss
} // namespace ad

// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2023 Intel Corporation and Robotec.ai
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include <memory>
#include <type_traits>
#include <variant>
#include "ad/rss/situation/SituationSnapshot.hpp"
#include "ad/rss/state/ProperResponse.hpp"
#include "ad/rss/state/RssStateSnapshot.hpp"
#include "ad/rss/world/WorldModel.hpp"

/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief namespace logging
 */
namespace logging {

template <typename E> constexpr auto to_underlying(E e) noexcept
{
  return static_cast<std::underlying_type_t<E>>(e);
}

enum class SituationTypeId
{
  NotRelevant = 0,
  SameDirection = 1,
  OppositeDirection = 2,
  IntersectionEgoHasPriority = 3,
  IntersectionObjectHasPriority = 4,
  IntersectionSamePriority = 5,
  Unstructured = 6
};

struct DataIntersection
{
  std::string longitudinal_relative_position;
  int longitudinal_relative_position_id;
  double npc_safe_distance_to_intersection;
  double npc_current_distance_to_intersection;
  double ego_safe_distance_to_intersection;
  double ego_current_distance_to_intersection;
  double npc_time_to_reach_intersection;
  double npc_time_to_leave_intersection;
  double ego_time_to_reach_intersection;
  double ego_time_to_leave_intersection;
  double same_direction_current_distance;
  double same_direction_safe_distance;
  std::string previous_intersection_state_type;
  int previous_intersection_state_type_id;
  std::string current_intersection_state_type;
  int current_intersection_state_type_id;

  static DataIntersection &getInstance()
  {
    static DataIntersection instance;
    return instance;
  }

  DataIntersection(DataIntersection const &) = delete;
  DataIntersection(DataIntersection &&) = delete;

private:
  DataIntersection(){};
};

struct DataNonIntersection
{
  bool is_ego_in_correct_lane; // Only for opposite direction
  std::string lateral_relative_position;
  int lateral_relative_position_id;
  std::string longitudinal_relative_position;
  int longitudinal_relative_position_id;

  double longitudinal_safe_distance;
  double longitudinal_current_distance;
  double lateral_left_safe_distance;
  double lateral_left_current_distance;
  double lateral_right_safe_distance;
  double lateral_right_current_distance;

  static DataNonIntersection &getInstance()
  {
    static DataNonIntersection instance;
    return instance;
  }

  DataNonIntersection(DataNonIntersection const &) = delete;
  DataNonIntersection(DataNonIntersection &&) = delete;

private:
  DataNonIntersection(){};
};

struct DataUnstructured
{
  std::string unstructured_response;
  int unstructured_response_id;

  bool is_safe_ego_must_brake;
  bool is_safe_other_must_brake;
  bool is_safe_brake_both;

  // Related to unsafe situation
  bool if_unsafe_are_both_car_at_full_stop = false;
  bool if_unsafe_other_is_moving_ego_continue_forward = false;
  bool if_unsafe_other_is_stopped_ego_drive_away = false;

  static DataUnstructured &getInstance()
  {
    static DataUnstructured instance;
    return instance;
  }

  DataUnstructured(DataUnstructured const &) = delete;
  DataUnstructured(DataUnstructured &&) = delete;

private:
  DataUnstructured(){};
};

/*!
 * @brief structure SituationData
 *
 * The structure SituationData contains information about the object
 */

struct SituationData
{
  std::string situation_type;
  SituationTypeId situation_type_id;
  int object_id;
  std::string object_name; // Data not accessible, always filled "Unknown"
  bool is_safe;

  std::variant<DataIntersection *, DataNonIntersection *, DataUnstructured *> data_variant_;

  void setSituationData(DataIntersection &data_variant);
  void setSituationData(DataNonIntersection &data_variant);
  void setSituationData(DataUnstructured &data_variant);
};

class ExtendedSituationData
{
public:
  static ExtendedSituationData &getInstance();
  void clear();

  ExtendedSituationData(ExtendedSituationData &other) = delete;
  void operator=(const ExtendedSituationData &) = delete;

  void setSituationData(SituationData &situation);

  bool is_evaluation_successful = false;
  std::vector<SituationData> situation_data{};

protected:
  ExtendedSituationData(){};
  ~ExtendedSituationData(){};

private:
  static ExtendedSituationData *instance_;
  static std::mutex mtx_;
};

} // namespace logging
} // namespace rss
} // namespace ad
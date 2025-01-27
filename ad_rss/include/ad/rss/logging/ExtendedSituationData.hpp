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
#include <optional>
#include <type_traits>
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
  std::string previous_intersection_state_type = "NotStored";
  int previous_intersection_state_type_id = -1;
  std::string current_intersection_state_type;
  int current_intersection_state_type_id;
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
};

struct DataUnstructured
{
  std::string unstructured_response;
  int unstructured_response_id;

  bool is_safe_ego_must_brake;
  bool is_safe_other_must_brake;
  bool is_safe_brake_both;

  // Related to unsafe situation
  bool are_both_car_at_full_stop = false;
  bool other_is_moving_ego_continue_forward = false;
  bool other_is_stopped_ego_drive_away = false;
};

/*!
 * @brief structure SituationData
 *
 * The structure SituationData contains information about the object
 */

class SituationData
{
public:
  DataIntersection &getDataIntersection();
  DataNonIntersection &getDataNonIntersection();
  DataUnstructured &getDataUnstructured();

  void setDataIntersection(DataIntersection data);
  void setDataNonIntersection(DataNonIntersection data);
  void setDataUnstructured(DataUnstructured data);

  std::string situation_type;
  SituationTypeId situation_type_id;
  int object_id;
  std::string object_name; // Not defined at ad-rss-lib, filled "Unknown"
  bool is_safe;

protected:
  std::optional<DataIntersection> data_intersection_;
  std::optional<DataNonIntersection> data_non_intersection_;
  std::optional<DataUnstructured> data_unstructured_;
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

  SituationData &safeGetLastSituationDataElement();

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
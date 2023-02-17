// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2021 Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "RssStructuredSceneIntersectionChecker.hpp"
#include <cmath>
#include <limits>
#include "ad/rss/logging/ExtendedSituationData.hpp"
#include "ad/rss/situation/Physics.hpp"
#include "ad/rss/situation/RssFormulas.hpp"

namespace ad {

using physics::Duration;

namespace rss {
namespace situation {

using situation::calculateTimeToCoverDistance;

bool RssStructuredSceneIntersectionChecker::checkLateralIntersect(Situation const &situation, bool &isSafe)
{
  isSafe = false;
  auto &extended_situation_data = logging::ExtendedSituationData::getInstance();
  auto &data_intersection = extended_situation_data.safeGetLastSituationDataElement().getDataIntersection();
  // auto &data_intersection = extended_situation_data.situation_data.back().data_intersection_.value();

  /**
   * Check if in any case the first vehicle has passed the intersection before the other
   * vehicle arrives at that point
   */

  Duration timeToReachEgo;
  Duration timeToReachOther;
  Duration timeToLeaveEgo;
  Duration timeToLeaveOther;

  bool result = calculateTimeToCoverDistance(situation.egoVehicleState.velocity.speedLon.maximum,
                                             situation.egoVehicleState.dynamics.maxSpeedOnAcceleration,
                                             situation.egoVehicleState.dynamics.responseTime,
                                             situation.egoVehicleState.dynamics.alphaLon.accelMax,
                                             situation.egoVehicleState.dynamics.alphaLon.brakeMin,
                                             situation.egoVehicleState.distanceToEnterIntersection,
                                             timeToReachEgo);

  result = result
    && calculateTimeToCoverDistance(situation.otherVehicleState.velocity.speedLon.maximum,
                                    situation.egoVehicleState.dynamics.maxSpeedOnAcceleration,
                                    situation.otherVehicleState.dynamics.responseTime,
                                    situation.otherVehicleState.dynamics.alphaLon.accelMax,
                                    situation.otherVehicleState.dynamics.alphaLon.brakeMin,
                                    situation.otherVehicleState.distanceToEnterIntersection,
                                    timeToReachOther);

  result = result
    && calculateTimeToCoverDistance(situation.egoVehicleState.velocity.speedLon.minimum,
                                    situation.egoVehicleState.dynamics.maxSpeedOnAcceleration,
                                    situation.egoVehicleState.dynamics.responseTime,
                                    situation.egoVehicleState.dynamics.alphaLon.brakeMax,
                                    situation.egoVehicleState.dynamics.alphaLon.brakeMax,
                                    situation.egoVehicleState.distanceToLeaveIntersection,
                                    timeToLeaveEgo);

  result = result
    && calculateTimeToCoverDistance(situation.otherVehicleState.velocity.speedLon.minimum,
                                    situation.egoVehicleState.dynamics.maxSpeedOnAcceleration,
                                    situation.otherVehicleState.dynamics.responseTime,
                                    situation.otherVehicleState.dynamics.alphaLon.brakeMax,
                                    situation.otherVehicleState.dynamics.alphaLon.brakeMax,
                                    situation.otherVehicleState.distanceToLeaveIntersection,
                                    timeToLeaveOther);

  data_intersection.ego_time_to_reach_intersection = static_cast<double>(timeToReachEgo);
  data_intersection.ego_time_to_leave_intersection = static_cast<double>(timeToLeaveEgo);
  data_intersection.npc_time_to_reach_intersection = static_cast<double>(timeToReachOther);
  data_intersection.npc_time_to_leave_intersection = static_cast<double>(timeToLeaveOther);

  if (result)
  {
    if ((timeToReachEgo > timeToLeaveOther) || (timeToReachOther > timeToLeaveEgo)
        || ((timeToReachEgo == std::numeric_limits<Duration>::max())
            && (timeToReachOther == std::numeric_limits<Duration>::max())))
    {
      /**
       * Currently we assume lateral overlap when there is a vehicle in an intersection
       * @todo This could be relaxed by calculating lateral distance here as well
       */
      isSafe = true;
    }
  }

  return result;
}

bool RssStructuredSceneIntersectionChecker::checkIntersectionSafe(Situation const &situation,
                                                                  state::RssStateInformation &rssStateInformation,
                                                                  bool &isSafe,
                                                                  IntersectionState &intersectionState)
{
  if ((situation.egoVehicleState.distanceToLeaveIntersection < situation.egoVehicleState.distanceToEnterIntersection)
      || (situation.otherVehicleState.distanceToLeaveIntersection
          < situation.otherVehicleState.distanceToEnterIntersection))
  {
    return false;
  }

  bool result = true;
  isSafe = false;
  const double not_calculated = -1.0;
  auto &extended_situation_data = logging::ExtendedSituationData::getInstance();
  auto &data_intersection = extended_situation_data.safeGetLastSituationDataElement().getDataIntersection();
  data_intersection.longitudinal_relative_position_id
    = logging::to_underlying(situation.relativePosition.longitudinalPosition);

  /**
   * Check if a non prio vehicle has safe distance to the intersection
   */
  if (!situation.egoVehicleState.hasPriority)
  {
    rssStateInformation.evaluator = state::RssStateEvaluator::IntersectionOtherPriorityEgoAbleToStop;
    rssStateInformation.currentDistance = situation.egoVehicleState.distanceToEnterIntersection;
    result = checkStopInFrontIntersection(situation.egoVehicleState, rssStateInformation.safeDistance, isSafe);

    data_intersection.ego_current_distance_to_intersection
      = static_cast<double>(situation.egoVehicleState.distanceToEnterIntersection);
    data_intersection.ego_safe_distance_to_intersection = static_cast<double>(rssStateInformation.safeDistance);
    data_intersection.npc_current_distance_to_intersection = not_calculated; // Not calculated
    data_intersection.npc_safe_distance_to_intersection = not_calculated;
  }
  if (result && !isSafe && !situation.otherVehicleState.hasPriority)
  {
    rssStateInformation.evaluator = state::RssStateEvaluator::IntersectionEgoPriorityOtherAbleToStop;
    rssStateInformation.currentDistance = situation.otherVehicleState.distanceToEnterIntersection;
    result = checkStopInFrontIntersection(situation.otherVehicleState, rssStateInformation.safeDistance, isSafe);

    data_intersection.ego_current_distance_to_intersection = not_calculated;
    data_intersection.ego_safe_distance_to_intersection = not_calculated;
    data_intersection.npc_current_distance_to_intersection
      = static_cast<double>(situation.otherVehicleState.distanceToEnterIntersection);
    data_intersection.npc_safe_distance_to_intersection = static_cast<double>(rssStateInformation.safeDistance);
  }

  if (isSafe)
  {
    intersectionState = IntersectionState::NonPrioAbleToBreak;
  }
  else if (result)
  {
    rssStateInformation.currentDistance = situation.relativePosition.longitudinalDistance;
    /**
     * Check if there is a safe longitudinal distance between the vehicles
     */
    if (situation.relativePosition.longitudinalPosition == LongitudinalRelativePosition::InFront)
    {
      rssStateInformation.evaluator = state::RssStateEvaluator::IntersectionEgoInFront;
      result = checkSafeLongitudinalDistanceSameDirection(situation.egoVehicleState,
                                                          situation.otherVehicleState,
                                                          situation.relativePosition.longitudinalDistance,
                                                          rssStateInformation.safeDistance,
                                                          isSafe);

      data_intersection.longitudinal_relative_position = "EgoInFront";
      data_intersection.longitudinal_relative_position_id
        = logging::to_underlying(situation.relativePosition.longitudinalPosition);
      data_intersection.same_direction_current_distance
        = static_cast<double>(situation.relativePosition.longitudinalDistance);
      data_intersection.same_direction_safe_distance = static_cast<double>(rssStateInformation.safeDistance);
    }
    else
    {
      rssStateInformation.evaluator = state::RssStateEvaluator::IntersectionOtherInFront;
      result = checkSafeLongitudinalDistanceSameDirection(situation.otherVehicleState,
                                                          situation.egoVehicleState,
                                                          situation.relativePosition.longitudinalDistance,
                                                          rssStateInformation.safeDistance,
                                                          isSafe);

      data_intersection.longitudinal_relative_position = "OtherInFront";
      data_intersection.longitudinal_relative_position_id
        = logging::to_underlying(situation.relativePosition.longitudinalPosition);
      data_intersection.same_direction_current_distance
        = static_cast<double>(situation.relativePosition.longitudinalDistance);
      data_intersection.same_direction_safe_distance = static_cast<double>(rssStateInformation.safeDistance);
    }
    if (isSafe)
    {
      intersectionState = IntersectionState::SafeLongitudinalDistance;
    }
    else if (result)
    {
      rssStateInformation.evaluator = state::RssStateEvaluator::IntersectionOverlap;
      rssStateInformation.currentDistance = physics::Distance(0.);
      rssStateInformation.safeDistance = physics::Distance(0.);
      result = checkLateralIntersect(situation, isSafe);

      if (isSafe)
      {
        intersectionState = IntersectionState::NoTimeOverlap;
      }
    }
  }
  data_intersection.current_intersection_state_type_id = static_cast<int>(logging::to_underlying(intersectionState));
  data_intersection.current_intersection_state_type = intersectionStateToString(intersectionState);

  return result;
}

bool RssStructuredSceneIntersectionChecker::calculateRssStateIntersection(world::TimeIndex const &timeIndex,
                                                                          Situation const &situation,
                                                                          state::RssState &rssState)
{
  if (situation.egoVehicleState.hasPriority && situation.otherVehicleState.hasPriority)
  {
    // both cannot have priority over the other at the same time
    return false;
  }
  bool result = false;
  auto &extended_situation_data = logging::ExtendedSituationData::getInstance();
  extended_situation_data.situation_data.push_back(logging::SituationData());
  logging::SituationData &situation_data = extended_situation_data.safeGetLastSituationDataElement();
  situation_data.setDataIntersection(logging::DataIntersection());

  if (situation.egoVehicleState.hasPriority)
  {
    situation_data.situation_type_id = logging::SituationTypeId::IntersectionEgoHasPriority;
    situation_data.situation_type = "IntersectionEgoHasPriority";
  }
  else if (situation.otherVehicleState.hasPriority)
  {
    situation_data.situation_type_id = logging::SituationTypeId::IntersectionObjectHasPriority;
    situation_data.situation_type = "IntersectionObjectHasPriority";
  }
  else
  {
    situation_data.situation_type_id = logging::SituationTypeId::IntersectionSamePriority;
    situation_data.situation_type = "IntersectionSamePriority";
  }

  try
  {
    if (timeIndex != mCurrentTimeIndex)
    {
      /**
       * next time step: current safe state map becomes last state now
       */
      mLastSafeStateMap.swap(mNewSafeStateMap);
      mNewSafeStateMap.clear();
      mCurrentTimeIndex = timeIndex;
    }

    rssState.situationId = situation.situationId;
    rssState.situationType = situation.situationType;
    rssState.objectId = situation.objectId;

    rssState.longitudinalState.isSafe = false;
    rssState.longitudinalState.response = state::LongitudinalResponse::BrakeMin;
    rssState.longitudinalState.alphaLon = situation.egoVehicleState.dynamics.alphaLon;

    /**
     * An intersection situation is lateral unsafe but usually doesn't require a lateral brake
     * @todo: if taking lateral intersection handling into account, this also has to be updated
     */
    rssState.lateralStateLeft.isSafe = false;
    rssState.lateralStateLeft.response = state::LateralResponse::None;
    rssState.lateralStateLeft.alphaLat = situation.egoVehicleState.dynamics.alphaLat;
    rssState.lateralStateLeft.rssStateInformation.evaluator = state::RssStateEvaluator::LateralDistance;
    rssState.lateralStateLeft.rssStateInformation.currentDistance = physics::Distance(0);
    rssState.lateralStateLeft.rssStateInformation.safeDistance = physics::Distance(0);
    rssState.lateralStateRight.isSafe = false;
    rssState.lateralStateRight.response = state::LateralResponse::None;
    rssState.lateralStateRight.alphaLat = situation.egoVehicleState.dynamics.alphaLat;
    rssState.lateralStateRight.rssStateInformation.evaluator = state::RssStateEvaluator::LateralDistance;
    rssState.lateralStateRight.rssStateInformation.currentDistance = physics::Distance(0);
    rssState.lateralStateRight.rssStateInformation.safeDistance = physics::Distance(0);

    bool isSafe = false;
    IntersectionState intersectionState = IntersectionState::NonPrioAbleToBreak;

    /**
     * Check if the intersection is safe and determine the intersection state of the situation
     */
    result
      = checkIntersectionSafe(situation, rssState.longitudinalState.rssStateInformation, isSafe, intersectionState);

    if (result)
    {
      rssState.longitudinalState.isSafe = isSafe;

      auto const previousIntersectionState = mLastSafeStateMap.find(situation.situationId);

      if (!isSafe)
      {
        /**
         * Situation is unsafe determine proper response
         */
        if (previousIntersectionState != mLastSafeStateMap.end())
        {
          auto &data_intersection = extended_situation_data.safeGetLastSituationDataElement().getDataIntersection();
          data_intersection.previous_intersection_state_type_id
            = static_cast<int>(logging::to_underlying(previousIntersectionState->second));
          data_intersection.previous_intersection_state_type
            = intersectionStateToString(previousIntersectionState->second);
          switch (previousIntersectionState->second)
          {
            case IntersectionState::NonPrioAbleToBreak:
            {
              if (situation.egoVehicleState.hasPriority)
              {
                rssState.longitudinalState.response = state::LongitudinalResponse::None;
              }
              break;
            }
            case IntersectionState::SafeLongitudinalDistance:
            {
              if (situation.relativePosition.longitudinalPosition == LongitudinalRelativePosition::InFront)
              {
                rssState.longitudinalState.response = state::LongitudinalResponse::None;
              }
              break;
            }
            case IntersectionState::NoTimeOverlap:
            {
              // @todo If we don't assume always lateral overlap we might need to brake laterally as well
              rssState.longitudinalState.response = state::LongitudinalResponse::BrakeMin;
              break;
            }
            default:
            {
              // LCOV_EXCL_START: unreachable code, keep to be on the safe side
              result = false;
              break;
              // LCOV_EXCL_STOP: unreachable code, keep to be on the safe side
            }
          }

          /**
           * Store the last safe intersection state for next time step
           */
          mNewSafeStateMap.insert(
            RssIntersectionStateMap::value_type(situation.situationId, previousIntersectionState->second));
        }
        else
        {
          /**
           * Situation is unsafe but there is no previous safe state. Therefore we can only decide to brake.
           */
          rssState.longitudinalState.response = state::LongitudinalResponse::BrakeMin;
        }
      }
      else
      {
        /**
         * Situation is safe
         */
        rssState.longitudinalState.response = state::LongitudinalResponse::None;

        // store the new safe state
        mNewSafeStateMap.insert(RssIntersectionStateMap::value_type(situation.situationId, intersectionState));
      }
    }
  }
  catch (...)
  {
    result = false;
  }
  situation_data.is_safe = rssState.longitudinalState.isSafe; // Intersection is always unsafe laterally
  situation_data.object_id = static_cast<int>(situation.objectId);
  situation_data.object_name = "Unknown";
  return result;
}

} // namespace situation
} // namespace rss
} // namespace ad

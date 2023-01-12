#ifndef OBSERVATION_MANAGER_IC_H
#define OBSERVATION_MANAGER_IC_H

#include "controllers/utility/OSP_messages.h"
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <tuple>
#include <optional>
#include <algorithm>
#include <numeric>
#include <string>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class FlockingObservationValue
{
private:
    std::tuple<CVector2, CVector2, std::size_t> contents;

public:
    FlockingObservationValue(CVector2 lbnd, CVector2 ubnd, std::size_t hop_count)
    {
        std::get<0>(contents) = lbnd;
        std::get<1>(contents) = ubnd;
        std::get<2>(contents) = hop_count;
    }
    FlockingObservationValue(FlockingObservationMessage o_msg, std::size_t hop_count) : FlockingObservationValue(o_msg.m_lbnd, o_msg.m_ubnd, hop_count) {}
    CVector2 GetLBnd() { return std::get<0>(contents); }
    CVector2 GetUBnd() { return std::get<1>(contents); }
    std::size_t GetHopCount() { return std::get<2>(contents); }
};

class FlockingObservationManager : public ObservationManager<FlockingObservationValue>
{
public:
    std::optional<CVector2> guess(Real, std::size_t);
    bool is_suspicious(OSP_Message o_msg,
                       std::size_t curr_t,
                       Real displacement,
                       Real radio_range,
                       Real observation_range,
                       CVector2 curr_pos,
                       bool obs_made);
};

#endif
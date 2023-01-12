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

class TimeSyncObservationValue
{
private:
    std::tuple<UInt32, std::size_t> contents;

public:
    TimeSyncObservationValue(UInt32 time, std::size_t hop_count)
    {
        std::get<UInt32>(contents) = time;
        std::get<std::size_t>(contents) = hop_count;
    }
    TimeSyncObservationValue(TimeSyncObservationMessage o_msg, std::size_t hop_count) : TimeSyncObservationValue(o_msg.time, hop_count) {}
    UInt32 GetTime() { return std::get<UInt32>(contents); }
    std::size_t GetHopCount() { return std::get<std::size_t>(contents); }
};

class TimeSyncObservationManager : public ObservationManager<TimeSyncObservationValue>
{
public:
    std::optional<Real> guess();
    bool is_suspicious(OSP_Message o_msg,
                       Real curr_t,
                       bool obs_made);
};

#endif
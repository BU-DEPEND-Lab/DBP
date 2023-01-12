#include "message_manager.h"
#include <argos3/core/utility/logging/argos_log.h>

std::optional<Real> TimeSyncObservationManager::guess()
{
    if (observation_map.size() == 0)
        return std::nullopt;
    auto res = std::max_element(observation_map.begin(), observation_map.end(), [](auto l, auto r)
                                { return l.second.GetTime() < r.second.GetTime(); });
    return res->second.GetTime();
}

bool TimeSyncObservationManager::is_suspicious(
    OSP_Message o_msg,
    Real curr_t,
    bool obs_made)
{
    auto obs = std::get<TimeSyncObservationMessage>(o_msg.m_msg_variant);
    return obs_made && (obs.time >= curr_t);
}

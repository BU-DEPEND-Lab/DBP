#include "message_manager.h"
#include <argos3/core/utility/logging/argos_log.h>

auto const ONE = CVector2(1., 1.);

CVector2 p_min(CVector2 a, CVector2 b)
{
    return CVector2(
        std::min<Real>(a.GetX(), b.GetX()),
        std::min<Real>(a.GetY(), b.GetY()));
}

CVector2 p_max(CVector2 a, CVector2 b)
{
    return CVector2::ZERO - p_min(-a, -b);
}

std::optional<Location> merge(Location a, Location b)
{
    Location ret;
    ret.first = p_max(a.first, b.first);
    ret.second = p_min(a.second, b.second);
    return (ret.second.GetX() > ret.first.GetX() &&
            ret.second.GetY() > ret.first.GetY())
               ? std::optional<Location>{ret}
               : std::nullopt;
}

std::optional<std::pair<Location, BaseObs>> LocalizationObservationManager::guess(Real displacement, Real radio_range)
{
    if (observation_map.size() == 0)
        return std::nullopt;
    std::vector<std::pair<MessageKey, LocalizationObservationValue>> obs_vec(observation_map.begin(), observation_map.end());
    std::sort(obs_vec.begin(), obs_vec.end(), [](auto l, auto r)
              {
        if (!l.second.is_direct || !r.second.is_direct) return false;
        return l.second.base_timestamp < r.second.base_timestamp; });
    std::stable_sort(obs_vec.begin(), obs_vec.end(), [](auto l, auto r)
                     { return l.second.is_direct && !r.second.is_direct; });
    Location res = std::make_pair(
        CVector2(-1000, -1000),
        CVector2(1000, 1000));
    for (auto obs : obs_vec)
    {
        Location other_box = {obs.second.lbnd - (displacement + radio_range) * ONE,
                              obs.second.ubnd + (displacement + radio_range) * ONE};
        auto merged = merge(res, other_box);
        if (!merged.has_value())
            break;
        res = merged.value();
    }
    BaseObs base_obs = make_tuple(
        obs_vec[0].second.is_direct ? obs_vec[0].first.GetOrigin() : obs_vec[0].second.base_origin.value(),
        obs_vec[0].second.is_direct ? obs_vec[0].first.GetTime() : obs_vec[0].second.base_timestamp.value(),
        obs_vec[0].second.is_direct ? obs_vec[0].second.lbnd : obs_vec[0].second.base_lbnd.value(),
        obs_vec[0].second.is_direct ? obs_vec[0].second.ubnd : obs_vec[0].second.base_ubnd.value());
    return std::make_pair(res, base_obs);
}

std::optional<std::string> LocalizationObservationManager::is_suspicious(OSP_Message o_msg,
                                                                         Real curr_t,
                                                                         bool obs_made,
                                                                         Real displacement,
                                                                         Real radio_range,
                                                                         CVector2 curr_pos)
{
    auto obs = std::get<LocalizationObservationMessage>(o_msg.m_msg_variant);
    // TODO too close direct observation violation
    if (obs_made)
    {
        // check byzantine direct observations
        if (obs.is_direct)
        {
            if (!merge(std::make_pair(curr_pos - displacement * ONE, curr_pos + displacement * ONE),
                       std::make_pair(obs.lbnd - radio_range * ONE, obs.ubnd + radio_range * ONE)) // lbnd and ubnd should be the same for direct obs
                     .has_value())
            {
                auto self_box = std::make_pair(curr_pos - displacement * ONE, curr_pos + displacement * ONE);
                auto other_box = std::make_pair(obs.lbnd - radio_range * ONE, obs.ubnd + radio_range * ONE);
                return o_msg.m_origin;
            }
        }
        else
        {
            if (!merge(std::make_pair(curr_pos - displacement * ONE, curr_pos + displacement * ONE),
                       std::make_pair(obs.base_lbnd.value() - (curr_t - obs.base_timestamp.value()) * radio_range * ONE,
                                      obs.base_ubnd.value() + (curr_t - obs.base_timestamp.value()) * radio_range * ONE))
                     .has_value())
                return obs.base_origin;
        }
    }
    // check for base / local mismatch TODO
    return std::nullopt;
}

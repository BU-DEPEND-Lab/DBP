#include "message_manager.h"
#include <argos3/core/utility/logging/argos_log.h>

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

std::optional<CVector2> FlockingObservationManager::guess(Real displacement, std::size_t curr_t)
{
    std::vector<std::pair<CVector2, CVector2>> boxes;
    auto ONE = CVector2(1., .1);
    for (auto obs : observation_map)
    {
        assert(curr_t > obs.first.GetTime());
        boxes.push_back({obs.second.GetLBnd() - (curr_t - obs.first.GetTime()) * displacement * ONE,
                         obs.second.GetUBnd() + (curr_t - obs.first.GetTime()) * displacement * ONE});
    }
    auto interior = std::accumulate(
        boxes.begin(),
        boxes.end(),
        std::make_pair(
            CVector2(-1000, -1000),
            CVector2(1000, 1000)),
        [](auto l, auto r)
        {
            auto ret = std::make_pair(
                CVector2(-1000, -1000),
                CVector2(1000, 1000));
            ret.first = p_max(l.first, r.first);
            ret.second = p_min(l.second, r.second);
            if (
                ret.second.GetX() > ret.first.GetX() &&
                ret.second.GetY() > ret.first.GetY())
                return ret;
            else
                return l;
        });
    if (
        interior.second.GetX() > interior.first.GetX() &&
        interior.second.GetY() > interior.first.GetY() &&
        interior.second.GetX() - interior.first.GetX() < 10 &&
        interior.second.GetY() - interior.first.GetY() < 10)
    {
        return (interior.first + interior.second) / 2;
    }
    return std::nullopt;
}

bool FlockingObservationManager::is_suspicious(
    OSP_Message o_msg,
    std::size_t curr_t,
    Real displacement,
    Real radio_range,
    Real observation_range,
    CVector2 curr_pos,
    bool obs_made)
{
    // reject if:
    //  - no observation was made locally, but should have made one
    //  - the observation would need to have moved faster  than cooperative nodes can propagate
    //  - observation contradicts what is observed locally
    auto r = Real(curr_t);
    auto obs = std::get<FlockingObservationMessage>(o_msg.m_msg_variant);
    auto delta_p = (curr_pos - std::get<FlockingObservationMessage>(o_msg.m_msg_variant).m_lbnd).Length();
    auto delta_t = Real(curr_t) - o_msg.m_timestamp;
    // if (o_msg.m_origin.find("fb") != std::string::npos)
    //     return false; // see if we have a soundness issue
    // speed of flood violation
    if (delta_t * radio_range + observation_range + 0.5 < delta_p)
        if (o_msg.m_origin.find("fb") != std::string::npos)
        {
            LOG << "echt waehlt echt :(" << std::endl;
            return true;
        }
        else
            return true;
    // missed observation
    if (delta_p < observation_range - (displacement - 0.1) * delta_t && !obs_made)
        if (o_msg.m_origin.find("fb") != std::string::npos)
        {
            LOG << "echt waehlt echt :(" << std::endl;
            return true;
        }
        else
            return true;
    // contradicts local observation
    if (delta_p > observation_range + (displacement + 0.1) * delta_t && obs_made)
        if (o_msg.m_origin.find("fb") != std::string::npos)
        {
            LOG << "echt waehlt echt :(" << std::endl;
            return true;
        }
        else
            return true;
    return false;
}

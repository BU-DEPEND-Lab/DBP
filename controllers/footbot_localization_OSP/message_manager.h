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

typedef std::pair<CVector2, CVector2> Location;

class LocalizationObservationValue
{
public:
    CVector2 lbnd;
    CVector2 ubnd;
    bool is_direct;
    // assume a signature from the origin
    // yikes... can't just include an optional OSP_Message (circular because of the MessageVariant needed the list of variant types)... that'd be nice tho
    std::optional<std::string> base_origin;
    std::optional<UInt32> base_timestamp;
    std::optional<CVector2> base_lbnd;
    std::optional<CVector2> base_ubnd;
    std::size_t hop_count;

    LocalizationObservationValue(CVector2 lbnd,
                                 CVector2 ubnd,
                                 bool is_direct,
                                 std::optional<std::string> base_origin,
                                 std::optional<UInt32> base_timestamp,
                                 std::optional<CVector2> base_lbnd,
                                 std::optional<CVector2> base_ubnd,
                                 std::size_t hop_count) : lbnd(lbnd),
                                                          ubnd(ubnd),
                                                          is_direct(is_direct),
                                                          base_origin(base_origin),
                                                          base_timestamp(base_timestamp),
                                                          base_lbnd(base_lbnd),
                                                          base_ubnd(base_ubnd),
                                                          hop_count(hop_count) {}
    LocalizationObservationValue(LocalizationObservationMessage o_msg, std::size_t hop_count) : LocalizationObservationValue(o_msg.lbnd,
                                                                                                                             o_msg.ubnd, o_msg.is_direct, o_msg.base_origin, o_msg.base_timestamp,
                                                                                                                             o_msg.base_lbnd, o_msg.base_ubnd, hop_count) {}
};

typedef std::tuple<std::string, UInt32, CVector2, CVector2> BaseObs;

class LocalizationObservationManager : public ObservationManager<LocalizationObservationValue>
{
public:
    std::optional<std::pair<Location, BaseObs>> guess(Real displacement, Real radio_range);
    std::optional<std::string> is_suspicious(OSP_Message o_msg,
                                             Real curr_t,
                                             bool obs_made,
                                             Real displacement,
                                             Real radio_range,
                                             CVector2 curr_pos);
};

typedef AccusationManager<LocalizationObservationValue> LocalizationAccusationManager;

#endif
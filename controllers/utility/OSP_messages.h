#ifndef OSP_MESSAGES_H
#define OSP_MESSAGES_H

#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/math/vector2.h>
#include <variant>
#include <cassert>
#include <boost/graph/adjacency_list.hpp>

using namespace boost;
using namespace argos;

typedef adjacency_list<vecS, vecS, undirectedS, std::string, no_property, no_property, listS> AccGraph;
typedef std::unordered_map<std::string, graph_traits<AccGraph>::vertex_descriptor> DescriptorMap;

template <typename... Ts>
[[nodiscard]] std::variant<Ts...>
expand_type(std::size_t i)
{
    assert(i < sizeof...(Ts));
    std::variant<Ts...> table[] = {Ts{}...};

    return table[i];
}

using namespace argos;

class Message
{
public:
    virtual void serialize(argos::CByteArray *target){};
    virtual void deserialize(argos::CByteArray *target){};
};
inline CByteArray vec_serialize(argos::CByteArray *target, CVector2 vec_value)
{
    *target << vec_value.GetX() << vec_value.GetY();
    return *target;
}
inline CByteArray vec_deserialize(argos::CByteArray *target, CVector2 &vec_value)
{
    Real x, y;
    *target >> x;
    *target >> y;
    vec_value.SetX(x);
    vec_value.SetY(y);
    return *target;
}

class FlockingObservationMessage : public Message
{
public:
    CVector2 m_lbnd;
    CVector2 m_ubnd;
    FlockingObservationMessage() {}
    FlockingObservationMessage(
        CVector2 lbnd,
        CVector2 ubnd) : m_lbnd(lbnd), m_ubnd(ubnd) {}
    void serialize(CByteArray *target) override
    {
        vec_serialize(target, m_lbnd);
        vec_serialize(target, m_ubnd);
    }
    void deserialize(CByteArray *target) override
    {
        vec_deserialize(target, m_lbnd);
        vec_deserialize(target, m_ubnd);
    }
};

class TimeSyncObservationMessage : public Message
{
public:
    UInt32 time;
    TimeSyncObservationMessage() {}
    TimeSyncObservationMessage(UInt32 time) : time(time) {}
    void serialize(CByteArray *target) override { *target << time; }
    void deserialize(CByteArray *target) override { *target >> time; }
};

class LocalizationObservationMessage : public Message
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
    LocalizationObservationMessage() {}
    LocalizationObservationMessage(CVector2 lbnd,
                                   CVector2 ubnd,
                                   bool is_direct,
                                   std::optional<std::string> base_origin = std::nullopt,
                                   std::optional<UInt32> base_timestamp = std::nullopt,
                                   std::optional<CVector2> base_lbnd = std::nullopt,
                                   std::optional<CVector2> base_ubnd = std::nullopt) : lbnd(lbnd), ubnd(ubnd), is_direct(is_direct), base_origin(base_origin), base_timestamp(base_timestamp), base_lbnd(base_lbnd), base_ubnd(base_ubnd) {}
    void serialize(CByteArray *target) override
    {
        vec_serialize(target, lbnd);
        vec_serialize(target, ubnd);
        *target << is_direct;
        if (!is_direct)
        {
            *target << base_origin.value() << base_timestamp.value();
            // assume indirect localization data is ignored without this
            vec_serialize(target, base_lbnd.value());
            vec_serialize(target, base_ubnd.value());
        }
    }
    void deserialize(CByteArray *target) override
    {
        SInt32 is_direct_sint32;
        vec_deserialize(target, lbnd);
        vec_deserialize(target, ubnd);
        *target >> is_direct_sint32;
        is_direct = is_direct_sint32;
        if (!is_direct)
        {
            std::string _base_origin;
            UInt32 _base_timestamp;
            CVector2 _base_lbnd, _base_ubnd;
            *target >> _base_origin;
            *target >> _base_timestamp;
            vec_deserialize(target, _base_lbnd);
            vec_deserialize(target, _base_ubnd);
            base_origin = _base_origin;
            base_timestamp = _base_timestamp;
            base_lbnd = _base_lbnd;
            base_ubnd = _base_ubnd;
        }
    }
};

class AccusationMessage : public Message
{
public:
    std::string m_accused;
    AccusationMessage() {}
    AccusationMessage(std::string accused) : m_accused(accused) {}
    void serialize(CByteArray *target) override
    {
        *target << m_accused;
    }
    void deserialize(CByteArray *target) override
    {
        *target >> m_accused;
    }
};

// Turns out that std::variant is also cursed
// the order you put the message types in here can have HUGE effect on performance
using MessageVariant = std::variant<FlockingObservationMessage, AccusationMessage, TimeSyncObservationMessage, LocalizationObservationMessage>;

template <typename>
struct tag
{
}; // <== this one IS literal

template <typename T, typename V>
struct get_index;

template <typename T, typename... Ts>
struct get_index<T, std::variant<Ts...>>
    : std::integral_constant<size_t, std::variant<tag<Ts>...>(tag<T>()).index()>
{
};

class OSP_Message : public Message
{
public:
    MessageVariant m_msg_variant;
    std::string m_origin;
    Real m_timestamp;
    std::size_t m_hop_count;
    OSP_Message() {}
    OSP_Message(MessageVariant msg_variant, std::string origin, Real timestamp, std::size_t hop_count) : m_msg_variant(msg_variant), m_origin(origin), m_timestamp(timestamp), m_hop_count(hop_count) {}
    void serialize(CByteArray *target)
    {
        *target << m_msg_variant.index() << m_origin << m_timestamp << m_hop_count;
        std::visit([target](auto &&visitor)
                   { visitor.serialize(target); },
                   m_msg_variant);
    }
    void deserialize(CByteArray *target)
    {
        std::size_t variant;
        *target >> variant;
        *target >> m_origin;
        *target >> m_timestamp;
        *target >> m_hop_count;
        m_msg_variant = expand_type<FlockingObservationMessage, AccusationMessage, TimeSyncObservationMessage, LocalizationObservationMessage>(variant);
        std::visit([target](auto &&visitor)
                   { visitor.deserialize(target); },
                   m_msg_variant);
    }
    void operator++()
    {
        m_hop_count++;
    }
    void operator++(int)
    {
        operator++();
    }
};

class Accusation
{
private:
    std::pair<std::string, std::string> contents;

public:
    std::string GetOrigin() { return contents.first; }
    std::string GetAccused() { return contents.second; }
    Accusation(std::string origin, std::string accused)
    {
        contents.first = origin;
        contents.second = accused;
    }
    Accusation(std::string origin, AccusationMessage a_msg) : Accusation(origin, a_msg.m_accused) {}
    bool operator<(const Accusation &other) const
    {
        if (contents.first < other.contents.first)
            return true;
        if (contents.first > other.contents.first)
            return false;
        return contents.second < other.contents.second;
    }
};

class MessageKey
{
private:
    std::size_t _variant;
    std::size_t _timestamp;
    std::string _origin;

public:
    MessageKey(std::size_t variant, std::size_t time, std::string id)
        : _variant(variant), _timestamp(time), _origin(id) {}
    MessageKey(OSP_Message msg) : MessageKey(msg.m_msg_variant.index(), msg.m_timestamp, msg.m_origin) {}
    bool operator==(const MessageKey &other) const
    {
        return (GetTime() == other.GetTime()) && (GetOrigin() == other.GetOrigin()) && (GetVariant() == other.GetVariant());
    }
    std::size_t GetTime() const { return _timestamp; }
    std::string GetOrigin() const { return _origin; }
    std::size_t GetVariant() const { return _variant; }
};

template <>
struct std::hash<MessageKey>
{
    std::size_t operator()(const MessageKey &mk) const
    {
        return std::hash<std::size_t>()(mk.GetTime()) ^
               std::hash<std::string>()(mk.GetOrigin()) ^
               std::hash<std::size_t>()(mk.GetVariant());
    }
};

template <class ObservationValue>
class ObservationManager
{
public:
    void add_observation(MessageKey, ObservationValue);
    std::optional<ObservationValue *> get_observation(MessageKey);
    void clear();
    void prune(std::string); // by origin
    void prune(std::unordered_set<std::string>);
    void prune(std::size_t, std::unordered_map<MessageKey, OSP_Message> &); // by age
protected:
    std::unordered_map<MessageKey, ObservationValue> observation_map;
};

template <class ObservationValue>
class AccusationManager
{
public:
    bool add_accused(Accusation, ObservationManager<ObservationValue> &);
    bool is_accused(std::string);
    void clear();

private:
    bool add_accused(std::string a, std::string b, DescriptorMap &dm, AccGraph &g);
    void update_accused_set();

public:
    std::set<Accusation> accusation_set;
    std::unordered_set<std::string> accused_set;
};

// IMPLEMENTATION
// this can't be in a separate file because C++ is cursed

#include <boost/graph/max_cardinality_matching.hpp>
#include <boost/property_map/property_map.hpp>

template <class ObservationValue>
bool AccusationManager<ObservationValue>::add_accused(Accusation accusation, ObservationManager<ObservationValue> &om)
{
    if (accusation_set.insert(accusation).second)
    {
        update_accused_set();
        om.prune(accused_set);
        return true;
    }
    return false;
}

template <class ObservationValue>
void AccusationManager<ObservationValue>::update_accused_set()
{
    accused_set.clear();
    graph_traits<AccGraph>::out_edge_iterator ei, ei_end;
    graph_traits<AccGraph>::vertex_iterator vi, vi_end, min_v;
    graph_traits<AccGraph>::degree_size_type min_degree;
    AccGraph g;
    DescriptorMap dm;

    for (auto accusation : accusation_set)
        add_accused(accusation.GetOrigin(), accusation.GetAccused(), dm, g);

    typedef graph_traits<AccGraph>::vertex_descriptor VD;
    std::map<VD, VD> match;
    associative_property_map<std::map<VD, VD>> mapAdapter(match);
    edmonds_maximum_cardinality_matching(g, mapAdapter);
    for (auto &i : match)
    {
        auto e = boost::edge(i.first, i.second, g);
        if (e.second)
        {
            accused_set.insert(g[i.first]);
            accused_set.insert(g[i.second]);
        }
    }
}

template <class ObservationValue>
bool AccusationManager<ObservationValue>::add_accused(std::string a, std::string b, DescriptorMap &dm, AccGraph &g)
{
    bool ret = false;
    if (dm.count(a) == 0)
        dm.insert({a, add_vertex(a, g)});
    if (dm.count(b) == 0)
        dm.insert({b, add_vertex(b, g)});
    if (!edge(dm[a], dm[b], g).second)
    {
        ret = true;
        add_edge(dm[a], dm[b], g);
    }
    return ret;
}

template <class ObservationValue>
bool AccusationManager<ObservationValue>::is_accused(std::string id)
{
    return accused_set.count(id) > 0;
}

template <class ObservationValue>
void AccusationManager<ObservationValue>::clear()
{
    accused_set.clear();
    accusation_set.clear();
}

template <class ObservationValue>
void ObservationManager<ObservationValue>::add_observation(MessageKey k, ObservationValue v)
{
    // for now, just keep one observation from each robot
    prune(k.GetOrigin());
    observation_map.emplace(k, v);
}

template <class ObservationValue>
std::optional<ObservationValue *> ObservationManager<ObservationValue>::get_observation(MessageKey k)
{
    auto elt = observation_map.find(k);
    if (elt == observation_map.end())
        return std::nullopt;
    return &(elt->second);
}

template <class ObservationValue>
void ObservationManager<ObservationValue>::clear()
{
    observation_map.clear();
}

template <class ObservationValue>
void ObservationManager<ObservationValue>::prune(std::string origin)
{
    for (auto it = observation_map.begin(); it != observation_map.end();)
    {
        if (it->first.GetOrigin() == origin)
        {
            it = observation_map.erase(it);
        }
        else
            ++it;
    }
}

template <class ObservationValue>
void ObservationManager<ObservationValue>::prune(std::size_t prune_age, std::unordered_map<MessageKey, OSP_Message> &messages)
{
    for (auto it = messages.begin(); it != messages.end();)
    {
        if (it->first.GetTime() < prune_age && it->first.GetVariant() != 1)
        {
            observation_map.erase(it->first);
            it = messages.erase(it);
        }
        else
            ++it;
    }
}

template <class ObservationValue>
void ObservationManager<ObservationValue>::prune(std::unordered_set<std::string> origins)
{
    for (auto origin : origins)
    {
        prune(origin);
    }
}

#endif
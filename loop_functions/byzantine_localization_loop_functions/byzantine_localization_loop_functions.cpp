#include "byzantine_localization_loop_functions.h"
#include <argos3/plugins/robots/foot-bot-networked/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include "../../controllers/footbot_localization_OSP/footbot_localization_OSP.h"

namespace argos
{

    /****************************************/
    /****************************************/

    void CTestLoopFunctions::Init(TConfigurationNode &t_tree)
    {
        try
        {
            try
            {
                simulation_time = std::stoi(t_tree.GetAttribute("simulation_time"));
            }
            catch (...)
            {
                THROW_ARGOSEXCEPTION("simulation_time not set");
            }
            tracking_error_output.open(
                t_tree.GetAttribute("tracking_error_output"));
            if (!tracking_error_output.is_open())
                THROW_ARGOSEXCEPTION("could not open tracking_error_output");
            blocklist_stats_output.open(
                t_tree.GetAttribute("blocklist_stats_output"));
            if (!blocklist_stats_output.is_open())
                THROW_ARGOSEXCEPTION("could not open blocklist_stats_output");
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("error parsing loop function parameters, ", ex);
        }
    }
    void CTestLoopFunctions::Destroy()
    {
        tracking_error_output.close();
        blocklist_stats_output.close();
    }
    void CTestLoopFunctions::PostStep()
    {
        auto target_loc = GetSpace().GetSimulationClock();
        for (auto ent : GetSpace().GetEntitiesByType("foot-bot-networked"))
        {
            CFootBotNetworkedEntity *fb = any_cast<CFootBotNetworkedEntity *>(ent.second);
            // if ((fb->GetId().find("fb") != std::string::npos) || (fb->GetId().find("byz") != std::string::npos))
            // {
            auto cont = (CFootBotLocalization_OSP *)&(fb->GetControllableEntity().GetController());
            CVector2 pos;
            cont->m_pcPosition->GetReading().Position.ProjectOntoXY(pos);
            tracking_error_output << GetSpace().GetSimulationClock() << "!!" << fb->GetId() << "!!";
            // the 8.0 here is the fixed tolerance in the controller... TODO fetch this from a controller variable
            tracking_error_output << (cont->GetPosition().has_value() ? abs(cont->GetPosition().value().GetX() - pos.GetX()) : 8.0) << "!!" << (cont->GetPosition().has_value() ? cont->GetPosition().value().GetX() : std::numeric_limits<double>::quiet_NaN()) << "!!" << pos.GetX();
            tracking_error_output << "!!" << cont->m_AccusationManager.accused_set.size() << std::endl;
            // }
        }
    }

    bool CTestLoopFunctions::IsExperimentFinished()
    {
        /* wait five ticks before evaluating the test */
        if (GetSpace().GetSimulationClock() < simulation_time)
        {
            return false;
        }
        else
        {
            auto bl_size = ComputeMinAndUnionBlocklistSize();
            blocklist_stats_output << "min blocklist size!!" << bl_size.first << std::endl;
            blocklist_stats_output << "blocklist union size!!" << bl_size.second << std::endl;
            return true;
        }
    }
    std::pair<std::size_t, std::size_t> CTestLoopFunctions::ComputeMinAndUnionBlocklistSize()
    {
        unordered_set<std::string> blocklist;
        int min_blocklist_size = INT_MAX;
        for (auto ent : GetSpace().GetEntitiesByType("foot-bot-networked"))
        {
            CFootBotNetworkedEntity *fb = any_cast<CFootBotNetworkedEntity *>(ent.second);
            if (fb->GetId().find("fb") != std::string::npos)
            {
                auto cont = (CFootBotLocalization_OSP *)&fb->GetControllableEntity().GetController();
                for (auto ign : cont->m_AccusationManager.accused_set)
                    blocklist.insert(ign);
                if (cont->m_AccusationManager.accused_set.size() < min_blocklist_size)
                    min_blocklist_size = cont->m_AccusationManager.accused_set.size();
            }
        }
        return std::make_pair(min_blocklist_size, blocklist.size());
    }
    /****************************************/
    /****************************************/

    REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "byzantine_localization_loop_functions");

}

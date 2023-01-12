#include "byzantine_time_sync_loop_functions.h"
#include <argos3/plugins/robots/foot-bot-networked/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include "../../controllers/footbot_time_sync_OSP/footbot_time_sync_OSP.h"
#include "../../controllers/footbot_time_sync_WMSR/footbot_time_sync_WMSR.h"

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
            if (t_tree.GetAttribute("controller_type") == "OSP")
                controller_type = OSP;
            else if (t_tree.GetAttribute("controller_type") == "WMSR")
                controller_type = WMSR;
            else
                THROW_ARGOSEXCEPTION("controller_type is not one of: OSP, WMSR");
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
            if (controller_type == OSP)
            {
                auto cont = (CFootBotTimeSync_OSP *)&(fb->GetControllableEntity().GetController());
                tracking_error_output << GetSpace().GetSimulationClock() << "!!" << fb->GetId() << "!!";
                tracking_error_output << (cont->local_time - target_loc) << "!!" << cont->local_time << "!!" << target_loc;
                tracking_error_output << "!!" << cont->m_AccusationManager.accused_set.size() << std::endl;
            }
            else if (controller_type == WMSR)
            {
                auto cont = (CFootBotTimeSync_WMSR *)&(fb->GetControllableEntity().GetController());
                tracking_error_output << GetSpace().GetSimulationClock() << "!!" << fb->GetId() << "!!";
                tracking_error_output << (cont->local_time - target_loc) << "!!" << cont->local_time << "!!" << target_loc;
                tracking_error_output << "!!" << 0 << std::endl;
            }
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
            if (controller_type == OSP)
            {
                auto bl_size = ComputeMinAndUnionBlocklistSize();
                blocklist_stats_output << "min blocklist size!!" << bl_size.first << std::endl;
                blocklist_stats_output << "blocklist union size!!" << bl_size.second << std::endl;
            }
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
                auto cont = (CFootBotTimeSync_OSP *)&fb->GetControllableEntity().GetController();
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

    REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "byzantine_time_sync_loop_functions");

}

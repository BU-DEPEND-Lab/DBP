#include "byzantine_flocking_loop_functions.h"
#include <argos3/plugins/robots/foot-bot-networked/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include "../../controllers/footbot_flocking_OSP_IC/footbot_flocking_OSP_IC.h"
#include "../../controllers/footbot_flocking_OSP_byzantine/footbot_flocking_OSP_byzantine.h"
#include "../../controllers/footbot_target/footbot_target.h"

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
        CVector2 target_loc;
        CFootBotEntity *target = dynamic_cast<CFootBotEntity *>(&GetSpace().GetEntity("target"));
        target_loc = ((CFootBotTarget *)(&target->GetControllableEntity().GetController()))->GetPosition();
        for (auto ent : GetSpace().GetEntitiesByType("foot-bot-networked"))
        {
            CFootBotNetworkedEntity *fb = any_cast<CFootBotNetworkedEntity *>(ent.second);
            if (fb->GetId().find("fb") != std::string::npos)
            {
                auto cont = (CFootBotFlocking_OSP_IC *)&(fb->GetControllableEntity().GetController());
                tracking_error_output << GetSpace().GetSimulationClock() << "!!" << fb->GetId() << "!!";
                if (cont->x.has_value())
                    tracking_error_output << abs(cont->x.value().GetX() - target_loc.GetX()) << "!!" << cont->x.value().GetX() << "!!" << target_loc.GetX();
                else
                    tracking_error_output << "!!!!";
                tracking_error_output << "!!" << cont->m_AccusationManager.accused_set.size() << std::endl;
            }
            else if (fb->GetId().find("byz") != std::string::npos)
            {
                auto cont = (CFootBotFlockingOSP_Byzantine *)&(fb->GetControllableEntity().GetController());
                tracking_error_output << GetSpace().GetSimulationClock() << "!!" << fb->GetId() << "!!";
                if (cont->x.has_value())
                    tracking_error_output << abs(cont->x.value().GetX() - target_loc.GetX()) << "!!" << cont->x.value().GetX() << "!!" << target_loc.GetX();
                else
                    tracking_error_output << "!!!!";
                tracking_error_output << "!!"
                                      << "0" << std::endl;
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
                auto cont = (CFootBotFlocking_OSP_IC *)&fb->GetControllableEntity().GetController();
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

    REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "byzantine_flocking_loop_functions");

}

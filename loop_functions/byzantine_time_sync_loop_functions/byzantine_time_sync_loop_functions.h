#ifndef TARGET_TRACKING_LOOP_FUNCTION_H
#define TARGET_TRACKING_LOOP_FUNCTION_H

#include <fstream>
#include <iostream>

namespace argos
{
    class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>

namespace argos
{

    class CTestLoopFunctions : public CLoopFunctions
    {

    public:
        CTestLoopFunctions() {}

        virtual ~CTestLoopFunctions() {}

        virtual void Init(TConfigurationNode &t_tree) override;

        virtual void Destroy() override;

        virtual void PostStep() override;

        virtual bool IsExperimentFinished() override;

        virtual std::pair<std::size_t, std::size_t> ComputeMinAndUnionBlocklistSize();

    private:
        std::ofstream tracking_error_output, blocklist_stats_output;
        int simulation_time;
        enum ControllerType
        {
            OSP,
            WMSR
        } controller_type;
    };
}

#endif
#pragma once

#include "MsgEmitter.hpp"
#include "GFMMState.hpp"
#include "UGVNavState.hpp"
#include "WaterFireExtState.hpp"
#include "GFIndoorFireDetectionStates.hpp"
#include "IntegerMsg.hpp"

class MissionStateManager : public msg_emitter
{
    private:
        GFMMState current_external_wall_fire_state;
    public:
        void updateMissionState(GFMMState);
        GFMMState getMissionState();
};

extern MissionStateManager mainMissionStateManager;
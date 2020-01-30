#pragma once

#include "MsgEmitter.hpp"
#include "GFMMState.hpp"
#include "UGVNavState.hpp"
#include "WaterFireExtState.hpp"
#include "GFIndoorFireDetectionStates.hpp"
#include "IntegerMsg.hpp"
#include "logger.hpp"

class MissionStateManager : public msg_emitter
{
    private:
        GFMMState current_state;
        void displayStateChange();
    public:
        void updateMissionState(GFMMState);
        GFMMState getMissionState();
};

extern MissionStateManager mainMissionStateManager;
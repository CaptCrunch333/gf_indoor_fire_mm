#include "MissionStateManager.hpp"
MissionStateManager mainMissionStateManager;

void MissionStateManager::updateMissionState(GFMMState t_current_state){
    current_external_wall_fire_state = t_current_state;
    IntegerMsg state_msg;
    state_msg.data=(int)t_current_state;
    emit_message(&state_msg);
}

GFMMState MissionStateManager::getMissionState(){
    return current_external_wall_fire_state;
}
#include "MissionStateManager.hpp"
MissionStateManager mainMissionStateManager;

void MissionStateManager::updateMissionState(GFMMState t_current_state){
    current_state = t_current_state;
    IntegerMsg state_msg;
    state_msg.data=(int)t_current_state;
    emit_message(&state_msg);
    displayStateChange();
}

GFMMState MissionStateManager::getMissionState(){
    return current_state;
}

void MissionStateManager::displayStateChange()
{
    switch (current_state)
    {
    case GFMMState::ERROR: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Error", LoggerLevel::Info);
        break;
    }
    case GFMMState::NOT_READY: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Not Ready", LoggerLevel::Info);
        break;
    }   
    case GFMMState::READY_TO_START: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Ready To Start", LoggerLevel::Info);
        break;
    }
    case GFMMState::HEADING_TOWARD_ENTRANCE: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Heading Towards Entrance", LoggerLevel::Info);
        break;
    }
    case GFMMState::SEARCHING_FOR_FIRE: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Searching For Fire", LoggerLevel::Info);
        break;
    }    
    case GFMMState::APPROACHING_FIRE: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Approaching Fire", LoggerLevel::Info);
        break;
    }
    case GFMMState::POSITIONING_UGV: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Extinguishing Fire", LoggerLevel::Info);
        break;
    }
    case GFMMState::EXTINGUISHING_FIRE: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Extinguishing Fire", LoggerLevel::Info);
        break;
    }
    case GFMMState::RETURNING_TO_BASE: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Returning To Base", LoggerLevel::Info);
        break;
    }
    case GFMMState::FINISHED: {
        Logger::getAssignedLogger()->log("GF MM State Changed To Finished", LoggerLevel::Info);
        break;
    }
    default:
        break;
    }
}
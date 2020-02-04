#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "FlightPipeline.hpp"
#include "FlightScenario.hpp"
#include "InternalSystemStateCondition.hpp"
#include "ChangeInternalState.hpp"
#include "ExternalSystemStateCondition.hpp"
#include "SendMessage.hpp"
#include "ROSUnit_Factory.hpp"
#include "MissionStateManager.hpp"
#include "UGVPatrolStates.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gf_indoor_fire_mm");
    ros::NodeHandle nh;
    // ************************************ LOGGER ************************************
    Logger::assignLogger(new StdLogger());
    Logger::getAssignedLogger()->log("start of logger", LoggerLevel::Info);
    //Logger::getAssignedLogger()->enableFileLog(LoggerLevel::Error);
    // ********************************************************************************
    // ************************************ ROSUNITS ************************************
    ROSUnit_Factory mainROSUnit_Factory_main{nh};
	ROSUnit* InternalStateUpdaterSrv = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/set_state");

    ROSUnit* FireDetectionStateUpdaterSrv = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_gf_indoor_fire_detection_state");
    ROSUnit* WaterExtStateUpdaterSrv = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_water_ext_state");
    ROSUnit* UGVNavCtrlUpdaterSrv = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_ugv_nav_state");
    ROSUnit* WaterLevelUpdaterSrv = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_water_level");

    ROSUnit* FireDetectionStateUpdaterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_detection/set_state");
	ROSUnit* WaterExtStateUpdaterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_mission_state");
    ROSUnit* WateLevelUpdateRequesterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/get_water_level"); 
    ROSUnit* UGVNavCtrlUpdaterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_mission_state");
    ROSUnit* UGVPatrolUpdaterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_patrol_mode");
    // ********************************************************************************
    // ******************************* FLIGHT ELEMENTS ********************************
    //Internal States //
    FlightElement* cs_to_error = new ChangeInternalState(GFMMState::ERROR);
    FlightElement* cs_to_not_ready = new ChangeInternalState(GFMMState::NOT_READY);
    FlightElement* cs_to_ready_to_start = new ChangeInternalState(GFMMState::READY_TO_START);
    FlightElement* cs_to_heading_toward_entrance = new ChangeInternalState(GFMMState::HEADING_TOWARD_ENTRANCE);
    FlightElement* cs_to_searching_for_fire = new ChangeInternalState(GFMMState::SEARCHING_FOR_FIRE);
    FlightElement* cs_to_fire_detected = new ChangeInternalState(GFMMState::FIRE_DETECTED);
    FlightElement* cs_to_approaching_fire = new ChangeInternalState(GFMMState::APPROACHING_FIRE);
    FlightElement* cs_to_extinguishing_fire = new ChangeInternalState(GFMMState::EXTINGUISHING_FIRE);
    FlightElement* cs_to_return_to_base = new ChangeInternalState(GFMMState::RETURNING_TO_BASE);
    FlightElement* cs_to_finished = new ChangeInternalState(GFMMState::FINISHED);
    ////////////////////

    //External States //
    //GF Fire Detection States
    IntegerMsg detection_ScanningWithNoDetection;
    detection_ScanningWithNoDetection.data = (int)FireDetectionState::SCANNING_NO_FIRE_DETECTED;
    FlightElement* detection_set_scanning_with_no_detection = new SendMessage((DataMessage*)&detection_ScanningWithNoDetection);

    //Water Extinguishing States
    IntegerMsg ext_ArmedIdle;
    ext_ArmedIdle.data = (int)WaterFireExtState::Armed_Idle;
    FlightElement* ext_set_armed_idle = new SendMessage((DataMessage*)&ext_ArmedIdle);
    IntegerMsg ext_Unarmed;
    ext_Unarmed.data = (int)WaterFireExtState::Unarmed;
    FlightElement* ext_set_unarmed_state = new SendMessage((DataMessage*)&ext_Unarmed);
    IntegerMsg ext_Idle; //resets water level
    ext_Idle.data = (int)WaterFireExtState::Idle;
    FlightElement* ext_set_idle_state = new SendMessage((DataMessage*)&ext_Idle);
    EmptyMsg ext_UploadWaterLevel;
    FlightElement* ext_upload_water_level = new SendMessage((DataMessage*)&ext_UploadWaterLevel); //TODO: use

    //UGV Nav TODO: add utility
    IntegerMsg ugv_HeadingTowardsEntrance;
    ugv_HeadingTowardsEntrance.data = (int)UGVNavState::HEADINGTOWARDSENTRANCE;
    FlightElement* ugv_set_heading_towards_entrance = new SendMessage((DataMessage*)&ugv_HeadingTowardsEntrance);

    IntegerMsg ugv_HeadingTowardsFireDirection;
    ugv_HeadingTowardsFireDirection.data = (int)UGVPatrolState::HEADINGTOWARDSFIREDIRECTION;
    FlightElement* ugv_set_heading_towards_fire_direction = new SendMessage((DataMessage*)&ugv_HeadingTowardsFireDirection);

    IntegerMsg ugv_PatrolingAreaCCW;
    ugv_PatrolingAreaCCW.data = (int)UGVPatrolState::PATROLINGCCW;
    FlightElement* ugv_set_patroling_area_ccw = new SendMessage((DataMessage*)&ugv_PatrolingAreaCCW);

    IntegerMsg ugv_HeadingTowardsFire;
    ugv_HeadingTowardsFire.data = (int)UGVNavState::HEADINGTOWARDSFIRE;
    FlightElement* ugv_set_heading_towards_fire = new SendMessage((DataMessage*)&ugv_HeadingTowardsFire);
    IntegerMsg ugv_ExtinguishingFire;
    ugv_ExtinguishingFire.data = (int)UGVNavState::EXTINGUISHINGFIRE;
    FlightElement* ugv_set_extinguishing_fire = new SendMessage((DataMessage*)&ugv_ExtinguishingFire);
    IntegerMsg ugv_ReturningToBase;
    ugv_ReturningToBase.data = (int)UGVNavState::RETURNINGTOBASE;
    FlightElement* ugv_set_returning_to_base = new SendMessage((DataMessage*)&ugv_ReturningToBase);
    ////////////////////

    InternalSystemStateCondition* error_condition = new InternalSystemStateCondition(GFMMState::ERROR);
    WaitForCondition* error_check = new WaitForCondition((Condition*)error_condition);

    InternalSystemStateCondition* not_ready_condition = new InternalSystemStateCondition(GFMMState::NOT_READY);
    WaitForCondition* not_ready_check = new WaitForCondition((Condition*)not_ready_condition);

    InternalSystemStateCondition* ready_to_start_condition = new InternalSystemStateCondition(GFMMState::READY_TO_START);
    WaitForCondition* ready_to_start_check = new WaitForCondition((Condition*)ready_to_start_condition);

    InternalSystemStateCondition* heading_towards_entrance_condition = new InternalSystemStateCondition(GFMMState::HEADING_TOWARD_ENTRANCE);
    WaitForCondition* heading_towards_entrance_check = new WaitForCondition((Condition*)heading_towards_entrance_condition);

    InternalSystemStateCondition* searching_for_fire_condition = new InternalSystemStateCondition(GFMMState::SEARCHING_FOR_FIRE);
    WaitForCondition* searching_for_fire_check = new WaitForCondition((Condition*)searching_for_fire_condition);

    InternalSystemStateCondition* fire_detected_condition = new InternalSystemStateCondition(GFMMState::FIRE_DETECTED);
    WaitForCondition* fire_detected_check = new WaitForCondition((Condition*)fire_detected_condition);

    InternalSystemStateCondition* approaching_fire_condition = new InternalSystemStateCondition(GFMMState::APPROACHING_FIRE);
    WaitForCondition* approaching_fire_check = new WaitForCondition((Condition*)approaching_fire_condition);

    InternalSystemStateCondition* extinguishing_fire_condition = new InternalSystemStateCondition(GFMMState::EXTINGUISHING_FIRE);
    WaitForCondition* extinguishing_fire_check = new WaitForCondition((Condition*)extinguishing_fire_condition);

    InternalSystemStateCondition* return_to_base_condition = new InternalSystemStateCondition(GFMMState::RETURNING_TO_BASE);
    WaitForCondition* return_to_base_check = new WaitForCondition((Condition*)return_to_base_condition);

    InternalSystemStateCondition* finished_condition = new InternalSystemStateCondition(GFMMState::FINISHED);
    WaitForCondition* finished_check = new WaitForCondition((Condition*)finished_condition);

    ExternalSystemStateCondition* FireDetection_Error = new ExternalSystemStateCondition((int)FireDetectionState::MALFUNCTION);
    WaitForCondition* fire_detection_error_check = new WaitForCondition((Condition*)FireDetection_Error);

    ExternalSystemStateCondition* FireDetection_Idle = new ExternalSystemStateCondition((int)FireDetectionState::IDLE);
    WaitForCondition* fire_detection_idle_check = new WaitForCondition((Condition*)FireDetection_Idle);

    ExternalSystemStateCondition* FireDetection_ScanningWithDetected = new ExternalSystemStateCondition((int)FireDetectionState::SCANNING_WITH_FIRE_DETECTED);
    WaitForCondition* fire_detection_scanning_with_detected_check = new WaitForCondition((Condition*)FireDetection_ScanningWithDetected);

    ExternalSystemStateCondition* FireDetection_ScanningWithLocated = new ExternalSystemStateCondition((int)FireDetectionState::SCANNING_WITH_FIRE_LOCATED);
    WaitForCondition* fire_detection_scanning_with_located_check = new WaitForCondition((Condition*)FireDetection_ScanningWithLocated);

    ExternalSystemStateCondition* WaterExt_Unarmed = new ExternalSystemStateCondition((int)WaterFireExtState::Unarmed);
    WaitForCondition* water_ext_unarmed_check = new WaitForCondition((Condition*)WaterExt_Unarmed);

    ExternalSystemStateCondition* WaterExt_ArmedIdle = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Idle);
    WaitForCondition* water_ext_armed_idle_check = new WaitForCondition((Condition*)WaterExt_ArmedIdle);

    ExternalSystemStateCondition* WaterExt_ArmedExtinguishing = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Extinguishing);
    WaitForCondition* water_ext_armed_extinguishing_check = new WaitForCondition((Condition*)WaterExt_ArmedExtinguishing);

    ExternalSystemStateCondition* WaterExt_ArmedExtinguished = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Extinguished);
    WaitForCondition* water_ext_armed_extinguished_check = new WaitForCondition((Condition*)WaterExt_ArmedExtinguished);

    ExternalSystemStateCondition* WaterExt_OutOfWater = new ExternalSystemStateCondition((int)WaterFireExtState::OutOfWater);
    WaitForCondition* water_ext_out_of_water_check = new WaitForCondition((Condition*)WaterExt_OutOfWater);

    ExternalSystemStateCondition* UGVNav_Idle = new ExternalSystemStateCondition((int)UGVNavState::IDLE);
    WaitForCondition* ugv_nav_idle_check = new WaitForCondition((Condition*)UGVNav_Idle);

    ExternalSystemStateCondition* UGVNav_SearchingForFire = new ExternalSystemStateCondition((int)UGVNavState::SEARCHINGFORFIRE);
    WaitForCondition* ugv_nav_searching_for_fire_check = new WaitForCondition((Condition*)UGVNav_SearchingForFire);

    // ExternalSystemStateCondition* UGVNav_HeadingTowardsFire = new ExternalSystemStateCondition((int)UGVNavState::HEADINGTOWARDSFIRE);
    // WaitForCondition* ugv_nav_heading_towards_fire_check = new WaitForCondition((Condition*)UGVNav_HeadingTowardsFire);

    ExternalSystemStateCondition* UGVNav_AlignedWithFire = new ExternalSystemStateCondition((int)UGVNavState::UGVALIGNEDWITHTARGET);
    WaitForCondition* ugv_nav_aligned_with_fire_check = new WaitForCondition((Condition*)UGVNav_AlignedWithFire);

    ExternalSystemStateCondition* UGVNav_ReachedBase = new ExternalSystemStateCondition((int)UGVNavState::REACHEDBASE);
    WaitForCondition* ugv_nav_reached_base_check = new WaitForCondition((Condition*)UGVNav_ReachedBase);

    // ********************************************************************************
    // ****************************** SYSTEM CONNECTIONS ******************************
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_error);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_not_ready);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_ready_to_start);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_heading_toward_entrance);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_searching_for_fire);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_fire_detected);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_approaching_fire);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_extinguishing_fire);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_return_to_base);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_finished);

    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_Error);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_Idle);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_ScanningWithDetected);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_ScanningWithLocated);

    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_Unarmed);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedIdle);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedExtinguishing);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedExtinguished);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_OutOfWater);

    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_Idle);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_SearchingForFire);
    //UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_HeadingTowardsFire);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_AlignedWithFire);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_ReachedBase);

    detection_set_scanning_with_no_detection->add_callback_msg_receiver((msg_receiver*) FireDetectionStateUpdaterClnt);

    ext_set_armed_idle->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    ext_set_unarmed_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    ext_set_idle_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    ext_upload_water_level->add_callback_msg_receiver((msg_receiver*) WateLevelUpdateRequesterClnt);

    ugv_set_heading_towards_entrance->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    ugv_set_heading_towards_fire->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    ugv_set_extinguishing_fire->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    ugv_set_returning_to_base->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    ugv_set_patroling_area_ccw->add_callback_msg_receiver((msg_receiver*) UGVPatrolUpdaterClnt);
    ugv_set_heading_towards_fire_direction->add_callback_msg_receiver((msg_receiver*) UGVPatrolUpdaterClnt);
    // ********************************************************************************
    // ********************************** PIPELINES ***********************************
    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, heading_towards_entrance_pipeline,
                   searching_for_fire_pipeline, fire_detected_pipeline, fire_located_pipeline, approaching_fire_pipeline, extinguishing_fire_pipeline, return_to_base_pipeline,
                   error_pipeline;// finished_pipeline;

    // TODO: add error check

    // not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    // not_ready_pipeline.addElement((FlightElement*)fire_detection_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)water_ext_unarmed_check);
    // not_ready_pipeline.addElement((FlightElement*)ugv_nav_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start);
    
    ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    ready_to_start_pipeline.addElement((FlightElement*)ugv_set_heading_towards_entrance);
    ready_to_start_pipeline.addElement((FlightElement*)cs_to_heading_toward_entrance);
    
    heading_towards_entrance_pipeline.addElement((FlightElement*)heading_towards_entrance_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)ugv_nav_searching_for_fire_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)detection_set_scanning_with_no_detection);
    heading_towards_entrance_pipeline.addElement((FlightElement*)cs_to_searching_for_fire);

    searching_for_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)fire_detection_scanning_with_detected_check);
    searching_for_fire_pipeline.addElement((FlightElement*)ugv_set_heading_towards_fire_direction);
    //searching_for_fire_pipeline.addElement((FlightElement*)ugv_set_patroling_area_ccw);
    searching_for_fire_pipeline.addElement((FlightElement*)cs_to_fire_detected);

    fire_detected_pipeline.addElement((FlightElement*)fire_detected_check);
    fire_detected_pipeline.addElement((FlightElement*)fire_detection_scanning_with_located_check);
    fire_detected_pipeline.addElement((FlightElement*)ugv_set_heading_towards_fire);
    fire_detected_pipeline.addElement((FlightElement*)cs_to_approaching_fire);

    approaching_fire_pipeline.addElement((FlightElement*)approaching_fire_check);
    approaching_fire_pipeline.addElement((FlightElement*)ugv_nav_aligned_with_fire_check);
    approaching_fire_pipeline.addElement((FlightElement*)ext_set_armed_idle);
    approaching_fire_pipeline.addElement((FlightElement*)ugv_set_extinguishing_fire);
    approaching_fire_pipeline.addElement((FlightElement*)cs_to_extinguishing_fire);

    // extinguishing_fire_pipeline.addElement((FlightElement*)extinguishing_fire_check);
    // extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_armed_extinguished_check);
    // extinguishing_fire_pipeline.addElement((FlightElement*)ext_set_unarmed_state);
    // extinguishing_fire_pipeline.addElement((FlightElement*)ugv_set_returning_to_base);
    // extinguishing_fire_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    extinguishing_fire_pipeline.addElement((FlightElement*)extinguishing_fire_check);
    extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_out_of_water_check);
    extinguishing_fire_pipeline.addElement((FlightElement*)ext_set_unarmed_state);
    extinguishing_fire_pipeline.addElement((FlightElement*)ugv_set_returning_to_base);
    extinguishing_fire_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    return_to_base_pipeline.addElement((FlightElement*) return_to_base_check);
    return_to_base_pipeline.addElement((FlightElement*) ugv_nav_reached_base_check);
    return_to_base_pipeline.addElement((FlightElement*) ext_set_idle_state);
    return_to_base_pipeline.addElement((FlightElement*) cs_to_finished);

    //TODO: ask about finish pipeline

    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    main_scenario.AddFlightPipeline(&heading_towards_entrance_pipeline);
    main_scenario.AddFlightPipeline(&searching_for_fire_pipeline);
    main_scenario.AddFlightPipeline(&fire_detected_pipeline);
    main_scenario.AddFlightPipeline(&approaching_fire_pipeline);
    main_scenario.AddFlightPipeline(&extinguishing_fire_pipeline);
    main_scenario.AddFlightPipeline(&return_to_base_pipeline);
    main_scenario.StartScenario();

    Logger::getAssignedLogger()->log("GF Indoor Fire Mission Management Node Started",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
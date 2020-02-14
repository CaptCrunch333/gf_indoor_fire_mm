#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "AddPipeline.hpp"
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
    ROSUnit_Factory mainROSUnit_Factory{nh};
	ROSUnit* InternalStateUpdaterSrv = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/set_state");

    ROSUnit* FireDetectionStateUpdaterSrv = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_gf_indoor_fire_detection_state");
    ROSUnit* WaterExtStateUpdaterSrv = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_water_ext_state");
    ROSUnit* UGVNavCtrlUpdaterSrv = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_ugv_nav_state");
    ROSUnit* WaterLevelUpdaterSrv = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_mm/update_water_level");

    ROSUnit* FireDetectionStateUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_detection/set_state");
	ROSUnit* FireDetectionVisualScanClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "gf_indoor_fire_detection/sweep_cmd");
    ROSUnit* WaterExtStateUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_mission_state");
    ROSUnit* WaterExtThermalScanClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Empty, "water_ext/trigger_scan");
    ROSUnit* WateLevelUpdateRequesterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/get_water_level"); 
    ROSUnit* UGVNavCtrlUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_mission_state");
    ROSUnit* UGVPatrolUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_patrol_mode");
    ROSUnit* UGVPositionAdjustmentClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Float, "ugv_nav/set_position_adjustment"); // TODO: add to IF
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
    FlightElement* cs_to_positioning_ugv = new ChangeInternalState(GFMMState::POSITIONING_UGV);
    FlightElement* cs_to_extinguishing_fire = new ChangeInternalState(GFMMState::EXTINGUISHING_FIRE);
    FlightElement* cs_to_return_to_base = new ChangeInternalState(GFMMState::RETURNING_TO_BASE);
    FlightElement* cs_to_finished = new ChangeInternalState(GFMMState::FINISHED);
    ////////////////////

    //External States //
    //GF Fire Detection States
    IntegerMsg detection_ScanningWithNoDetection;
    detection_ScanningWithNoDetection.data = (int)FireDetectionState::SCANNING_NO_FIRE_DETECTED;
    FlightElement* cs_fire_detection_scanning_with_no_detection = new SendMessage((DataMessage*)&detection_ScanningWithNoDetection);
    EmptyMsg detection_VisualScan;
    FlightElement* cs_fire_detection_start_visual_scan = new SendMessage((DataMessage*)&detection_VisualScan);
    Wait* detection_visual_scanning_wait = new Wait;
    detection_visual_scanning_wait->wait_time_ms = 30000;

    //Water Extinguishing States
    IntegerMsg ext_Idle; //resets water level
    ext_Idle.data = (int)WaterFireExtState::Idle;
    FlightElement* cs_water_ext_idle_state = new SendMessage((DataMessage*)&ext_Idle);
    IntegerMsg ext_Unarmed;
    ext_Unarmed.data = (int)WaterFireExtState::Unarmed;
    FlightElement* cs_water_ext_unarmed_state = new SendMessage((DataMessage*)&ext_Unarmed);
    IntegerMsg ext_ArmedIdle;
    ext_ArmedIdle.data = (int)WaterFireExtState::Armed_Idle;
    FlightElement* cs_water_ext_armed_idle = new SendMessage((DataMessage*)&ext_ArmedIdle);
    EmptyMsg ext_ThermalScan;
    FlightElement* cs_water_ext_thermal_scan = new SendMessage((DataMessage*)&ext_ThermalScan);
    EmptyMsg ext_UploadWaterLevel;
    FlightElement* cmd_water_ext_upload_water_level = new SendMessage((DataMessage*)&ext_UploadWaterLevel); //TODO: use

    //UGV Nav TODO: add utility
    IntegerMsg ugv_HeadingTowardsEntrance;
    ugv_HeadingTowardsEntrance.data = (int)UGVNavState::HEADINGTOWARDSENTRANCE;
    FlightElement* cs_ugv_nav_heading_towards_entrance = new SendMessage((DataMessage*)&ugv_HeadingTowardsEntrance);
    IntegerMsg ugv_HeadingTowardsFireDirection;
    ugv_HeadingTowardsFireDirection.data = (int)UGVPatrolState::HEADINGTOWARDSFIREDIRECTION;
    FlightElement* cs_ugv_nav_heading_towards_fire_direction = new SendMessage((DataMessage*)&ugv_HeadingTowardsFireDirection);
    IntegerMsg ugv_PatrolingAreaCCW;
    ugv_PatrolingAreaCCW.data = (int)UGVPatrolState::PATROLINGCCW;
    FlightElement* cmd_ugv_nav_patroling_area_ccw = new SendMessage((DataMessage*)&ugv_PatrolingAreaCCW);
    IntegerMsg ugv_HeadingTowardsFire;
    ugv_HeadingTowardsFire.data = (int)UGVNavState::HEADINGTOWARDSFIRE;
    FlightElement* cs_ugv_nav_heading_towards_fire = new SendMessage((DataMessage*)&ugv_HeadingTowardsFire);
    FloatMsg ugv_AdjustingPosition;
    ugv_AdjustingPosition.data = (float)0.25; //moves the robot 0.5 meters ccw, for cw use negative values
    FlightElement* cmd_ugv_nav_position_adjustment = new SendMessage((DataMessage*)&ugv_AdjustingPosition);
    IntegerMsg ugv_ExtinguishingFire;
    ugv_ExtinguishingFire.data = (int)UGVNavState::EXTINGUISHINGFIRE;
    FlightElement* cs_ugv_nav_extinguishing_fire = new SendMessage((DataMessage*)&ugv_ExtinguishingFire);
    IntegerMsg ugv_ReturningToBase;
    ugv_ReturningToBase.data = (int)UGVNavState::RETURNINGTOBASE;
    FlightElement* cs_ugv_nav_returning_to_base = new SendMessage((DataMessage*)&ugv_ReturningToBase);

    Wait* ugv_position_adjust_wait = new Wait;
    ugv_position_adjust_wait->wait_time_ms = 1000;

    ////////////////////

    Wait* ros_comm_wait = new Wait;
    ros_comm_wait->wait_time_ms = 1000;

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

    InternalSystemStateCondition* positioning_ugv_condition = new InternalSystemStateCondition(GFMMState::POSITIONING_UGV);
    WaitForCondition* positioning_ugv_check = new WaitForCondition((Condition*)positioning_ugv_condition);

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

    ExternalSystemStateCondition* WaterExt_FireDetected = new ExternalSystemStateCondition((int)WaterFireExtState::Detected);
    WaitForCondition* water_ext_fire_detected_check = new WaitForCondition((Condition*)WaterExt_FireDetected);

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

    ExternalSystemStateCondition* UGVNav_HeadingTowardsFire = new ExternalSystemStateCondition((int)UGVNavState::HEADINGTOWARDSFIRE);
    WaitForCondition* ugv_nav_heading_towards_fire_check = new WaitForCondition((Condition*)UGVNav_HeadingTowardsFire);

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
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_positioning_ugv);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_extinguishing_fire);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_return_to_base);
    InternalStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*)cs_to_finished);

    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_Error);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_Idle);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_ScanningWithDetected);
    FireDetectionStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) FireDetection_ScanningWithLocated);

    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_Unarmed);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_FireDetected);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedIdle);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedExtinguishing);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_ArmedExtinguished);
    WaterExtStateUpdaterSrv->add_callback_msg_receiver((msg_receiver*) WaterExt_OutOfWater);

    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_Idle);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_SearchingForFire);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_HeadingTowardsFire);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_AlignedWithFire);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_ReachedBase);

    detection_cs_fire_detection_scanning_with_no_detection->add_callback_msg_receiver((msg_receiver*) FireDetectionStateUpdaterClnt);
    cs_fire_detection_start_visual_scan->add_callback_msg_receiver((msg_receiver*)FireDetectionVisualScanClnt);

    cs_water_ext_unarmed_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cs_water_ext_thermal_scan->add_callback_msg_receiver((msg_receiver*) WaterExtThermalScanClnt);
    cs_water_ext_armed_idle->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cs_water_ext_idle_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cmd_water_ext_upload_water_level->add_callback_msg_receiver((msg_receiver*) WateLevelUpdateRequesterClnt);

    cs_ugv_nav_heading_towards_entrance->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    cs_ugv_nav_heading_towards_fire->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    cs_ugv_nav_extinguishing_fire->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    cs_ugv_nav_returning_to_base->add_callback_msg_receiver((msg_receiver*) UGVNavCtrlUpdaterClnt);
    cmd_ugv_nav_patroling_area_ccw->add_callback_msg_receiver((msg_receiver*) UGVPatrolUpdaterClnt);
    cs_ugv_nav_heading_towards_fire_direction->add_callback_msg_receiver((msg_receiver*) UGVPatrolUpdaterClnt);
    cmd_ugv_nav_position_adjustment->add_callback_msg_receiver((msg_receiver*) UGVPositionAdjustmentClnt);
    // ********************************************************************************
    // ********************************** PIPELINES ***********************************
    FlightScenario main_scenario;
    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, heading_towards_entrance_pipeline,
                   searching_for_fire_pipeline, detecting_fire_pipeline, fire_detected_pipeline, locating_fire_pipeline, 
                   fire_located_pipeline, approaching_fire_pipeline, positioning_ugv_success_pipepline,
                   positioning_ugv_failure_pipeline,extinguishing_fire_pipeline, return_to_base_pipeline, error_pipeline;// finished_pipeline;
    
    FlightElement* add_searching_fire_pipeline = new AddPipeline(&searching_for_fire_pipeline, &main_scenario);
    FlightElement* add_fire_detected_pipeline = new AddPipeline(&fire_detected_pipeline, &main_scenario);
    FlightElement* add_positioning_ugv_failure_pipeline = new AddPipeline(&positioning_ugv_failure_pipeline, &main_scenario);
    FlightElement* add_approaching_fire_pipeline = new AddPipeline(&approaching_fire_pipeline, &main_scenario);
    
    // TODO: add error check

    // not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    // not_ready_pipeline.addElement((FlightElement*)fire_detection_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)water_ext_unarmed_check);
    // not_ready_pipeline.addElement((FlightElement*)ugv_nav_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start); //TODO: enable this check
    
    ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    ready_to_start_pipeline.addElement((FlightElement*)cs_ugv_nav_heading_towards_entrance);
    ready_to_start_pipeline.addElement((FlightElement*)cs_to_heading_toward_entrance);
    
    heading_towards_entrance_pipeline.addElement((FlightElement*)heading_towards_entrance_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)ugv_nav_searching_for_fire_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)cs_fire_detection_scanning_with_no_detection);
    heading_towards_entrance_pipeline.addElement((FlightElement*)cs_to_searching_for_fire);

    searching_for_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)cmd_ugv_nav_position_adjustment);
    //TODO: CHANGE UGV NAV SUBSYSTEM STATES TO BE RESTRICTED (MOVE, REACHEDGOAL)
    //searching_for_fire_pipeline.addElement((FlightElement*)ros_comm_wait);
    searching_for_fire_pipeline.addElement((FlightElement*)ugv_nav_searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)cs_fire_detection_start_visual_scan);
    searching_for_fire_pipeline.addElement((FlightElement*)detection_visual_scanning_wait);
    searching_for_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)add_searching_fire_pipeline);

    detecting_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    detecting_fire_pipeline.addElement((FlightElement*)fire_detection_scanning_with_detected_check);
    detecting_fire_pipeline.addElement((FlightElement*)cs_to_fire_detected);

    fire_detected_pipeline.addElement((FlightElement*)fire_detected_check);
    fire_detected_pipeline.addElement((FlightElement*)cmd_ugv_nav_position_adjustment);
    //fire_detected_pipeline.addElement((FlightElement*)ros_comm_wait);
    fire_detected_pipeline.addElement((FlightElement*)ugv_nav_searching_for_fire_check);
    //TODO: REMOVE IMPLICIT FIRE CHASING AND HANDLE DEFLECTION POINT
    fire_detected_pipeline.addElement((FlightElement*)fire_detected_check);
    fire_detected_pipeline.addElement((FlightElement*)add_fire_detected_pipeline);

    locating_fire_pipeline.addElement((FlightElement*)fire_detected_check);
    locating_fire_pipeline.addElement((FlightElement*)fire_detection_scanning_with_located_check);
    locating_fire_pipeline.addElement((FlightElement*)cs_ugv_nav_heading_towards_fire);
    locating_fire_pipeline.addElement((FlightElement*)cs_to_approaching_fire);

    approaching_fire_pipeline.addElement((FlightElement*)approaching_fire_check);
    approaching_fire_pipeline.addElement((FlightElement*)ugv_nav_aligned_with_fire_check); // Add a pipeline to adjust positioning of ugv to hit fire, this should adjust to the functionality of the water_ext and ugv nav 
    approaching_fire_pipeline.addElement((FlightElement*)cs_water_ext_armed_idle);
    approaching_fire_pipeline.addElement((FlightElement*)cs_ugv_nav_extinguishing_fire);
    approaching_fire_pipeline.addElement((FlightElement*)cs_to_extinguishing_fire);

    extinguishing_fire_pipeline.addElement((FlightElement*)extinguishing_fire_check);
    extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_out_of_water_check);
    //extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_armed_extinguished_check);
    //extinguishing_fire_pipeline.addElement((FlightElement*)cs_water_ext_unarmed_state);
    extinguishing_fire_pipeline.addElement((FlightElement*)cs_ugv_nav_returning_to_base);
    extinguishing_fire_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    return_to_base_pipeline.addElement((FlightElement*) return_to_base_check);
    return_to_base_pipeline.addElement((FlightElement*) ugv_nav_reached_base_check);
    //return_to_base_pipeline.addElement((FlightElement*) cs_water_ext_idle_state);
    return_to_base_pipeline.addElement((FlightElement*) cs_to_finished);

    //TODO: ask about finish pipeline
    //main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    main_scenario.AddFlightPipeline(&heading_towards_entrance_pipeline);
    main_scenario.AddFlightPipeline(&searching_for_fire_pipeline);
    main_scenario.AddFlightPipeline(&detecting_fire_pipeline);
    main_scenario.AddFlightPipeline(&fire_detected_pipeline);
    main_scenario.AddFlightPipeline(&locating_fire_pipeline);
    main_scenario.AddFlightPipeline(&approaching_fire_pipeline);
    main_scenario.AddFlightPipeline(&extinguishing_fire_pipeline);
    main_scenario.AddFlightPipeline(&return_to_base_pipeline);
    main_scenario.StartScenario();

    Logger::getAssignedLogger()->log("GF Indoor Fire Mission Management Node Started",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
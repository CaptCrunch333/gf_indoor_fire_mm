#include "logger.hpp"
#include "std_logger.hpp"
#include "ros_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "AddPipeline.hpp"
#include "ResetPipeline.hpp"
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
    Logger::assignLogger(new RosLogger);
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
	ROSUnit* FireDetectionVisualScanClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Empty, "gf_indoor_fire_detection/sweep_cmd");
    ROSUnit* WaterExtStateUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/set_mission_state");
    ROSUnit* WaterExtThermalScanClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Empty, "water_ext/trigger_scan");
    ROSUnit* WateLevelUpdateRequesterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "water_ext/get_water_level"); 
    ROSUnit* UGVNavCtrlUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_mission_state");
    ROSUnit* UGVPatrolUpdaterClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_patrol_mode");
    ROSUnit* UGVPositionAdjustmentClnt = mainROSUnit_Factory.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Float, "ugv_nav/set_position_adjustment"); // TODO: add to IF
    ROSUnit* UGVChangePoseClnt = mainROSFactory->CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_Pose, "ugv_nav/move_to_goal");
    ROSUnit* UGVGoToFireLocationClnt = mainROSUnit_Factory->CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_Empty, "ugv_nav/go_to_fire_location");
    // ********************************************************************************
    // ******************************* FLIGHT ELEMENTS ********************************
    //Internal States //
    FlightElement* cs_to_error = new ChangeInternalState(GFMMState::ERROR);
    cs_to_error->set_perform_msg("cs_to_error completed");
    FlightElement* cs_to_not_ready = new ChangeInternalState(GFMMState::NOT_READY);
    cs_to_not_ready->set_perform_msg("cs_to_not_readey completed");
    FlightElement* cs_to_ready_to_start = new ChangeInternalState(GFMMState::READY_TO_START);
    cs_to_ready_to_start->set_perform_msg("cs_to_ready_to_start completed");
    FlightElement* cs_to_heading_toward_entrance = new ChangeInternalState(GFMMState::HEADING_TOWARD_ENTRANCE);
    cs_to_heading_toward_entrance->set_perform_msg("cs_to_heading_toward_entrance completed");
    FlightElement* cs_to_searching_for_fire = new ChangeInternalState(GFMMState::SEARCHING_FOR_FIRE);
    cs_to_searching_for_fire->set_perform_msg("cs_to_searching_for_fire completed");
    FlightElement* cs_to_fire_detected = new ChangeInternalState(GFMMState::FIRE_DETECTED);
    cs_to_fire_detected->set_perform_msg("cs_to_fire_detected completed");
    FlightElement* cs_to_approaching_fire = new ChangeInternalState(GFMMState::APPROACHING_FIRE);
    cs_to_approaching_fire->set_perform_msg("cs_to_approaching_fire completed");
    FlightElement* cs_to_positioning_ugv = new ChangeInternalState(GFMMState::POSITIONING_UGV);
    cs_to_positioning_ugv->set_perform_msg("cs_to_positioning_ugv completed");
    FlightElement* cs_to_extinguishing_fire = new ChangeInternalState(GFMMState::EXTINGUISHING_FIRE);
    cs_to_extinguishing_fire->set_perform_msg("cs_to_extinguishing_fire completed");
    FlightElement* cs_to_return_to_base = new ChangeInternalState(GFMMState::RETURNING_TO_BASE);
    cs_to_return_to_base->set_perform_msg("cs_to_return_to_base completed");
    FlightElement* cs_to_finished = new ChangeInternalState(GFMMState::FINISHED);
    cs_to_finished->set_perform_msg("cs_to_finished completed");
    ////////////////////

    //External States //
    //GF Fire Detection States
    IntegerMsg detection_ScanningWithNoDetection;
    detection_ScanningWithNoDetection.data = (int)FireDetectionState::SCANNING_NO_FIRE_DETECTED;
    FlightElement* cs_fire_detection_scanning_with_no_detection = new SendMessage((DataMessage*)&detection_ScanningWithNoDetection);
    cs_fire_detection_scanning_with_no_detection->set_perform_msg("cs_fire_detection_scanning_with_no_detection completed");
    EmptyMsg detection_VisualScan;
    FlightElement* cmd_fire_detection_start_visual_scan = new SendMessage((DataMessage*)&detection_VisualScan);
    cmd_fire_detection_start_visual_scan->set_perform_msg("cmd_fire_detection_start_visual_scan completed");
    Wait* detection_visual_scanning_wait = new Wait;
    detection_visual_scanning_wait->set_perform_msg("detection_visual_scanning_wait in progress");
    detection_visual_scanning_wait->wait_time_ms = 40000;

    //Water Extinguishing States
    IntegerMsg ext_Idle; //resets water level
    ext_Idle.data = (int)WaterFireExtState::Idle;
    FlightElement* cs_water_ext_idle_state = new SendMessage((DataMessage*)&ext_Idle);
    cs_water_ext_idle_state->set_perform_msg("cs_water_ext_idle_state completed");
    IntegerMsg ext_Unarmed;
    ext_Unarmed.data = (int)WaterFireExtState::Unarmed;
    FlightElement* cs_water_ext_unarmed_state = new SendMessage((DataMessage*)&ext_Unarmed);
    cs_water_ext_unarmed_state->set_perform_msg("cs_water_ext_unarmed_state completed");
    IntegerMsg ext_ArmedIdle;
    ext_ArmedIdle.data = (int)WaterFireExtState::Armed_Idle;
    FlightElement* cs_water_ext_armed_idle = new SendMessage((DataMessage*)&ext_ArmedIdle);
    cs_water_ext_armed_idle->set_perform_msg("cs_water_ext_armed_idle completed");
    EmptyMsg ext_ThermalScan;
    FlightElement* cs_water_ext_thermal_scan = new SendMessage((DataMessage*)&ext_ThermalScan);
    cs_water_ext_thermal_scan->set_perform_msg("cs_water_ext_thermal_scan completed");
    EmptyMsg ext_UploadWaterLevel;
    FlightElement* cmd_water_ext_upload_water_level = new SendMessage((DataMessage*)&ext_UploadWaterLevel); //TODO: use
    cmd_water_ext_upload_water_level->set_perform_msg("cmd_water_ext_upload_water_level completed");
    
    //UGV Nav TODO: add utility
    IntegerMsg ugv_PatrolingAreaCCW;
    ugv_PatrolingAreaCCW.data = (int)UGVPatrolState::PATROLINGCCW;
    FlightElement* cmd_ugv_nav_patroling_area_ccw = new SendMessage((DataMessage*)&ugv_PatrolingAreaCCW);
    cmd_ugv_nav_patroling_area_ccw->set_perform_msg("cmd_ugv_nav_patroling_area_ccw completed");
    FloatMsg ugv_AdjustingPosition;
    ugv_AdjustingPosition.data = (float)0.25; //moves the robot 0.5 meters ccw, for cw use negative values
    FlightElement* cmd_ugv_nav_position_adjustment = new SendMessage((DataMessage*)&ugv_AdjustingPosition);
    cmd_ugv_nav_position_adjustment->set_perform_msg("cmd_ugv_nav_position_adjustment completed");
    PoseMsg ugv_ReturnToBase;
    ugv_ReturnToBase.pose.x = 2.33;
    ugv_ReturnToBase.pose.y = 2.33;
    ugv_ReturnToBase.pose.z = 0;
    ugv_ReturnToBase.pose.yaw = 0;
    FlightElement* cmd_ugv_nav_move_to_base = new SendMessage((DataMessage*)&ugv_ReturnToBase);
    cmd_ugv_nav_move_to_base->set_perform_msg("cmd_ugv_nav_move_to_base completed");
    PoseMsg ugv_GoToEntrance;
    ugv_GoToEntrance.pose.x = 5.8;
    ugv_GoToEntrance.pose.y = 5.8;
    ugv_GoToEntrance.pose.z = 0;
    ugv_GoToEntrance.pose.yaw = 0;
    FlightElement* cmd_ugv_nav_go_to_entrance = new SendMessage((DataMessage*)&ugv_GoToEntrance);
    cmd_ugv_nav_go_to_entrance->set_perform_msg("cmd_ugv_nav_go_to_entrance completed");
    EmptyMsg* ugv_GoToFireLocation;
    FlightElement cmd_ugv_nav_go_to_fire_location = new SendMessage((DataMessage*)&ugv_GoToFireLocation);
    cmd_ugv_nav_go_to_fire_location->set_perform_msg("cmd_ugv_nav_go_to_fire_location completed");

    Wait* ugv_position_adjust_wait = new Wait;
    ugv_position_adjust_wait->set_perform_msg("ugv_position_adjust_wait in progress");
    ugv_position_adjust_wait->wait_time_ms = 1000;

    ////////////////////

    Wait* ros_comm_wait = new Wait;
    ros_comm_wait->set_perform_msg("ros_comm_wait in progress");
    ros_comm_wait->wait_time_ms = 1000;

    ////////////////////

    InternalSystemStateCondition* error_condition = new InternalSystemStateCondition(GFMMState::ERROR);
    WaitForCondition* error_check = new WaitForCondition((Condition*)error_condition);
    error_check->set_perform_msg("error_check completed");

    InternalSystemStateCondition* not_ready_condition = new InternalSystemStateCondition(GFMMState::NOT_READY);
    WaitForCondition* not_ready_check = new WaitForCondition((Condition*)not_ready_condition);
    not_ready_check->set_perform_msg("not_ready_check completed");
    
    InternalSystemStateCondition* ready_to_start_condition = new InternalSystemStateCondition(GFMMState::READY_TO_START);
    WaitForCondition* ready_to_start_check = new WaitForCondition((Condition*)ready_to_start_condition);
    ready_to_start_check->set_perform_msg("ready_to_start_check completed");
    
    InternalSystemStateCondition* heading_towards_entrance_condition = new InternalSystemStateCondition(GFMMState::HEADING_TOWARD_ENTRANCE);
    WaitForCondition* heading_towards_entrance_check = new WaitForCondition((Condition*)heading_towards_entrance_condition);
    heading_towards_entrance_check->set_perform_msg("heading_towards_entrance_check completed");
    
    InternalSystemStateCondition* searching_for_fire_condition = new InternalSystemStateCondition(GFMMState::SEARCHING_FOR_FIRE);
    WaitForCondition* searching_for_fire_check = new WaitForCondition((Condition*)searching_for_fire_condition);
    searching_for_fire_check->set_perform_msg("searching_for_fire_check completed");
    
    InternalSystemStateCondition* fire_detected_condition = new InternalSystemStateCondition(GFMMState::FIRE_DETECTED);
    WaitForCondition* fire_detected_check = new WaitForCondition((Condition*)fire_detected_condition);
    fire_detected_check->set_perform_msg("fire_detected_check completed");
    
    InternalSystemStateCondition* approaching_fire_condition = new InternalSystemStateCondition(GFMMState::APPROACHING_FIRE);
    WaitForCondition* approaching_fire_check = new WaitForCondition((Condition*)approaching_fire_condition);
    approaching_fire_check->set_perform_msg("approaching_fire_check completed");
    
    InternalSystemStateCondition* positioning_ugv_condition = new InternalSystemStateCondition(GFMMState::POSITIONING_UGV);
    WaitForCondition* positioning_ugv_check = new WaitForCondition((Condition*)positioning_ugv_condition);
    positioning_ugv_check->set_perform_msg("positioning_ugv_check completed");
    
    InternalSystemStateCondition* extinguishing_fire_condition = new InternalSystemStateCondition(GFMMState::EXTINGUISHING_FIRE);
    WaitForCondition* extinguishing_fire_check = new WaitForCondition((Condition*)extinguishing_fire_condition);
    extinguishing_fire_check->set_perform_msg("extinguishing_fire_check completed");
    
    InternalSystemStateCondition* return_to_base_condition = new InternalSystemStateCondition(GFMMState::RETURNING_TO_BASE);
    WaitForCondition* return_to_base_check = new WaitForCondition((Condition*)return_to_base_condition);
    return_to_base_check->set_perform_msg("return_to_base_check completed");
    
    InternalSystemStateCondition* finished_condition = new InternalSystemStateCondition(GFMMState::FINISHED);
    WaitForCondition* finished_check = new WaitForCondition((Condition*)finished_condition);
    finished_check->set_perform_msg("finished_check completed");
    
    ExternalSystemStateCondition* FireDetection_Error = new ExternalSystemStateCondition((int)FireDetectionState::MALFUNCTION);
    WaitForCondition* fire_detection_error_check = new WaitForCondition((Condition*)FireDetection_Error);
    fire_detection_error_check->set_perform_msg("fire_detection_error_check completed");
    
    ExternalSystemStateCondition* FireDetection_Idle = new ExternalSystemStateCondition((int)FireDetectionState::IDLE);
    WaitForCondition* fire_detection_idle_check = new WaitForCondition((Condition*)FireDetection_Idle);
    fire_detection_idle_check->set_perform_msg("fire_detection_idle_check completed");
    
    ExternalSystemStateCondition* FireDetection_ScanningWithDetected = new ExternalSystemStateCondition((int)FireDetectionState::SCANNING_WITH_FIRE_DETECTED);
    WaitForCondition* fire_detection_scanning_with_detected_check = new WaitForCondition((Condition*)FireDetection_ScanningWithDetected);
    fire_detection_scanning_with_detected_check->set_perform_msg("fire_detection_scanning_with_detected_check completed");
    
    ExternalSystemStateCondition* FireDetection_ScanningWithLocated = new ExternalSystemStateCondition((int)FireDetectionState::SCANNING_WITH_FIRE_LOCATED);
    WaitForCondition* fire_detection_scanning_with_located_check = new WaitForCondition((Condition*)FireDetection_ScanningWithLocated);
    fire_detection_scanning_with_located_check->set_perform_msg("fire_detection_scanning_with_located_check completed");
    
    ExternalSystemStateCondition* WaterExt_Unarmed = new ExternalSystemStateCondition((int)WaterFireExtState::Unarmed);
    WaitForCondition* water_ext_unarmed_check = new WaitForCondition((Condition*)WaterExt_Unarmed);
    water_ext_unarmed_check->set_perform_msg("water_ext_unarmed_check completed");
    
    ExternalSystemStateCondition* WaterExt_FireDetected = new ExternalSystemStateCondition((int)WaterFireExtState::Detected);
    WaitForCondition* water_ext_fire_detected_check = new WaitForCondition((Condition*)WaterExt_FireDetected);
    water_ext_fire_detected_check->set_perform_msg("water_ext_fire_detected_check completed");
    
    ExternalSystemStateCondition* WaterExt_ArmedIdle = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Idle);
    WaitForCondition* water_ext_armed_idle_check = new WaitForCondition((Condition*)WaterExt_ArmedIdle);
    water_ext_armed_idle_check->set_perform_msg("water_ext_armed_idle_check completed");
    
    ExternalSystemStateCondition* WaterExt_ArmedExtinguishing = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Extinguishing);
    WaitForCondition* water_ext_armed_extinguishing_check = new WaitForCondition((Condition*)WaterExt_ArmedExtinguishing);
    water_ext_armed_extinguishing_check->set_perform_msg("water_ext_armed_extinguishing_check completed");
    
    ExternalSystemStateCondition* WaterExt_ArmedExtinguished = new ExternalSystemStateCondition((int)WaterFireExtState::Armed_Extinguished);
    WaitForCondition* water_ext_armed_extinguished_check = new WaitForCondition((Condition*)WaterExt_ArmedExtinguished);
    water_ext_armed_extinguished_check->set_perform_msg("water_ext_armed_extinguished_check completed");
    
    ExternalSystemStateCondition* WaterExt_OutOfWater = new ExternalSystemStateCondition((int)WaterFireExtState::OutOfWater);
    WaitForCondition* water_ext_out_of_water_check = new WaitForCondition((Condition*)WaterExt_OutOfWater);
    water_ext_out_of_water_check->set_perform_msg("water_ext_out_of_water_check completed");
    
    ExternalSystemStateCondition* UGVNav_Idle = new ExternalSystemStateCondition((int)UGVNavState::IDLE);
    WaitForCondition* ugv_nav_idle_check = new WaitForCondition((Condition*)UGVNav_Idle);
    ugv_nav_idle_check->set_perform_msg("ugv_nav_idle_check completed");
    
    ExternalSystemStateCondition* UGVNav_ReachedGoal = new ExternalSystemStateCondition((int)UGVNavState::REACHEDGOAL);
    WaitForCondition* ugv_nav_reached_goal_check = new WaitForCondition((Condition*)UGVNav_ReachedGoal);
    ugv_nav_reached_goal_check->set_perform_msg("ugv_nav_reached_goal_check completed");
    
    ExternalSystemStateCondition* UGVNav_Moving = new ExternalSystemStateCondition((int)UGVNavState::MOVING);
    WaitForCondition* ugv_nav_moving_check = new WaitForCondition((Condition*)UGVNav_Moving);
    ugv_nav_moving_check->set_perform_msg("ugv_nav_moving_check completed");
    
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
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_Moving);
    UGVNavCtrlUpdaterSrv->add_callback_msg_receiver((msg_receiver*) UGVNav_ReachedGoal);

    cs_fire_detection_scanning_with_no_detection->add_callback_msg_receiver((msg_receiver*) FireDetectionStateUpdaterClnt);
    cmd_fire_detection_start_visual_scan->add_callback_msg_receiver((msg_receiver*)FireDetectionVisualScanClnt);

    cs_water_ext_unarmed_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cs_water_ext_thermal_scan->add_callback_msg_receiver((msg_receiver*) WaterExtThermalScanClnt);
    cs_water_ext_armed_idle->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cs_water_ext_idle_state->add_callback_msg_receiver((msg_receiver*) WaterExtStateUpdaterClnt);
    cmd_water_ext_upload_water_level->add_callback_msg_receiver((msg_receiver*) WateLevelUpdateRequesterClnt);

    cmd_ugv_nav_patroling_area_ccw->add_callback_msg_receiver((msg_receiver*) UGVPatrolUpdaterClnt);
    cmd_ugv_nav_position_adjustment->add_callback_msg_receiver((msg_receiver*) UGVPositionAdjustmentClnt);
    cmd_ugv_nav_go_to_entrance->add_callback_msg_receiver((msg_receiver*) UGVChangePoseClnt);
    cmd_ugv_nav_move_to_base->add_callback_msg_receiver((msg_receiver*) UGVChangePoseClnt);
    cmd_ugv_nav_go_to_fire_location->add_callback_msg_receiver((msg_receiver*) UGVGoToFireLocationClnt);
    // ********************************************************************************
    // ********************************** PIPELINES ***********************************
    FlightScenario main_scenario;
    FlightPipeline not_ready_pipeline, ready_to_start_pipeline, heading_towards_entrance_pipeline,
                   searching_for_fire_pipeline, detecting_fire_pipeline, fire_detected_pipeline, locating_fire_pipeline, 
                   fire_located_pipeline, approaching_fire_pipeline, positioning_ugv_success_pipepline,
                   positioning_ugv_failure_pipeline,extinguishing_fire_pipeline, return_to_base_pipeline, error_pipeline;// finished_pipeline;
    
    not_ready_pipeline.set_msg("not_ready_pipeline");
    ready_to_start_pipeline.set_msg("ready_to_start_pipeline");
    heading_towards_entrance_pipeline.set_msg("heading_towards_entrance_pipeline");
    searching_for_fire_pipeline.set_msg("searching_for_fire_pipeline");
    detecting_fire_pipeline.set_msg("detecting_fire_pipeline");
    fire_detected_pipeline.set_msg("fire_detected_pipeline");
    locating_fire_pipeline.set_msg("locating_fire_pipeline");
    fire_located_pipeline.set_msg("fire_located_pipeline");
    approaching_fire_pipeline.set_msg("approaching_fire_pipeline");
    positioning_ugv_success_pipepline.set_msg("positioning_ugv_success_pipepline");
    positioning_ugv_failure_pipeline.set_msg("positioning_ugv_failure_pipeline");
    extinguishing_fire_pipeline.set_msg("extinguishing_fire_pipeline");
    return_to_base_pipeline.set_msg("return_to_base_pipeline");
    error_pipeline.set_msg("error_pipeline");

    FlightElement* reset_searching_fire_pipeline = new ResetPipeline(&searching_for_fire_pipeline);
    reset_searching_fire_pipeline->set_perform_msg("searching_for_fire_pipeline reset");
    FlightElement* reset_fire_detected_pipeline = new ResetPipeline(&fire_detected_pipeline);
    reset_fire_detected_pipeline->set_perform_msg("fire_detected_pipeline reset");
    FlightElement* reset_positioning_ugv_failure_pipeline = new ResetPipeline(&positioning_ugv_failure_pipeline);
    reset_positioning_ugv_failure_pipeline->set_perform_msg("positioning_ugv_failure_pipeline reset");
    FlightElement* reset_approaching_fire_pipeline = new ResetPipeline(&approaching_fire_pipeline);
    reset_approaching_fire_pipeline->set_perform_msg("approaching_fire_pipeline reset");

    // TODO: add error check

    // not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    // not_ready_pipeline.addElement((FlightElement*)fire_detection_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)water_ext_unarmed_check);
    // not_ready_pipeline.addElement((FlightElement*)ugv_nav_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start); //TODO: enable this check
    
    ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    ready_to_start_pipeline.addElement((FlightElement*)cmd_ugv_nav_go_to_entrance);
    ready_to_start_pipeline.addElement((FlightElement*)cs_to_heading_toward_entrance);
    
    heading_towards_entrance_pipeline.addElement((FlightElement*)heading_towards_entrance_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)ugv_nav_reached_goal_check);
    heading_towards_entrance_pipeline.addElement((FlightElement*)cs_fire_detection_scanning_with_no_detection);
    heading_towards_entrance_pipeline.addElement((FlightElement*)cs_to_searching_for_fire);

    searching_for_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)cmd_fire_detection_start_visual_scan);
    searching_for_fire_pipeline.addElement((FlightElement*)detection_visual_scanning_wait);
    searching_for_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    searching_for_fire_pipeline.addElement((FlightElement*)cmd_ugv_nav_position_adjustment);
    searching_for_fire_pipeline.addElement((FlightElement*)ugv_nav_reached_goal_check);
    searching_for_fire_pipeline.addElement((FlightElement*)reset_searching_fire_pipeline);

    locating_fire_pipeline.addElement((FlightElement*)searching_for_fire_check);
    locating_fire_pipeline.addElement((FlightElement*)fire_detection_scanning_with_located_check);
    //locating_fire_pipeline.addElement((FlightElement*)ros_comm_wait); //TODO: check comm
    locating_fire_pipeline.addElement((FlightElement*)cmd_ugv_nav_go_to_fire_location);
    locating_fire_pipeline.addElement((FlightElement*)cs_to_approaching_fire);

    approaching_fire_pipeline.addElement((FlightElement*)approaching_fire_check);
    approaching_fire_pipeline.addElement((FlightElement*)ugv_nav_reached_goal_check); // Add a pipeline to adjust positioning of ugv to hit fire, this should adjust to the functionality of the water_ext and ugv nav 
    approaching_fire_pipeline.addElement((FlightElement*)cs_water_ext_armed_idle);
    approaching_fire_pipeline.addElement((FlightElement*)cs_to_extinguishing_fire);

    extinguishing_fire_pipeline.addElement((FlightElement*)extinguishing_fire_check);
    extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_out_of_water_check);
    //extinguishing_fire_pipeline.addElement((FlightElement*)water_ext_armed_extinguished_check);
    //extinguishing_fire_pipeline.addElement((FlightElement*)cs_water_ext_unarmed_state);
    extinguishing_fire_pipeline.addElement((FlightElement*)cmd_ugv_nav_move_to_base);
    extinguishing_fire_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    return_to_base_pipeline.addElement((FlightElement*) return_to_base_check);
    return_to_base_pipeline.addElement((FlightElement*) ugv_nav_reached_goal_check);
    //return_to_base_pipeline.addElement((FlightElement*) cs_water_ext_idle_state);
    return_to_base_pipeline.addElement((FlightElement*) cs_to_finished);

    //TODO: ask about finish pipeline
    //main_scenario.AddFlightPipeline(&not_ready_pipeline);
    main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    main_scenario.AddFlightPipeline(&heading_towards_entrance_pipeline);
    main_scenario.AddFlightPipeline(&searching_for_fire_pipeline);
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
 #include "ros/ros.h"
#include <iostream>
#include "ROSUnit.hpp"
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
    ROSUnit* UGVNavCtrlUpdaterClnt = mainROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ugv_nav/set_mission_state");

    // ********************************************************************************
    // ******************************* FLIGHT ELEMENTS ********************************
    //Internal States //
    FlightElement* cs_to_error = new ChangeInternalState(GFMMState::ERROR);
    FlightElement* cs_to_not_ready = new ChangeInternalState(GFMMState::NOT_READY);
    FlightElement* cs_to_ready_to_start = new ChangeInternalState(GFMMState::READY_TO_START);
    FlightElement* cs_to_heading_toward_entrance = new ChangeInternalState(GFMMState::HEADING_TOWARD_ENTRANCE);
    FlightElement* cs_to_searching_for_fire = new ChangeInternalState(GFMMState::SEARCHING_FOR_FIRE);
    FlightElement* cs_to_approaching_fire = new ChangeInternalState(GFMMState::APPROACHING_FIRE);
    FlightElement* cs_to_extinguishing_fire = new ChangeInternalState(GFMMState::EXTINGUISHING_FIRE);
    FlightElement* cs_to_return_to_base = new ChangeInternalState(GFMMState::RETURNING_TO_BASE);
    FlightElement* cs_to_finished = new ChangeInternalState(GFMMState::FINISHED);
    //                //

    //External States //
    //GF Fire Detection States
    IntegerMsg detection_ScanningWithNoDetection;
    detection_ScanningWithNoDetection.data = (int)FireDetectionState::SCANNING_NO_FIRE_DETECTED;
    FlightElement* set_scanning_with_no_detection = new SendMessage((DataMessage*)&detection_ScanningWithNoDetection);

    //Water Extinguishing States
    IntegerMsg ext_ArmedIdle;
    ext_ArmedIdle.data = (int)WaterFireExtState::Armed_Idle;
    FlightElement* set_armed_idles = new SendMessage((DataMessage*)&ext_ArmedIdle);
    IntegerMsg ext_Idle; //resets water level
    ext_Idle.data = (int)WaterFireExtState::Idle;
    FlightElement* set_idle_state = new SendMessage((DataMessage*)&ext_Idle);
    EmptyMsg ext_UploadWaterLevel;
    FlightElement* upload_water_level = new SendMessage((DataMessage*)&ext_UploadWaterLevel);

    //UGV Nav TODO: add utility
    IntegerMsg ugv_HeadingTowardsEntrance;
    ugv_HeadingTowardsEntrance.data = (int)UGVNavState::HEADINGTOWARDSENTRANCE;
    FlightElement* set_heading_towards_entrance = new SendMessage((DataMessage*)&ugv_HeadingTowardsEntrance);
    IntegerMsg ugv_ReturningToBase;
    ugv_ReturningToBase.data = (int)UGVNavState::RETURNINGTOBASE;
    FlightElement* set_returning_to_base = new SendMessage((DataMessage*)&ugv_ReturningToBase);
    //                //


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

    InternalSystemStateCondition* approaching_fire_condition = new InternalSystemStateCondition(GFMMState::APPROACHING_FIRE);
    WaitForCondition* approaching_fire_check = new WaitForCondition((Condition*)approaching_fire_condition);

    InternalSystemStateCondition* extinguishing_fire_condition = new InternalSystemStateCondition(GFMMState::EXTINGUISHING_FIRE);
    WaitForCondition* extinguishing_fire_check = new WaitForCondition((Condition*)extinguishing_fire_condition);

    InternalSystemStateCondition* return_to_base_condition = new InternalSystemStateCondition(GFMMState::RETURNING_TO_BASE);
    WaitForCondition* return_to_base_check = new WaitForCondition((Condition*)return_to_base_condition);

    InternalSystemStateCondition* finished_condition = new InternalSystemStateCondition(GFMMState::FINISHED);
    WaitForCondition* finished_check = new WaitForCondition((Condition*)finished_condition);

    // ExternalSystemStateCondition* FireDetection_ScanningWithNoDetection = new ExternalSystemStateCondition(0);
    // WaitForCondition* outdoor_wall_fire_detection_idle_check = new WaitForCondition((Condition*)outdoor_wall_fire_detection_idle);
    
    // ExternalSystemStateCondition* uav_control_landed = new ExternalSystemStateCondition(1);
    // WaitForCondition* uav_control_landed_check = new WaitForCondition((Condition*)uav_control_landed);
    
    // ExternalSystemStateCondition* uav_control_following_trajectory = new ExternalSystemStateCondition(6);
    // WaitForCondition* uav_control_following_trajectory_check = new WaitForCondition((Condition*)uav_control_following_trajectory);

    // ExternalSystemStateCondition* uav_control_hovering = new ExternalSystemStateCondition(7);
    // WaitForCondition* uav_control_hovering_check = new WaitForCondition((Condition*)uav_control_hovering);

    // ExternalSystemStateCondition* water_fire_extinguishing_idle = new ExternalSystemStateCondition(0);
    // WaitForCondition* water_fire_extinguishing_idle_check = new WaitForCondition((Condition*)water_fire_extinguishing_idle);

    // ExternalSystemStateCondition* water_fire_extinguishing_extinguished = new ExternalSystemStateCondition(4);
    // WaitForCondition* water_fire_extinguishing_extinguished_check = new WaitForCondition((Condition*)water_fire_extinguishing_extinguished);
    
    // ExternalSystemStateCondition* outdoor_navigation_idle = new ExternalSystemStateCondition(0);
    // WaitForCondition* outdoor_navigation_idle_check = new WaitForCondition((Condition*)outdoor_navigation_idle);
    
    // ExternalSystemStateCondition* outdoor_navigation_all_wall_fire = new ExternalSystemStateCondition(1);
    // WaitForCondition* outdoor_navigation_all_wall_fire_check = new WaitForCondition((Condition*)outdoor_navigation_all_wall_fire);

    // //******************Connections******************

    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_not_ready);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_ready_to_start);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_scanning_outdoor);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_approaching_outdoor);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_extinguishing_outdoor);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_return_to_base);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_finished);
    // ros_set_system_state_srv->add_callback_msg_receiver((msg_receiver*)cs_to_error);

    // ros_updt_fire_detection_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_wall_fire_detection_idle);

    // ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_landed);
    // ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_following_trajectory);
    // ros_updt_uav_control_state_srv->add_callback_msg_receiver((msg_receiver*)uav_control_hovering);

    // ros_updt_water_ext_state_srv->add_callback_msg_receiver((msg_receiver*)water_fire_extinguishing_idle);
    // ros_updt_water_ext_state_srv->add_callback_msg_receiver((msg_receiver*)water_fire_extinguishing_extinguished);

    // ros_updt_outdoor_nav_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_navigation_idle);
    // ros_updt_outdoor_nav_state_srv->add_callback_msg_receiver((msg_receiver*)outdoor_navigation_all_wall_fire);

    // set_ignoring_state_outdoor_fire_detection->add_callback_msg_receiver((msg_receiver*)ros_set_fire_detection_state_clnt);
    // trigger_upload_uav_scan_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_scan_path_clnt);
    // set_scanning_state_outdoor_fire_detection->add_callback_msg_receiver((msg_receiver*)ros_set_fire_detection_state_clnt);
    // trigger_upload_uav_fire_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_fire_path_clnt);
    // set_arming_ext_state_fire_extinguishing->add_callback_msg_receiver((msg_receiver*)set_fire_extinguishing_state_clnt);
    // trigger_upload_uav_home_path->add_callback_msg_receiver((msg_receiver*)ros_trigger_uav_home_path_clnt);
    // set_taking_off_state_uav_control->add_callback_msg_receiver((msg_receiver*)ros_set_mission_state_clnt);
    // set_landing_state_uav_control->add_callback_msg_receiver((msg_receiver*)ros_set_mission_state_clnt);
    
    // //**********************************************
    // FlightPipeline not_ready_pipeline, ready_to_start_pipeline, scanning_outdoor_pipeline,
    //                approach_outdoor_pipeline, extinguish_outdoor_pipeline, return_to_base_pipeline,
    //                error_pipeline, finished_pipeline;

    // //Check Current Mission State
    // not_ready_pipeline.addElement((FlightElement*)not_ready_check);
    // //Check if all systems are ready
    // not_ready_pipeline.addElement((FlightElement*)outdoor_wall_fire_detection_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)uav_control_landed_check);
    // not_ready_pipeline.addElement((FlightElement*)water_fire_extinguishing_idle_check);
    // not_ready_pipeline.addElement((FlightElement*)outdoor_navigation_idle_check);
    // //Change internal state to READY_TO_START
    // not_ready_pipeline.addElement((FlightElement*)cs_to_ready_to_start);
    
    // //Check Current Mission State
    // ready_to_start_pipeline.addElement((FlightElement*)ready_to_start_check);
    // //Call set_mission_state (Outdoor Fire Detection) and set to Ignore
    // ready_to_start_pipeline.addElement((FlightElement*)set_ignoring_state_outdoor_fire_detection);
    // //Call set_mission_state (UAV_Control) and set to Taking_Off
    // ready_to_start_pipeline.addElement((FlightElement*)set_taking_off_state_uav_control);
    // //Trigger Upload_UAV_Scan_Path
    // ready_to_start_pipeline.addElement((FlightElement*)trigger_upload_uav_scan_path);
    // //Check if UAV is at "Following Trajectory"
    // ready_to_start_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    // //Change internal state to SCANNING_OUTDOOR
    // ready_to_start_pipeline.addElement((FlightElement*)cs_to_scanning_outdoor);
    // //Call set_mission_state (Outdoor Fire Detection) and set to Scanning
    // ready_to_start_pipeline.addElement((FlightElement*)set_scanning_state_outdoor_fire_detection);
    
    // //Check Current Mission State
    // scanning_outdoor_pipeline.addElement((FlightElement*)scanning_outdoor_check);
    // //Check Outdoor Navigation is at "All wall fire detected"
    // scanning_outdoor_pipeline.addElement((FlightElement*)outdoor_navigation_all_wall_fire_check);
    // //Trigger Upload_UAV_Fire_Paths with a fire tag
    // scanning_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_fire_path);
    // //Check if UAV is at "Following Trajectory"
    // scanning_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    // //Change internal state to APPROACHING_OUTDOOR
    // scanning_outdoor_pipeline.addElement((FlightElement*)cs_to_approaching_outdoor);

    // //Check Current Mission State
    // approach_outdoor_pipeline.addElement((FlightElement*)approach_outdoor_check);
    // //Check if UAV is at "Hovering"
    // approach_outdoor_pipeline.addElement((FlightElement*)uav_control_hovering_check);
    // //Call set_mission_state (Fire Extinguishing) and set to Armed w/ Extinguishing
    // approach_outdoor_pipeline.addElement((FlightElement*)set_arming_ext_state_fire_extinguishing);
    // //Change internal state to EXTINGUISHING_OUTDOOR
    // approach_outdoor_pipeline.addElement((FlightElement*)cs_to_extinguishing_outdoor);

    // //Check Current Mission State
    // extinguish_outdoor_pipeline.addElement((FlightElement*)extinguish_outdoor_check);
    // //Check if Fire Extinguished is at "Extinguished"
    // extinguish_outdoor_pipeline.addElement((FlightElement*)water_fire_extinguishing_extinguished_check);
    // //Trigger UAV to go home
    // extinguish_outdoor_pipeline.addElement((FlightElement*)trigger_upload_uav_home_path);
    // //Check if UAV is at "Following Trajectory"
    // extinguish_outdoor_pipeline.addElement((FlightElement*)uav_control_following_trajectory_check);
    // //Change internal state to RETURNING_TO_BASE
    // extinguish_outdoor_pipeline.addElement((FlightElement*)cs_to_return_to_base);

    // //Check Current Mission State
    // return_to_base_pipeline.addElement((FlightElement*)return_to_base_check);
    // //Check if UAV is at "Hovering"
    // return_to_base_pipeline.addElement((FlightElement*)uav_control_hovering_check);
    // //Call set_mission_state (UAV_Control) and set to Landing
    // return_to_base_pipeline.addElement((FlightElement*)set_landing_state_uav_control);
    // //Change internal state to FINISHED
    // return_to_base_pipeline.addElement((FlightElement*)cs_to_finished);

    // //TODO Error Pipeline

    // Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    // FlightScenario main_scenario;
    // main_scenario.AddFlightPipeline(&not_ready_pipeline);
    // main_scenario.AddFlightPipeline(&ready_to_start_pipeline);
    // main_scenario.AddFlightPipeline(&scanning_outdoor_pipeline);
    // main_scenario.AddFlightPipeline(&approach_outdoor_pipeline);
    // main_scenario.AddFlightPipeline(&extinguish_outdoor_pipeline);
    // main_scenario.AddFlightPipeline(&return_to_base_pipeline);
    // main_scenario.StartScenario();
    // Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

// This file is a modified copy from
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/src
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/src

#include <abb_rws_client/rws_service_provider_ros.hpp>

#include <abb_robot_msgs/msg/service_responses.hpp>

#include <abb_rws_client/mapping.hpp>
#include <abb_hardware_interface/utilities.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <type_traits>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cctype>

using RAPIDSymbols = abb::rws::RWSStateMachineInterface::ResourceIdentifiers::RAPID::Symbols;

namespace abb_rws_client
{
// Static member definition
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> RWSServiceProviderROS::tf_broadcaster_;
RWSServiceProviderROS::RWSServiceProviderROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip,
                                             unsigned short robot_port)
  : node_(node)
  , rws_manager_{ robot_ip, robot_port, abb::rws::SystemConstants::General::DEFAULT_USERNAME,
                  abb::rws::SystemConstants::General::DEFAULT_PASSWORD }
{
  std::string robot_id = node_->get_parameter("robot_nickname").as_string();
  bool no_connection_timeout = node_->get_parameter("no_connection_timeout").as_bool();
  robot_controller_description_ =
      abb::robot::utilities::establishRWSConnection(rws_manager_, robot_id, no_connection_timeout);
  abb::robot::utilities::verifyRobotWareVersion(robot_controller_description_.header().robot_ware_version());

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  // Static member definition

  system_state_sub_ = node_->create_subscription<abb_robot_msgs::msg::SystemState>(
      "~/system_states", 10, std::bind(&RWSServiceProviderROS::systemStateCallback, this, std::placeholders::_1));
  runtime_state_sub_ = node_->create_subscription<abb_rapid_sm_addin_msgs::msg::RuntimeState>(
      "~/sm_addin/runtime_states", 10,
      std::bind(&RWSServiceProviderROS::runtimeStateCallback, this, std::placeholders::_1));

  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRobotControllerDescription>(
      "~/get_robot_controller_description",
      std::bind(&RWSServiceProviderROS::getRCDescription, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetFileContents>(
      "~/get_file_contents",
      std::bind(&RWSServiceProviderROS::getFileContents, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetIOSignal>(
      "~/get_io_signal",
      std::bind(&RWSServiceProviderROS::getIOSignal, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDBool>(
      "~/get_rapid_bool",
      std::bind(&RWSServiceProviderROS::getRAPIDBool, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDDnum>(
      "~/get_rapid_dnum",
      std::bind(&RWSServiceProviderROS::getRAPIDDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDNum>(
      "~/get_rapid_num",
      std::bind(&RWSServiceProviderROS::getRAPIDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDString>(
      "~/get_rapid_string",
      std::bind(&RWSServiceProviderROS::getRAPIDString, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDSymbol>(
      "~/get_rapid_symbol",
      std::bind(&RWSServiceProviderROS::getRAPIDSymbol, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetSpeedRatio>(
      "~/get_speed_ratio",
      std::bind(&RWSServiceProviderROS::getSpeedRatio, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/pp_to_main", std::bind(&RWSServiceProviderROS::ppToMain, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetFileContents>(
      "~/set_file_contents",
      std::bind(&RWSServiceProviderROS::setFileContents, this, std::placeholders::_1, std::placeholders::_2)));
  // Register the new GetRAPIDWobj service
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDWobj>(
      "~/get_wobjdata_tf",
      std::bind(&RWSServiceProviderROS::getWObjDataTF, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::GetRAPIDTool>(
      "~/get_tooldata_tf",
      std::bind(&RWSServiceProviderROS::getToolDataTF, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetIOSignal>(
      "~/set_io_signal",
      std::bind(&RWSServiceProviderROS::setIOSignal, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/set_motors_off",
      std::bind(&RWSServiceProviderROS::setMotorsOff, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/set_motors_on",
      std::bind(&RWSServiceProviderROS::setMotorsOn, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDBool>(
      "~/set_rapid_bool",
      std::bind(&RWSServiceProviderROS::setRAPIDBool, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDDnum>(
      "~/set_rapid_dnum",
      std::bind(&RWSServiceProviderROS::setRAPIDDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDNum>(
      "~/set_rapid_num",
      std::bind(&RWSServiceProviderROS::setRAPIDNum, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDString>(
      "~/set_rapid_string",
      std::bind(&RWSServiceProviderROS::setRAPIDString, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetRAPIDSymbol>(
      "~/set_rapid_symbol",
      std::bind(&RWSServiceProviderROS::setRAPIDSymbol, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::SetSpeedRatio>(
      "~/set_speed_ratio",
      std::bind(&RWSServiceProviderROS::setSpeedRatio, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/start_rapid",
      std::bind(&RWSServiceProviderROS::startRAPID, this, std::placeholders::_1, std::placeholders::_2)));
  core_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
      "~/stop_rapid", std::bind(&RWSServiceProviderROS::stopRAPID, this, std::placeholders::_1, std::placeholders::_2)));

  const auto& system_indicators = robot_controller_description_.system_indicators();

  auto has_sm_1_0 = system_indicators.addins().has_state_machine_1_0();
  auto has_sm_1_1 = system_indicators.addins().has_state_machine_1_1();
  if (has_sm_1_0)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "StateMachine Add-In 1.0 detected");
  }
  else if (has_sm_1_1)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "StateMachine Add-In 1.1 detected");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "No StateMachine Add-In detected");
  }

  if (has_sm_1_0 || has_sm_1_1)
  {
    sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
        "~/run_rapid_routine",
        std::bind(&RWSServiceProviderROS::runRAPIDRoutine, this, std::placeholders::_1, std::placeholders::_2)));
    sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine>(
        "~/set_rapid_routine",
        std::bind(&RWSServiceProviderROS::setRAPIDRoutine, this, std::placeholders::_1, std::placeholders::_2)));
    if (system_indicators.options().egm())
    {
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::GetEGMSettings>(
          "~/get_egm_settings",
          std::bind(&RWSServiceProviderROS::getEGMSettings, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetEGMSettings>(
          "~/set_egm_settings",
          std::bind(&RWSServiceProviderROS::setEGMSettings, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "~/start_egm_joint",
          std::bind(&RWSServiceProviderROS::startEGMJoint, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "~/start_egm_pose",
          std::bind(&RWSServiceProviderROS::startEGMPose, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "~/stop_egm", std::bind(&RWSServiceProviderROS::stopEGM, this, std::placeholders::_1, std::placeholders::_2)));
      if (has_sm_1_1)
      {
        sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
            "~/start_egm_stream",
            std::bind(&RWSServiceProviderROS::startEGMStream, this, std::placeholders::_1, std::placeholders::_2)));
        sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
            "~/stop_egm_stream",
            std::bind(&RWSServiceProviderROS::stopEGMStream, this, std::placeholders::_1, std::placeholders::_2)));
      }
    }

    if (system_indicators.addins().smart_gripper())
    {
      sm_services_.push_back(node_->create_service<abb_robot_msgs::srv::TriggerWithResultCode>(
          "~/run_sg_routine",
          std::bind(&RWSServiceProviderROS::runSGRoutine, this, std::placeholders::_1, std::placeholders::_2)));
      sm_services_.push_back(node_->create_service<abb_rapid_sm_addin_msgs::srv::SetSGCommand>(
          "~/set_sg_command",
          std::bind(&RWSServiceProviderROS::setSGCommand, this, std::placeholders::_1, std::placeholders::_2)));
    }
  }
  RCLCPP_INFO(node_->get_logger(), "RWS client services initialized!");
}

void RWSServiceProviderROS::systemStateCallback(const abb_robot_msgs::msg::SystemState& msg)
{
  system_state_ = msg;
}

void RWSServiceProviderROS::runtimeStateCallback(const abb_rapid_sm_addin_msgs::msg::RuntimeState& msg)
{
  runtime_state_ = msg;
}

/*************************************************************************************************************
 **************************************** RWS core services methods ******************************************
 ************************************************************************************************************/

bool RWSServiceProviderROS::getRCDescription(
    const abb_robot_msgs::srv::GetRobotControllerDescription::Request::SharedPtr req,
    abb_robot_msgs::srv::GetRobotControllerDescription::Response::SharedPtr res)
{
  res->description = robot_controller_description_.DebugString();

  res->message = abb_robot_msgs::msg::ServiceResponses::SUCCESS;
  res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;

  return true;
}

bool RWSServiceProviderROS::getFileContents(const abb_robot_msgs::srv::GetFileContents::Request::SharedPtr req,
                                            abb_robot_msgs::srv::GetFileContents::Response::SharedPtr res)
{
  if (!verifyArgumentFilename(req->filename, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.getFile(abb::rws::RWSClient::FileResource(req->filename), &res->contents))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getIOSignal(const abb_robot_msgs::srv::GetIOSignal::Request::SharedPtr req,
                                        abb_robot_msgs::srv::GetIOSignal::Response::SharedPtr res)
{
  if (!verifyArgumentSignal(req->signal, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    res->value = interface.getIOSignal(req->signal);

    if (!res->value.empty())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRAPIDBool(const abb_robot_msgs::srv::GetRAPIDBool::Request::SharedPtr req,
                                         abb_robot_msgs::srv::GetRAPIDBool::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDBool rapid_bool;
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_bool))
    {
      res->value = rapid_bool.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRAPIDDNum(const abb_robot_msgs::srv::GetRAPIDDnum::Request::SharedPtr req,
                                         abb_robot_msgs::srv::GetRAPIDDnum::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDDnum rapid_dnum;
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_dnum))
    {
      res->value = rapid_dnum.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRAPIDNum(const abb_robot_msgs::srv::GetRAPIDNum::Request::SharedPtr req,
                                        abb_robot_msgs::srv::GetRAPIDNum::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum rapid_num{};
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_num))
    {
      res->value = rapid_num.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRAPIDString(const abb_robot_msgs::srv::GetRAPIDString::Request::SharedPtr req,
                                           abb_robot_msgs::srv::GetRAPIDString::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDString rapid_string{};
    if (interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_string))
    {
      res->value = rapid_string.value;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getRAPIDSymbol(const abb_robot_msgs::srv::GetRAPIDSymbol::Request::SharedPtr req,
                                           abb_robot_msgs::srv::GetRAPIDSymbol::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    res->value = interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol);

    if (!res->value.empty())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::getSpeedRatio(const abb_robot_msgs::srv::GetSpeedRatio::Request::SharedPtr,
                                          abb_robot_msgs::srv::GetSpeedRatio::Response::SharedPtr res)
{
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    try
    {
      res->speed_ratio = interface.getSpeedRatio();
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    catch (const std::runtime_error& exception)
    {
      res->message = exception.what();
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::ppToMain(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDStopped(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.resetRAPIDProgramPointer())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setFileContents(const abb_robot_msgs::srv::SetFileContents::Request::SharedPtr req,
                                            abb_robot_msgs::srv::SetFileContents::Response::SharedPtr res)
{
  if (!verifyArgumentFilename(req->filename, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.uploadFile(abb::rws::RWSClient::FileResource(req->filename), req->contents))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setIOSignal(const abb_robot_msgs::srv::SetIOSignal::Request::SharedPtr req,
                                        abb_robot_msgs::srv::SetIOSignal::Response::SharedPtr res)
{
  if (!verifyArgumentSignal(req->signal, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setIOSignal(req->signal, req->value))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setMotorsOff(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyMotorsOn(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setMotorsOff())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setMotorsOn(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                        abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyMotorsOff(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setMotorsOn())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDBool(const abb_robot_msgs::srv::SetRAPIDBool::Request::SharedPtr req,
                                         abb_robot_msgs::srv::SetRAPIDBool::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDBool rapid_bool = static_cast<bool>(req->value);
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_bool))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDDNum(const abb_robot_msgs::srv::SetRAPIDDnum::Request::SharedPtr req,
                                         abb_robot_msgs::srv::SetRAPIDDnum::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDDnum rapid_dnum = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_dnum))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDNum(const abb_robot_msgs::srv::SetRAPIDNum::Request::SharedPtr req,
                                        abb_robot_msgs::srv::SetRAPIDNum::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum rapid_num = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_num))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDString(const abb_robot_msgs::srv::SetRAPIDString::Request::SharedPtr req,
                                           abb_robot_msgs::srv::SetRAPIDString::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDString rapid_string = req->value;
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, rapid_string))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDSymbol(const abb_robot_msgs::srv::SetRAPIDSymbol::Request::SharedPtr req,
                                           abb_robot_msgs::srv::SetRAPIDSymbol::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDSymbolPath(req->path, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.setRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, req->value))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setSpeedRatio(const abb_robot_msgs::srv::SetSpeedRatio::Request::SharedPtr req,
                                          abb_robot_msgs::srv::SetSpeedRatio::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    try
    {
      if (interface.setSpeedRatio(req->speed_ratio))
      {
        res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
      }
      else
      {
        res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
        res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      }
    }
    catch (const std::exception& exception)
    {
      res->message = exception.what();
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyMotorsOn(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.startRAPIDExecution())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.stopRAPIDExecution())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

/*************************************************************************************************************
 ******************************** RWS State Machine Add-in services methods **********************************
 ************************************************************************************************************/

bool RWSServiceProviderROS::getEGMSettings(const abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Request::SharedPtr req,
                                           abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDTask(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskExist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RWSStateMachineInterface::EGMSettings settings;

    if (interface.services().egm().getSettings(req->task, &settings))
    {
      res->settings = abb::robot::utilities::map(settings);
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setEGMSettings(const abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Request::SharedPtr req,
                                           abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDTask(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskExist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskInitialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RWSStateMachineInterface::EGMSettings settings = abb::robot::utilities::map(req->settings);

    if (interface.services().egm().setSettings(req->task, settings))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::runRAPIDRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                            abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().rapid().signalRunRAPIDRoutine())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::runSGRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().sg().signalRunSGRoutine())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setRAPIDRoutine(const abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Request::SharedPtr req,
                                            abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDTask(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskExist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskInitialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().rapid().setRoutineName(req->task, req->routine))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::setSGCommand(const abb_rapid_sm_addin_msgs::srv::SetSGCommand::Request::SharedPtr req,
                                         abb_rapid_sm_addin_msgs::srv::SetSGCommand::Response::SharedPtr res)
{
  if (!verifyArgumentRAPIDTask(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskExist(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinTaskInitialized(req->task, res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  unsigned int req_command = 0;
  try
  {
    req_command = abb::robot::utilities::mapStateMachineSGCommand(req->command);
  }
  catch (const std::runtime_error& exception)
  {
    res->message = exception.what();
    res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::RAPIDNum sg_command_input = static_cast<float>(req_command);
    abb::rws::RAPIDNum sg_target_position_input = req->target_position;

    if (interface.setRAPIDSymbolData(req->task, RAPIDSymbols::SG_COMMAND_INPUT, sg_command_input) &&
        interface.setRAPIDSymbolData(req->task, RAPIDSymbols::SG_TARGET_POSTION_INPUT, sg_target_position_input))
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMJoint(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                          abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartJoint())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMPose(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                         abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartPose())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::startEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                           abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifySMAddinRuntimeStates(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStartStream())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopEGM(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runPriorityService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStop())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProviderROS::stopEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr,
                                          abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res)
{
  if (!verifyAutoMode(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRAPIDRunning(res->result_code, res->message))
  {
    return true;
  }
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    if (interface.services().egm().signalEGMStopStream())
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      res->message = abb_robot_msgs::msg::ServiceResponses::FAILED;
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), interface.getLogTextLatestEvent());
    }
  });

  return true;
}
bool RWSServiceProviderROS::getWObjDataTF(
    const abb_robot_msgs::srv::GetRAPIDWobj::Request::SharedPtr req,
    abb_robot_msgs::srv::GetRAPIDWobj::Response::SharedPtr res)
{
    if (!verifyRWSManagerReady(res->result_code, res->message))
    {
        return true;
    }

    bool found = false;
    geometry_msgs::msg::TransformStamped tf_msg;
    std::string user_frame_id;
    std::string object_frame_id;

    // Helpers
    auto trim = [](const std::string& s) {
        size_t a = 0, b = s.size();
        while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
        while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
        return s.substr(a, b - a);
    };

    auto stripQuotes = [](const std::string& s) {
        if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\'')))
            return s.substr(1, s.size() - 2);
        return s;
    };

    auto parseNumberArray = [&](const std::string& token) {
        std::vector<double> nums;
        std::string s = trim(token);
        if (!s.empty() && s.front() == '[' && s.back() == ']') s = s.substr(1, s.size() - 2);
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            try { nums.push_back(std::stod(trim(item))); }
            catch (...) {}
        }
        return nums;
    };

    auto toBoolToken = [&](const std::string& tok) {
        std::string t = stripQuotes(trim(tok));
        std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c){ return std::toupper(c); });
        return (t == "TRUE" || t == "1");
    };

    rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
        std::string raw = interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Raw RAPID response: " << raw);

        std::string s = trim(raw);
        if (s.empty())
        {
            res->message = "Failed to get WObjData";
            res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
            return;
        }

        // Split top-level elements: uf, cf, name, user frame, tool frame
        std::vector<std::string> elems;
        int depth = 0;
        bool in_string = false;
        std::string cur;
        for (char c : s)
        {
            cur.push_back(c);
            if (c == '"') in_string = !in_string;
            else if (!in_string)
            {
                if (c == '[') ++depth;
                else if (c == ']') --depth;
                else if (c == ',' && depth == 1)
                {
                    cur.pop_back();
                    elems.push_back(trim(cur));
                    cur.clear();
                }
            }
        }
        if (!cur.empty()) elems.push_back(trim(cur));

        if (elems.size() < 5)
        {
            res->message = "Unexpected WObjData format";
            res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "WObjData parse failed: " << raw);
            return;
        }

        try
        {
            res->robhold = toBoolToken(elems[0]);
            res->ufprog = toBoolToken(elems[1]);
            res->ufmec = stripQuotes(trim(elems[2]));

            auto parseFrame = [&](const std::string& frame_str,
                                  std::array<double, 3>& pos_out,
                                  std::array<double, 4>& quat_out,
                                  bool fill_tf = false)
            {
                std::string inner = trim(frame_str);
                if (inner.front() == '[' && inner.back() == ']') inner = inner.substr(1, inner.size() - 2);

                // find first [ ] for pos
                size_t pos_start = inner.find('[');
                size_t pos_end = inner.find(']');
                size_t quat_start = inner.rfind('[');
                size_t quat_end = inner.rfind(']');

                auto pos_vec = parseNumberArray(inner.substr(pos_start, pos_end - pos_start + 1));
                auto quat_vec = parseNumberArray(inner.substr(quat_start, quat_end - quat_start + 1));

                if (pos_vec.size() >= 3)
                {
                  // RWS returns positions in millimetres; convert to metres
                  pos_out[0] = pos_vec[0] / 1000.0;
                  pos_out[1] = pos_vec[1] / 1000.0;
                  pos_out[2] = pos_vec[2] / 1000.0;
                }
                if (quat_vec.size() >= 4)
                { // real part first
                    quat_out[0] = quat_vec[1]; 
                    quat_out[1] = quat_vec[2];
                    quat_out[2] = quat_vec[3];
                    quat_out[3] = quat_vec[0];
                }

                if (fill_tf)
                {
                    tf_msg.transform.translation.x = pos_out[0];
                    tf_msg.transform.translation.y = pos_out[1];
                    tf_msg.transform.translation.z = pos_out[2];
                    tf_msg.transform.rotation.x = quat_out[0];
                    tf_msg.transform.rotation.y = quat_out[1];
                    tf_msg.transform.rotation.z = quat_out[2];
                    tf_msg.transform.rotation.w = quat_out[3];
                }
            };

            // Parse user frame
            parseFrame(elems[3], res->user_frame_position, res->user_frame_orientation, true);

            // Parse tool frame
            parseFrame(elems[4], res->object_frame_position, res->object_frame_orientation, false);

            // Populate TF
	            tf_msg.header.stamp = node_->now();
	            tf_msg.header.frame_id = req->parent_frame_id.empty() ? "base_link" : req->parent_frame_id;
	            user_frame_id = req->user_frame_id.empty() ? (res->ufmec.empty() ? req->path.symbol : res->ufmec)
	                                                       : req->user_frame_id;
	            object_frame_id =
	                req->object_frame_id.empty() ? user_frame_id + std::string("_object") : req->object_frame_id;
	            tf_msg.child_frame_id = user_frame_id;

            found = true;
            res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
            res->message = "Parsed WObjData";
        }
        catch (const std::exception& ex)
        {
            res->message = std::string("WObjData parse exception: ") + ex.what();
            res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "WObjData parse exception: " << ex.what());
        }
    });

    if (found)
    {
      // also publish object frame as child of the user frame
      geometry_msgs::msg::TransformStamped obj_tf;
      obj_tf.header.stamp = tf_msg.header.stamp;
	      obj_tf.header.frame_id = tf_msg.child_frame_id; // user frame as parent
	      obj_tf.child_frame_id = object_frame_id;
      obj_tf.transform.translation.x = res->object_frame_position[0];
      obj_tf.transform.translation.y = res->object_frame_position[1];
      obj_tf.transform.translation.z = res->object_frame_position[2];
      obj_tf.transform.rotation.x = res->object_frame_orientation[0];
      obj_tf.transform.rotation.y = res->object_frame_orientation[1];
      obj_tf.transform.rotation.z = res->object_frame_orientation[2];
      obj_tf.transform.rotation.w = res->object_frame_orientation[3];

      std::vector<geometry_msgs::msg::TransformStamped> tfs;
      tfs.push_back(tf_msg);
      tfs.push_back(obj_tf);
      tf_broadcaster_->sendTransform(tfs);
      if (res->result_code != abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS)
      {
        res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
      }
    }

	    return true;
}

bool RWSServiceProviderROS::getToolDataTF(
    const abb_robot_msgs::srv::GetRAPIDTool::Request::SharedPtr req,
    abb_robot_msgs::srv::GetRAPIDTool::Response::SharedPtr res)
{
  if (!verifyRWSManagerReady(res->result_code, res->message))
  {
    return true;
  }

  bool found = false;
  geometry_msgs::msg::TransformStamped tf_msg;

  rws_manager_.runService([&](abb::rws::RWSStateMachineInterface& interface) {
    abb::rws::ToolData rapid_tool;
    if (!interface.getRAPIDSymbolData(req->path.task, req->path.module, req->path.symbol, &rapid_tool))
    {
      res->message = "Failed to get ToolData";
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_FAILED;
      return;
    }

    const auto& pos = rapid_tool.tframe.pos;
    const auto& rot = rapid_tool.tframe.rot;

    // RAPID tooldata positions are expressed in mm relative to tool0.
    res->tool_frame_position[0] = pos.x.value / 1000.0;
    res->tool_frame_position[1] = pos.y.value / 1000.0;
    res->tool_frame_position[2] = pos.z.value / 1000.0;
    // RAPID quaternion stores real part first; ROS expects x, y, z, w.
    res->tool_frame_orientation[0] = rot.q2.value;
    res->tool_frame_orientation[1] = rot.q3.value;
    res->tool_frame_orientation[2] = rot.q4.value;
    res->tool_frame_orientation[3] = rot.q1.value;

    tf_msg.header.stamp = node_->now();
    tf_msg.header.frame_id = req->parent_frame_id.empty() ? "tool0" : req->parent_frame_id;
    tf_msg.child_frame_id = req->tool_frame_id.empty() ? req->path.symbol : req->tool_frame_id;
    tf_msg.transform.translation.x = res->tool_frame_position[0];
    tf_msg.transform.translation.y = res->tool_frame_position[1];
    tf_msg.transform.translation.z = res->tool_frame_position[2];
    tf_msg.transform.rotation.x = res->tool_frame_orientation[0];
    tf_msg.transform.rotation.y = res->tool_frame_orientation[1];
    tf_msg.transform.rotation.z = res->tool_frame_orientation[2];
    tf_msg.transform.rotation.w = res->tool_frame_orientation[3];

    found = true;
    res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    res->message = "Parsed ToolData";
  });

  if (found)
  {
    tf_broadcaster_->sendTransform(tf_msg);
    if (res->result_code != abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS)
    {
      res->result_code = abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS;
    }
  }

  return true;
}


/*************************************************************************************************************
 **************************************** Verification helper methods ****************************************
 ************************************************************************************************************/

bool RWSServiceProviderROS::verifyAutoMode(uint16_t& result_code, std::string& message)
{
  if (!system_state_.auto_mode)
  {
    message = abb_robot_msgs::msg::ServiceResponses::NOT_IN_AUTO_MODE;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_NOT_IN_AUTO_MODE;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentFilename(const std::string& filename, uint16_t& result_code,
                                                   std::string& message)
{
  if (filename.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_FILENAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_FILENAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentRAPIDSymbolPath(const abb_robot_msgs::msg::RAPIDSymbolPath& path,
                                                          uint16_t& result_code, std::string& message)
{
  if (path.task.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  if (path.module.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_MODULE_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_MODULE_NAME;
    return false;
  }

  if (path.symbol.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_SYMBOL_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_SYMBOL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentRAPIDTask(const std::string& task, uint16_t& result_code,
                                                    std::string& message)
{
  if (task.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message)
{
  if (signal.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::EMPTY_SIGNAL_NAME;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_EMPTY_SIGNAL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyMotorsOff(uint16_t& result_code, std::string& message)
{
  if (system_state_.motors_on)
  {
    message = abb_robot_msgs::msg::ServiceResponses::MOTORS_ARE_ON;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_MOTORS_ARE_ON;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyMotorsOn(uint16_t& result_code, std::string& message)
{
  if (!system_state_.motors_on)
  {
    message = abb_robot_msgs::msg::ServiceResponses::MOTORS_ARE_OFF;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_MOTORS_ARE_OFF;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddinRuntimeStates(uint16_t& result_code, std::string& message)
{
  if (runtime_state_.state_machines.empty())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_RUNTIME_STATES_MISSING;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_RUNTIME_STATES_MISSING;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddinTaskExist(const std::string& task, uint16_t& result_code, std::string& message)
{
  auto it = std::find_if(runtime_state_.state_machines.begin(), runtime_state_.state_machines.end(),
                         [&](const auto& sm) { return sm.rapid_task == task; });

  if (it == runtime_state_.state_machines.end())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_UNKNOWN_RAPID_TASK;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_UNKNOWN_RAPID_TASK;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifySMAddinTaskInitialized(const std::string& task, uint16_t& result_code,
                                                         std::string& message)
{
  if (!verifySMAddinTaskExist(task, result_code, message))
  {
    return false;
  }

  auto it = std::find_if(runtime_state_.state_machines.begin(), runtime_state_.state_machines.end(),
                         [&](const auto& sm) { return sm.rapid_task == task; });

  if (it->sm_state == abb_rapid_sm_addin_msgs::msg::StateMachineState::SM_STATE_UNKNOWN ||
      it->sm_state == abb_rapid_sm_addin_msgs::msg::StateMachineState::SM_STATE_INITIALIZE)
  {
    message = abb_robot_msgs::msg::ServiceResponses::SM_UNINITIALIZED;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SM_UNINITIALIZED;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRAPIDRunning(uint16_t& result_code, std::string& message)
{
  if (!system_state_.rapid_running)
  {
    message = abb_robot_msgs::msg::ServiceResponses::RAPID_NOT_RUNNING;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_RAPID_NOT_RUNNING;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRAPIDStopped(uint16_t& result_code, std::string& message)
{
  if (system_state_.rapid_running)
  {
    message = abb_robot_msgs::msg::ServiceResponses::RAPID_NOT_STOPPED;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_RAPID_NOT_STOPPED;
    return false;
  }

  return true;
}

bool RWSServiceProviderROS::verifyRWSManagerReady(uint16_t& result_code, std::string& message)
{
  if (!rws_manager_.isInterfaceReady())
  {
    message = abb_robot_msgs::msg::ServiceResponses::SERVER_IS_BUSY;
    result_code = abb_robot_msgs::msg::ServiceResponses::RC_SERVER_IS_BUSY;
    return false;
  }

  return true;
}
}  // namespace abb_rws_client

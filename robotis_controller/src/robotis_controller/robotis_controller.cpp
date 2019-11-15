/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * robotis_controller.cpp
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#include <ros/package.h>
#include <ros/callback_queue.h>

#include "robotis_controller/robotis_controller.h"

using namespace robotis_framework;

RobotisController::RobotisController()
  : is_timer_running_(false),
    stop_timer_(false),
    init_pose_loaded_(false),
    timer_thread_(0),
    controller_mode_(MotionModuleMode),
    DEBUG_PRINT(false),
    robot_(0),
    gazebo_mode_(false),
    gazebo_robot_name_("robotis")
{
  direct_sync_write_.clear();
}

void RobotisController::initializeSyncWrite()
{
  if (gazebo_mode_ == true)
    return;

  //ROS_INFO("FIRST BULKREAD");
  for (auto& it : port_to_bulk_read_)
    it.second->txRxPacket();
  for(auto& it : port_to_bulk_read_)
  {
    int error_count = 0;
    int result = COMM_SUCCESS;
    do
    {
      if (++error_count > 10)
      {
        ROS_ERROR("[RobotisController] first bulk read fail!!");
        exit(-1);
      }
      usleep(10 * 1000);
      result = it.second->txRxPacket();
    } while (result != COMM_SUCCESS);
  }
  init_pose_loaded_ = true;
  //ROS_INFO("FIRST BULKREAD END");

  // clear syncwrite param setting
  for (auto& it : port_to_sync_write_position_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_current_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }

  // set init syncwrite param(from data of bulkread)
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel *dxl = it.second;

    for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
    {
      uint32_t  read_data = 0;
      uint8_t   sync_write_data[4];

      if (port_to_bulk_read_[dxl->port_name_]->isAvailable(dxl->id_,
                                                          dxl->bulk_read_items_[i]->address_,
                                                          dxl->bulk_read_items_[i]->data_length_) == true)
      {
        read_data = port_to_bulk_read_[dxl->port_name_]->getData(dxl->id_,
                                                                dxl->bulk_read_items_[i]->address_,
                                                                dxl->bulk_read_items_[i]->data_length_);

        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

        if ((dxl->present_position_item_ != 0) &&
            (dxl->bulk_read_items_[i]->item_name_ == dxl->present_position_item_->item_name_))
        {
          dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(read_data) - dxl->dxl_state_->position_offset_;   // remove offset
          dxl->dxl_state_->goal_position_ = dxl->dxl_state_->present_position_;

          port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
        }
        else if ((dxl->position_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_p_gain_ = read_data;
        }
        else if ((dxl->position_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_i_gain_ = read_data;
        }
        else if ((dxl->position_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_d_gain_ = read_data;
        }
        else if ((dxl->present_velocity_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_velocity_item_->item_name_))
        {
          dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(read_data);
          dxl->dxl_state_->goal_velocity_ = dxl->dxl_state_->present_velocity_;
        }
        else if ((dxl->velocity_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_p_gain_ = read_data;
        }
        else if ((dxl->velocity_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_i_gain_ = read_data;
        }
        else if ((dxl->velocity_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_d_gain_ = read_data;
        }
        else if ((dxl->present_current_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_current_item_->item_name_))
        {
          dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(read_data);
          dxl->dxl_state_->goal_torque_ = dxl->dxl_state_->present_torque_;
        }
      }
    }
  }
}

bool RobotisController::initialize(const std::string robot_file_path, const std::string init_file_path)
{
  std::string dev_desc_dir_path = ros::package::getPath("robotis_device") + "/devices";

  // load robot info : port , device
  Robot *new_robot = new Robot(robot_file_path, dev_desc_dir_path);
  initialize(new_robot, init_file_path);
}


bool RobotisController::initialize(Robot *robot, const std::string init_file_path)
{
  robot_ = robot;
  if (gazebo_mode_ == true)
  {
    queue_thread_ = boost::thread(boost::bind(&RobotisController::msgQueueThread, this));
    return true;
  }

  for (auto& it : robot_->ports_)
  {
    std::string               port_name           = it.first;
    dynamixel::PortHandler   *port                = it.second;
    dynamixel::PacketHandler *default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port->setBaudRate(port->getBaudRate()) == false)
    {
      ROS_ERROR("PORT [%s] SETUP ERROR! (baudrate: %d)", port_name.c_str(), port->getBaudRate());
      exit(-1);
    }

    // get the default device info of the port
    std::string default_device_name = robot_->port_default_device_[port_name];
    auto dxl_it = robot_->dxls_.find(default_device_name);
    auto sensor_it = robot_->sensors_.find(default_device_name);
    if (dxl_it != robot_->dxls_.end())
    {
      Dynamixel *default_device = dxl_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);

      if (default_device->goal_position_item_ != 0)
      {
        port_to_sync_write_position_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_position_item_->address_,
                                            default_device->goal_position_item_->data_length_);
      }

      if (default_device->position_p_gain_item_ != 0)
      {
        port_to_sync_write_position_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_p_gain_item_->address_,
                                            default_device->position_p_gain_item_->data_length_);
      }

      if (default_device->position_i_gain_item_ != 0)
      {
        port_to_sync_write_position_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_i_gain_item_->address_,
                                            default_device->position_i_gain_item_->data_length_);
      }

      if (default_device->position_d_gain_item_ != 0)
      {
        port_to_sync_write_position_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_d_gain_item_->address_,
                                            default_device->position_d_gain_item_->data_length_);
      }

      if (default_device->goal_velocity_item_ != 0)
      {
        port_to_sync_write_velocity_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_velocity_item_->address_,
                                            default_device->goal_velocity_item_->data_length_);
      }

      if (default_device->velocity_p_gain_item_ != 0)
      {
        port_to_sync_write_velocity_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_p_gain_item_->address_,
                                            default_device->velocity_p_gain_item_->data_length_);
      }

      if (default_device->velocity_i_gain_item_ != 0)
      {
        port_to_sync_write_velocity_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_i_gain_item_->address_,
                                            default_device->velocity_i_gain_item_->data_length_);
      }

      if (default_device->velocity_d_gain_item_ != 0)
      {
        port_to_sync_write_velocity_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_d_gain_item_->address_,
                                            default_device->velocity_d_gain_item_->data_length_);
      }

      if (default_device->goal_current_item_ != 0)
      {
        port_to_sync_write_current_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_current_item_->address_,
                                            default_device->goal_current_item_->data_length_);
      }
    }
    else if (sensor_it != robot_->sensors_.end())
    {
      Sensor *_default_device = sensor_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(_default_device->protocol_version_);
    }

    port_to_bulk_read_[port_name] = new dynamixel::GroupBulkRead(port, default_pkt_handler);
  }

  // (for loop) check all dxls are connected.
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (ping(joint_name) != 0)
    {
      usleep(10 * 1000);
      if (ping(joint_name) != 0)
        ROS_ERROR("JOINT[%s] does NOT respond!!", joint_name.c_str());
    }
  }

  initializeDevice(init_file_path);

  queue_thread_ = boost::thread(boost::bind(&RobotisController::msgQueueThread, this));
  return true;
}

void RobotisController::initializeDevice(const std::string init_file_path)
{
  // device initialize
  if (DEBUG_PRINT)
    ROS_WARN("INIT FILE LOAD");

  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(init_file_path.c_str());

    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++)
    {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];
      if (joint_node.size() == 0)
        continue;

      Dynamixel *dxl = NULL;
      auto dxl_it = robot_->dxls_.find(joint_name);
      if (dxl_it != robot_->dxls_.end())
        dxl = dxl_it->second;

      if (dxl == NULL)
      {
        ROS_WARN("Joint [%s] was not found.", joint_name.c_str());
        continue;
      }
      if (DEBUG_PRINT)
        ROS_INFO("JOINT_NAME: %s", joint_name.c_str());

      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++)
      {
        std::string item_name = it_joint->first.as<std::string>();

        if (DEBUG_PRINT)
          ROS_INFO("  ITEM_NAME: %s", item_name.c_str());

        uint32_t value = it_joint->second.as<uint32_t>();

        ControlTableItem *item = dxl->ctrl_table_[item_name];
        if (item == NULL)
        {
          ROS_WARN("Control Item [%s] was not found.", item_name.c_str());
          continue;
        }

        if (item->memory_type_ == EEPROM)
        {
          uint8_t   data8 = 0;
          uint16_t  data16 = 0;
          uint32_t  data32 = 0;

          switch (item->data_length_)
          {
          case 1:
            read1Byte(joint_name, item->address_, &data8);
            if (data8 == value)
              continue;
            break;
          case 2:
            read2Byte(joint_name, item->address_, &data16);
            if (data16 == value)
              continue;
            break;
          case 4:
            read4Byte(joint_name, item->address_, &data32);
            if (data32 == value)
              continue;
            break;
          default:
            break;
          }
        }

        switch (item->data_length_)
        {
        case 1:
          write1Byte(joint_name, item->address_, (uint8_t) value);
          break;
        case 2:
          write2Byte(joint_name, item->address_, (uint16_t) value);
          break;
        case 4:
          write4Byte(joint_name, item->address_, value);
          break;
        default:
          break;
        }

        if (item->memory_type_ == EEPROM)
        {
          // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
          usleep(item->data_length_ * 55 * 1000);
        }
      }
    }
  } catch (const std::exception& e)
  {
    ROS_INFO("Dynamixel Init file not found.");
  }

  // [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
  for (auto& it : robot_->ports_)
  {
    if (port_to_bulk_read_[it.first] != 0)
      port_to_bulk_read_[it.first]->clearParam();
  }
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (dxl == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

//    // bulk read default : present position
//    if(dxl->present_position_item != 0)
//    {
//        bulkread_start_addr    = dxl->present_position_item->address;
//        bulkread_data_length   = dxl->present_position_item->data_length;
//    }

    // calculate bulk read start address & data length
    auto indirect_addr_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != dxl->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr_leng = dxl->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            // ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", joint_name.c_str(), indirect_addr, dxl->ctrl_table[dxl->bulk_read_items[i]->item_name]->address + _l);

            read2Byte(joint_name, indirect_addr, &data16);
            if (data16 != dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(joint_name, indirect_addr, dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = dxl->bulk_read_items_[0];

        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr = dxl->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = dxl->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

//    ROS_WARN("[%12s] start_addr: %d, data_length: %d", joint_name.c_str(), bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[dxl->port_name_]->addParam(dxl->id_, bulkread_start_addr, bulkread_data_length);

    // Torque ON
    if (writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1) != COMM_SUCCESS)
      writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1);
  }

  for (auto& it : robot_->sensors_)
  {
    std::string sensor_name = it.first;
    Sensor     *sensor      = it.second;

    if (sensor == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    // calculate bulk read start address & data length
    auto indirect_addr_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != sensor->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr_leng = sensor->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
//            ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", sensor_name.c_str(), indirect_addr, sensor->ctrl_table[sensor->bulk_read_items[i]->item_name]->address + _l);
            read2Byte(sensor_name, indirect_addr, &data16);
            if (data16 != sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(sensor_name,
                         indirect_addr,
                         sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = sensor->bulk_read_items_[0];

        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr = sensor->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = sensor->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    //ROS_WARN("[%12s] start_addr: %d, data_length: %d", sensor_name.c_str(), bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[sensor->port_name_]->addParam(sensor->id_, bulkread_start_addr, bulkread_data_length);
  }
}

void RobotisController::gazeboTimerThread()
{
  ros::Rate gazebo_rate(1000 / robot_->getControlCycle());

  while (!stop_timer_)
  {
    if (init_pose_loaded_ == true)
      process();
    gazebo_rate.sleep();
  }
}

void RobotisController::msgQueueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  ros::Subscriber write_control_table_sub = ros_node.subscribe("/robotis/write_control_table", 5,
                                                               &RobotisController::writeControlTableCallback, this);
  ros::Subscriber sync_write_item_sub     = ros_node.subscribe("/robotis/sync_write_item", 10,
                                                               &RobotisController::syncWriteItemCallback, this);
  ros::Subscriber joint_ctrl_modules_sub  = ros_node.subscribe("/robotis/set_joint_ctrl_modules", 10,
                                                               &RobotisController::setJointCtrlModuleCallback, this);
  ros::Subscriber enable_ctrl_module_sub  = ros_node.subscribe("/robotis/enable_ctrl_module", 10,
                                                               &RobotisController::setCtrlModuleCallback, this);
  ros::Subscriber control_mode_sub        = ros_node.subscribe("/robotis/set_control_mode", 10,
                                                               &RobotisController::setControllerModeCallback, this);
  ros::Subscriber joint_states_sub        = ros_node.subscribe("/robotis/set_joint_states", 10,
                                                               &RobotisController::setJointStatesCallback, this);

  ros::Subscriber gazebo_joint_states_sub;
  if (gazebo_mode_ == true)
    gazebo_joint_states_sub = ros_node.subscribe("/" + gazebo_robot_name_ + "/joint_states", 10,
                                                 &RobotisController::gazeboJointStatesCallback, this);

  /* publisher */
  goal_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
  present_joint_state_pub_  = ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_states", 10);
  current_module_pub_       = ros_node.advertise<robotis_controller_msgs::JointCtrlModule>(
                                                              "/robotis/present_joint_ctrl_modules", 10);

  if (gazebo_mode_ == true)
  {
    for (auto& it : robot_->dxls_)
    {
      gazebo_joint_position_pub_[it.first] = ros_node.advertise<std_msgs::Float64>(
                                                "/" + gazebo_robot_name_ + "/" + it.first + "_position/command", 1);
      gazebo_joint_velocity_pub_[it.first] = ros_node.advertise<std_msgs::Float64>(
                                                "/" + gazebo_robot_name_ + "/" + it.first + "_velocity/command", 1);
      gazebo_joint_effort_pub_[it.first]   = ros_node.advertise<std_msgs::Float64>(
                                                "/" + gazebo_robot_name_ + "/" + it.first + "_effort/command", 1);
    }
  }

  /* service */
  ros::ServiceServer get_joint_module_server = ros_node.advertiseService("/robotis/get_present_joint_ctrl_modules",
                                                        &RobotisController::getJointCtrlModuleService, this);
  ros::ServiceServer set_joint_module_server = ros_node.advertiseService("/robotis/set_present_joint_ctrl_modules",
                                                        &RobotisController::setJointCtrlModuleService, this);
  ros::ServiceServer set_module_server = ros_node.advertiseService("/robotis/set_present_ctrl_modules",
                                                        &RobotisController::setCtrlModuleService, this);

  ros::WallDuration duration(robot_->getControlCycle() / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void *RobotisController::timerThread(void *param)
{
  RobotisController *controller = (RobotisController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  ROS_DEBUG("controller::thread_proc started");

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while (!controller->stop_timer_)
  {
    next_time.tv_sec += (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) % 1000000000;

    controller->process();

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
    if (delta_nsec < -100000)
    {
      if (controller->DEBUG_PRINT == true)
      {
        fprintf(stderr, "[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]",
                         delta_nsec / 1000000.0, (long )next_time.tv_sec, (long )next_time.tv_nsec,
                         (long )curr_time.tv_sec, (long )curr_time.tv_nsec);
      }

      // next_time = curr_time + 3 msec
      next_time.tv_sec = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
      next_time.tv_nsec = (curr_time.tv_nsec + 3000000) % 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
  return 0;
}

void RobotisController::startTimer()
{
  if (this->is_timer_running_ == true)
    return;

  if (this->gazebo_mode_ == true)
  {
    // create and start the thread
    gazebo_thread_ = boost::thread(boost::bind(&RobotisController::gazeboTimerThread, this));
  }
  else
  {
    initializeSyncWrite();

    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }

    usleep(8 * 1000);

    int error;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if (error != 0)
      ROS_ERROR("pthread_attr_setschedpolicy error = %d\n", error);
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (error != 0)
      ROS_ERROR("pthread_attr_setinheritsched error = %d\n", error);

    memset(&param, 0, sizeof(param));
    param.sched_priority = 31;    // RT
    error = pthread_attr_setschedparam(&attr, &param);
    if (error != 0)
      ROS_ERROR("pthread_attr_setschedparam error = %d\n", error);

    // create and start the thread
    if ((error = pthread_create(&this->timer_thread_, &attr, this->timerThread, this)) != 0)
    {
      ROS_ERROR("Creating timer thread failed!!");
      exit(-1);
    }
  }

  this->is_timer_running_ = true;
}

void RobotisController::stopTimer()
{
  int error = 0;

  // set the flag to stop the thread
  if (this->is_timer_running_)
  {
    this->stop_timer_ = true;

    if (this->gazebo_mode_ == false)
    {
      // wait until the thread is stopped.
      if ((error = pthread_join(this->timer_thread_, NULL)) != 0)
        exit(-1);

      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
          it.second->rxPacket();
      }

      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
    }
    else
    {
      // wait until the thread is stopped.
      gazebo_thread_.join();
    }

    this->stop_timer_ = false;
    this->is_timer_running_ = false;
  }
}

bool RobotisController::isTimerRunning()
{
  return this->is_timer_running_;
}

void RobotisController::loadOffset(const std::string path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load offset yaml.");
    return;
  }

  YAML::Node offset_node = doc["offset"];
  if (offset_node.size() == 0)
    return;

  ROS_INFO("Load offsets...");
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); it++)
  {
    std::string joint_name = it->first.as<std::string>();
    double offset = it->second.as<double>();

    auto dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl_it->second->dxl_state_->position_offset_ = offset;
  }
}

void RobotisController::process()
{
  // avoid duplicated function call
  static bool is_process_running = false;
  if (is_process_running == true)
    return;
  is_process_running = true;

  // ROS_INFO("Controller::Process()");

  ros::Time start_time;
  ros::Duration time_duration;

  if (DEBUG_PRINT)
    start_time = ros::Time::now();

  sensor_msgs::JointState goal_state;
  sensor_msgs::JointState present_state;

  present_state.header.stamp = ros::Time::now();
  goal_state.header.stamp = present_state.header.stamp;

  if (controller_mode_ == MotionModuleMode)
  {
    if (gazebo_mode_ == false)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        robot_->ports_[it.first]->setPacketTimeout(0.0);
        it.second->rxPacket();
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel  *dxl         = dxl_it.second;
          std::string port_name   = dxl_it.second->port_name_;
          std::string joint_name  = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            bool      updated = false;
            uint32_t  data    = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_; // remove offset
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_; // remove offset
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      // -> save to robot->sensors_[]->sensor_state_
      if (robot_->sensors_.size() > 0)
      {
        for (auto& sensor_it : robot_->sensors_)
        {
          Sensor     *sensor      = sensor_it.second;
          std::string port_name   = sensor_it.second->port_name_;
          std::string sensor_name = sensor_it.first;

          if (sensor->bulk_read_items_.size() > 0)
          {
            bool      updated = false;
            uint32_t  data    = 0;
            for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = sensor->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(sensor->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(sensor->id_, item->address_, item->data_length_);

                // change sensor_state
                sensor->sensor_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              sensor->sensor_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      if (DEBUG_PRINT)
      {
        time_duration = ros::Time::now() - start_time;
        fprintf(stderr, "(%2.6f) BulkRead Rx & update state \n", time_duration.nsec * 0.000001);
      }

      // SyncWrite
      queue_mutex_.lock();

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
      }

      if (port_to_sync_write_position_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }

      queue_mutex_.unlock();

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
        it.second->txPacket();

      if (DEBUG_PRINT)
      {
        time_duration = ros::Time::now() - start_time;
        fprintf(stderr, "(%2.6f) SyncWrite & BulkRead Tx \n", time_duration.nsec * 0.000001);
      }
    }
    else if (gazebo_mode_ == true)
    {
      for (auto module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++)
      {
        if ((*module_it)->getModuleEnable() == false)
          continue;

        std_msgs::Float64 joint_msg;

        for (auto& dxl_it : robot_->dxls_)
        {
          std::string     joint_name  = dxl_it.first;
          Dynamixel      *dxl         = dxl_it.second;
          DynamixelState *dxl_state   = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == (*module_it)->getModuleName())
          {
            if ((*module_it)->getControlMode() == PositionControl)
            {
              joint_msg.data = dxl_state->goal_position_;
              gazebo_joint_position_pub_[joint_name].publish(joint_msg);
            }
            else if ((*module_it)->getControlMode() == VelocityControl)
            {
              joint_msg.data = dxl_state->goal_velocity_;
              gazebo_joint_velocity_pub_[joint_name].publish(joint_msg);
            }
            else if ((*module_it)->getControlMode() == TorqueControl)
            {
              joint_msg.data = dxl_state->goal_torque_;
              gazebo_joint_effort_pub_[joint_name].publish(joint_msg);
            }
          }
        }
      }
    }
  }
  else if (controller_mode_ == DirectControlMode)
  {
    if(gazebo_mode_ == false)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        robot_->ports_[it.first]->setPacketTimeout(0.0);
        it.second->rxPacket();
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel  *dxl         = dxl_it.second;
          std::string port_name   = dxl_it.second->port_name_;
          std::string joint_name  = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_; // remove offset
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_; // remove offset
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      queue_mutex_.lock();

//      for (auto& it : port_to_sync_write_position_)
//      {
//        it.second->txPacket();
//        it.second->clearParam();
//      }

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
      }

      queue_mutex_.unlock();

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
        it.second->txPacket();
    }
  }

  // Call SensorModule Process()
  // -> for loop : call SensorModule list -> Process()
  if (sensor_modules_.size() > 0)
  {
    for (auto module_it = sensor_modules_.begin(); module_it != sensor_modules_.end(); module_it++)
    {
      (*module_it)->process(robot_->dxls_, robot_->sensors_);

      for (auto& it : (*module_it)->result_)
        sensor_result_[it.first] = it.second;
    }
  }

  if (DEBUG_PRINT)
  {
    time_duration = ros::Time::now() - start_time;
    fprintf(stderr, "(%2.6f) SensorModule Process() & save result \n", time_duration.nsec * 0.000001);
  }

  if (controller_mode_ == MotionModuleMode)
  {
    // Call MotionModule Process()
    // -> for loop : call MotionModule list -> Process()
    if (motion_modules_.size() > 0)
    {
      queue_mutex_.lock();

      for (auto module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++)
      {
        if ((*module_it)->getModuleEnable() == false)
          continue;

        (*module_it)->process(robot_->dxls_, sensor_result_);

        // for loop : joint list
        for (auto& dxl_it : robot_->dxls_)
        {
          std::string     joint_name  = dxl_it.first;
          Dynamixel      *dxl         = dxl_it.second;
          DynamixelState *dxl_state   = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == (*module_it)->getModuleName())
          {
            //do_sync_write = true;
            DynamixelState *result_state = (*module_it)->result_[joint_name];

            if (result_state == NULL)
            {
              ROS_ERROR("[%s] %s ", (*module_it)->getModuleName().c_str(), joint_name.c_str());
              continue;
            }

            // TODO: check update time stamp ?

            if ((*module_it)->getControlMode() == PositionControl)
            {
              dxl_state->goal_position_ = result_state->goal_position_;

              if (gazebo_mode_ == false)
              {
                // add offset
                uint32_t pos_data = dxl->convertRadian2Value(dxl_state->goal_position_ + dxl_state->position_offset_);
                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

                if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                  port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if position p gain value is changed -> sync write
                if (dxl_state->position_p_gain_ != result_state->position_p_gain_)
                {
                  dxl_state->position_p_gain_ = result_state->position_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_p_gain_));

                  if (port_to_sync_write_position_p_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position i gain value is changed -> sync write
                if (dxl_state->position_i_gain_ != result_state->position_i_gain_)
                {
                  dxl_state->position_i_gain_ = result_state->position_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_i_gain_));

                  if (port_to_sync_write_position_i_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position d gain value is changed -> sync write
                if (dxl_state->position_d_gain_ != result_state->position_d_gain_)
                {
                  dxl_state->position_d_gain_ = result_state->position_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_d_gain_));

                  if (port_to_sync_write_position_d_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
              }
            }
            else if ((*module_it)->getControlMode() == VelocityControl)
            {
              dxl_state->goal_velocity_ = result_state->goal_velocity_;

              if (gazebo_mode_ == false)
              {
                uint32_t vel_data = dxl->convertVelocity2Value(dxl_state->goal_velocity_);
                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

                if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                  port_to_sync_write_velocity_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if velocity p gain gain value is changed -> sync write
                if (dxl_state->velocity_p_gain_ != result_state->velocity_p_gain_)
                {
                  dxl_state->velocity_p_gain_ = result_state->velocity_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));

                  if (port_to_sync_write_velocity_p_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity i gain value is changed -> sync write
                if (dxl_state->velocity_i_gain_ != result_state->velocity_i_gain_)
                {
                  dxl_state->velocity_i_gain_ = result_state->velocity_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));

                  if (port_to_sync_write_velocity_i_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity d gain value is changed -> sync write
                if (dxl_state->velocity_d_gain_ != result_state->velocity_d_gain_)
                {
                  dxl_state->velocity_d_gain_ = result_state->velocity_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));

                  if (port_to_sync_write_velocity_d_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
              }
            }
            else if ((*module_it)->getControlMode() == TorqueControl)
            {
              dxl_state->goal_torque_ = result_state->goal_torque_;

              if (gazebo_mode_ == false)
              {
                uint32_t curr_data = dxl->convertTorque2Value(dxl_state->goal_torque_);
                uint8_t sync_write_data[2] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(curr_data);
                sync_write_data[1] = DXL_HIBYTE(curr_data);

                if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                  port_to_sync_write_current_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
              }
            }
          }
        }
      }

      queue_mutex_.unlock();
    }

    if (DEBUG_PRINT)
    {
      time_duration = ros::Time::now() - start_time;
      fprintf(stderr, "(%2.6f) MotionModule Process() & save result \n", time_duration.nsec * 0.000001);
    }
  }

  // publish present & goal position
  for (auto& dxl_it : robot_->dxls_)
  {
    std::string joint_name  = dxl_it.first;
    Dynamixel  *dxl         = dxl_it.second;

    present_state.name.push_back(joint_name);
    present_state.position.push_back(dxl->dxl_state_->present_position_);
    present_state.velocity.push_back(dxl->dxl_state_->present_velocity_);
    present_state.effort.push_back(dxl->dxl_state_->present_torque_);

    goal_state.name.push_back(joint_name);
    goal_state.position.push_back(dxl->dxl_state_->goal_position_);
    goal_state.velocity.push_back(dxl->dxl_state_->goal_velocity_);
    goal_state.effort.push_back(dxl->dxl_state_->goal_torque_);
  }

  // -> publish present joint_states & goal joint states topic
  present_joint_state_pub_.publish(present_state);
  goal_joint_state_pub_.publish(goal_state);

  if (DEBUG_PRINT)
  {
    time_duration = ros::Time::now() - start_time;
    fprintf(stderr, "(%2.6f) Process() DONE \n", time_duration.nsec * 0.000001);
  }

  is_process_running = false;
}

void RobotisController::addMotionModule(MotionModule *module)
{
  // check whether the module name already exists
  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      ROS_ERROR("Motion Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  motion_modules_.push_back(module);
  motion_modules_.unique();
}

void RobotisController::removeMotionModule(MotionModule *module)
{
  motion_modules_.remove(module);
}

void RobotisController::addSensorModule(SensorModule *module)
{
  // check whether the module name already exists
  for (auto m_it = sensor_modules_.begin(); m_it != sensor_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      ROS_ERROR("Sensor Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  sensor_modules_.push_back(module);
  sensor_modules_.unique();
}

void RobotisController::removeSensorModule(SensorModule *module)
{
  sensor_modules_.remove(module);
}

void RobotisController::writeControlTableCallback(const robotis_controller_msgs::WriteControlTable::ConstPtr &msg)
{
  Device *device = NULL;

  if (DEBUG_PRINT)
    fprintf(stderr, "[WriteControlTable] led control msg received\n");

  auto dev_it1 = robot_->dxls_.find(msg->joint_name);
  if(dev_it1 != robot_->dxls_.end())
  {
    device = dev_it1->second;
  }
  else
  {
    auto dev_it2 = robot_->sensors_.find(msg->joint_name);
    if(dev_it2 != robot_->sensors_.end())
    {
      device = dev_it2->second;
    }
    else
    {
      ROS_WARN("[WriteControlTable] Unknown device : %s", msg->joint_name.c_str());
      return;
    }
  }

  ControlTableItem *item = NULL;
  auto item_it = device->ctrl_table_.find(msg->start_item_name);
  if(item_it != device->ctrl_table_.end())
  {
    item = item_it->second;
  }
  else
  {
    ROS_WARN("[WriteControlTable] Unknown item : %s", msg->start_item_name.c_str());
    return;
  }

  dynamixel::PortHandler   *port           = robot_->ports_[device->port_name_];
  dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

  if (item->access_type_ == Read)
    return;

  queue_mutex_.lock();

  direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, msg->data_length));
  direct_sync_write_[direct_sync_write_.size() - 1]->addParam(device->id_, (uint8_t *)(msg->data.data()));
  
//  fprintf(stderr, "[WriteControlTable] %s -> %s : ", msg->joint_name.c_str(), msg->start_item_name.c_str());
//  for (auto &dt : msg->data)
//	  fprintf(stderr, "%02X ", dt);
//  fprintf(stderr, "\n");

  queue_mutex_.unlock();

}

void RobotisController::syncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr &msg)
{
  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    Device           *device;

    auto d_it1 = robot_->dxls_.find(msg->joint_name[i]);
    if (d_it1 != robot_->dxls_.end())
    {
      device = d_it1->second;
    }
    else
    {
      auto d_it2 = robot_->sensors_.find(msg->joint_name[i]);
      if (d_it2 != robot_->sensors_.end())
      {
        device = d_it2->second;
      }
      else
      {
        ROS_WARN("[SyncWriteItem] Unknown device : %s", msg->joint_name[i].c_str());
        continue;
      }
    }

//    ControlTableItem *item  = device->ctrl_table_[msg->item_name];
    ControlTableItem *item  = NULL;
    auto item_it = device->ctrl_table_.find(msg->item_name);
    if(item_it != device->ctrl_table_.end())
    {
      item = item_it->second;
    }
    else
    {
      ROS_WARN("SyncWriteItem] Unknown item : %s", msg->item_name.c_str());
      continue;
    }

    dynamixel::PortHandler   *port           = robot_->ports_[device->port_name_];
    dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

    if (item->access_type_ == Read)
      continue;

    queue_mutex_.lock();

    int idx = 0;
    if (direct_sync_write_.size() == 0)
    {
      direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
      idx = 0;
    }
    else
    {
      for (idx = 0; idx < direct_sync_write_.size(); idx++)
      {
        if (direct_sync_write_[idx]->getPortHandler() == port && direct_sync_write_[idx]->getPacketHandler() == packet_handler)
          break;
      }

      if (idx == direct_sync_write_.size())
        direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
    }

    uint8_t *data = new uint8_t[item->data_length_];
    if (item->data_length_ == 1)
      data[0] = (uint8_t) msg->value[i];
    else if (item->data_length_ == 2)
    {
      data[0] = DXL_LOBYTE((uint16_t )msg->value[i]);
      data[1] = DXL_HIBYTE((uint16_t )msg->value[i]);
    }
    else if (item->data_length_ == 4)
    {
      data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
      data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
    }
    direct_sync_write_[idx]->addParam(device->id_, data);
    delete[] data;

    queue_mutex_.unlock();
  }
}

void RobotisController::setControllerModeCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "DirectControlMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      robot_->ports_[it.first]->setPacketTimeout(0.0);
      it.second->rxPacket();
    }
    controller_mode_ = DirectControlMode;
  }
  else if (msg->data == "MotionModuleMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }
    controller_mode_ = MotionModuleMode;
  }
}

void RobotisController::setJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (controller_mode_ != DirectControlMode)
    return;

  queue_mutex_.lock();

  for (int i = 0; i < msg->name.size(); i++)
  {
    int32_t pos = 0;

    Dynamixel *dxl = robot_->dxls_[msg->name[i]];
    if (dxl == NULL)
      continue;

    dxl->dxl_state_->goal_position_ = msg->position[i];
    pos = dxl->convertRadian2Value((double) msg->position[i]);

    uint8_t sync_write_data[4];
    sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos));
    sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos));
    sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos));
    sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos));

    if (port_to_sync_write_position_[dxl->port_name_] != NULL)
      port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
  }

  queue_mutex_.unlock();
}

void RobotisController::setCtrlModuleCallback(const std_msgs::String::ConstPtr &msg)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  std::string _module_name_to_set = msg->data;

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setCtrlModuleThread, this, _module_name_to_set));
}

void RobotisController::setCtrlModule(std::string module_name)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setCtrlModuleThread, this, module_name));
}
void RobotisController::setJointCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
  if (msg->joint_name.size() != msg->module_name.size())
    return;

  if(set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setJointCtrlModuleThread, this, msg));
}

bool RobotisController::getJointCtrlModuleService(robotis_controller_msgs::GetJointModule::Request &req,
    robotis_controller_msgs::GetJointModule::Response &res)
{
  for (unsigned int idx = 0; idx < req.joint_name.size(); idx++)
  {
    auto d_it = robot_->dxls_.find((std::string) (req.joint_name[idx]));
    if (d_it != robot_->dxls_.end())
    {
      res.joint_name.push_back(req.joint_name[idx]);
      res.module_name.push_back(d_it->second->ctrl_module_name_);
    }
  }

  if (res.joint_name.size() == 0)
    return false;

  return true;
}

bool RobotisController::setJointCtrlModuleService(robotis_controller_msgs::SetJointModule::Request &req, robotis_controller_msgs::SetJointModule::Response &res)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  robotis_controller_msgs::JointCtrlModule modules;
  modules.joint_name = req.joint_name;
  modules.module_name = req.module_name;

  robotis_controller_msgs::JointCtrlModule::ConstPtr msg_ptr(new robotis_controller_msgs::JointCtrlModule(modules));

  if (modules.joint_name.size() != modules.module_name.size())
    return false;

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setJointCtrlModuleThread, this, msg_ptr));

  set_module_thread_.join();

  return true;
}

bool RobotisController::setCtrlModuleService(robotis_controller_msgs::SetModule::Request &req, robotis_controller_msgs::SetModule::Response &res)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  std::string _module_name_to_set = req.module_name;

  set_module_thread_ = boost::thread(boost::bind(&RobotisController::setCtrlModuleThread, this, _module_name_to_set));

  set_module_thread_.join();

  return true;
}

void RobotisController::setJointCtrlModuleThread(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
  // stop module list
  std::list<MotionModule *> _stop_modules;
  std::list<MotionModule *> _enable_modules;

  for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    Dynamixel *_dxl = NULL;
    std::map<std::string, Dynamixel*>::iterator _dxl_it = robot_->dxls_.find((std::string)(msg->joint_name[idx]));
    if(_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // enqueue
    if(_dxl->ctrl_module_name_ != msg->module_name[idx])
    {
      for(std::list<MotionModule *>::iterator _stop_m_it = motion_modules_.begin(); _stop_m_it != motion_modules_.end(); _stop_m_it++)
      {
        if((*_stop_m_it)->getModuleName() == _dxl->ctrl_module_name_ && (*_stop_m_it)->getModuleEnable() == true)
          _stop_modules.push_back(*_stop_m_it);
      }
    }
  }

  // stop the module
  _stop_modules.unique();
  for(std::list<MotionModule *>::iterator _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->stop();
  }

  // wait to stop
  for(std::list<MotionModule *>::iterator _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    while((*_stop_m_it)->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for(std::list<MotionModule *>::iterator _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    std::string ctrl_module = msg->module_name[idx];
    std::string joint_name = msg->joint_name[idx];

    Dynamixel *_dxl = NULL;
    std::map<std::string, Dynamixel*>::iterator _dxl_it = robot_->dxls_.find(joint_name);
    if(_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // none
    if(ctrl_module == "" || ctrl_module == "none")
    {
      _dxl->ctrl_module_name_ = "none";

      if(gazebo_mode_ == true)
        continue;

      uint32_t _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_);
      uint8_t _sync_write_data[4];
      _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
      _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

      if(port_to_sync_write_position_[_dxl->port_name_] != NULL)
        port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

      if(port_to_sync_write_current_[_dxl->port_name_] != NULL)
        port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
      if(port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
        port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
    }
    else
    {
      // check whether the module exist
      for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
      {
        // if it exist
        if((*_m_it)->getModuleName() == ctrl_module)
        {
          std::map<std::string, DynamixelState*>::iterator _result_it = (*_m_it)->result_.find(joint_name);
          if(_result_it == (*_m_it)->result_.end())
            break;

          _dxl->ctrl_module_name_ = ctrl_module;

          // enqueue enable module list
          _enable_modules.push_back(*_m_it);
          ControlMode _mode = (*_m_it)->getControlMode();

          if(gazebo_mode_ == true)
            break;

          if(_mode == PositionControl)
          {
            uint32_t _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

            if(port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if(_mode == VelocityControl)
          {
            uint32_t _vel_data = _dxl->convertVelocity2Value(_dxl->dxl_state_->goal_velocity_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_vel_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_vel_data));

            if(port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if(_mode == TorqueControl)
          {
            uint32_t _curr_data = _dxl->convertTorque2Value(_dxl->dxl_state_->goal_torque_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_curr_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_curr_data));

            if(port_to_sync_write_current_[_dxl->port_name_] != NULL)
              port_to_sync_write_current_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_velocity_[_dxl->port_name_] != NULL)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_position_[_dxl->port_name_] != NULL)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          break;
        }
      }
    }
  }

  // enable module(s)
  _enable_modules.unique();
  for(std::list<MotionModule *>::iterator _m_it = _enable_modules.begin(); _m_it != _enable_modules.end(); _m_it++)
  {
    (*_m_it)->setModuleEnable(true);
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::JointCtrlModule _current_module_msg;
  for(std::map<std::string, Dynamixel *>::iterator _dxl_iter = robot_->dxls_.begin(); _dxl_iter  != robot_->dxls_.end(); ++_dxl_iter)
  {
    _current_module_msg.joint_name.push_back(_dxl_iter->first);
    _current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name_);
  }

  if(_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
    current_module_pub_.publish(_current_module_msg);
}

void RobotisController::setCtrlModuleThread(std::string ctrl_module)
{
  // stop module
  std::list<MotionModule *> stop_modules;

  if (ctrl_module == "" || ctrl_module == "none")
  {
    // enqueue all modules in order to stop
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      if ((*m_it)->getModuleEnable() == true)
        stop_modules.push_back(*m_it);
    }
  }
  else
  {
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        // enqueue the module which lost control of joint in order to stop
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);

          if (d_it != robot_->dxls_.end())
          {
            // enqueue
            if (d_it->second->ctrl_module_name_ != ctrl_module)
            {
              for (auto stop_m_it = motion_modules_.begin(); stop_m_it != motion_modules_.end(); stop_m_it++)
              {
                if (((*stop_m_it)->getModuleName() == d_it->second->ctrl_module_name_) &&
                    ((*stop_m_it)->getModuleEnable() == true))
                {
                  stop_modules.push_back(*stop_m_it);
                }
              }
            }
          }
        }

        break;
      }
    }
  }

  // stop the module
  stop_modules.unique();
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    (*stop_m_it)->stop();
  }

  // wait to stop
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    while ((*stop_m_it)->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for(std::list<MotionModule *>::iterator _stop_m_it = stop_modules.begin(); _stop_m_it != stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }


  // set ctrl module
  queue_mutex_.lock();

  if (DEBUG_PRINT)
    ROS_INFO_STREAM("set module : " << ctrl_module);

  // none
  if ((ctrl_module == "") || (ctrl_module == "none"))
  {
    // set dxl's control module to "none"
    for (auto& d_it : robot_->dxls_)
    {
      Dynamixel *dxl = d_it.second;
      dxl->ctrl_module_name_ = "none";

      if (gazebo_mode_ == true)
        continue;

      uint32_t pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_);
      uint8_t sync_write_data[4] = { 0 };
      sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
      sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
      sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
      sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

      if (port_to_sync_write_position_[dxl->port_name_] != NULL)
        port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

      if (port_to_sync_write_current_[dxl->port_name_] != NULL)
        port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
      if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
        port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
    }
  }
  else
  {
    // check whether the module exist
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        ControlMode mode = (*m_it)->getControlMode();
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);
          if (d_it != robot_->dxls_.end())
          {
            Dynamixel *dxl = d_it->second;
            dxl->ctrl_module_name_ = ctrl_module;

            if (gazebo_mode_ == true)
              continue;

            if (mode == PositionControl)
            {
              uint32_t pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == VelocityControl)
            {
              uint32_t vel_data = dxl->convertVelocity2Value(dxl->dxl_state_->goal_velocity_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == TorqueControl)
            {
              uint32_t curr_data = dxl->convertTorque2Value(dxl->dxl_state_->goal_torque_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(curr_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(curr_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(curr_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(curr_data));

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
          }
        }

        break;
      }
    }
  }

  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    // set all used modules -> enable
    for (auto& d_it : robot_->dxls_)
    {
      if (d_it.second->ctrl_module_name_ == (*m_it)->getModuleName())
      {
        (*m_it)->setModuleEnable(true);
        break;
      }
    }
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::JointCtrlModule current_module_msg;
  for (auto& dxl_iter : robot_->dxls_)
  {
    current_module_msg.joint_name.push_back(dxl_iter.first);
    current_module_msg.module_name.push_back(dxl_iter.second->ctrl_module_name_);
  }

  if (current_module_msg.joint_name.size() == current_module_msg.module_name.size())
    current_module_pub_.publish(current_module_msg);
}

void RobotisController::gazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  queue_mutex_.lock();

  for (unsigned int i = 0; i < msg->name.size(); i++)
  {
    auto d_it = robot_->dxls_.find((std::string) msg->name[i]);
    if (d_it != robot_->dxls_.end())
    {
      d_it->second->dxl_state_->present_position_ = msg->position[i];
      d_it->second->dxl_state_->present_velocity_ = msg->velocity[i];
      d_it->second->dxl_state_->present_torque_ = msg->effort[i];
    }
  }

  if (init_pose_loaded_ == false)
  {
    for (auto& it : robot_->dxls_)
      it.second->dxl_state_->goal_position_ = it.second->dxl_state_->present_position_;
    init_pose_loaded_ = true;
  }

  queue_mutex_.unlock();
}

bool RobotisController::isTimerStopped()
{
  if (this->is_timer_running_)
  {
    if (DEBUG_PRINT == true)
      ROS_WARN("Process Timer is running.. STOP the timer first.");
    return false;
  }
  return true;
}

int RobotisController::ping(const std::string joint_name, uint8_t *error)
{
  return ping(joint_name, 0, error);
}
int RobotisController::ping(const std::string joint_name, uint16_t* model_number, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->ping(port_handler, dxl->id_, model_number, error);
}

int RobotisController::action(const std::string joint_name)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->action(port_handler, dxl->id_);
}
int RobotisController::reboot(const std::string joint_name, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->reboot(port_handler, dxl->id_, error);
}
int RobotisController::factoryReset(const std::string joint_name, uint8_t option, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->factoryReset(port_handler, dxl->id_, option, error);
}

int RobotisController::read(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->readTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem *item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  switch (item->data_length_)
  {
  case 1:
  {
    uint8_t read_data = 0;
    result = pkt_handler->read1ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  case 2:
  {
    uint16_t read_data = 0;
    result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  case 4:
  {
    uint32_t read_data = 0;
    result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  default:
    break;
  }
  return result;
}

int RobotisController::read1Byte(const std::string joint_name, uint16_t address, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read2Byte(const std::string joint_name, uint16_t address, uint16_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read4Byte(const std::string joint_name, uint16_t address, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->writeTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem *item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  int       result      = COMM_NOT_AVAILABLE;
  uint8_t  *write_data  = new uint8_t[item->data_length_];
  if (item->data_length_ == 1)
  {
    write_data[0] = (uint8_t) data;
    result = pkt_handler->write1ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 2)
  {
    write_data[0] = DXL_LOBYTE((uint16_t )data);
    write_data[1] = DXL_HIBYTE((uint16_t )data);
    result = pkt_handler->write2ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 4)
  {
    write_data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)data));
    write_data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)data));
    write_data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)data));
    write_data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)data));
    result = pkt_handler->write4ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  delete[] write_data;
  return result;
}

int RobotisController::write1Byte(const std::string joint_name, uint16_t address, uint8_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write2Byte(const std::string joint_name, uint16_t address, uint16_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write4Byte(const std::string joint_name, uint16_t address, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::regWrite(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data,
    uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->regWriteTxRx(port_handler, dxl->id_, address, length, data, error);
}


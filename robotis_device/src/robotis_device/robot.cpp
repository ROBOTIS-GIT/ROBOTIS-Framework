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
 * robot.cpp
 *
 *  Created on: 2015. 12. 11.
 *      Author: zerom
 */

#include <fstream>
#include <iostream>
#include <algorithm>

#include "robotis_device/robot.h"

using namespace robotis_framework;

static inline std::string &ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
  return s;
}
static inline std::string &rtrim(std::string &s)
{
  auto it = std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  });
  s.erase(it.base(), s.end());
  return s;
}
static inline std::string &trim(std::string &s)
{
  return ltrim(rtrim(s));
}

static inline std::vector<std::string> split(const std::string &text, char sep)
{
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;

  while ((end = text.find(sep, start)) != (std::string::npos))
  {
    tokens.push_back(text.substr(start, end - start));
    trim(tokens.back());
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  trim(tokens.back());

  return tokens;
}

Robot::Robot(std::string robot_file_path, std::string dev_desc_dir_path)
  : control_cycle_msec_(DEFAULT_CONTROL_CYCLE)
{
  if (dev_desc_dir_path.compare(dev_desc_dir_path.size() - 1, 1, "/") != 0)
    dev_desc_dir_path += "/";

  std::ifstream file(robot_file_path.c_str());
  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;
    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
        input_str = input_str.substr(0, pos);

      // trim
      input_str = trim(input_str);

      // find session
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size() - 1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size() - 2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }

      if (session == SESSION_CONTROL_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '=');
        if (tokens.size() != 2)
          continue;

        if (tokens[0] == "control_cycle")
          control_cycle_msec_ = std::atoi(tokens[1].c_str());
      }
      else if (session == SESSION_PORT_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 3)
          continue;

        std::cout << tokens[0] << " added. (baudrate: " << tokens[1] << ")" << std::endl;

        ports_[tokens[0]] = dynamixel::PortHandler::getPortHandler(tokens[0].c_str());
        ports_[tokens[0]]->setBaudRate(std::atoi(tokens[1].c_str()));
        port_default_device_[tokens[0]] = tokens[2];
      }
      else if (session == SESSION_DEVICE_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 7)
          continue;

        if (tokens[0] == DYNAMIXEL)
        {
          std::string file_path = dev_desc_dir_path + tokens[0] + "/" + tokens[3] + ".device";
          int         id        = std::atoi(tokens[2].c_str());
          std::string port      = tokens[1];
          float       protocol  = std::atof(tokens[4].c_str());
          std::string dev_name  = tokens[5];

          dxls_[dev_name] = getDynamixel(file_path, id, port, protocol);

          Dynamixel *dxl = dxls_[dev_name];
          std::vector<std::string> sub_tokens = split(tokens[6], ',');
          if (sub_tokens.size() > 0 && sub_tokens[0] != "")
          {
            std::map<std::string, ControlTableItem *>::iterator indirect_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
            if (indirect_it != dxl->ctrl_table_.end())    // INDIRECT_ADDRESS_1 exist
            {
              uint16_t indirect_data_addr = dxl->ctrl_table_[INDIRECT_DATA_1]->address_;
              for (int _i = 0; _i < sub_tokens.size(); _i++)
              {
                std::map<std::string, ControlTableItem *>::iterator bulkread_it = dxl->ctrl_table_.find(sub_tokens[_i]);
                if(bulkread_it == dxl->ctrl_table_.end())
                {
                  fprintf(stderr, "\n  ##### BULK READ ITEM [ %s ] NOT FOUND!! #####\n\n", sub_tokens[_i].c_str());
                  continue;
                }

                dxl->bulk_read_items_.push_back(new ControlTableItem());
                ControlTableItem *dest_item = dxl->bulk_read_items_[_i];
                ControlTableItem *src_item  = dxl->ctrl_table_[sub_tokens[_i]];

                dest_item->item_name_       = src_item->item_name_;
                dest_item->address_         = indirect_data_addr;
                dest_item->access_type_     = src_item->access_type_;
                dest_item->memory_type_     = src_item->memory_type_;
                dest_item->data_length_     = src_item->data_length_;
                dest_item->data_min_value_  = src_item->data_min_value_;
                dest_item->data_max_value_  = src_item->data_max_value_;
                dest_item->is_signed_       = src_item->is_signed_;

                indirect_data_addr += dest_item->data_length_;
              }
            }
            else    // INDIRECT_ADDRESS_1 not exist
            {
              for (int i = 0; i < sub_tokens.size(); i++)
              {
                if (dxl->ctrl_table_[sub_tokens[i]] != NULL)
                  dxl->bulk_read_items_.push_back(dxl->ctrl_table_[sub_tokens[i]]);
              }
            }
          }
        }
        else if (tokens[0] == SENSOR)
        {
          std::string file_path = dev_desc_dir_path + tokens[0] + "/" + tokens[3] + ".device";
          int         id        = std::atoi(tokens[2].c_str());
          std::string port      = tokens[1];
          float       protocol  = std::atof(tokens[4].c_str());
          std::string dev_name  = tokens[5];

          sensors_[dev_name] = getSensor(file_path, id, port, protocol);

          Sensor *sensor = sensors_[dev_name];
          std::vector<std::string> sub_tokens = split(tokens[6], ',');
          if (sub_tokens.size() > 0 && sub_tokens[0] != "")
          {
            std::map<std::string, ControlTableItem *>::iterator indirect_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
            if (indirect_it != sensor->ctrl_table_.end())    // INDIRECT_ADDRESS_1 exist
            {
              uint16_t indirect_data_addr = sensor->ctrl_table_[INDIRECT_DATA_1]->address_;
              for (int i = 0; i < sub_tokens.size(); i++)
              {
                sensor->bulk_read_items_.push_back(new ControlTableItem());
                ControlTableItem *dest_item = sensor->bulk_read_items_[i];
                ControlTableItem *src_item  = sensor->ctrl_table_[sub_tokens[i]];

                dest_item->item_name_       = src_item->item_name_;
                dest_item->address_         = indirect_data_addr;
                dest_item->access_type_     = src_item->access_type_;
                dest_item->memory_type_     = src_item->memory_type_;
                dest_item->data_length_     = src_item->data_length_;
                dest_item->data_min_value_  = src_item->data_min_value_;
                dest_item->data_max_value_  = src_item->data_max_value_;
                dest_item->is_signed_       = src_item->is_signed_;

                indirect_data_addr += dest_item->data_length_;
              }
            }
            else    // INDIRECT_ADDRESS_1 exist
            {
              for (int i = 0; i < sub_tokens.size(); i++)
                sensor->bulk_read_items_.push_back(sensor->ctrl_table_[sub_tokens[i]]);
            }
          }
        }
      }
    }
    file.close();
  }
  else
  {
    std::cout << "Unable to open file : " + robot_file_path << std::endl;
  }
}

Sensor *Robot::getSensor(std::string path, int id, std::string port, float protocol_version)
{
  Sensor *sensor;

  // load file model_name.device
  std::ifstream file(path.c_str());
  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;

    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
        input_str = input_str.substr(0, pos);

      // trim
      input_str = trim(input_str);
      if (input_str == "")
        continue;

      // find _session
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size() - 1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size() - 2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }

      if (session == SESSION_DEVICE_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '=');
        if (tokens.size() != 2)
          continue;

        if (tokens[0] == "model_name")
          sensor = new Sensor(id, tokens[1], protocol_version);
      }
      else if (session == SESSION_CONTROL_TABLE)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 8)
          continue;

        ControlTableItem *item = new ControlTableItem();
        item->item_name_    = tokens[1];
        item->address_      = std::atoi(tokens[0].c_str());
        item->data_length_  = std::atoi(tokens[2].c_str());

        if (tokens[3] == "R")
          item->access_type_ = Read;
        else if (tokens[3] == "RW")
          item->access_type_ = ReadWrite;

        if (tokens[4] == "EEPROM")
          item->memory_type_ = EEPROM;
        else if (tokens[4] == "RAM")
          item->memory_type_ = RAM;

        item->data_min_value_ = std::atol(tokens[5].c_str());
        item->data_max_value_ = std::atol(tokens[6].c_str());

        if (tokens[7] == "Y")
          item->is_signed_ = true;
        else if (tokens[7] == "N")
          item->is_signed_ = false;
        sensor->ctrl_table_[tokens[1]] = item;
      }
    }
    sensor->port_name_ = port;

    fprintf(stderr, "(%s) [ID:%3d] %14s added. \n", port.c_str(), sensor->id_, sensor->model_name_.c_str());
    //std::cout << "[ID:" << (int)(_sensor->id) << "] " << _sensor->model_name << " added. (" << port << ")" << std::endl;
    file.close();
  }
  else
    std::cout << "Unable to open file : " + path << std::endl;

  return sensor;
}

Dynamixel *Robot::getDynamixel(std::string path, int id, std::string port, float protocol_version)
{
  Dynamixel *dxl;

  // load file model_name.device
  std::ifstream file(path.c_str());
  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;

    std::string torque_enable_item_name = "";
    std::string present_position_item_name = "";
    std::string present_velocity_item_name = "";
    std::string present_current_item_name = "";
    std::string goal_position_item_name = "";
    std::string goal_velocity_item_name = "";
    std::string goal_current_item_name = "";
    std::string position_d_gain_item_name = "";
    std::string position_i_gain_item_name = "";
    std::string position_p_gain_item_name = "";
    std::string velocity_d_gain_item_name = "";
    std::string velocity_i_gain_item_name = "";
    std::string velocity_p_gain_item_name = "";

    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
        input_str = input_str.substr(0, pos);

      // trim
      input_str = trim(input_str);
      if (input_str == "")
        continue;

      // find session
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size() - 1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size() - 2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }

      if (session == SESSION_DEVICE_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '=');
        if (tokens.size() != 2)
          continue;

        if (tokens[0] == "model_name")
          dxl = new Dynamixel(id, tokens[1], protocol_version);
      }
      else if (session == SESSION_TYPE_INFO)
      {
        std::vector<std::string> tokens = split(input_str, '=');
        if (tokens.size() != 2)
          continue;

        if (tokens[0] == "torque_to_current_value_ratio")
          dxl->torque_to_current_value_ratio_ = std::atof(tokens[1].c_str());
        else if (tokens[0] == "velocity_to_value_ratio")
          dxl->velocity_to_value_ratio_ = std::atof(tokens[1].c_str());

        else if (tokens[0] == "value_of_0_radian_position")
          dxl->value_of_0_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "value_of_min_radian_position")
          dxl->value_of_min_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "value_of_max_radian_position")
          dxl->value_of_max_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "min_radian")
          dxl->min_radian_ = std::atof(tokens[1].c_str());
        else if (tokens[0] == "max_radian")
          dxl->max_radian_ = std::atof(tokens[1].c_str());

        else if (tokens[0] == "torque_enable_item_name")
          torque_enable_item_name = tokens[1];
        else if (tokens[0] == "present_position_item_name")
          present_position_item_name = tokens[1];
        else if (tokens[0] == "present_velocity_item_name")
          present_velocity_item_name = tokens[1];
        else if (tokens[0] == "present_current_item_name")
          present_current_item_name = tokens[1];
        else if (tokens[0] == "goal_position_item_name")
          goal_position_item_name = tokens[1];
        else if (tokens[0] == "goal_velocity_item_name")
          goal_velocity_item_name = tokens[1];
        else if (tokens[0] == "goal_current_item_name")
          goal_current_item_name = tokens[1];
        else if (tokens[0] == "position_d_gain_item_name")
          position_d_gain_item_name = tokens[1];
        else if (tokens[0] == "position_i_gain_item_name")
          position_i_gain_item_name = tokens[1];
        else if (tokens[0] == "position_p_gain_item_name")
          position_p_gain_item_name = tokens[1];
        else if (tokens[0] == "velocity_d_gain_item_name")
          velocity_d_gain_item_name = tokens[1];
        else if (tokens[0] == "velocity_i_gain_item_name")
          velocity_i_gain_item_name = tokens[1];
        else if (tokens[0] == "velocity_p_gain_item_name")
          velocity_p_gain_item_name = tokens[1];
      }
      else if (session == SESSION_CONTROL_TABLE)
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if (tokens.size() != 8)
          continue;

        ControlTableItem *item = new ControlTableItem();
        item->item_name_    = tokens[1];
        item->address_      = std::atoi(tokens[0].c_str());
        item->data_length_  = std::atoi(tokens[2].c_str());

        if (tokens[3] == "R")
          item->access_type_ = Read;
        else if (tokens[3] == "RW")
          item->access_type_ = ReadWrite;

        if (tokens[4] == "EEPROM")
          item->memory_type_ = EEPROM;
        else if (tokens[4] == "RAM")
          item->memory_type_ = RAM;

        item->data_min_value_ = std::atol(tokens[5].c_str());
        item->data_max_value_ = std::atol(tokens[6].c_str());

        if (tokens[7] == "Y")
          item->is_signed_ = true;
        else if (tokens[7] == "N")
          item->is_signed_ = false;
        dxl->ctrl_table_[tokens[1]] = item;
      }
    }
    dxl->port_name_ = port;

    if (dxl->ctrl_table_[torque_enable_item_name] != NULL)
      dxl->torque_enable_item_ = dxl->ctrl_table_[torque_enable_item_name];
    if (dxl->ctrl_table_[present_position_item_name] != NULL)
      dxl->present_position_item_ = dxl->ctrl_table_[present_position_item_name];
    if (dxl->ctrl_table_[present_velocity_item_name] != NULL)
      dxl->present_velocity_item_ = dxl->ctrl_table_[present_velocity_item_name];
    if (dxl->ctrl_table_[present_current_item_name] != NULL)
      dxl->present_current_item_ = dxl->ctrl_table_[present_current_item_name];
    if (dxl->ctrl_table_[goal_position_item_name] != NULL)
      dxl->goal_position_item_ = dxl->ctrl_table_[goal_position_item_name];
    if (dxl->ctrl_table_[goal_velocity_item_name] != NULL)
      dxl->goal_velocity_item_ = dxl->ctrl_table_[goal_velocity_item_name];
    if (dxl->ctrl_table_[goal_current_item_name] != NULL)
      dxl->goal_current_item_ = dxl->ctrl_table_[goal_current_item_name];
    if (dxl->ctrl_table_[position_d_gain_item_name] != NULL)
      dxl->position_d_gain_item_ = dxl->ctrl_table_[position_d_gain_item_name];
    if (dxl->ctrl_table_[position_i_gain_item_name] != NULL)
      dxl->position_i_gain_item_ = dxl->ctrl_table_[position_i_gain_item_name];
    if (dxl->ctrl_table_[position_p_gain_item_name] != NULL)
      dxl->position_p_gain_item_ = dxl->ctrl_table_[position_p_gain_item_name];
    if (dxl->ctrl_table_[velocity_d_gain_item_name] != NULL)
      dxl->velocity_d_gain_item_ = dxl->ctrl_table_[velocity_d_gain_item_name];
    if (dxl->ctrl_table_[velocity_i_gain_item_name] != NULL)
      dxl->velocity_i_gain_item_ = dxl->ctrl_table_[velocity_i_gain_item_name];
    if (dxl->ctrl_table_[velocity_p_gain_item_name] != NULL)
      dxl->velocity_p_gain_item_ = dxl->ctrl_table_[velocity_p_gain_item_name];

    fprintf(stderr, "(%s) [ID:%3d] %14s added. \n", port.c_str(), dxl->id_, dxl->model_name_.c_str());
    //std::cout << "[ID:" << (int)(_dxl->id) << "] " << _dxl->model_name << " added. (" << port << ")" << std::endl;
    file.close();
  }
  else
    std::cout << "Unable to open file : " + path << std::endl;

  return dxl;
}

int Robot::getControlCycle()
{
  return control_cycle_msec_;
}

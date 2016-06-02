/*
 * RobotisController.cpp
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#include <ros/package.h>
#include <ros/callback_queue.h>

#include "robotis_controller/RobotisController.h"

using namespace ROBOTIS;

RobotisController::RobotisController()
    : is_timer_running_(false),
      stop_timer_(false),
      init_pose_loaded_(false),
      timer_thread_(0),
      controller_mode_(MOTION_MODULE_MODE),
      DEBUG_PRINT(false),
      robot(0),
      gazebo_mode(false),
      gazebo_robot_name("robotis")
{
    direct_sync_write_.clear();
}

void RobotisController::InitSyncWrite()
{
    if(gazebo_mode == true)
        return;

    ROS_INFO("FIRST BULKREAD");
    // bulkread twice
    for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
        _it->second->TxRxPacket();
    for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
    {
        int _error_cnt = 0;
        int _result = COMM_SUCCESS;
        do {
            if(++_error_cnt > 10)
            {
                ROS_ERROR("[RobotisController] bulk read fail!!");
                exit(-1);
            }
            usleep(10*1000);
            _result = _it->second->TxRxPacket();
        } while (_result != COMM_SUCCESS);
    }
    init_pose_loaded_ = true;
    ROS_INFO("FIRST BULKREAD END");

    // clear syncwrite param setting
    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position.begin(); _it != port_to_sync_write_position.end(); _it++)
    {
        if(_it->second != NULL)
            _it->second->ClearParam();
    }
    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position_p_gain.begin(); _it != port_to_sync_write_position_p_gain.end(); _it++)
    {
        if(_it->second != NULL)
            _it->second->ClearParam();
    }
    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_velocity.begin(); _it != port_to_sync_write_velocity.end(); _it++)
    {
        if(_it->second != NULL)
            _it->second->ClearParam();
    }
    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_torque.begin(); _it != port_to_sync_write_torque.end(); _it++)
    {
        if(_it->second != NULL)
            _it->second->ClearParam();
    }

    // set init syncwrite param(from data of bulkread)
    for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
    {
        std::string _joint_name = _it->first;
        Dynamixel  *_dxl        = _it->second;

        for(int _i = 0; _i < _dxl->bulk_read_items.size(); _i++)
        {
            UINT32_T _read_data = 0;
            UINT8_T _sync_write_data[4];

            if(port_to_bulk_read[_dxl->port_name]->IsAvailable(_dxl->id,
                                                               _dxl->bulk_read_items[_i]->address,
                                                               _dxl->bulk_read_items[_i]->data_length) == true)
            {
                _read_data = port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id,
                                                                         _dxl->bulk_read_items[_i]->address,
                                                                         _dxl->bulk_read_items[_i]->data_length);

                _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_read_data));
                _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_read_data));
                _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_read_data));
                _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_read_data));

                if(_dxl->present_position_item != 0 && _dxl->bulk_read_items[_i]->item_name == _dxl->present_position_item->item_name)
                {
                    _dxl->dxl_state->present_position = _dxl->ConvertValue2Radian(_read_data) - _dxl->dxl_state->position_offset;   // remove offset
                    _dxl->dxl_state->goal_position = _dxl->dxl_state->present_position;

                    port_to_sync_write_position[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);
                }
                else if(_dxl->position_p_gain_item != 0 && _dxl->bulk_read_items[_i]->item_name == _dxl->position_p_gain_item->item_name)
                {
                    _dxl->dxl_state->position_p_gain = _read_data;
                }
                else if(_dxl->present_velocity_item != 0 && _dxl->bulk_read_items[_i]->item_name == _dxl->present_velocity_item->item_name)
                {
                    _dxl->dxl_state->present_velocity = _dxl->ConvertValue2Velocity(_read_data);
                    _dxl->dxl_state->goal_velocity = _dxl->dxl_state->present_velocity;
                }
                else if(_dxl->present_current_item != 0 && _dxl->bulk_read_items[_i]->item_name == _dxl->present_current_item->item_name)
                {
                    _dxl->dxl_state->present_current = _dxl->ConvertValue2Current(_read_data);
                    _dxl->dxl_state->goal_current = _dxl->dxl_state->present_current;
                }
            }
        }
    }
}

bool RobotisController::Initialize(const std::string robot_file_path, const std::string init_file_path)
{
    std::string _dev_desc_dir_path  = ros::package::getPath("robotis_device") + "/devices";

    // load robot info : port , device
    robot = new Robot(robot_file_path, _dev_desc_dir_path);

    if(gazebo_mode == true)
    {
        queue_thread_ = boost::thread(boost::bind(&RobotisController::QueueThread, this));
        return true;
    }

    for(std::map<std::string, PortHandler *>::iterator _it = robot->ports.begin(); _it != robot->ports.end(); _it++)
    {
        std::string     _port_name  = _it->first;
        PortHandler    *_port       = _it->second;

        PacketHandler  *_default_pkt_handler    = PacketHandler::GetPacketHandler(2.0);

        if(_port->SetBaudRate(_port->GetBaudRate()) == false)
        {
            ROS_ERROR("PORT [%s] SETUP ERROR! (baudrate: %d)", _port_name.c_str(), _port->GetBaudRate());
            exit(-1);
        }

        // get the default device info of the port
        std::string _default_device_name = robot->port_default_device[_port_name];
        std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find(_default_device_name);
        std::map<std::string, Sensor*>::iterator _sensor_it = robot->sensors.find(_default_device_name);
        if(_dxl_it != robot->dxls.end())
        {
            Dynamixel *_default_device = _dxl_it->second;
            _default_pkt_handler = PacketHandler::GetPacketHandler(_default_device->protocol_version);

            if(_default_device->goal_position_item != 0)
            {
                port_to_sync_write_position[_port_name] = new GroupSyncWrite(_port,
                                                                             _default_pkt_handler,
                                                                             _default_device->goal_position_item->address,
                                                                             _default_device->goal_position_item->data_length);
            }

            if(_default_device->position_p_gain_item != 0)
            {
                port_to_sync_write_position_p_gain[_port_name] = new GroupSyncWrite(_port,
                                                                             _default_pkt_handler,
                                                                             _default_device->position_p_gain_item->address,
                                                                             _default_device->position_p_gain_item->data_length);
            }

            if(_default_device->goal_velocity_item != 0)
            {
                port_to_sync_write_velocity[_port_name] = new GroupSyncWrite(_port,
                                                                             _default_pkt_handler,
                                                                             _default_device->goal_velocity_item->address,
                                                                             _default_device->goal_velocity_item->data_length);
            }

            if(_default_device->goal_current_item != 0)
            {
                port_to_sync_write_torque[_port_name]   = new GroupSyncWrite(_port,
                                                                             _default_pkt_handler,
                                                                             _default_device->goal_current_item->address,
                                                                             _default_device->goal_current_item->data_length);
            }
        }
        else if(_sensor_it != robot->sensors.end())
        {
            Sensor *_default_device = _sensor_it->second;
            _default_pkt_handler = PacketHandler::GetPacketHandler(_default_device->protocol_version);
        }

        port_to_bulk_read[_port_name] = new GroupBulkRead(_port, _default_pkt_handler);
    }

    // (for loop) check all dxls are connected.
    for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
    {
        std::string _joint_name = _it->first;
        Dynamixel  *_dxl        = _it->second;

        if(Ping(_joint_name) != 0)
        {
            usleep(10*1000);
            if(Ping(_joint_name) != 0)
                ROS_ERROR("JOINT[%s] does NOT respond!!", _joint_name.c_str());
        }
    }

    InitDevice(init_file_path);

    // [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
    for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
    {
        std::string _joint_name = _it->first;
        Dynamixel  *_dxl        = _it->second;

        if(_dxl == NULL)
            continue;

        int _bulkread_start_addr    = 0;
        int _bulkread_data_length   = 0;

//        // bulk read default : present position
//        if(_dxl->present_position_item != 0)
//        {
//            _bulkread_start_addr    = _dxl->present_position_item->address;
//            _bulkread_data_length   = _dxl->present_position_item->data_length;
//        }

        // calculate bulk read start address & data length
        std::map<std::string, ControlTableItem *>::iterator _indirect_addr_it = _dxl->ctrl_table.find(INDIRECT_ADDRESS_1);
        if(_indirect_addr_it != _dxl->ctrl_table.end())    // INDIRECT_ADDRESS_1 exist
        {
            if(_dxl->bulk_read_items.size() != 0)
            {
                _bulkread_start_addr    = _dxl->bulk_read_items[0]->address;
                _bulkread_data_length   = 0;

                // set indirect address
                int _indirect_addr = _indirect_addr_it->second->address;
                for(int _i = 0; _i < _dxl->bulk_read_items.size(); _i++)
                {
                    int _addr_leng = _dxl->bulk_read_items[_i]->data_length;

                    _bulkread_data_length += _addr_leng;
                    for(int _l = 0; _l < _addr_leng; _l++)
                    {
//                        ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", _joint_name.c_str(), _indirect_addr, _dxl->ctrl_table[_dxl->bulk_read_items[_i]->item_name]->address + _l);
                        Write2Byte(_joint_name, _indirect_addr, _dxl->ctrl_table[_dxl->bulk_read_items[_i]->item_name]->address + _l);
                        _indirect_addr += 2;
                    }
                }
            }
        }
        else    // INDIRECT_ADDRESS_1 NOT exist
        {
            if(_dxl->bulk_read_items.size() != 0)
            {
                _bulkread_start_addr    = _dxl->bulk_read_items[0]->address;
                _bulkread_data_length   = 0;

                ControlTableItem *_last_item = _dxl->bulk_read_items[0];

                for(int _i = 0; _i < _dxl->bulk_read_items.size(); _i++)
                {
                    int _addr = _dxl->bulk_read_items[_i]->address;
                    if(_addr < _bulkread_start_addr)
                        _bulkread_start_addr = _addr;
                    else if(_last_item->address < _addr)
                        _last_item      = _dxl->bulk_read_items[_i];
                }

                _bulkread_data_length = _last_item->address - _bulkread_start_addr + _last_item->data_length;
            }
        }

//        ROS_WARN("[%12s] start_addr: %d, data_length: %d", _joint_name.c_str(), _bulkread_start_addr, _bulkread_data_length);
        if(_bulkread_start_addr != 0)
            port_to_bulk_read[_dxl->port_name]->AddParam(_dxl->id, _bulkread_start_addr, _bulkread_data_length);

        // Torque ON
        if(WriteCtrlItem(_joint_name, _dxl->torque_enable_item->item_name, 1) != COMM_SUCCESS)
            WriteCtrlItem(_joint_name, _dxl->torque_enable_item->item_name, 1);
    }

    for(std::map<std::string, Sensor*>::iterator _it = robot->sensors.begin(); _it != robot->sensors.end(); _it++)
    {
        std::string _sensor_name    = _it->first;
        Sensor     *_sensor         = _it->second;

        if(_sensor == NULL)
            continue;

        int _bulkread_start_addr    = 0;
        int _bulkread_data_length   = 0;

        // calculate bulk read start address & data length
        std::map<std::string, ControlTableItem *>::iterator _indirect_addr_it = _sensor->ctrl_table.find(INDIRECT_ADDRESS_1);
        if(_indirect_addr_it != _sensor->ctrl_table.end())    // INDIRECT_ADDRESS_1 exist
        {
            if(_sensor->bulk_read_items.size() != 0)
            {
                _bulkread_start_addr    = _sensor->bulk_read_items[0]->address;
                _bulkread_data_length   = 0;

                // set indirect address
                int _indirect_addr = _indirect_addr_it->second->address;
                for(int _i = 0; _i < _sensor->bulk_read_items.size(); _i++)
                {
                    int _addr_leng = _sensor->bulk_read_items[_i]->data_length;

                    _bulkread_data_length += _addr_leng;
                    for(int _l = 0; _l < _addr_leng; _l++)
                    {
//                        ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d", _sensor_name.c_str(), _indirect_addr, _sensor->ctrl_table[_sensor->bulk_read_items[_i]->item_name]->address + _l);
                        Write2Byte(_sensor_name, _indirect_addr, _sensor->ctrl_table[_sensor->bulk_read_items[_i]->item_name]->address + _l);
                        _indirect_addr += 2;
                    }
                }
            }
        }
        else    // INDIRECT_ADDRESS_1 NOT exist
        {
            if(_sensor->bulk_read_items.size() != 0)
            {
                _bulkread_start_addr    = _sensor->bulk_read_items[0]->address;
                _bulkread_data_length   = 0;

                ControlTableItem *_last_item = _sensor->bulk_read_items[0];

                for(int _i = 0; _i < _sensor->bulk_read_items.size(); _i++)
                {
                    int _addr = _sensor->bulk_read_items[_i]->address;
                    if(_addr < _bulkread_start_addr)
                        _bulkread_start_addr = _addr;
                    else if(_last_item->address < _addr)
                        _last_item      = _sensor->bulk_read_items[_i];
                }

                _bulkread_data_length = _last_item->address - _bulkread_start_addr + _last_item->data_length;
            }
        }

        //ROS_WARN("[%12s] start_addr: %d, data_length: %d", _sensor_name.c_str(), _bulkread_start_addr, _bulkread_data_length);
        if(_bulkread_start_addr != 0)
            port_to_bulk_read[_sensor->port_name]->AddParam(_sensor->id, _bulkread_start_addr, _bulkread_data_length);
    }

    queue_thread_ = boost::thread(boost::bind(&RobotisController::QueueThread, this));
    return true;
}

void RobotisController::InitDevice(const std::string init_file_path)
{
    // device initialize
    if(DEBUG_PRINT) ROS_WARN("INIT FILE LOAD");

    YAML::Node _doc;
    try{
        _doc = YAML::LoadFile(init_file_path.c_str());

        for(YAML::const_iterator _it_doc = _doc.begin(); _it_doc != _doc.end(); _it_doc++)
        {
            std::string _joint_name = _it_doc->first.as<std::string>();

            YAML::Node _joint_node = _doc[_joint_name];
            if(_joint_node.size() == 0)
                continue;

            Dynamixel *_dxl = NULL;
            std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find(_joint_name);
            if(_dxl_it != robot->dxls.end())
                _dxl = _dxl_it->second;

            if(_dxl == NULL)
            {
                ROS_WARN("Joint [%s] was not found.", _joint_name.c_str());
                continue;
            }
            if(DEBUG_PRINT) ROS_INFO("JOINT_NAME: %s", _joint_name.c_str());

            for(YAML::const_iterator _it_joint = _joint_node.begin(); _it_joint != _joint_node.end(); _it_joint++)
            {
                std::string _item_name  = _it_joint->first.as<std::string>();

                if(DEBUG_PRINT) ROS_INFO("  ITEM_NAME: %s", _item_name.c_str());

                UINT32_T    _value      = _it_joint->second.as<UINT32_T>();

                ControlTableItem *_item = _dxl->ctrl_table[_item_name];
                if(_item == NULL)
                {
                    ROS_WARN("Control Item [%s] was not found.", _item_name.c_str());
                    continue;
                }

                if(_item->memory_type == EEPROM)
                {
                    UINT8_T     _data8  = 0;
                    UINT16_T    _data16 = 0;
                    UINT32_T    _data32 = 0;

                    switch(_item->data_length)
                    {
                    case 1:
                        Read1Byte(_joint_name, _item->address, &_data8);
                        if(_data8 == _value)
                            continue;
                        break;
                    case 2:
                        Read2Byte(_joint_name, _item->address, &_data16);
                        if(_data16 == _value)
                            continue;
                        break;
                    case 4:
                        Read4Byte(_joint_name, _item->address, &_data32);
                        if(_data32 == _value)
                            continue;
                        break;
                    default:
                        break;
                    }
                }

                switch(_item->data_length)
                {
                case 1:
                    Write1Byte(_joint_name, _item->address, (UINT8_T)_value);
                    break;
                case 2:
                    Write2Byte(_joint_name, _item->address, (UINT16_T)_value);
                    break;
                case 4:
                    Write4Byte(_joint_name, _item->address, _value);
                    break;
                default:
                    break;
                }

                if(_item->memory_type == EEPROM)
                {
                    // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
                    usleep(_item->data_length * 55 * 1000);
                }
            }
        }
    } catch(const std::exception& e) {
        ROS_INFO("Dynamixel Init file not found.");
    }
}

void RobotisController::GazeboThread()
{
    ros::Rate gazebo_rate(1000 / CONTROL_CYCLE_MSEC);

    while(!stop_timer_)
    {
        if(init_pose_loaded_ == true)
            Process();
        gazebo_rate.sleep();
    }
}

void RobotisController::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    ros::Subscriber _sync_write_item_sub    = _ros_node.subscribe("/robotis/sync_write_item", 10, &RobotisController::SyncWriteItemCallback, this);
    ros::Subscriber _joint_ctrl_modules_sub = _ros_node.subscribe("/robotis/set_joint_ctrl_modules", 10, &RobotisController::SetJointCtrlModuleCallback, this);
    ros::Subscriber _enable_ctrl_module_sub	= _ros_node.subscribe("/robotis/enable_ctrl_module", 10, &RobotisController::SetCtrlModuleCallback, this);
    ros::Subscriber _control_mode_sub       = _ros_node.subscribe("/robotis/set_control_mode", 10, &RobotisController::SetControllerModeCallback, this);
    ros::Subscriber _joint_states_sub       = _ros_node.subscribe("/robotis/set_joint_states", 10, &RobotisController::SetJointStatesCallback, this);

    ros::Subscriber _gazebo_joint_states_sub;
    if(gazebo_mode == true)
        _gazebo_joint_states_sub = _ros_node.subscribe("/" + gazebo_robot_name + "/joint_states", 10, &RobotisController::GazeboJointStatesCallback, this);

    /* publisher */
    goal_joint_state_pub        = _ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
    present_joint_state_pub     = _ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_states", 10);
    current_module_pub          = _ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/present_joint_ctrl_modules", 10);

    if(gazebo_mode == true)
    {
        for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
            gazebo_joint_pub[_it->first] = _ros_node.advertise<std_msgs::Float64>("/" + gazebo_robot_name + "/" + _it->first + "_pos/command", 1);
    }

    /* service */
    ros::ServiceServer _joint_module_server = _ros_node.advertiseService("/robotis/get_present_joint_ctrl_modules", &RobotisController::GetCtrlModuleCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
        usleep(100);
    }
}

void *RobotisController::ThreadProc(void *param)
{
    RobotisController *controller = (RobotisController *)param;
    static struct timespec next_time;
    static struct timespec curr_time;

    ROS_INFO("controller::thread_proc");

    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while(!controller->stop_timer_)
    {
        next_time.tv_sec   += (next_time.tv_nsec + CONTROL_CYCLE_MSEC * 1000000) / 1000000000;
        next_time.tv_nsec   = (next_time.tv_nsec + CONTROL_CYCLE_MSEC * 1000000) % 1000000000;

        controller->Process();

        clock_gettime(CLOCK_MONOTONIC, &curr_time);
        long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
        if(delta_nsec < -100000 )
        {
            if(controller->DEBUG_PRINT == true)
                ROS_WARN("[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]", delta_nsec/1000000.0, (long)next_time.tv_sec, (long)next_time.tv_nsec, (long)curr_time.tv_sec, (long)curr_time.tv_nsec);
            // next_time = curr_time + 3 msec
            next_time.tv_sec    = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
            next_time.tv_nsec   = (curr_time.tv_nsec + 3000000) % 1000000000;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    return 0;
}

void RobotisController::StartTimer()
{
    if(this->is_timer_running_ == true)
        return;

    if(this->gazebo_mode == true)
    {
        // create and start the thread
        gazebo_thread_ = boost::thread(boost::bind(&RobotisController::GazeboThread, this));
    }
    else
    {
        InitSyncWrite();

        for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
            _it->second->TxPacket();

        int error;
        struct sched_param param;
        pthread_attr_t attr;

        pthread_attr_init(&attr);

        error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
        if(error != 0)
            ROS_ERROR("pthread_attr_setschedpolicy error = %d\n",error);
        error = pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
        if(error != 0)
            ROS_ERROR("pthread_attr_setinheritsched error = %d\n",error);

        memset(&param, 0, sizeof(param));
        param.sched_priority = 31;// RT
        error = pthread_attr_setschedparam(&attr, &param);
        if(error != 0)
            ROS_ERROR("pthread_attr_setschedparam error = %d\n",error);

        // create and start the thread
        if((error = pthread_create(&this->timer_thread_, &attr, this->ThreadProc, this))!= 0) {
            ROS_ERROR("timer thread create fail!!");
            exit(-1);
        }
    }

    this->is_timer_running_ = true;
}

void RobotisController::StopTimer()
{
    int error = 0;

    // set the flag to stop the thread
    if(this->is_timer_running_)
    {
        this->stop_timer_ = true;

        if(this->gazebo_mode == false)
        {
            // wait until the thread is stopped.
            if((error = pthread_join(this->timer_thread_, NULL)) != 0)
                exit(-1);

            for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
            {
                if(_it->second != NULL)
                    _it->second->RxPacket();
            }

            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position.begin(); _it != port_to_sync_write_position.end(); _it++)
            {
                if(_it->second != NULL)
                    _it->second->ClearParam();
            }
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position_p_gain.begin(); _it != port_to_sync_write_position_p_gain.end(); _it++)
            {
                if(_it->second != NULL)
                    _it->second->ClearParam();
            }
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_velocity.begin(); _it != port_to_sync_write_velocity.end(); _it++)
            {
                if(_it->second != NULL)
                    _it->second->ClearParam();
            }
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_torque.begin(); _it != port_to_sync_write_torque.end(); _it++)
            {
                if(_it->second != NULL)
                    _it->second->ClearParam();
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

bool RobotisController::IsTimerRunning()
{
    return this->is_timer_running_;
}

void RobotisController::LoadOffset(const std::string path)
{
    YAML::Node _doc;
    try{
        _doc = YAML::LoadFile(path.c_str());
    } catch(const std::exception& e) {
        ROS_ERROR("Fail to load offset yaml.");
        return;
    }

    YAML::Node _offset_node = _doc["offset"];
    if(_offset_node.size() == 0)
        return;

    ROS_INFO("Load offsets...");
    for(YAML::const_iterator _it = _offset_node.begin(); _it != _offset_node.end(); _it++)
    {
        std::string _joint_name = _it->first.as<std::string>();
        double      _offset     = _it->second.as<double>();

        std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find(_joint_name);
        if(_dxl_it != robot->dxls.end())
            _dxl_it->second->dxl_state->position_offset = _offset;
    }
}

void RobotisController::Process()
{
    // avoid duplicated function call
    static bool _is_process_running = false;
    if(_is_process_running == true)
        return;
    _is_process_running = true;

    // ROS_INFO("Controller::Process()");
    bool _do_sync_write = false;

    ros::Time _start_time;
    ros::Duration _time_duration;

    if(DEBUG_PRINT)
        _start_time = ros::Time::now();

    sensor_msgs::JointState _goal_state;
    sensor_msgs::JointState _present_state;

    _present_state.header.stamp = ros::Time::now();
    _goal_state.header.stamp    = _present_state.header.stamp;

    // BulkRead Rx
    // -> save to Robot->dxls[]->dynamixel_state.present_position
    if(gazebo_mode == false)
    {
        // BulkRead Rx
        for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
        {
            robot->ports[_it->first]->SetPacketTimeout(0.0);
            _it->second->RxPacket();
        }

        if(robot->dxls.size() > 0)
        {
            for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
            {
                Dynamixel  *_dxl        = dxl_it->second;
                std::string _port_name  = dxl_it->second->port_name;
                std::string _joint_name = dxl_it->first;

                if(_dxl->bulk_read_items.size() > 0)
                {
                    UINT32_T _data = 0;
                    for(int _i = 0; _i < _dxl->bulk_read_items.size(); _i++)
                    {
                        ControlTableItem   *_item   = _dxl->bulk_read_items[_i];
                        if(port_to_bulk_read[_port_name]->IsAvailable(_dxl->id, _item->address, _item->data_length) == true)
                        {
                            _data = port_to_bulk_read[_port_name]->GetData(_dxl->id, _item->address, _item->data_length);

                            // change dxl_state
                            if(_dxl->present_position_item != 0 && _item->item_name == _dxl->present_position_item->item_name)
                                _dxl->dxl_state->present_position = _dxl->ConvertValue2Radian(_data) - _dxl->dxl_state->position_offset; // remove offset
                            else if(_dxl->present_velocity_item != 0 && _item->item_name == _dxl->present_velocity_item->item_name)
                                _dxl->dxl_state->present_velocity = _dxl->ConvertValue2Velocity(_data);
                            else if(_dxl->present_current_item != 0 && _item->item_name == _dxl->present_current_item->item_name)
                                _dxl->dxl_state->present_current = _dxl->ConvertValue2Current(_data);
                            else if(_dxl->goal_position_item != 0 && _item->item_name == _dxl->goal_position_item->item_name)
                                _dxl->dxl_state->goal_position = _dxl->ConvertValue2Radian(_data) - _dxl->dxl_state->position_offset; // remove offset
                            else if(_dxl->goal_velocity_item != 0 && _item->item_name == _dxl->goal_velocity_item->item_name)
                                _dxl->dxl_state->goal_velocity = _dxl->ConvertValue2Velocity(_data);
                            else if(_dxl->goal_current_item != 0 && _item->item_name == _dxl->goal_current_item->item_name)
                                _dxl->dxl_state->goal_current = _dxl->ConvertValue2Current(_data);
                            else
                                _dxl->dxl_state->bulk_read_table[_item->item_name] = _data;
                        }
                    }

                    // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
                    _dxl->dxl_state->update_time_stamp = TimeStamp(_present_state.header.stamp.sec, _present_state.header.stamp.nsec);
                }
            }
        }

        if(robot->sensors.size() > 0)
        {
            for(std::map<std::string, Sensor *>::iterator sensor_it = robot->sensors.begin(); sensor_it != robot->sensors.end(); sensor_it++)
            {
                Sensor     *_sensor         = sensor_it->second;
                std::string _port_name      = sensor_it->second->port_name;
                std::string _sensor_name    = sensor_it->first;

                if(_sensor->bulk_read_items.size() > 0)
                {
                    UINT32_T _data = 0;
                    for(int _i = 0; _i < _sensor->bulk_read_items.size(); _i++)
                    {
                        ControlTableItem *_item = _sensor->bulk_read_items[_i];
                        if(port_to_bulk_read[_port_name]->IsAvailable(_sensor->id, _item->address, _item->data_length) == true)
                        {
                            _data = port_to_bulk_read[_port_name]->GetData(_sensor->id, _item->address, _item->data_length);

                            // change sensor_state
                            _sensor->sensor_state->bulk_read_table[_item->item_name] = _data;
                        }
                    }

                    // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
                    _sensor->sensor_state->update_time_stamp = TimeStamp(_present_state.header.stamp.sec, _present_state.header.stamp.nsec);
                }
            }
        }
    }

    if(DEBUG_PRINT)
    {
        _time_duration = ros::Time::now() - _start_time;
        ROS_INFO("(%2.6f) BulkRead Rx & update state", _time_duration.nsec * 0.000001);
    }

    // Call SensorModule Process()
    // -> for loop : call SensorModule list -> Process()
    if(sensor_modules_.size() > 0)
    {
        for(std::list<SensorModule*>::iterator _module_it = sensor_modules_.begin(); _module_it != sensor_modules_.end(); _module_it++)
        {
            (*_module_it)->Process(robot->dxls, robot->sensors);

            for(std::map<std::string, double>::iterator _it = (*_module_it)->result.begin(); _it != (*_module_it)->result.end(); _it++)
                sensor_result_[_it->first] = _it->second;
        }
    }

    if(DEBUG_PRINT)
    {
        _time_duration = ros::Time::now() - _start_time;
        ROS_INFO("(%2.6f) SensorModule Process() & save result", _time_duration.nsec * 0.000001);
    }

    if(controller_mode_ == MOTION_MODULE_MODE)
    {
        // Call MotionModule Process()
        // -> for loop : call MotionModule list -> Process()
        if(motion_modules_.size() > 0)
        {
            queue_mutex_.lock();

            for(std::list<MotionModule*>::iterator module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++)
            {
                if((*module_it)->GetModuleEnable() == false)
                    continue;

                (*module_it)->Process(robot->dxls, sensor_result_);

                // for loop : joint list
                for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
                {
                    std::string     _joint_name = dxl_it->first;
                    Dynamixel      *_dxl        = dxl_it->second;
                    DynamixelState *_dxl_state  = dxl_it->second->dxl_state;

                    if(_dxl->ctrl_module_name == (*module_it)->GetModuleName())
                    {
                        _do_sync_write = true;
                        DynamixelState *_result_state = (*module_it)->result[_joint_name];

                        if(_result_state == NULL) {
                            ROS_INFO("[%s] %s", (*module_it)->GetModuleName().c_str(), _joint_name.c_str());
                            continue;
                        }

                        // TODO: check update time stamp ?

                        if((*module_it)->GetControlMode() == POSITION_CONTROL)
                        {
//                            if(_result_state->goal_position == 0 && _dxl->id == 3)
//                                ROS_INFO("[MODULE:%s][ID:%2d] goal position = %f", (*module_it)->GetModuleName().c_str(), _dxl->id, _dxl_state->goal_position);
                            _dxl_state->goal_position = _result_state->goal_position;

                            if(gazebo_mode == false)
                            {
                                // add offset
                                UINT32_T _pos_data = _dxl->ConvertRadian2Value(_dxl_state->goal_position + _dxl_state->position_offset);
                                UINT8_T  _sync_write_data[4] = {0};
                                _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
                                _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
                                _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
                                _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

                                if(abs(_pos_data) > 151800)
                                    printf("goal_pos : %f |  position_offset : %f | pos_data : %d\n", _dxl_state->goal_position , _dxl_state->position_offset, _pos_data);

                                if(port_to_sync_write_position[_dxl->port_name] != NULL)
                                    port_to_sync_write_position[_dxl->port_name]->ChangeParam(_dxl->id, _sync_write_data);

                                // if position p gain value is changed -> sync write
                                if(_dxl_state->position_p_gain != _result_state->position_p_gain)
                                {
                                    _dxl_state->position_p_gain = _result_state->position_p_gain;

                                    UINT8_T _sync_write_p_gain[4] = {0};
                                    _sync_write_p_gain[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_state->position_p_gain));
                                    _sync_write_p_gain[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_state->position_p_gain));
                                    _sync_write_p_gain[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_state->position_p_gain));
                                    _sync_write_p_gain[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_state->position_p_gain));

                                    if(port_to_sync_write_position_p_gain[_dxl->port_name] != NULL)
                                        port_to_sync_write_position_p_gain[_dxl->port_name]->AddParam(_dxl->id, _sync_write_p_gain);
                                }
                            }
                        }
                        else if((*module_it)->GetControlMode() == VELOCITY_CONTROL)
                        {
                            _dxl_state->goal_velocity = _result_state->goal_velocity;

                            if(gazebo_mode == false)
                            {
                                UINT32_T _vel_data = _dxl->ConvertVelocity2Value(_dxl_state->goal_velocity);
                                UINT8_T  _sync_write_data[4] = {0};
                                _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_vel_data));
                                _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_vel_data));
                                _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_vel_data));
                                _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_vel_data));

                                if(port_to_sync_write_velocity[_dxl->port_name] != NULL)
                                    port_to_sync_write_velocity[_dxl->port_name]->ChangeParam(_dxl->id, _sync_write_data);
                            }
                        }
                        else if((*module_it)->GetControlMode() == CURRENT_CONTROL)
                        {
                            _dxl_state->goal_current = _result_state->goal_current;

                            if(gazebo_mode == false)
                            {
                                UINT32_T _torq_data = _dxl->ConvertCurrent2Value(_dxl_state->goal_current);
                                UINT8_T _sync_write_data[2];
                                _sync_write_data[0] = DXL_LOBYTE(_torq_data);
                                _sync_write_data[1] = DXL_HIBYTE(_torq_data);

                                if(port_to_sync_write_torque[_dxl->port_name] != NULL)
                                    port_to_sync_write_torque[_dxl->port_name]->ChangeParam(_dxl->id, _sync_write_data);
                            }
                        }
                    }
                }
            }

            queue_mutex_.unlock();
        }

        if(DEBUG_PRINT)
        {
            _time_duration = ros::Time::now() - _start_time;
            ROS_INFO("(%2.6f) MotionModule Process() & save result", _time_duration.nsec * 0.000001);
        }

        // SyncWrite
        if(gazebo_mode == false && _do_sync_write)
        {
            if(port_to_sync_write_position_p_gain.size() > 0)
            {
                for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position_p_gain.begin(); _it != port_to_sync_write_position_p_gain.end(); _it++)
                {
                    _it->second->TxPacket();
                    _it->second->ClearParam();
                }
            }
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position.begin(); _it != port_to_sync_write_position.end(); _it++)
                if(_it->second != NULL) _it->second->TxPacket();
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_velocity.begin(); _it != port_to_sync_write_velocity.end(); _it++)
                if(_it->second != NULL) _it->second->TxPacket();
            for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_torque.begin(); _it != port_to_sync_write_torque.end(); _it++)
                if(_it->second != NULL) _it->second->TxPacket();
        }
        else if(gazebo_mode == true)
        {
            std_msgs::Float64 _joint_msg;

            for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
            {
                _joint_msg.data = dxl_it->second->dxl_state->goal_position;
                gazebo_joint_pub[dxl_it->first].publish(_joint_msg);
            }
        }
    }
    else if(controller_mode_ == DIRECT_CONTROL_MODE)
    {
        queue_mutex_.lock();

        for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_position.begin(); _it != port_to_sync_write_position.end(); _it++)
        {
            _it->second->TxPacket();
            _it->second->ClearParam();
        }

        if(direct_sync_write_.size() > 0)
        {
            for(int _i = 0; _i < direct_sync_write_.size(); _i++)
            {
                direct_sync_write_[_i]->TxPacket();
                direct_sync_write_[_i]->ClearParam();
            }
            direct_sync_write_.clear();
        }

        queue_mutex_.unlock();
    }

    // TODO: User Read/Write

    // BulkRead Tx
    if(gazebo_mode == false)
    {
        for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
            _it->second->TxPacket();
    }

    if(DEBUG_PRINT)
    {
        _time_duration = ros::Time::now() - _start_time;
        ROS_INFO("(%2.6f) SyncWrite & BulkRead Tx", _time_duration.nsec * 0.000001);
    }

    // publish present & goal position
    for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
    {
        std::string _joint_name = dxl_it->first;
        Dynamixel  *_dxl        = dxl_it->second;

        _present_state.name.push_back(_joint_name);
        _present_state.position.push_back(_dxl->dxl_state->present_position);
        _present_state.velocity.push_back(_dxl->dxl_state->present_velocity);
        _present_state.effort.push_back(_dxl->dxl_state->present_current);

        _goal_state.name.push_back(_joint_name);
        _goal_state.position.push_back(_dxl->dxl_state->goal_position);
        _goal_state.velocity.push_back(_dxl->dxl_state->goal_velocity);
        _goal_state.effort.push_back(_dxl->dxl_state->goal_current);
    }

    // -> publish present joint_states & goal joint states topic
    present_joint_state_pub.publish(_present_state);
    goal_joint_state_pub.publish(_goal_state);

    if(DEBUG_PRINT)
    {
        _time_duration = ros::Time::now() - _start_time;
        ROS_WARN("(%2.6f) Process() DONE", _time_duration.nsec * 0.000001);
    }

    _is_process_running = false;
}

void RobotisController::AddMotionModule(MotionModule *module)
{
    // check whether the module name already exists
    for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
    {
        if((*_m_it)->GetModuleName() == module->GetModuleName())
        {
            ROS_ERROR("Motion Module Name [%s] already exist !!", module->GetModuleName().c_str());
            return;
        }
    }

    module->Initialize(CONTROL_CYCLE_MSEC, robot);
    motion_modules_.push_back(module);
    motion_modules_.unique();
}

void RobotisController::RemoveMotionModule(MotionModule *module)
{
    motion_modules_.remove(module);
}

void RobotisController::AddSensorModule(SensorModule *module)
{
    // check whether the module name already exists
    for(std::list<SensorModule *>::iterator _m_it = sensor_modules_.begin(); _m_it != sensor_modules_.end(); _m_it++)
    {
        if((*_m_it)->GetModuleName() == module->GetModuleName())
        {
            ROS_ERROR("Sensor Module Name [%s] already exist !!", module->GetModuleName().c_str());
            return;
        }
    }

    module->Initialize(CONTROL_CYCLE_MSEC, robot);
    sensor_modules_.push_back(module);
    sensor_modules_.unique();
}

void RobotisController::RemoveSensorModule(SensorModule *module)
{
    sensor_modules_.remove(module);
}

void RobotisController::SyncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr &msg)
{
    for(int _i = 0; _i < msg->joint_name.size(); _i++)
    {
        Dynamixel           *_dxl           = robot->dxls[msg->joint_name[_i]];
        ControlTableItem    *_item          = _dxl->ctrl_table[msg->item_name];

        PortHandler         *_port          = robot->ports[_dxl->port_name];
        PacketHandler       *_packet_handler= PacketHandler::GetPacketHandler(_dxl->protocol_version);

        if(_item->access_type == READ)
            continue;

        int _idx = 0;
        if(direct_sync_write_.size() == 0)
        {
            direct_sync_write_.push_back(new GroupSyncWrite(_port, _packet_handler, _item->address, _item->data_length));
            _idx = 0;
        }
        else
        {
            for(_idx = 0; _idx < direct_sync_write_.size(); _idx++)
            {
                if(direct_sync_write_[_idx]->GetPortHandler() == _port &&
                        direct_sync_write_[_idx]->GetPacketHandler() == _packet_handler)
                    break;
            }

            if(_idx == direct_sync_write_.size())
                direct_sync_write_.push_back(new GroupSyncWrite(_port, _packet_handler, _item->address, _item->data_length));
        }

        UINT8_T *_data = new UINT8_T[_item->data_length];
        if(_item->data_length == 1)
            _data[0] = (UINT8_T)msg->value[_i];
        else if(_item->data_length == 2)
        {
            _data[0] = DXL_LOBYTE((UINT16_T)msg->value[_i]);
            _data[1] = DXL_HIBYTE((UINT16_T)msg->value[_i]);
        }
        else if(_item->data_length == 4)
        {
            _data[0] = DXL_LOBYTE(DXL_LOWORD((UINT32_T)msg->value[_i]));
            _data[1] = DXL_HIBYTE(DXL_LOWORD((UINT32_T)msg->value[_i]));
            _data[2] = DXL_LOBYTE(DXL_HIWORD((UINT32_T)msg->value[_i]));
            _data[3] = DXL_HIBYTE(DXL_HIWORD((UINT32_T)msg->value[_i]));
        }
        direct_sync_write_[_idx]->AddParam(_dxl->id, _data);
        delete[] _data;
    }
}

void RobotisController::SetControllerModeCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "DIRECT_CONTROL_MODE")
        controller_mode_ = DIRECT_CONTROL_MODE;
    else if(msg->data == "MOTION_MODULE_MODE")
        controller_mode_ = MOTION_MODULE_MODE;
}

void RobotisController::SetJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if(controller_mode_ != DIRECT_CONTROL_MODE)
        return;

    queue_mutex_.lock();

    for(int _i = 0; _i < msg->name.size(); _i++)
    {
        INT32_T _pos = 0;

        Dynamixel *_dxl = robot->dxls[msg->name[_i]];
        if(_dxl == NULL)
            continue;

        _dxl->dxl_state->goal_position = msg->position[_i];
        _pos = _dxl->ConvertRadian2Value((double)msg->position[_i]);

        UINT8_T _sync_write_data[4];
        _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos));
        _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos));
        _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos));
        _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos));

        if(port_to_sync_write_position[_dxl->port_name] != NULL)
            port_to_sync_write_position[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);
    }

    queue_mutex_.unlock();
}

void RobotisController::SetCtrlModuleCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string _module_name_to_set = msg->data;

    set_module_thread_ = boost::thread(boost::bind(&RobotisController::SetCtrlModuleThread, this, _module_name_to_set));
}

void RobotisController::SetJointCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
    if(msg->joint_name.size() != msg->module_name.size())
        return;

    queue_mutex_.lock();

    for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
    {
        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find((std::string)(msg->joint_name[idx]));
        if(_dxl_it != robot->dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

        // none
        if(msg->module_name[idx] == "" || msg->module_name[idx] == "none")
        {
            _dxl->ctrl_module_name = msg->module_name[idx];
            continue;
        }

        // check whether the module is using this joint
        for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
        {
            if((*_m_it)->GetModuleName() == msg->module_name[idx])
            {
                if((*_m_it)->result.find(msg->joint_name[idx]) != (*_m_it)->result.end())
                {
                    _dxl->ctrl_module_name = msg->module_name[idx];
                    break;
                }
            }
        }
    }

    for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
    {
        // set all modules -> disable
        (*_m_it)->SetModuleEnable(false);

        // set all used modules -> enable
        for(std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.begin(); _d_it != robot->dxls.end(); _d_it++)
        {
            if(_d_it->second->ctrl_module_name == (*_m_it)->GetModuleName())
            {
                (*_m_it)->SetModuleEnable(true);
                break;
            }
        }
    }

    // TODO: set indirect address
    // -> check module's control_mode

    queue_mutex_.unlock();

    robotis_controller_msgs::JointCtrlModule _current_module_msg;
    for(std::map<std::string, Dynamixel *>::iterator _dxl_iter = robot->dxls.begin(); _dxl_iter  != robot->dxls.end(); ++_dxl_iter)
    {
        _current_module_msg.joint_name.push_back(_dxl_iter->first);
        _current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name);
    }

    if(_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
        current_module_pub.publish(_current_module_msg);
}

bool RobotisController::GetCtrlModuleCallback(robotis_controller_msgs::GetJointModule::Request &req, robotis_controller_msgs::GetJointModule::Response &res)
{
    for(unsigned int idx = 0; idx < req.joint_name.size(); idx++)
    {
        std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.find((std::string)(req.joint_name[idx]));
        if(_d_it != robot->dxls.end())
        {
            res.joint_name.push_back(req.joint_name[idx]);
            res.module_name.push_back(_d_it->second->ctrl_module_name);
        }
    }

    if(res.joint_name.size() == 0) return false;

    return true;
}

void RobotisController::SetCtrlModuleThread(std::string ctrl_module)
{
    // stop module
    std::list<MotionModule *> _stop_modules;

    if(ctrl_module == "" || ctrl_module == "none")
    {
        // enqueue all modules in order to stop
        for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
        {
            if((*_m_it)->GetModuleEnable() == true) _stop_modules.push_back(*_m_it);
        }
    }
    else
    {
        for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
        {
            // if it exist
            if((*_m_it)->GetModuleName() == ctrl_module)
            {
                // enqueue the module which lost control of joint in order to stop
                for(std::map<std::string, DynamixelState*>::iterator _result_it = (*_m_it)->result.begin(); _result_it != (*_m_it)->result.end(); _result_it++)
                {
                    std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.find(_result_it->first);

                    if(_d_it != robot->dxls.end())
                    {
                        // enqueue
                        if(_d_it->second->ctrl_module_name != ctrl_module)
                        {
                            for(std::list<MotionModule *>::iterator _stop_m_it = motion_modules_.begin(); _stop_m_it != motion_modules_.end(); _stop_m_it++)
                            {
                                if((*_stop_m_it)->GetModuleName() == _d_it->second->ctrl_module_name && (*_stop_m_it)->GetModuleEnable() == true)
                                    _stop_modules.push_back(*_stop_m_it);
                            }
                        }
                    }
                }

                break;
            }
        }
    }

    // stop the module
    _stop_modules.unique();
    for(std::list<MotionModule *>::iterator _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
    {
        (*_stop_m_it)->Stop();
    }

    // wait to stop
    for(std::list<MotionModule *>::iterator _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
    {
        while((*_stop_m_it)->IsRunning())
            usleep(CONTROL_CYCLE_MSEC * 1000);
    }

    // set ctrl module
    queue_mutex_.lock();

    if(DEBUG_PRINT) ROS_INFO_STREAM("set module : " << ctrl_module);

    // none
    if(ctrl_module == "" || ctrl_module == "none")
    {
        // set all modules -> disable
        for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
        {
            (*_m_it)->SetModuleEnable(false);
        }

        // set dxl's control module to "none"
        for(std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.begin(); _d_it != robot->dxls.end(); _d_it++)
        {
            Dynamixel *_dxl = _d_it->second;
            _dxl->ctrl_module_name = "none";

            if(gazebo_mode == true)
                continue;

            UINT32_T _pos_data = _dxl->ConvertRadian2Value(_dxl->dxl_state->goal_position + _dxl->dxl_state->position_offset);
            UINT8_T _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

            if(port_to_sync_write_position[_dxl->port_name] != NULL)
                port_to_sync_write_position[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);

            if(port_to_sync_write_torque[_dxl->port_name] != NULL)
                port_to_sync_write_torque[_dxl->port_name]->RemoveParam(_dxl->id);
            if(port_to_sync_write_velocity[_dxl->port_name] != NULL)
                port_to_sync_write_velocity[_dxl->port_name]->RemoveParam(_dxl->id);
        }
    }
    else
    {
        // check whether the module exist
        for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
        {
            // if it exist
            if((*_m_it)->GetModuleName() == ctrl_module)
            {
                CONTROL_MODE _mode = (*_m_it)->GetControlMode();
                for(std::map<std::string, DynamixelState*>::iterator _result_it = (*_m_it)->result.begin(); _result_it != (*_m_it)->result.end(); _result_it++)
                {
                    std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.find(_result_it->first);
                    if(_d_it != robot->dxls.end())
                    {
                        Dynamixel *_dxl = _d_it->second;
                        _dxl->ctrl_module_name = ctrl_module;

                        if(gazebo_mode == true)
                            continue;

                        if(_mode == POSITION_CONTROL)
                        {
                            UINT32_T _pos_data = _dxl->ConvertRadian2Value(_dxl->dxl_state->goal_position + _dxl->dxl_state->position_offset);
                            UINT8_T _sync_write_data[4];
                            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
                            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
                            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
                            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

                            if(port_to_sync_write_position[_dxl->port_name] != NULL)
                                port_to_sync_write_position[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);

                            if(port_to_sync_write_torque[_dxl->port_name] != NULL)
                                port_to_sync_write_torque[_dxl->port_name]->RemoveParam(_dxl->id);
                            if(port_to_sync_write_velocity[_dxl->port_name] != NULL)
                                port_to_sync_write_velocity[_dxl->port_name]->RemoveParam(_dxl->id);
                        }
                        else if(_mode == VELOCITY_CONTROL)
                        {
                            UINT32_T _vel_data = _dxl->ConvertVelocity2Value(_dxl->dxl_state->goal_velocity);
                            UINT8_T _sync_write_data[4];
                            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_vel_data));
                            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_vel_data));
                            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_vel_data));
                            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_vel_data));

                            if(port_to_sync_write_velocity[_dxl->port_name] != NULL)
                                port_to_sync_write_velocity[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);

                            if(port_to_sync_write_torque[_dxl->port_name] != NULL)
                                port_to_sync_write_torque[_dxl->port_name]->RemoveParam(_dxl->id);
                            if(port_to_sync_write_position[_dxl->port_name] != NULL)
                                port_to_sync_write_position[_dxl->port_name]->RemoveParam(_dxl->id);
                        }
                        else if(_mode == CURRENT_CONTROL)
                        {
                            UINT32_T _curr_data = _dxl->ConvertCurrent2Value(_dxl->dxl_state->goal_current);
                            UINT8_T _sync_write_data[4];
                            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_curr_data));
                            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_curr_data));
                            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_curr_data));
                            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_curr_data));

                            if(port_to_sync_write_torque[_dxl->port_name] != NULL)
                                port_to_sync_write_torque[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);

                            if(port_to_sync_write_velocity[_dxl->port_name] != NULL)
                                port_to_sync_write_velocity[_dxl->port_name]->RemoveParam(_dxl->id);
                            if(port_to_sync_write_position[_dxl->port_name] != NULL)
                                port_to_sync_write_position[_dxl->port_name]->RemoveParam(_dxl->id);
                        }
                    }
                }

                break;
            }
        }
    }

    for(std::list<MotionModule *>::iterator _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
    {
        // set all modules -> disable
        (*_m_it)->SetModuleEnable(false);

        // set all used modules -> enable
        for(std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.begin(); _d_it != robot->dxls.end(); _d_it++)
        {
            if(_d_it->second->ctrl_module_name == (*_m_it)->GetModuleName())
            {
                (*_m_it)->SetModuleEnable(true);
                break;
            }
        }
    }

    // TODO: set indirect address
    // -> check module's control_mode

    queue_mutex_.unlock();

    // publish current module
    robotis_controller_msgs::JointCtrlModule _current_module_msg;
    for(std::map<std::string, Dynamixel *>::iterator _dxl_iter = robot->dxls.begin(); _dxl_iter  != robot->dxls.end(); ++_dxl_iter)
    {
        _current_module_msg.joint_name.push_back(_dxl_iter->first);
        _current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name);
    }

    if(_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
        current_module_pub.publish(_current_module_msg);
}

void RobotisController::GazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    queue_mutex_.lock();

    for(unsigned int _i = 0; _i < msg->name.size(); _i++)
    {
        std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.find((std::string)msg->name[_i]);
        if(_d_it != robot->dxls.end())
        {
            _d_it->second->dxl_state->present_position  = msg->position[_i];
            _d_it->second->dxl_state->present_velocity  = msg->velocity[_i];
            _d_it->second->dxl_state->present_current   = msg->effort[_i];
        }
    }

    if(init_pose_loaded_ == false)
    {
        for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
            _it->second->dxl_state->goal_position = _it->second->dxl_state->present_position;
        init_pose_loaded_ = true;
    }

    queue_mutex_.unlock();
}

bool RobotisController::CheckTimerStop()
{
    if(this->is_timer_running_)
    {
        if(DEBUG_PRINT == true)
            ROS_WARN("Process Timer is running.. STOP the timer first.");
        return false;
    }
    return true;
}

int RobotisController::Ping(const std::string joint_name, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Ping(_port_handler, _dxl->id, error);
}
int RobotisController::Ping(const std::string joint_name, UINT16_T* model_number, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Ping(_port_handler, _dxl->id, model_number, error);
}

int RobotisController::Action(const std::string joint_name)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Action(_port_handler, _dxl->id);
}
int RobotisController::Reboot(const std::string joint_name, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Reboot(_port_handler, _dxl->id, error);
}
int RobotisController::FactoryReset(const std::string joint_name, UINT8_T option, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->FactoryReset(_port_handler, _dxl->id, option, error);
}

int RobotisController::Read(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->ReadTxRx(_port_handler, _dxl->id, address, length, data, error);
}

int RobotisController::ReadCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    ControlTableItem*_item          = _dxl->ctrl_table[item_name];
    if(_item == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    int _result = COMM_NOT_AVAILABLE;
    switch(_item->data_length)
    {
    case 1:
    {
        UINT8_T _data = 0;
        _result = _pkt_handler->Read1ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
        if(_result == COMM_SUCCESS)
            *data = _data;
        break;
    }
    case 2:
    {
        UINT16_T _data = 0;
        _result = _pkt_handler->Read2ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
        if(_result == COMM_SUCCESS)
            *data = _data;
        break;
    }
    case 4:
    {
        UINT32_T _data = 0;
        _result = _pkt_handler->Read4ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
        if(_result == COMM_SUCCESS)
            *data = _data;
        break;
    }
    default:
        break;
    }
    return _result;
}

int RobotisController::Read1Byte(const std::string joint_name, UINT16_T address, UINT8_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Read1ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Read2Byte(const std::string joint_name, UINT16_T address, UINT16_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Read2ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Read4Byte(const std::string joint_name, UINT16_T address, UINT32_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Read4ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->WriteTxRx(_port_handler, _dxl->id, address, length, data, error);
}

int RobotisController::WriteCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    ControlTableItem*_item          = _dxl->ctrl_table[item_name];
    if(_item == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    int _result = COMM_NOT_AVAILABLE;
    UINT8_T *_data = new UINT8_T[_item->data_length];
    if(_item->data_length == 1)
    {
        _data[0] = (UINT8_T)data;
        _result = _pkt_handler->Write1ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    else if(_item->data_length == 2)
    {
        _data[0] = DXL_LOBYTE((UINT16_T)data);
        _data[1] = DXL_HIBYTE((UINT16_T)data);
        _result = _pkt_handler->Write2ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    else if(_item->data_length == 4)
    {
        _data[0] = DXL_LOBYTE(DXL_LOWORD((UINT32_T)data));
        _data[1] = DXL_HIBYTE(DXL_LOWORD((UINT32_T)data));
        _data[2] = DXL_LOBYTE(DXL_HIWORD((UINT32_T)data));
        _data[3] = DXL_HIBYTE(DXL_HIWORD((UINT32_T)data));
        _result = _pkt_handler->Write4ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    delete[] _data;
    return _result;
}

int RobotisController::Write1Byte(const std::string joint_name, UINT16_T address, UINT8_T data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Write1ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write2Byte(const std::string joint_name, UINT16_T address, UINT16_T data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Write2ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write4Byte(const std::string joint_name, UINT16_T address, UINT32_T data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->Write4ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::RegWrite(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    return _pkt_handler->RegWriteTxRx(_port_handler, _dxl->id, address, length, data, error);
}


#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

namespace yhs_tool
{

  CanControl::CanControl()
  {
    ros::NodeHandle private_node("~");

    std::string ultrasonic_numbers_str;
    private_node.getParam("ultrasonic_number", ultrasonic_numbers_str);

    std::istringstream iss(ultrasonic_numbers_str);
    int number;
    while (iss >> number)
    {
      ultrasonic_number_.push_back(number);
    }
  }

  CanControl::~CanControl()
  {
  }

  // io控制回调函数
  void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
  {
    static unsigned char count_1 = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    memset(sendData_u_io_, 0, 8);

    sendData_u_io_[0] = 0xff;
    if (msg.io_cmd_lamp_ctrl)
      sendData_u_io_[0] &= 0xff;
    else
      sendData_u_io_[0] &= 0xfe;
    if (msg.io_cmd_unlock)
      sendData_u_io_[0] &= 0xff;
    else
      sendData_u_io_[0] &= 0xfd;

    sendData_u_io_[1] = 0xff;
    if (msg.io_cmd_lower_beam_headlamp)
      sendData_u_io_[1] &= 0xff;
    else
      sendData_u_io_[1] &= 0xfe;
    if (msg.io_cmd_upper_beam_headlamp)
      sendData_u_io_[1] &= 0xff;
    else
      sendData_u_io_[1] &= 0xfd;

    if (msg.io_cmd_turn_lamp == 0)
      sendData_u_io_[1] &= 0xf3;
    if (msg.io_cmd_turn_lamp == 1)
      sendData_u_io_[1] &= 0xf7;
    if (msg.io_cmd_turn_lamp == 2)
      sendData_u_io_[1] &= 0xfb;

    if (msg.io_cmd_braking_lamp)
      sendData_u_io_[1] &= 0xff;
    else
      sendData_u_io_[1] &= 0xef;
    if (msg.io_cmd_clearance_lamp)
      sendData_u_io_[1] &= 0xff;
    else
      sendData_u_io_[1] &= 0xdf;
    if (msg.io_cmd_fog_lamp)
      sendData_u_io_[1] &= 0xff;
    else
      sendData_u_io_[1] &= 0xbf;

    sendData_u_io_[2] = msg.io_cmd_speaker;

    count_1++;
    if (count_1 == 16)
      count_1 = 0;

    sendData_u_io_[6] = count_1 << 4;

    sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

    send_frames_.can_id = 0x98C4D7D0;
    send_frames_.can_dlc = 8;

    memcpy(send_frames_.data, sendData_u_io_, 8);

    int ret = write(dev_handler_, &send_frames_, sizeof(send_frames_));
    if (ret <= 0)
    {
      ROS_ERROR("send message failed, error code: %d", ret);
    }
  }

  void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
    const unsigned char gear = msg.ctrl_cmd_gear;
    const short ctrl_cmd_x_linear = msg.ctrl_cmd_x_linear * 1000;
    const short ctrl_cmd_z_angular = msg.ctrl_cmd_z_angular * 100;
    const short ctrl_cmd_y_linear = msg.ctrl_cmd_y_linear * 1000;
    static unsigned char count = 0;

    // 打印接收到的速度信息
    ROS_INFO("Received speed control command:");
    ROS_INFO("  Gear: %d", gear);
    ROS_INFO("  Linear X: %.3f m/s", msg.ctrl_cmd_x_linear);
    ROS_INFO("  Linear Y: %.3f m/s", msg.ctrl_cmd_y_linear);
    ROS_INFO("  Angular Z: %.3f rad/s", msg.ctrl_cmd_z_angular);

    std::lock_guard<std::mutex> lock(mutex_);

    memset(sendData_u_vel_, 0, 8);

    sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & gear);

    sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((ctrl_cmd_x_linear & 0x0f) << 4));

    sendData_u_vel_[1] = (ctrl_cmd_x_linear >> 4) & 0xff;

    sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (ctrl_cmd_x_linear >> 12));

    sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((ctrl_cmd_z_angular & 0x0f) << 4));

    sendData_u_vel_[3] = (ctrl_cmd_z_angular >> 4) & 0xff;

    sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (ctrl_cmd_z_angular >> 12));

    sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((ctrl_cmd_y_linear & 0x0f) << 4));

    sendData_u_vel_[5] = (ctrl_cmd_y_linear >> 4) & 0xff;

    sendData_u_vel_[6] = sendData_u_vel_[6] | (0x0f & (ctrl_cmd_y_linear >> 12));

    count++;

    if (count == 16)
        count = 0;

    sendData_u_vel_[6] = sendData_u_vel_[6] | (count << 4);

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_.can_id = 0x98C4D1D0;
    send_frames_.can_dlc = 8;

    memcpy(send_frames_.data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_, sizeof(send_frames_));
    if (ret <= 0)
    {
        ROS_ERROR("send message failed, error code: %d", ret);
    }
}

  void CanControl::steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg)
  {
    const unsigned char gear = msg.ctrl_cmd_gear;
    const short steering_ctrl_cmd_velocity = msg.steering_ctrl_cmd_velocity * 1000;
    const short steering_ctrl_cmd_steering = msg.steering_ctrl_cmd_steering * 100;
    static unsigned char count_2 = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    unsigned char sendData_u_tem[8] = {0};

    sendData_u_tem[0] = sendData_u_tem[0] | (0x0f & gear);

    sendData_u_tem[0] = sendData_u_tem[0] | (0xf0 & ((steering_ctrl_cmd_velocity & 0x0f) << 4));

    sendData_u_tem[1] = (steering_ctrl_cmd_velocity >> 4) & 0xff;

    sendData_u_tem[2] = sendData_u_tem[2] | (0x0f & (steering_ctrl_cmd_velocity >> 12));

    sendData_u_tem[2] = sendData_u_tem[2] | (0xf0 & ((steering_ctrl_cmd_steering & 0x0f) << 4));

    sendData_u_tem[3] = (steering_ctrl_cmd_steering >> 4) & 0xff;

    sendData_u_tem[4] = sendData_u_tem[4] | (0x0f & (steering_ctrl_cmd_steering >> 12));

    count_2++;

    if (count_2 == 16)
      count_2 = 0;

    sendData_u_tem[6] = count_2 << 4;

    sendData_u_tem[7] = sendData_u_tem[0] ^ sendData_u_tem[1] ^ sendData_u_tem[2] ^ sendData_u_tem[3] ^ sendData_u_tem[4] ^ sendData_u_tem[5] ^ sendData_u_tem[6];

    send_frames_.can_id = 0x98C4D2D0;
    send_frames_.can_dlc = 8;

    memcpy(send_frames_.data, sendData_u_tem, 8);

    int ret = write(dev_handler_, &send_frames_, sizeof(send_frames_));
    if (ret <= 0)
    {
      ROS_ERROR("send message failed, error code: %d", ret);
    }
  }

  // 数据接收解析线程
  void CanControl::recvData()
  {

    while (ros::ok())
    {

      if (read(dev_handler_, &recv_frames_, sizeof(recv_frames_)) >= 0)
      {
        switch (recv_frames_.can_id)
        {
        //
        case 0x98C4D1EF:
        {
          yhs_can_msgs::ctrl_fb msg;
          msg.ctrl_fb_gear = 0x0f & recv_frames_.data[0];

          msg.ctrl_fb_x_linear = (float)((short)((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;

          msg.ctrl_fb_z_angular = (float)((short)((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          msg.ctrl_fb_y_linear = (float)((short)((recv_frames_.data[6] & 0x0f) << 12 | recv_frames_.data[5] << 4 | (recv_frames_.data[4] & 0xf0) >> 4)) / 1000;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            ctrl_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4D2EF:
        {
          yhs_can_msgs::steering_ctrl_fb msg;
          msg.steering_ctrl_fb_gear = 0x0f & recv_frames_.data[0];

          msg.steering_ctrl_fb_velocity = (float)((short)((recv_frames_.data[2] & 0x0f) << 12 | recv_frames_.data[1] << 4 | (recv_frames_.data[0] & 0xf0) >> 4)) / 1000;

          msg.steering_ctrl_fb_steering = (float)((short)((recv_frames_.data[4] & 0x0f) << 12 | recv_frames_.data[3] << 4 | (recv_frames_.data[2] & 0xf0) >> 4)) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            steering_ctrl_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4D6EF:
        {
          yhs_can_msgs::lf_wheel_fb msg;
          msg.lf_wheel_fb_velocity = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000;

          msg.lf_wheel_fb_pulse = (int)(recv_frames_.data[5] << 24 | recv_frames_.data[4] << 16 | recv_frames_.data[3] << 8 | recv_frames_.data[2]);

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            lf_wheel_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4D7EF:
        {
          yhs_can_msgs::lr_wheel_fb msg;
          msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000;

          msg.lr_wheel_fb_pulse = (int)(recv_frames_.data[5] << 24 | recv_frames_.data[4] << 16 | recv_frames_.data[3] << 8 | recv_frames_.data[2]);

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            lr_wheel_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4D8EF:
        {
          yhs_can_msgs::rr_wheel_fb msg;
          msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000;

          msg.rr_wheel_fb_pulse = (int)(recv_frames_.data[5] << 24 | recv_frames_.data[4] << 16 | recv_frames_.data[3] << 8 | recv_frames_.data[2]);

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            rr_wheel_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4D9EF:
        {
          yhs_can_msgs::rf_wheel_fb msg;
          msg.rf_wheel_fb_velocity = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000;

          msg.rf_wheel_fb_pulse = (int)(recv_frames_.data[5] << 24 | recv_frames_.data[4] << 16 | recv_frames_.data[3] << 8 | recv_frames_.data[2]);

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            rf_wheel_fb_pub_.publish(msg);
          }

          break;
        }

        // io反馈
        case 0x98C4DAEF:
        {
          yhs_can_msgs::io_fb msg;
          if (0x01 & recv_frames_.data[0])
            msg.io_fb_lamp_ctrl = true;
          else
            msg.io_fb_lamp_ctrl = false;

          if (0x02 & recv_frames_.data[1])
            msg.io_fb_unlock = true;
          else
            msg.io_fb_unlock = false;

          if (0x01 & recv_frames_.data[1])
            msg.io_fb_lower_beam_headlamp = true;
          else
            msg.io_fb_lower_beam_headlamp = false;

          if (0x02 & recv_frames_.data[1])
            msg.io_fb_upper_beam_headlamp = true;
          else
            msg.io_fb_upper_beam_headlamp = false;

          msg.io_fb_turn_lamp = (0x0c & recv_frames_.data[1]) >> 2;

          if (0x10 & recv_frames_.data[1])
            msg.io_fb_braking_lamp = true;
          else
            msg.io_fb_braking_lamp = false;

          if (0x20 & recv_frames_.data[1])
            msg.io_fb_clearance_lamp = true;
          else
            msg.io_fb_clearance_lamp = false;

          if (0x40 & recv_frames_.data[1])
            msg.io_fb_fog_lamp = true;
          else
            msg.io_fb_fog_lamp = false;

          if (0x01 & recv_frames_.data[2])
            msg.io_fb_speaker = true;
          else
            msg.io_fb_speaker = false;

          if (0x01 & recv_frames_.data[3])
            msg.io_fb_fl_impact_sensor = true;
          else
            msg.io_fb_fl_impact_sensor = false;

          if (0x02 & recv_frames_.data[3])
            msg.io_fb_fm_impact_sensor = true;
          else
            msg.io_fb_fm_impact_sensor = false;

          if (0x04 & recv_frames_.data[3])
            msg.io_fb_fr_impact_sensor = true;
          else
            msg.io_fb_fr_impact_sensor = false;

          if (0x08 & recv_frames_.data[3])
            msg.io_fb_rl_impact_sensor = true;
          else
            msg.io_fb_rl_impact_sensor = false;

          if (0x10 & recv_frames_.data[3])
            msg.io_fb_rm_impact_sensor = true;
          else
            msg.io_fb_rm_impact_sensor = false;

          if (0x20 & recv_frames_.data[3])
            msg.io_fb_rr_impact_sensor = true;
          else
            msg.io_fb_rr_impact_sensor = false;

          if (0x01 & recv_frames_.data[4])
            msg.io_fb_fl_drop_sensor = true;
          else
            msg.io_fb_fl_drop_sensor = false;

          if (0x02 & recv_frames_.data[4])
            msg.io_fb_fm_drop_sensor = true;
          else
            msg.io_fb_fm_drop_sensor = false;

          if (0x04 & recv_frames_.data[4])
            msg.io_fb_fr_drop_sensor = true;
          else
            msg.io_fb_fr_drop_sensor = false;

          if (0x08 & recv_frames_.data[4])
            msg.io_fb_rl_drop_sensor = true;
          else
            msg.io_fb_rl_drop_sensor = false;

          if (0x10 & recv_frames_.data[4])
            msg.io_fb_rm_drop_sensor = true;
          else
            msg.io_fb_rm_drop_sensor = false;

          if (0x20 & recv_frames_.data[4])
            msg.io_fb_rr_drop_sensor = true;
          else
            msg.io_fb_rr_drop_sensor = false;

          if (0x01 & recv_frames_.data[5])
            msg.io_fb_estop = true;
          else
            msg.io_fb_estop = false;

          if (0x02 & recv_frames_.data[5])
            msg.io_fb_joypad_ctrl = true;
          else
            msg.io_fb_joypad_ctrl = false;

          if (0x04 & recv_frames_.data[5])
            msg.io_fb_charge_state = true;
          else
            msg.io_fb_charge_state = false;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            io_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4DCEF:
        {
          yhs_can_msgs::front_angle_fb msg;
          msg.front_angle_fb_l = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.front_angle_fb_r = (float)((short)(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            front_angle_fb_pub_.publish(msg);
          }

          break;
        }

        //
        case 0x98C4DDEF:
        {
          yhs_can_msgs::rear_angle_fb msg;
          msg.rear_angle_fb_l = (float)((short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.rear_angle_fb_r = (float)((short)(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            rear_angle_fb_pub_.publish(msg);
          }

          break;
        }

        // bms反馈
        case 0x98C4E1EF:
        {
          yhs_can_msgs::bms_fb msg;
          msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 100;

          msg.bms_fb_current = (float)((short)(recv_frames_.data[3] << 8 | recv_frames_.data[2])) / 100;

          msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_.data[5] << 8 | recv_frames_.data[4])) / 100;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            bms_fb_pub_.publish(msg);
          }

          break;
        }

        // bms_flag反馈
        case 0x98C4E2EF:
        {
          yhs_can_msgs::bms_flag_fb msg;
          msg.bms_flag_fb_soc = recv_frames_.data[0];

          if (0x01 & recv_frames_.data[1])
            msg.bms_flag_fb_single_ov = true;
          else
            msg.bms_flag_fb_single_ov = false;

          if (0x02 & recv_frames_.data[1])
            msg.bms_flag_fb_single_uv = true;
          else
            msg.bms_flag_fb_single_uv = false;

          if (0x04 & recv_frames_.data[1])
            msg.bms_flag_fb_ov = true;
          else
            msg.bms_flag_fb_ov = false;

          if (0x08 & recv_frames_.data[1])
            msg.bms_flag_fb_uv = true;
          else
            msg.bms_flag_fb_uv = false;

          if (0x10 & recv_frames_.data[1])
            msg.bms_flag_fb_charge_ot = true;
          else
            msg.bms_flag_fb_charge_ot = false;

          if (0x20 & recv_frames_.data[1])
            msg.bms_flag_fb_charge_ut = true;
          else
            msg.bms_flag_fb_charge_ut = false;

          if (0x40 & recv_frames_.data[1])
            msg.bms_flag_fb_discharge_ot = true;
          else
            msg.bms_flag_fb_discharge_ot = false;

          if (0x80 & recv_frames_.data[1])
            msg.bms_flag_fb_discharge_ut = true;
          else
            msg.bms_flag_fb_discharge_ut = false;

          if (0x01 & recv_frames_.data[2])
            msg.bms_flag_fb_charge_oc = true;
          else
            msg.bms_flag_fb_charge_oc = false;

          if (0x02 & recv_frames_.data[2])
            msg.bms_flag_fb_discharge_oc = true;
          else
            msg.bms_flag_fb_discharge_oc = false;

          if (0x04 & recv_frames_.data[2])
            msg.bms_flag_fb_short = true;
          else
            msg.bms_flag_fb_short = false;

          if (0x08 & recv_frames_.data[2])
            msg.bms_flag_fb_ic_error = true;
          else
            msg.bms_flag_fb_ic_error = false;

          if (0x10 & recv_frames_.data[2])
            msg.bms_flag_fb_lock_mos = true;
          else
            msg.bms_flag_fb_lock_mos = false;

          if (0x20 & recv_frames_.data[2])
            msg.bms_flag_fb_charge_flag = true;
          else
            msg.bms_flag_fb_charge_flag = false;

          msg.bms_flag_fb_hight_temperature = (float)((short)(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4)) / 10;

          msg.bms_flag_fb_low_temperature = (float)((short)((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5])) / 10;

          unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

          if (crc == recv_frames_.data[7])
          {

            bms_flag_fb_pub_.publish(msg);
          }

          break;
        }

          // ultrasonic
          static unsigned short ultra_data[8] = {0};
        case 0x98C4E8EF:
        {
          ultra_data[0] = (unsigned short)((recv_frames_.data[1] & 0x0f) << 8 | recv_frames_.data[0]);
          ultra_data[1] = (unsigned short)(recv_frames_.data[2] << 4 | ((recv_frames_.data[1] & 0xf0) >> 4));

          ultra_data[2] = (unsigned short)((recv_frames_.data[4] & 0x0f) << 8 | recv_frames_.data[3]);
          ultra_data[3] = (unsigned short)(recv_frames_.data[5] << 4 | ((recv_frames_.data[4] & 0xf0) >> 4));
          break;
        }

        case 0x98C4E9EF:
        {
          ultra_data[4] = (unsigned short)((recv_frames_.data[1] & 0x0f) << 8 | recv_frames_.data[0]);
          ultra_data[5] = (unsigned short)(recv_frames_.data[2] << 4 | ((recv_frames_.data[1] & 0xf0) >> 4));

          ultra_data[6] = (unsigned short)((recv_frames_.data[4] & 0x0f) << 8 | recv_frames_.data[3]);
          ultra_data[7] = (unsigned short)(recv_frames_.data[5] << 4 | ((recv_frames_.data[4] & 0xf0) >> 4));

          yhs_can_msgs::ultrasonic ultra_msg;

          ultra_msg.front_right = ultra_data[ultrasonic_number_[0]];
          ultra_msg.front_left = ultra_data[ultrasonic_number_[1]];
          ultra_msg.left_front = ultra_data[ultrasonic_number_[2]];
          ultra_msg.left_rear = ultra_data[ultrasonic_number_[3]];

          ultra_msg.rear_left = ultra_data[ultrasonic_number_[4]];
          ultra_msg.rear_right = ultra_data[ultrasonic_number_[5]];
          ultra_msg.right_rear = ultra_data[ultrasonic_number_[6]];
          ultra_msg.right_front = ultra_data[ultrasonic_number_[7]];

          ultrasonic_pub_.publish(ultra_msg);
        }

        default:
          break;
        }
      }
    }
  }

  // 数据发送线程
  void CanControl::sendData()
  {
    ros::Rate loop(100);

    while (ros::ok())
    {
      loop.sleep();
    }
  }

  void CanControl::run()
  {
    ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
    steering_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::steering_ctrl_cmd>("steering_ctrl_cmd", 5, &CanControl::steering_ctrl_cmdCallBack, this);

    ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5);
    io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb", 5);
    lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb", 5);
    lf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lf_wheel_fb>("lf_wheel_fb", 5);
    rf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rf_wheel_fb>("rf_wheel_fb", 5);
    rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb", 5);
    bms_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_fb>("bms_fb", 5);
    bms_flag_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_fb>("bms_flag_fb", 5);
    steering_ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::steering_ctrl_fb>("steering_ctrl_fb", 5);
    front_angle_fb_pub_ = nh_.advertise<yhs_can_msgs::front_angle_fb>("front_angle_fb", 5);
    rear_angle_fb_pub_ = nh_.advertise<yhs_can_msgs::rear_angle_fb>("rear_angle_fb", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic", 5);

    // 打开设备
    dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (dev_handler_ < 0)
    {
      ROS_ERROR(">>open can deivce error!");
      return;
    }
    else
    {
      ROS_INFO(">>open can deivce success!");
    }

    struct ifreq ifr;

    std::string can_name("can0");

    strcpy(ifr.ifr_name, can_name.c_str());

    ioctl(dev_handler_, SIOCGIFINDEX, &ifr);

    // bind socket to network interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (ret < 0)
    {
      ROS_ERROR(">>bind dev_handler error!\r\n");
      return;
    }

    // 创建接收发送数据线程
    boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
    //	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

    ros::spin();

    close(dev_handler_);
  }

}

// 主函数
int main(int argc, char **argv)
{
  ros::init(argc, argv, "yhs_can_control_node");

  yhs_tool::CanControl cancontrol;
  cancontrol.run();

  return 0;
}

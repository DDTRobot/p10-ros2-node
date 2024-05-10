/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/28/20.
//
#include "can_control_actuator/common/can_interface/can_bus.h"
#include "can_control_actuator/common/tools/math_utilities.h"

#include <string>

namespace can_interface {
#define OUTPUT_POSITION_0   0

    static int16_t motor_actual_pos=0;
    static int16_t output_actual_pos=0;
    static int16_t round = 0;


    CanBus::CanBus(const std::string &bus_name, CanDataPtr data_ptr,
                   int thread_priority)
            : bus_name_(bus_name), data_ptr_(data_ptr),
              logger_(rclcpp::get_logger("CanBus")) {
        // Initialize device at can_device, false for no loop back.
        while (!socket_can_.open(
                bus_name,
                std::bind(&CanBus::frameCallback, this, std::placeholders::_1),
                thread_priority) &&
               rclcpp::ok()) {
        }
        logger_ = rclcpp::get_logger("CanBus");

        RCLCPP_INFO(logger_, "Successfully connected to %s.", bus_name.c_str());
        // Set up CAN package header

        ddt_frame0_.can_id = 0x032;
        ddt_frame0_.can_dlc = 8;
        ddt_frame1_.can_id = 0x033;
        ddt_frame1_.can_dlc = 8;
    }

    void CanBus::start() {
        for (auto &item: *data_ptr_.id2act_data_) {
            if (item.second.type.find("ddt") != std::string::npos) {
                can_frame mode_set_frame{}, zero_point_set_frame{}, enable_frame{}, dynamics_offset{};
                mode_set_frame.data[0] = 0x01;
                mode_set_frame.data[1] = 0x1C;
                mode_set_frame.data[2] = 0x04;
                mode_set_frame.data[3] = 0x00;
                mode_set_frame.data[4] = 0x00;
                mode_set_frame.data[5] = 0x00;
                mode_set_frame.data[6] = 0xFF;
                mode_set_frame.data[7] = 0xFF;
                mode_set_frame.can_id = 0x036;
                mode_set_frame.can_dlc = 8;
                socket_can_.write(&mode_set_frame);


//                for (int i = 0; i < 8; i++) {
//                    zero_point_set_frame.data[i] = 0x00; // Once enable all ddt motor
//                }
//                zero_point_set_frame.data[1] = 0x01;
//                zero_point_set_frame.can_id = 0x039;
//                zero_point_set_frame.can_dlc = 8;
//                socket_can_.write(&zero_point_set_frame);




                for (int i = 0; i < 8; i++) {
                    enable_frame.data[i] = 0x02; // Once enable all ddt motor
                }
                enable_frame.can_id = 0x038;
                enable_frame.can_dlc = 8;
                socket_can_.write(&enable_frame);
                // TODO : Add the callback info check 0xA0+id's info

//                for (int i = 0; i < 8; i++) {
//                    dynamics_offset.data[i] = 0x00; // Once enable all ddt motor
//                }
//                dynamics_offset.data[0] = 0x0D;
//                dynamics_offset.can_id = 0x038;
//                dynamics_offset.can_dlc = 8;
//                socket_can_.write(&dynamics_offset);
            }
        }
    }



    void CanBus::close() {
        for (auto &item: *data_ptr_.id2act_data_) {
            if (item.second.type.find("ddt") != std::string::npos) {
                can_frame frame{};
                for (int i = 0; i < 8; i++) {
                    frame.data[i] = 0x01; // Once disable all ddt motor
                }
                frame.can_id = 0x038;

                //RCLCPP_INFO(logger_, std::to_string(motor_actual_pos));
                frame.can_dlc = 8;
                socket_can_.write(&frame);
            }
        }
    }

    void CanBus::test() {
        for (auto &item: *data_ptr_.id2act_data_) {
            if (item.second.type.find("dm") != std::string::npos) {
                //       test effort
                //            item.second.exe_cmd = 1;

                //       test pos
                //            item.second.cmd_pos = 0.;
                //            item.second.cmd_kp = 1.;

                //       test vel
                float vel = 3;
                item.second.cmd_vel = vel;
                item.second.cmd_kd = 1;
            }
            if (item.second.type.find("ddt") != std::string::npos) {
                float vel = 3;
                item.second.exe_cmd = 0;
            }
        }
        write();
    }

    void CanBus::write() {
        bool ddt_has_write_frame0 = false, ddt_has_write_frame1 = false;
        static double zero_point_round=0;
        static uint8_t init_flag=2;
        static int16_t last_motor_actual_pos=0;
        static int16_t last_output_actual_pos=0;
        static int16_t zero_round=0;


        int back_zero_id = 0x51 - 0x050;//default id is 01
        int16_t back_zero_cmd;

        uint16_t round_back_zero,motor_pulse_back_zero=0;
        can_frame check_motor_position{};
        can_frame set_zero_point_frame{};
        //RCLCPP_INFO(logger_, "motor "+std::to_string(motor_actual_pos)+"output "+std::to_string(output_actual_pos));

        switch (init_flag) {
            case 0:
                close();
                set_zero_point_frame.data[0]=0;
                set_zero_point_frame.data[1]=1;
                set_zero_point_frame.can_id = 0x039;
                set_zero_point_frame.can_dlc = 8;
                socket_can_.write(&set_zero_point_frame);
                break;


            //common cmd while motor working
            case 1:
                std::fill(std::begin(ddt_frame0_.data), std::end(ddt_frame0_.data), 0);
                std::fill(std::begin(ddt_frame1_.data), std::end(ddt_frame1_.data), 0);

                for (auto &item: *data_ptr_.id2act_data_) {
                    if (item.second.type.find("ddt") != std::string::npos) {
                        const ActCoeff &act_coeff =
                                data_ptr_.type2act_coeffs_->find(item.second.type)->second;
                        int id = item.first - 0x050;
                        double cmd;
                        if (item.second.mode == "position")
                        {
                            cmd = minAbs(act_coeff.pos2act * (item.second.exe_cmd+zero_round/1000.0),
                                         act_coeff.max_out); // add max_range to act_data

                        }
                        else if (item.second.mode == "effort")
                            cmd = minAbs(act_coeff.effort2act * item.second.exe_cmd,
                                         act_coeff.max_out); // add max_range to act_data
                        if (-1 < id && id < 4) {
                            ddt_frame0_.data[2 * (id - 1)] =
                                    static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                            ddt_frame0_.data[2 * (id - 1) + 1] = static_cast<uint8_t>(static_cast<int16_t>(cmd));
                            ddt_has_write_frame0 = true;
                        } else if (3 < id && id < 8) {
                            ddt_frame1_.data[2 * (id - 5)] =
                                    static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                            ddt_frame1_.data[2 * (id - 5) + 1] = static_cast<uint8_t>(static_cast<int16_t>(cmd));
                            ddt_has_write_frame1 = true;
                        }
                    }
                }
                if (ddt_has_write_frame0)
                    socket_can_.write(&ddt_frame0_);
                if (ddt_has_write_frame1)
                    socket_can_.write(&ddt_frame1_);
                break;
            //back to zero point(for target)
            case 2:
                //wait for the new data coming
                if(last_motor_actual_pos!=motor_actual_pos)
                {
                    //get the rounds that the motor needs to rotate
                    zero_point_round=(OUTPUT_POSITION_0-output_actual_pos)*10.0/32768.0;
                    //rotate less than 10 rounds
                    if(zero_point_round>10) zero_point_round-=10;
                    if(zero_point_round<-10) zero_point_round+=10;
                    //get inferior arc
                    if(zero_point_round>10/2)
                        zero_point_round-=10;
                    if(zero_point_round<-10/2)
                        zero_point_round+=10;
                    zero_point_round+=(round/100.0);
                    init_flag=3;
                }
                last_output_actual_pos=output_actual_pos;
                last_motor_actual_pos=motor_actual_pos;
                //RCLCPP_INFO(logger_, "zero_point_round "+std::to_string(zero_point_round)+"output "+std::to_string(round));

                break;

                //set CAN frame for the motor
            case 3:
                std::fill(std::begin(ddt_frame0_.data), std::end(ddt_frame0_.data), 0);
                std::fill(std::begin(ddt_frame1_.data), std::end(ddt_frame1_.data), 0);
                back_zero_cmd = (int16_t )minAbs((double)zero_point_round*100.0,1e10); // add max_range to act_data
                ddt_frame0_.data[2 * (back_zero_id - 1)] =static_cast<uint8_t>(back_zero_cmd >> 8u);
                ddt_frame0_.data[2 * (back_zero_id - 1) + 1] = static_cast<uint8_t>(back_zero_cmd);
                ddt_frame0_.can_id=0x32;
                ddt_frame0_.can_dlc=8;
                socket_can_.write(&ddt_frame0_);
                //record the position of motor when the target arrives the zero point
                zero_round=round;
                //RCLCPP_INFO(logger_, "motor "+std::to_string(motor_actual_pos)+"output "+std::to_string(output_actual_pos));
                if(abs(output_actual_pos-OUTPUT_POSITION_0)<60||abs(output_actual_pos-(32768.0-OUTPUT_POSITION_0))<60)
                    init_flag=1;
                break;
        }


        //check the position of motor and target,and the rounds of motor
        for (int i = 0; i < 8; i++) {
            check_motor_position.data[i] = 0x00; // Once enable all ddt motor
        }
        check_motor_position.data[0] = 0x04;
        // pos
        check_motor_position.data[1] = 0x0D;
        check_motor_position.data[2] = 0x0B;
        check_motor_position.can_id = 0x035;
        check_motor_position.can_dlc = 8;
        socket_can_.write(&check_motor_position);
    }

    void CanBus::read(rclcpp::Time time) {
        std::lock_guard<std::mutex> guard(mutex_);
        can_frame check_motor_position{};
        for (const auto &frame_stamp: read_buffer_) {
            can_frame frame = frame_stamp.frame;
            if (frame.can_id == 0x071)
                std::memcpy(check_motor_position.data, frame.data, sizeof(check_motor_position.data));
                motor_actual_pos=(check_motor_position.data[0] << 8u) | check_motor_position.data[1];
                output_actual_pos=(check_motor_position.data[2] << 8u) | check_motor_position.data[3];
                round = (check_motor_position.data[4] << 8u) | check_motor_position.data[5];
        }


        for (const auto &frame_stamp: read_buffer_) {
            can_frame frame = frame_stamp.frame;
            if (data_ptr_.id2act_data_->find(frame.can_id) !=
                data_ptr_.id2act_data_->end()) {
                ActData &act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
                if ((frame_stamp.stamp - act_data.stamp).seconds() < 0.0005)
                    continue;
                const ActCoeff &act_coeff =
                        data_ptr_.type2act_coeffs_->find(act_data.type)->second;


                if (act_data.type.find("ddt") != std::string::npos) {
                    act_data.q_raw = (check_motor_position.data[0] << 8u) | check_motor_position.data[1];
                    act_data.qd_raw = (frame.data[0] << 8u) | frame.data[1];
                    int16_t cur = (frame.data[2] << 8u) | frame.data[3];
                    // Multiple circle
                    if (act_data.seq != 0) // not the first receive
                    {
                        if (act_data.q_raw - act_data.q_last > 16384)
                            act_data.q_circle--;
                        else if (act_data.q_raw - act_data.q_last < -16384)
                            act_data.q_circle++;
                    }
                    int pos_offset_hex = std::stoi(act_coeff.pos_offset_hex, 0, 16);
                    act_data.seq++;
                    act_data.q_last = act_data.q_raw;
                    // Converter raw CAN data to position velocity and effort.
                    double pos_resolution_ratio = 0.1, vel_resolution_ratio = 0.1, cur_resolution_ratio = 0.01;
                    act_data.pos =
                            act_coeff.act2pos * pos_resolution_ratio *
                            static_cast<double>(act_data.q_raw + 32768 * act_data.q_circle - pos_offset_hex) /
                            6.28;
                    act_data.vel = act_coeff.act2vel *
                                   static_cast<double>(act_data.qd_raw) *
                                   vel_resolution_ratio;
                    act_data.effort = act_coeff.act2effort * static_cast<double>(cur) *
                                      cur_resolution_ratio;
                    // Low pass filter
                    //                    rclcpp::Time current = rclcpp::Clock().now();
                    //                    act_data.lp_filter->input(act_data.vel, current);
                    //                    act_data.vel = act_data.lp_filter->output();
                    continue;
                }

            }
        }
        read_buffer_.clear();
    }

    void CanBus::write(can_frame *frame) { socket_can_.write(frame); }

    void CanBus::frameCallback(const can_frame &frame) {
        std::lock_guard<std::mutex> guard(mutex_);
        rclcpp::Time current_time = rclcpp::Clock().now();
        CanFrameStamp can_frame_stamp{.frame = frame, .stamp = current_time};
        read_buffer_.push_back(can_frame_stamp);
    }

} // namespace can_interface

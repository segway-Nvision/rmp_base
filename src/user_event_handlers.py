"""--------------------------------------------------------------------
COPYRIGHT 2013 SEGWAY Inc.

Software License Agreement:

The software supplied herewith by Segway Inc. (the "Company") for its
RMP Robotic Platforms is intended and supplied to you, the Company's
customer, for use solely and exclusively with Segway products. The
software is owned by the Company and/or its supplier, and is protected
under applicable copyright laws.  All rights are reserved. Any use in
violation of the foregoing restrictions may subject the user to criminal
sanctions under applicable laws, as well as to civil liability for the
breach of the terms and conditions of this license. The Company may
immediately terminate this Agreement upon your use of the software with
any products that are not Segway products.

The software was written using Python programming language.  Your use
of the software is therefore subject to the terms and conditions of the
OSI- approved open source license viewable at http://www.python.org/.
You are solely responsible for ensuring your compliance with the Python
open source license.

You shall indemnify, defend and hold the Company harmless from any claims,
demands, liabilities or expenses, including reasonable attorneys fees, incurred
by the Company as a result of any claim or proceeding against the Company
arising out of or based upon:

(i) The combination, operation or use of the software by you with any hardware,
    products, programs or data not supplied or approved in writing by the Company,
    if such claim or proceeding would have been avoided but for such combination,
    operation or use.

(ii) The modification of the software by or on behalf of you

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 \file   user_event_handlers.py

 \brief  This module allows the user to define how to handle events generated
         in rmp_interface.py.

 \Platform: Cross Platform
--------------------------------------------------------------------"""
from system_defines import *
import time, sys, os
from geometry_msgs.msg import TwistStamped

"""
Define some general parameters for the example like various commands
"""
RMP_SET_TRACTOR = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,TRACTOR_REQUEST]
RMP_SET_STANDBY = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,STANDBY_REQUEST]
RMP_SET_BALANCE = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,BALANCE_REQUEST]

RMP_STOP_SONG = [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_NO_SONG,MOTOR_AUDIO_PLAY_NO_SONG]
RMP_SONGS = [[RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_POWER_ON_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_POWER_OFF_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ALARM_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_MODE_UP_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_MODE_DOWN_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_FINAL_SHUTDOWN_SONG],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_CORRECT_ISSUE],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ISSUE_CORRECTED],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_CORRECT_ISSUE_REPEATING],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_BEGINNER_ACK],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_EXPERT_ACK],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_ENTER_FOLLOW],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_TEST_SWEEP],
             [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_SIMULATE_MOTOR_NOISE]]

"""
Velocity limit of RMP
"""

MAX_LINEAR_VEL = 0.6
MAX_ANGULAR_VEL = 3.15

"""
This is the class that defines how to handle events passed up by the RMP class.
These events currently include:	 
RMP_TX_RDY: The RMP class can accept a new command. Commands sent before this event
			will be queued and executed asyncronously. The user should only post new
			commands once this event has been triggered
			
RMP_RSP_DATA_RDY: A response packet is ready for the user.
"""
class RMPEventHandlers:
    def __init__(self, cmd_queue, rsp_queue, in_flags, rmp_thread):
        self.start_time = time.time()
        self.song_playing = False
        self.idx = 0
        self.vel = TwistStamped()
        self.vel.twist.linear.x = 0.0
        self.vel.twist.angular.z = 0.0
        self.cmd_queue = cmd_queue
        self.rsp_queue = rsp_queue
        self.in_flags = in_flags

        """
        This is the dictionary that the outflags get passed to. Each one can be
        redefined to be passed to whatever user function you would like
        """

        self.handle_event = dict({RMP_KILL:shutdown,
                                  RMP_TX_RDY:self.send_motion_cmd,
                                  RMP_RSP_DATA_RDY:self.get_rsp,
                                  RMP_GOTO_STANDBY:self.goto_standby,
                                  RMP_GOTO_TRACTOR:self.goto_tractor,
			                      RMP_GOTO_BALANCE:self.goto_balance})

    def shutdown(self):
        in_flags.put(RMP_KILL) # Kill rmp_thread

    def send_motion_cmd(self):
        RMP_MOTION_CMD = [RMP_MOTION_CMD_ID, self.vel.twist.linear.x, self.vel.twist.angular.z]
        self.cmd_queue.put(RMP_MOTION_CMD)

    def get_rsp(self):
        fb_dict = self.rsp_queue.get()

        my_data = [['operational_time   : ', fb_dict["operational_time"]],
                   ['inertial_x_acc_g   : ', fb_dict["inertial_x_acc_g"]],
                   ['inertial_y_acc_g   : ', fb_dict["inertial_y_acc_g"]],
                   ['inertial_x_rate_rps: ', fb_dict["inertial_x_rate_rps"]],
                   ['inertial_y_rate_rps: ', fb_dict["inertial_y_rate_rps"]],
                   ['inertial_z_rate_rps: ', fb_dict["inertial_z_rate_rps"]],
                   ['pse_pitch_deg      : ', fb_dict["pse_pitch_deg"]],
                   ['pse_roll_deg       : ', fb_dict["pse_roll_deg"]],
                   ['pse_roll_rate_dps  : ', fb_dict["pse_roll_rate_dps"]],
                   ['pse_yaw_rate_dps   : ', fb_dict["pse_yaw_rate_dps"]]]

        temp = ""
        for i in range(0, len(my_data)):
            temp += my_data[i][0] + str(my_data[i][1]) + "\n"

        os.system('cls')
        print temp

    def goto_standby(self):
        self.cmd_queue.put(RMP_SET_STANDBY)

    def goto_tractor(self):
        self.cmd_queue.put(RMP_SET_TRACTOR)

    def goto_balance(self):
        self.cmd_queue.put(RMP_SET_BALANCE)

    def update_vel(self, new_vel):
        if (abs(msg.twist.linear.x) < MAX_LINEAR_VEL): # Update if the new velocity is in bound
            self.vel.twist.linear.x = new_vel.twist.linear.x
        else:
            print "Linear velocity out of limit +-%f" % MAX_LINEAR_VEL
        if (abs(msg.twist.angular.z) < MAX_ANGULAR_VEL):
            self.vel.twist.linear.z = new_vel.twist.angular.z
        else:
            print "Angular velocity out of limit +-%f" % MAX_ANGULAR_VEL

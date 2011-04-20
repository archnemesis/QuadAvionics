/**
 * ---------------------------------------------------------------------
 * QuadAvionics
 * ---------------------------------------------------------------------
 * UAV Flight Control System
 * Copyright (C) 2011 by Robin Gingras
 * ---------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.    If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * File:                QuadAvionics.pde
 * Description: Main setup and control loop.
 * Created:         Tuesday, April 19 2011
 * Author:            Robin Gingras <robin.gingras@me.com>
 */

#include <Servo.h>
#include "mavserial.h"
#include "mavlink.h"

const int target_id = 100;
const int component_id = 200;

//
// loop counting
//
unsigned long fastLoopTimer = 0;
unsigned long fastLoopTimeStamp = 0;
uint8_t fastLoopDelta = 0;

unsigned long mediumLoopTimer = 0;
uint8_t mediumLoopDelta = 0;
byte mediumLoopCounter = 0;

byte oneHzCounter = 0x00;

//
// performance measuring
//
float system_load = 0;

//
// servo outputs for the ESCs
//
const int esc1_servo = 2;
const int esc2_servo = 3;
const int esc3_servo = 7;
const int esc4_servo = 8;

//
// PWM limits for control surfaces (microseconds)
//
#define ROTOR_TILT_MIN 1000
#define ROTOR_TILT_MAX 2000
#define ROTOR_TILT_MID 1500
#define ESC_THROTTLE_MIN 1250   // the highest value possible while motor is not spinning at all
#define ESC_THROTTLE_MAX 1900   // the highest value possible speed the motor will go (set to a safe limit)
#define ESC_THROTTLE_IDLE 1350  // idle speed for the motors (about 5-10% thrust)

//
// Analog input pins
//
#define BATT_1_INPUT 0
#define BATT_2_INPUT 1
#define BATT_3_INPUT 2
#define BATT_4_INPUT 3

#define BATT_THRESHOLD 9.6
#define INPUT_VOLTAGE 5.0
#define VOLTAGE_DIVIDER_RATIO 3.56
#define BATTERY_VOLTAGE(x) (x * (INPUT_VOLTAGE / 1024.0)) * VOLTAGE_DIVIDER_RATIO

const int system_type = MAV_QUADROTOR;
const int autopilot_type = MAV_AUTOPILOT_GENERIC;

static int mav_packet_drops = 0;
static int mav_mode = MAV_MODE_UNINIT;
static int mav_nav_mode = MAV_NAV_GROUNDED;
static int mav_state = MAV_STATE_UNINIT;

struct battery_voltage {
    float batt_1 = BATT_THRESHOLD * 1.05;
    float batt_2 = BATT_THRESHOLD * 1.05;
    float batt_3 = BATT_THRESHOLD * 1.05;
    float batt_4 = BATT_THRESHOLD * 1.05;
};

//
// outputs for main rotor speed controllers
//
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

//
// outputs for main rotor tilt servos
//
Servo tilt1;
Servo tilt2;
Servo tilt3;
Servo tilt4;

void setup()
{
    Serial.begin(9600);
    Serial.println("QuadAvionics v1.0: Starting up...");
    
    esc1.attach(esc1_servo);
    esc2.attach(esc2_servo);
    esc3.attach(esc3_servo);
    esc4.attach(esc4_servo);
    
    //
    // make sure motor throttle is at 0%
    //
    esc1.writeMicroseconds(ESC_THROTTLE_MIN);
    esc2.writeMicroseconds(ESC_THROTTLE_MIN);
    esc3.writeMicroseconds(ESC_THROTTLE_MIN);
    esc4.writeMicroseconds(ESC_THROTTLE_MIN);
    
    //
    // set tilt servos to center (90 degrees)
    //
    tilt1.writeMicroseconds(ROTOR_TILT_MID);
    tilt2.writeMicroseconds(ROTOR_TILT_MID);
    tilt3.writeMicroseconds(ROTOR_TILT_MID);
    tilt4.writeMicroseconds(ROTOR_TILT_MID);
    
    //
    // set up telemetry serial port
    //
    Serial1.begin(9600);
    
}

void loop()
{
    if (millis() - fastLoopTimer > 19) {
        fastLoopDelta = millis() - fastLoopTimer;
        system_load = (float)(fastLoopTimeStamp - fastLoopTimer) / fastLoopDelta;
        fastLoopTimer = millis();
        
        fast_loop();
        
        oneHzCounter++;
        if (oneHzCounter == 50) {
            one_second_loop();
            oneHzCounter = 0;
        }
        
        fastLoopTimeStamp = millis();
    }
}

/**
 * 50Hz Fast Loop
 */
void fast_loop()
{
    //
    // Check for incoming commands or data
    //
    
    mavlink_message_t msg;
    
    if (Serial1.available() > 0) {
        uint8_t c = Serial1.read();
        
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) {
            handle_mavlink(MAVLINK_COMM_0, &msg);
        }
    }
    
    medium_loop();
}

/*
 * 10Hz 1/5th Loop
 */
void medium_loop()
{
    switch (mediumLoopCounter) {
        case 0:
            mediumLoopCounter++;
            
            // TODO: GPS Update
            
            break;
            
        case 1:
            mediumLoopCounter++;
            
            //
            // update battery voltage readings
            //
            battery_voltage.batt_1 = BATTERY_VOLTAGE(analogRead(BATT_1_INPUT));
            battery_voltage.batt_2 = BATTERY_VOLTAGE(analogRead(BATT_2_INPUT));
            battery_voltage.batt_3 = BATTERY_VOLTAGE(analogRead(BATT_3_INPUT));
            battery_voltage.batt_4 = BATTERY_VOLTAGE(analogRead(BATT_4_INPUT));
            
            float sys_voltage = (battery_voltage.batt_1
                + battery_voltage.batt_2
                + battery_voltage.batt_3
                + battery_voltage.batt_4) / 4.0;
            
            mavlink_msg_sys_status_send(MAV_COMM_0,
                mav_mode,
                mav_nav_mode,
                mav_state,
                system_load,
                sys_voltage,
                false,
                mavlink_status.packet_rx_drop_count
            );
            
            break;
            
        case 2:
        case 3:
            break;
            
        case 4:
            mediumLoopCounter = 0;
            break;
    }
}

void one_second_loop()
{
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type);
}

void handle_mavlink(mavlink_channel_t chan, mavlink_message_t *msg)
{
    switch (msg->msgid) {
        case MAV_MSG_ID_ACTION:
            mavlink_action_t action;
            mavlink_msg_action_decode(msg, &action);
            
            switch (action.action) {
                case MAV_ACTION_MOTORS_START:
                    // Put the main rotors into idle mode.
                    esc1.writeMicroseconds(ESC_THROTTLE_IDLE);
                    esc2.writeMicroseconds(ESC_THROTTLE_IDLE);
                    esc3.writeMicroseconds(ESC_THROTTLE_IDLE);
                    esc4.writeMicroseconds(ESC_THROTTLE_IDLE);
                    
                    mavlink_msg_action_ack_send(MAV_COMM_0, action.action, 1);
                    
                    break;
                    
                case MAV_ACTION_MOTORS_STOP:
                    esc1.writeMicroseconds(ESC_THROTTLE_MIN);
                    esc2.writeMicroseconds(ESC_THROTTLE_MIN);
                    esc3.writeMicroseconds(ESC_THROTTLE_MIN);
                    esc4.writeMicroseconds(ESC_THROTTLE_MIN);
                    
                    mavlink_msg_action_ack_send(MAV_COMM_0, action.action, 1);
                    
                    break;
            }
            
            break;
            
        case MAV_MSG_ID_SYS_STATUS:
            
    }
}

#include <Servo.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "Defines.h"
#include "MavSerial.h"
#include <mavlink.h>

struct _system_info_t {
    float load;
    uint16_t    packet_loss;
    uint8_t     mode;
    uint8_t     nav_mode;
    uint8_t     state;
} system_info;

struct _loop_counters_t {
    unsigned long   fast_timer;
    unsigned long   fast_timestamp;
    unsigned long   medium_timer;
    uint8_t         fast_delta;
    uint8_t         medium_delta;
    byte            medium_counter;
    byte            onehz_counter;
} loop_counters;

struct _telemetry_settings_t {
    bool            servo_output_raw;
    bool            sys_status;
} telemetry_settings;

typedef struct _control_output_t {
    uint16_t        esc1;
    uint16_t        esc2;
    uint16_t        esc3;
    uint16_t        esc4;
    uint16_t        tilt1;
    uint16_t        tilt2;
    uint16_t        tilt3;
    uint16_t        tilt4;
} control_output_t;

control_output_t control_output;

//
// outputs for main rotor speed controllers
//
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup()
{
    Serial.begin(115200);
    Serial.println("Beginning setup sequence...");
    
    mavlink_system.sysid            = 100;
    mavlink_system.compid           = 200;
    
    system_info.load                = 0;
    system_info.packet_loss         = 0;
    system_info.mode                = MAV_MODE_UNINIT;
    system_info.nav_mode            = MAV_NAV_GROUNDED;
    system_info.state               = MAV_STATE_BOOT;

    loop_counters.fast_timer        = 0;
    loop_counters.fast_timestamp    = 0;
    loop_counters.medium_timer      = 0;
    loop_counters.fast_delta        = 0;
    loop_counters.medium_delta      = 0;
    loop_counters.medium_counter    = 0;
    loop_counters.onehz_counter     = 0;
    
    control_output.esc1 = ESC_THROTTLE_MIN;
    control_output.esc2 = ESC_THROTTLE_MIN;
    control_output.esc3 = ESC_THROTTLE_MIN;
    control_output.esc4 = ESC_THROTTLE_MIN;

    esc1.attach(ESC_OUTPUT_1);
    esc2.attach(ESC_OUTPUT_2);
    esc3.attach(ESC_OUTPUT_3);
    esc4.attach(ESC_OUTPUT_4);

    esc1.writeMicroseconds(ESC_THROTTLE_MIN);
    esc2.writeMicroseconds(ESC_THROTTLE_MIN);
    esc3.writeMicroseconds(ESC_THROTTLE_MIN);
    esc4.writeMicroseconds(ESC_THROTTLE_MIN);
    
    system_info.state = MAV_STATE_STANDBY;
}

void loop()
{
    if (millis() - loop_counters.fast_timer > 19) {
        loop_counters.fast_delta = (millis() - loop_counters.fast_timer);
        system_info.load = (float)(loop_counters.fast_timestamp - loop_counters.fast_timer) / loop_counters.fast_delta;
        loop_counters.fast_timer = millis();

        fast_loop();
        medium_loop();

        loop_counters.onehz_counter++;
        if (loop_counters.onehz_counter == 50) {
            one_second_loop();
            loop_counters.onehz_counter = 0;
        }

        loop_counters.fast_timestamp = millis();
    }
}

/**
 * This loop runs at about 20Hz.
 * here.
 */
void fast_loop()
{
    //
    // Check for incoming commands or data
    //
    
    static mavlink_message_t msg;
    
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) {
            mavlink_dispatch(msg);
        }
    }
}

/**
 * This loop runs at about 10Hz.
 */
void medium_loop()
{
    switch (loop_counters.medium_counter) {
        case 0:
            loop_counters.medium_counter++;
            
            mavlink_msg_servo_output_raw_send(
                MAVLINK_COMM_0,
                control_output.esc1,
                control_output.esc2,
                control_output.esc3,
                control_output.esc4,
                0,
                0,
                0,
                0
            );
            
            break;
            
        case 1:
            loop_counters.medium_counter = 0;

            control_update();
            
            break;
    }
}

/**
 * This loop runs once per second.
 */
void one_second_loop()
{
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_QUADROTOR, MAV_AUTOPILOT_GENERIC);
    mavlink_msg_sys_status_send(
        MAVLINK_COMM_0, 
        system_info.mode, 
        system_info.nav_mode,
        system_info.state,
        system_info.load,
        12, 100,
        system_info.packet_loss
    );
}

void control_update()
{
    static control_output_t last_output;
    
    esc1.writeMicroseconds(control_output.esc1);
    esc2.writeMicroseconds(control_output.esc2);
    esc3.writeMicroseconds(control_output.esc3);
    esc4.writeMicroseconds(control_output.esc4);

    if (last_output.esc1 != control_output.esc1 ||
        last_output.esc2 != control_output.esc2 ||
        last_output.esc3 != control_output.esc3 ||
        last_output.esc4 != control_output.esc4) 
    {
        // do stuff here with change in servo position
        
        last_output = control_output;
    }

}

/**
 * Handles MavLink messages.
 */
void mavlink_dispatch(mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_SET_MODE:
            mavlink_set_mode_t new_mode;
            mavlink_msg_set_mode_decode(&msg, &new_mode);
            
            if (new_mode.target == mavlink_system.sysid) {
                system_info.mode = new_mode.mode;
            }
            
            break;
            
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
            mavlink_change_operator_control_t change;
            mavlink_msg_change_operator_control_decode(&msg, &change);
            
            break;
        
        case MAVLINK_MSG_ID_COMMAND:
            mavlink_command_t cmd;
            mavlink_msg_command_decode(&msg, &cmd);
          
            switch (cmd.command) {
                case MAV_CMD_DO_SET_MODE:
                {
                    uint8_t new_mode = (uint8_t)cmd.param1;
                  
                    //
                    // prevent accepting an invalid mode
                    //
                    if (new_mode <= MAV_MODE_RC_TRAINING) {
                        system_info.mode = new_mode;
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_MODE, 0);
                    }
                    else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_MODE, 1);
                    }

                    break;
                }
                case MAV_CMD_DO_SET_SERVO:
                {
                    uint8_t servo = cmd.param1;
                    uint16_t pos  = cmd.param2;

                    if (servo > 0 && servo < 9 && pos >= 1000 && pos <= 2000) {
                        switch (servo) {
                            case 1:
                                control_output.esc1 = pos;
                                break;
                            case 2:
                                control_output.esc2 = pos;
                                break;
                            case 3:
                                control_output.esc3 = pos;
                                break;
                            case 4:
                                control_output.esc4 = pos;
                                break;
                        }

                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_SERVO, 0);
                    }
                    else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_SERVO, 1);
                    }

                    break;
                }
          }
            
    }
}

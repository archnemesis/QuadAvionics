#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "Defines.h"
#include "PulseRadio.h"
#include "MavSerial.h"
#include <mavlink.h>

struct _system_info_t {
    float load;
    uint16_t    packet_loss;
    uint8_t     mode;
    uint8_t     nav_mode;
    uint8_t     state;
    
    float       batt1_voltage;
    float       batt2_voltage;
    float       batt3_voltage;
    float       batt4_voltage;
    
    float       batt1_current;
    float       batt2_current;
    float       batt3_current;
    float       batt4_current;
    
    float       batt1_charge;
    float       batt2_charge;
    float       batt3_charge;
    float       batt4_charge;
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

uint16_t rc_input_min[8];
uint16_t rc_input_max[8];
uint16_t rc_input[8];

uint16_t servo_output[8];
int servo_output_trim[8];

//
// outputs for main rotor speed controllers
//
//Servo esc_servo[4];
//Servo tilt_servo[4];

void setup()
{
    Serial.begin(9600);
    Serial1.begin(38400);
    
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
    
    rc_input_min[0] = 1300;
    rc_input_max[0] = 1857;
    rc_input_min[1] = 1157;
    rc_input_max[1] = 1915;
    rc_input_min[2] = 1130;
    rc_input_max[2] = 1890;
    
    servo_output[0] = 1000;
    servo_output[1] = 1000;
    servo_output[2] = 1000;
    servo_output[3] = 1000;
    servo_output[4] = 1500;
    servo_output[5] = 1500;
    servo_output[6] = 1500;
    servo_output[7] = 1500;
    
    servo_output_trim[0] = 0;
    servo_output_trim[1] = 0;
    servo_output_trim[2] = 0;
    servo_output_trim[3] = 0;
    servo_output_trim[4] = 0;
    servo_output_trim[5] = 0;
    servo_output_trim[6] = 0;
    servo_output_trim[7] = 0;
    
    PulseRadio.init();
    
    for (int i = 0; i < 8; i++) {
        PulseRadio.outputCh(i, servo_output[i]);
    }
    
    system_info.state = MAV_STATE_STANDBY;
    system_info.mode = MAV_MODE_LOCKED;
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
    
    while (Serial1.available() > 0) {
        uint8_t c = Serial1.read();
        
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
        {
            loop_counters.medium_counter++;
            
            //
            // Read battery voltage and current draw
            //
            float b1_voltage_raw = (((analogRead(BATT1_VOLTAGE_PIN) / 1024.0f) * 5.0f) / 0.06369f) * 1000.0f;
            float b1_current_raw = (((analogRead(BATT1_CURRENT_PIN) / 1024.0f) * 5.0f) / 0.03660f) * 1000.0f;
            
            system_info.batt1_voltage = int(b1_voltage_raw);
            system_info.batt1_current = int(b1_current_raw);
            system_info.batt1_charge  = map(b1_voltage_raw, 8100, 12690, 0, 100);
            
            break;
        }    
        case 1:
        {
            loop_counters.medium_counter++;
            
            if (PulseRadio.ready()) {
                for (int i = 0; i < 8; i++) {
                    rc_input[i] = PulseRadio.inputCh(i);
                }
            }
            
            break;
        }
        case 2:
        {
            loop_counters.medium_counter = 0;

            control_update();
            
            break;
        }
    }
}

/**
 * This loop runs once per second.
 */
void one_second_loop()
{
    //
    // wait for XBee to warm up before transmitting...
    //

    if (millis() > 5000) {
        mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_QUADROTOR, MAV_AUTOPILOT_GENERIC);
        
        mavlink_msg_sys_status_send(
            MAVLINK_COMM_0, 
            system_info.mode, 
            system_info.nav_mode,
            system_info.state,
            system_info.load,
            system_info.batt1_voltage, 
            system_info.batt1_charge,
            system_info.packet_loss
        );
        
        mavlink_msg_servo_output_raw_send(
            MAVLINK_COMM_0,
            servo_output[0],
            servo_output[1],
            servo_output[2],
            servo_output[3],
            servo_output[4],
            servo_output[5],
            servo_output[6],
            servo_output[7]
        );
        
        mavlink_msg_rc_channels_raw_send(
            MAVLINK_COMM_0,
            rc_input[0],
            rc_input[1],
            rc_input[2],
            rc_input[3],
            rc_input[4],
            rc_input[5],
            rc_input[6],
            rc_input[7],
            0
        );
    }
}

void control_update()
{
    static uint16_t throttle_stab[THROTTLE_RC_FILTER_COUNT];
    static uint8_t throttle_stab_counter = 0;
    
    if (system_info.mode == MAV_MODE_MANUAL) {
        for (int chan = 0; chan < 8; chan++) {
            switch (chan) {
                case 0: // forward/aft thruster tilt
                {
                    uint16_t angle = rc_input[chan];
                    angle = constrain(angle, rc_input_min[chan], rc_input_max[chan]);
                    angle = map(angle, rc_input_min[chan], rc_input_max[chan], 1000, 2000);
                    
                    PulseRadio.outputCh(4, angle);
                    servo_output[4] = angle;
                    
                    angle = map(angle, 1000, 2000, 2000, 1000);
                    PulseRadio.outputCh(6, angle);
                    servo_output[6] = angle;
                    
                    break;
                }
                case 1: // left/right thruster tilt
                {
                    uint16_t angle = rc_input[chan];
                    angle = constrain(angle, rc_input_min[chan], rc_input_max[chan]);
                    angle = map(angle, rc_input_min[chan], rc_input_max[chan], 1000, 2000);
                    
                    PulseRadio.outputCh(5, angle);
                    PulseRadio.outputCh(7, angle);
                    servo_output[5] = angle;
                    servo_output[7] = angle;
                    
                    break;
                }
                case 2: // throttle
                {
                    
                    uint16_t throttle = rc_input[chan];
                    uint16_t throttle_avg;
                    
                    throttle = constrain(throttle, rc_input_min[chan], rc_input_max[chan]);
                    throttle = map(throttle, rc_input_min[chan], rc_input_max[chan], 1000, 2000);
                    
                    for (uint8_t i = 0; i < THROTTLE_RC_FILTER_COUNT; i++) {
                        throttle_avg += throttle_stab[i];
                        if (i < 4) {
                            throttle_stab[i] = throttle_stab[i+1];
                        }
                        else {
                            throttle_stab[i] = throttle;
                        }
                    }
                    
                    throttle = round(throttle_avg / (float)THROTTLE_RC_FILTER_COUNT);
                    
                    for (int esc_chan = 0; esc_chan < 4; esc_chan++) {
                        PulseRadio.outputCh(esc_chan, throttle);
                        servo_output[esc_chan] = throttle;
                    }
                    
                    break;
                }
                case 3: // yaw
                {
                    break;
                }
                case 4: // switch
                {
                    break;
                }
            }
        }
    }
    else if (system_info.mode == MAV_MODE_AUTO || system_info.mode == MAV_MODE_GUIDED) {
        for (int chan = 0; chan < 8; chan++) {
            PulseRadio.outputCh(chan, servo_output[chan]);
        }
    }
}

/**
 * Handles MavLink messages.
 */
void mavlink_dispatch(mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_SET_MODE:
        {
            mavlink_set_mode_t new_mode;
            mavlink_msg_set_mode_decode(&msg, &new_mode);
            
            if (new_mode.target == mavlink_system.sysid) {
                system_info.mode = new_mode.mode;
            }
            
            break;
        }    
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        {
            mavlink_change_operator_control_t change;
            mavlink_msg_change_operator_control_decode(&msg, &change);
            
            break;
        }
        case MAVLINK_MSG_ID_ACTION:
        {
            mavlink_action_t action;
            mavlink_msg_action_decode(&msg, &action);
            
            switch (action.action) {
            
            }
            
            break;
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        {
            mavlink_manual_control_t ctrl;
            mavlink_msg_manual_control_decode(&msg, &ctrl);
            
            uint16_t thrust = (uint16_t)ctrl.thrust;
            
            if (thrust <= 1000) {
                thrust += 1000;
                
                servo_output[0] = thrust;
                servo_output[1] = thrust;
                servo_output[2] = thrust;
                servo_output[3] = thrust;
            }
            
            break;
        }
        case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t pset;
            mavlink_msg_param_set_decode(&msg, &cmd);
            
            switch (pset.param_id[0]) {
                case P_SERVO:
                    switch (pset.param_id[1]) {
                        case K_TRIM_CHANNEL_1:
                            servo_output_trim[0] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_2:
                            servo_output_trim[1] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_3:
                            servo_output_trim[2] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_4:
                            servo_output_trim[3] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_5:
                            servo_output_trim[4] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_6:
                            servo_output_trim[5] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_7:
                            servo_output_trim[6] = (int)pset.param_value;
                            break;
                        case K_TRIM_CHANNEL_8:
                            servo_output_trim[7] = (int)pset.param_value;
                            break;
                    }
                    
                    break;
            }
            
            break;
        }
        case MAVLINK_MSG_ID_COMMAND:
        {
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

                    if (servo >= 0 && servo < 8 && pos >= 1000 && pos <= 2000) {
                        servo_output[servo] = pos;
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_SERVO, 0);
                    }
                    else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_DO_SET_SERVO, 1);
                    }

                    break;
                }
            }
            
            break;
        }           
    }
}

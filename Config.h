#ifndef CONFIG_H
#define CONFIG_H

struct _config_storage_t {
    char version[4];
    
    uint16_t servo_output_min[8];
    uint16_t servo_output_max[8];
    uint16_t servo_output_trim[8];
    
    uint16_t rc_input_min[8];
    uint16_t rc_input_max[8];
    uint16_t rc_input_trim[8];
    
    uint16_t throttle_limit;
    uint16_t tilt_limit;
    
    uint8_t telem_servo_output;
    uint8_t telem_power_status;
    uint8_t telem_system_status;
    uint8_t telem_heartbeat;
    uint8_t telem_rc_input;
    uint8_t telem_gps_output;
    uint8_t telem_accel_output;
    uint8_t telem_gyro_output;
    
    
} config_storage_t;

class Config_Class
{
    public:
        Config_Class();
        void save();
        
}

#endif

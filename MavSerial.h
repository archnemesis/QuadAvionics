#ifndef MAVSERIAL_H
#define MAVSERIAL_H

#include <WProgram.h>
#include <mavlink_types.h>

mavlink_system_t mavlink_system;
mavlink_status_t mavlink_status;

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0) {
    Serial1.write(ch);
  }
  
  if (chan == MAVLINK_COMM_1) {
    Serial2.write(ch);
  }
}

#endif

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include "protocol.h"
#include "can_fc.h"
class CANFeedc0de
{
public:
  void poll();
  void handle_frame(uint16_t id, uint8_t* frame, uint8_t len);
  void send_feedback(const Feedback& in);
  bool get_command(Command& out);
private:
  Command command;
  Feedback feedback;

  FCSender feedc0de_fcs = FCSender(CAN_ID_FEEDBACK_BACK_TO_STW);
  FCReceiver feedc0de_fcr = FCReceiver(CAN_ID_COMMAND_BACK_TO_STW, ((uint8_t *)&command) + 2, sizeof(command) - 4);
};

extern "C"
{
#endif

void can_feedc0de_handle_frame(uint16_t id, uint8_t* frame, uint8_t len);
void can_feedc0de_poll();

#ifdef __cplusplus
}
#endif

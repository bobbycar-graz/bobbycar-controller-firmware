#include <cstdint>
#include <cstddef>
#include <cstring>

#include "protocol.h"
#include "can_fc.h"

#include "can_feedc0de.h"


static_assert((sizeof(Command) + FC_CHUNK_SIZE - 1) / FC_CHUNK_SIZE < 15);

void CANFeedc0de::poll()
{
  if (feedc0de_fcs.tx_pending())
    feedc0de_fcs.tx();

  if (feedc0de_fcr.ack_pending())
    feedc0de_fcr.ack();
}

void CANFeedc0de::send_feedback(const Feedback& in)
{
  feedback = in;
  feedc0de_fcs.reset(((uint8_t*)&feedback) + 2, sizeof(feedback) - 4);
}

bool CANFeedc0de::get_command(Command& out)
{
  if (!feedc0de_fcr.transfer_finished())
    return false;

  out = command;
  feedc0de_fcr.reset(((uint8_t*)&command) + 2, sizeof(command) - 4);

  return true;
}

void CANFeedc0de::handle_frame(uint16_t id, uint8_t* frame, uint8_t len)
{
  switch (id)
  {
  case CAN_ID_FEEDBACK_STW_TO_BACK:
    feedc0de_fcs.handle_frame(frame, len);
    break;
  case CAN_ID_COMMAND_STW_TO_BACK:
    feedc0de_fcr.handle_frame(frame, len);
    break;
  default:
    Error_Handler();
  }
}

extern "C"
{

void can_feedc0de_handle_frame(uint16_t id, uint8_t* frame, uint8_t len)
{
  extern CANFeedc0de can_feedc0de;
  can_feedc0de.handle_frame(id, frame, len);
}

void can_feedc0de_poll()
{
  extern CANFeedc0de can_feedc0de;
  can_feedc0de.poll();
}

}

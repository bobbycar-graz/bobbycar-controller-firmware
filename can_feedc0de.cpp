#include <cstdint>
#include <cstddef>
#include <cstring>

#include "config.h"
#include "protocol.h"
#include "can_fc.h"

#include "can_feedc0de.h"


static_assert((sizeof(Command) + FC_CHUNK_SIZE - 1) / FC_CHUNK_SIZE < 15);

template <class Sent, class Received>
void CANFeedc0de<Sent, Received>::poll()
{
  feedc0de_fcs.tx();
  feedc0de_fcr.ack();
}

template <class Sent, class Received>
void CANFeedc0de<Sent, Received>::send(const Sent& in)
{
  sent = in;
  feedc0de_fcs.reset(((uint8_t*)&sent) + 2, sizeof(sent) - 4);
}

template <class Sent, class Received>
bool CANFeedc0de<Sent, Received>::get(Received& out)
{
  if (!feedc0de_fcr.transfer_finished())
    return false;

  out = received;
  feedc0de_fcr.reset(((uint8_t*)&received) + 2, sizeof(received) - 4);

  return true;
}

template <class Sent, class Received>
bool CANFeedc0de<Sent, Received>::handle_frame(uint16_t id, uint8_t* frame, uint8_t len)
{
  if (id == fcs_rx_can_id)
  {
    feedc0de_fcs.handle_frame(frame, len);
    return true;
  }
  else if (id == fcr_rx_can_id)
  {
    feedc0de_fcr.handle_frame(frame, len);
    return true;
  }

  return false;
}

#ifdef CAN_FEEDCODE_STW
template class CANFeedc0de<Command, Feedback>;

extern "C"
{

void can_feedc0de_handle_frame(uint16_t id, uint8_t* frame, uint8_t len)
{
  extern CANFeedc0de<Command, Feedback> can_instances[NUM_BOARDS_MAX];

  bool handled = false;
  for (size_t i = 0; i < NUM_BOARDS_MAX; i++)
  {
    if (can_instances[i].handle_frame(id, frame, len))
      handled = true;
  }

  if (!handled)
    Error_Handler();
}

}

#else
template class CANFeedc0de<Feedback, Command>;

extern "C"
{

void can_feedc0de_handle_frame(uint16_t id, uint8_t* frame, uint8_t len)
{
  extern CANFeedc0de<Feedback, Command> can_feedc0de;
  if (!can_feedc0de.handle_frame(id, frame, len))
    Error_Handler();
}

void can_feedc0de_poll()
{
  extern CANFeedc0de<Feedback, Command> can_feedc0de;
  can_feedc0de.poll();
}

}
#endif

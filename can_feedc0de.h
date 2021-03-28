#pragma once

#ifdef __cplusplus
#include <cstdint>
#include "protocol.h"
#include "can_fc.h"

template <class Sent, class Received>
class CANFeedc0de
{
public:
  CANFeedc0de(uint16_t fcs_tx_can_id, uint16_t fcs_rx_can_id,
              uint16_t fcr_tx_can_id, uint16_t fcr_rx_can_id)
    : fcs_rx_can_id(fcs_rx_can_id),
      fcr_rx_can_id(fcr_rx_can_id),
      feedc0de_fcs(fcs_tx_can_id),
      feedc0de_fcr(fcr_tx_can_id, (uint8_t *)&received + 2, sizeof(received) - 4)
  {
  };

  void poll();
  bool handle_frame(uint16_t id, uint8_t* frame, uint8_t len);
  void send(const Sent& in);
  bool get(Received& out);

private:
  Sent sent;
  Received received;

  uint16_t fcs_rx_can_id;
  uint16_t fcr_rx_can_id;

  FCSender feedc0de_fcs;
  FCReceiver feedc0de_fcr;
};

extern "C"
{
#endif

void can_feedc0de_handle_frame(uint16_t id, uint8_t* frame, uint8_t len);
void can_feedc0de_poll();

#ifdef __cplusplus
}
#endif

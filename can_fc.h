#pragma once

#ifdef __cplusplus
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>

#include "can.h"

//#define FC_STRICT

#define FC_PROTOCOL_ERROR()
#define Error_Handler() while(1)

constexpr const uint8_t FC_CHUNK_SIZE = 7;
constexpr const uint8_t FC_TAG_INIT = 0x45;
constexpr const uint8_t FC_TAG_END = 0x7d;

static uint8_t next_tag(uint8_t tag)
{
  if (tag == 0)
  {
    return FC_TAG_INIT;
  }
  else
  {
    return tag + 0x04;
  }
}

class FCSender
{
public:
  FCSender(uint16_t id) : id(id)
  {
  }

  bool tx_pending()
  {
    return pos_sent != len && pos_sent == pos_acked;
  }

  bool transfer_in_progress()
  {
    return data != nullptr && pos_acked != len;
  }

  bool transfer_finished()
  {
    return data != nullptr && pos_acked == len;
  }

  void reset(const uint8_t* data, size_t len)
  {
    pos_sent = 0;
    pos_acked = 0;
    tag_sent = 0;

    this->data = data;
    this->len = len;
  }

  void handle_frame(const uint8_t* payload, uint8_t payload_len)
  {
    if (pos_sent == 0)
      return;

    if (payload_len != 1)
      return;

    if (payload[0] != tag_sent)
      return;

    pos_acked = pos_sent;
  }

  void tx()
  {
    uint8_t payload[1 + FC_CHUNK_SIZE];

    // Still waiting for ack, or done transmitting
    if (!tx_pending())
      return;

    size_t new_pos_sent = std::min(pos_sent + FC_CHUNK_SIZE, len);
    uint8_t sent_size = new_pos_sent - pos_sent;
    uint8_t new_tag_sent = next_tag(tag_sent);
    if (new_tag_sent == FC_TAG_END)
      Error_Handler();

    payload[0] = new_tag_sent;
    memcpy(&payload[1], data + pos_sent, sent_size);
    can_tx(id, payload, 1 + sent_size);

    pos_sent = new_pos_sent;
    tag_sent = new_tag_sent;
  }

private:
  uint16_t id;
  const uint8_t* data;
  size_t len;
  size_t pos_sent;
  size_t pos_acked;
  uint8_t tag_sent;
};

class FCReceiver
{
public:
  FCReceiver(uint16_t id, uint8_t* data, size_t len) : id(id)
  {
    reset(data, len);
  }

  bool ack_pending()
  {
    return pos_received != pos_acked;
  }

  bool transfer_in_progress()
  {
    return ready && data != nullptr && pos_acked != len;
  }

  bool transfer_finished()
  {
    return ready && data != nullptr && pos_acked == len;
  }

  void reset(uint8_t* data, size_t len)
  {
    // Poor man's mutex
    ready = false;

    pos_received = 0;
    pos_acked = 0;
    tag_expected = next_tag(0);

    this->data = data;
    this->len = len;

    ready = true;
  }

  void handle_frame(const uint8_t* payload, uint8_t payload_len)
  {
    if (!transfer_in_progress())
      return;

    if (tag_expected == FC_TAG_END)
      return;

    if (payload_len < 1)
      return;

    // Reset receiver if frame with FC_TAG_INIT received when expecting different one
    if (payload[0] == FC_TAG_INIT && tag_expected != FC_TAG_INIT)
      reset(data, len);

    // Don't accept new data if we haven't acked the previous data yet
    if (ack_pending())
      return;

    uint8_t data_len = payload_len - 1;
    if (data_len > len - pos_received)
      return;

    // Ignore all other tags
    if (payload[0] != tag_expected)
      return;

    memcpy(&data[pos_received], &payload[1], data_len);

    pos_received += data_len;
  }

  void ack()
  {
    uint8_t payload[1];

    if (!ack_pending())
      return;

    // Poor man's mutex
    ready = false;

    // ISR may have called reset() in the mean time
    if (!ack_pending())
    {
      ready = true;
      return;
    }

    payload[0] = tag_expected;
    can_tx(id, payload, sizeof(payload));

    pos_acked = pos_received;
    tag_expected = next_tag(tag_expected);

    ready = true;
  }

private:
  bool ready;

  uint16_t id;
  uint8_t* data;
  size_t len;
  size_t pos_received;
  size_t pos_acked;
  uint8_t tag_expected;
};

#endif // __cplusplus

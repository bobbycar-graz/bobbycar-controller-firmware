#pragma once

#define BOARD_INDEX 0

#define CAN_ID_BOARD                    0x300

#define CAN_SUBID_COMMAND               0x000
#define CAN_SUBID_FEEDBACK              0x002
#define CAN_SUBID_RESERVED0             0x004
#define CAN_SUBID_RESERVED1             0x006
#define CAN_SUBID_RESERVED2             0x008
#define CAN_SUBID_RESERVED3             0x00a
#define CAN_SUBID_RESERVED4             0x00c
#define CAN_SUBID_RESERVED5             0x00e

#define CAN_DIRECTION_STW_TO_BOARD      0x000
#define CAN_DIRECTION_BOARD_TO_STW      0x001

#define CAN_ID_COMMAND_STW_TO_BOARD(board_index) \
    (CAN_ID_BOARD | ((board_index) << 4) | CAN_SUBID_COMMAND | CAN_DIRECTION_STW_TO_BOARD)
#define CAN_ID_COMMAND_BOARD_TO_STW(board_index) \
    (CAN_ID_BOARD | ((board_index) << 4) | CAN_SUBID_COMMAND | CAN_DIRECTION_BOARD_TO_STW)
#define CAN_ID_FEEDBACK_STW_TO_BOARD(board_index) \
    (CAN_ID_BOARD | ((board_index) << 4) | CAN_SUBID_FEEDBACK | CAN_DIRECTION_STW_TO_BOARD)
#define CAN_ID_FEEDBACK_BOARD_TO_STW(board_index) \
    (CAN_ID_BOARD | ((board_index) << 4) | CAN_SUBID_FEEDBACK | CAN_DIRECTION_BOARD_TO_STW)

#ifdef __cplusplus
extern "C"
{
#endif

void can_init(void);
void can_tx(uint16_t address, const uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

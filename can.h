#pragma once

#define CAN_ID_COMMAND_STW_TO_BACK  0x311
#define CAN_ID_COMMAND_BACK_TO_STW  0x312
#define CAN_ID_COMMAND_STW_TO_FRONT 0x315
#define CAN_ID_COMMAND_FRONT_TO_STW 0x316

#define CAN_ID_FEEDBACK_STW_TO_BACK  0x319
#define CAN_ID_FEEDBACK_BACK_TO_STW  0x31A
#define CAN_ID_FEEDBACK_STW_TO_FRONT 0x31D
#define CAN_ID_FEEDBACK_FRONT_TO_STW 0x31E

#define CAN_ID_HB_BACK_TO_STW       0x331
#define CAN_ID_HB_FRONT_TO_STW      0x333


#ifdef __cplusplus
extern "C"
{
#endif

void can_config(void);
void can_test(void);
void can_tx(uint16_t id, const uint8_t* data, uint8_t len);


#ifdef __cplusplus
}
#endif

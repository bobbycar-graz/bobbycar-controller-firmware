#include "stm32f1xx_hal.h"

#include "can_feedc0de.h"
#include "can.h"

/* Definition for CANx clock resources */
#define CANx                           CAN1
#define CANx_CLK_ENABLE()              __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define CANx_FORCE_RESET()             __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()           __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for CANx Pins */
#define CANx_TX_PIN                    GPIO_PIN_9
#define CANx_TX_GPIO_PORT              GPIOB
#define CANx_RX_PIN                    GPIO_PIN_8
#define CANx_RX_GPIO_PORT              GPIOB

/* Definition for CANx AFIO Remap */
#define CANx_AFIO_REMAP_CLK_ENABLE()   __HAL_RCC_AFIO_CLK_ENABLE()
#define CANx_AFIO_REMAP_RX_TX_PIN()    __HAL_AFIO_REMAP_CAN1_2()

/* Definition for CAN's NVIC */
#define CANx_RX_IRQn                   USB_LP_CAN1_RX0_IRQn
#define CANx_RX_IRQHandler             USB_LP_CAN1_RX0_IRQHandler
#define CANx_TX_IRQn                   USB_HP_CAN1_TX_IRQn
#define CANx_TX_IRQHandler             USB_HP_CAN1_TX_IRQHandler


/**
  ******************************************************************************
  * @file    CAN/CAN_Networking/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to configure the CAN peripheral
  *          to send and receive CAN frames in normal mode. The sent frames
  *          are used to control Leds by pressing KEY Push Button.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Private variables ---------------------------------------------------------*/
uint8_t ubKeyNumber = 0x0;
CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/* Private function prototypes -----------------------------------------------*/
static void __NO_RETURN Error_Handler(void);
static void CAN_MspInit(CAN_HandleTypeDef *hcan);
static void CAN_MspDeInit(CAN_HandleTypeDef *hcan);
static void CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
static void CAN_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan);

/* Private functions ---------------------------------------------------------*/

static uint32_t len_to_dlc(uint8_t len)
{
  uint32_t len_u32 = len;

  if (len_u32 <= 8)
    return len_u32;

  // CAN-FD-only lengths currently not supported
  Error_Handler();
}

static uint8_t dlc_to_len(uint32_t dlc)
{
  // Check if DLC valid
  if (dlc & ~0xFu)
    Error_Handler();

  // CAN-FD-only lengths currently not supported
  if (dlc > 8)
    Error_Handler();

  return dlc;
}

void can_tx(uint16_t id, const uint8_t* data, uint8_t len)
{
  TxHeader.StdId = id;
  TxHeader.DLC = len_to_dlc(len);

  while (HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle) < 1)
  {
  }

  /* Start the Transmission process */
  if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, (uint8_t *)data, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void __NO_RETURN Error_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void can_init(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /* Configure the CAN peripheral */
  CanHandle.Instance = CANx;

  CanHandle.MspInitCallback = CAN_MspInit;
  CanHandle.MspDeInitCallback = CAN_MspDeInit;

  CanHandle.Init.TimeTriggeredMode = DISABLE;
  CanHandle.Init.AutoBusOff = ENABLE;
  CanHandle.Init.AutoWakeUp = DISABLE;
  CanHandle.Init.AutoRetransmission = ENABLE;
  CanHandle.Init.ReceiveFifoLocked = DISABLE;
  CanHandle.Init.TransmitFifoPriority = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  CanHandle.Init.TimeSeg1 = CAN_BS1_6TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_5TQ;
  CanHandle.Init.Prescaler = 4;

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = CAN_ID_BOARD | (BOARD_INDEX << 4) | CAN_DIRECTION_STW_TO_BOARD;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x07F1;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  if (HAL_CAN_RegisterCallback(&CanHandle, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_RxFifo0MsgPendingCallback) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_RegisterCallback(&CanHandle, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, CAN_TxMailboxCompleteCallback) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_RegisterCallback(&CanHandle, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, CAN_TxMailboxCompleteCallback) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_RegisterCallback(&CanHandle, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, CAN_TxMailboxCompleteCallback) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* Activate CAN TX notification */
  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* Configure Transmission process */
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;
}

/**
  * @brief  Rx Fifo 0 message pending callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
static void CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  if (RxHeader.IDE == CAN_ID_STD &&
      (RxHeader.StdId == CAN_ID_COMMAND_STW_TO_BOARD(BOARD_INDEX) ||
       RxHeader.StdId == CAN_ID_FEEDBACK_STW_TO_BOARD(BOARD_INDEX)))
  {
    can_feedc0de_handle_frame(RxHeader.StdId, RxData, dlc_to_len(RxHeader.DLC));
  }

  // slightly yucky, but we don't want to block inside the IRQ handler
  if (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) >= 2)
  {
    can_feedc0de_poll();
  }
}

static void CAN_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan)
{
  // slightly yucky, but we don't want to block inside the IRQ handler
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) >= 2)
  {
    can_feedc0de_poll();
  }
}

/**
  * @brief CAN MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for DMA interrupt request enable
  * @param hcan: CAN handle pointer
  * @retval None
  */
static void CAN_MspInit(CAN_HandleTypeDef *hcan)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* CAN1 Periph clock enable */
  CANx_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  CANx_GPIO_CLK_ENABLE();
  /* Enable AFIO clock and Remap CAN PINs to PB8 and PB9*******/
  CANx_AFIO_REMAP_CLK_ENABLE();
  CANx_AFIO_REMAP_RX_TX_PIN();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* CAN1 TX GPIO pin configuration */
  GPIO_InitStruct.Pin = CANx_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init(CANx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* CAN1 RX GPIO pin configuration */
  GPIO_InitStruct.Pin = CANx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init(CANx_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC #################################################*/
  /* NVIC configuration for CAN1 Reception complete interrupt */
  HAL_NVIC_SetPriority(CANx_RX_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CANx_RX_IRQn);

  HAL_NVIC_SetPriority(CANx_TX_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CANx_TX_IRQn);
}

/**
  * @brief CAN MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hcan: CAN handle pointer
  * @retval None
  */
static void CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
  /*##-1- Reset peripherals ##################################################*/
  CANx_FORCE_RESET();
  CANx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the CAN1 TX GPIO pin */
  HAL_GPIO_DeInit(CANx_TX_GPIO_PORT, CANx_TX_PIN);
  /* De-initialize the CAN1 RX GPIO pin */
  HAL_GPIO_DeInit(CANx_RX_GPIO_PORT, CANx_RX_PIN);

  /*##-4- Disable the NVIC for CAN reception #################################*/
  HAL_NVIC_DisableIRQ(CANx_RX_IRQn);
}

/**
* @brief  This function handles CAN1 RX0 interrupt request.
* @param  None
* @retval None
*/
void CANx_RX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&CanHandle);
}

void CANx_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&CanHandle);
}

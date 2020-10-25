/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
* Copyright (C) 2019-2020 Daniel Brunner <daniel@brunner.ninja>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>

#include "stm32f1xx_hal.h"

#include "defines.h"
#include "config.h"
#include "protocol.h"

extern "C" {
#include "BLDC_controller.h"
extern const P rtP_Left; // default settings defined in BLDC_controller_data.c
}

namespace {
TIM_HandleTypeDef htim_right;
TIM_HandleTypeDef htim_left;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

//DMA_HandleTypeDef hdma_usart3_rx;
//DMA_HandleTypeDef hdma_usart3_tx;

volatile struct {
    uint16_t dcr;
    uint16_t dcl;
    uint16_t rl1;
    uint16_t rl2;
    uint16_t rr1;
    uint16_t rr2;
    uint16_t batt1;
    uint16_t l_tx2;
    uint16_t temp;
    uint16_t l_rx2;
} adc_buffer;

// ###############################################################################

volatile uint32_t timeout;
int16_t timeoutCntSerial   = 0;  // Timeout counter for Rx Serial command

uint32_t main_loop_counter;

uint16_t offsetcount = 0;
int16_t offsetrl1    = 2000;
int16_t offsetrl2    = 2000;
int16_t offsetrr1    = 2000;
int16_t offsetrr2    = 2000;
int16_t offsetdcl    = 2000;
int16_t offsetdcr    = 2000;

int16_t batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 20;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

int32_t board_temp_adcFixdt = adc_buffer.temp << 20;  // Fixed-point filter output initialized with current ADC converted to fixed-point
int16_t board_temp_adcFilt  = adc_buffer.temp;
int16_t board_temp_deg_c;

struct {
    RT_MODEL rtM; /* Real-time model */
    P     rtP;    /* Block parameters (auto storage) */
    DW    rtDW;   /* Observable states */
    ExtU  rtU;    /* External inputs */
    ExtY  rtY;    /* External outputs */

    MotorState state;

    uint32_t chops = 0;
} left, right;

struct {
    BuzzerState state;

    uint32_t timer = 0;
} buzzer;

Command command;
Feedback feedback;



void filtLowPass32(int16_t u, uint16_t coef, int32_t *y);

void SystemClock_Config();

void UART2_Init();

//void UART3_Init();

void MX_GPIO_Init();

void MX_TIM_Init();

void MX_ADC1_Init();

void MX_ADC2_Init();

void poweroff();

void parseCommand();

void sendFeedback();

} // anonymous namespace

int main()
{
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();

    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    enum { CurrentMeasAB, CurrentMeasBC, CurrentMeasAC };

    left.rtP = rtP_Left;
#ifdef PETERS_PLATINE
    left.rtP.z_selPhaCurMeasABC  = CurrentMeasBC;            // Left motor measured current phases = {iB, iC} -> do NOT change
#else
    left.rtP.z_selPhaCurMeasABC  = CurrentMeasAB;            // Left motor measured current phases = {iA, iB} -> do NOT change
#endif
    left.rtP.z_ctrlTypSel        = uint8_t(left.state.ctrlTyp);
    left.rtP.b_diagEna           = DIAG_ENA;
    left.rtP.i_max               = (left.state.iMotMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
    left.rtP.n_max               = left.state.nMotMax << 4;                       // fixdt(1,16,4)
    left.rtP.b_fieldWeakEna      = FIELD_WEAK_ENA;
    left.rtP.id_fieldWeakMax     = (left.state.fieldWeakMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
    left.rtP.a_phaAdvMax         = left.state.phaseAdvMax << 4;                   // fixdt(1,16,4)
    left.rtP.r_fieldWeakHi       = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
    left.rtP.r_fieldWeakLo       = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)
    left.rtM.defaultParam        = &left.rtP;
    left.rtM.dwork               = &left.rtDW;
    left.rtM.inputs              = &left.rtU;
    left.rtM.outputs             = &left.rtY;

    right.rtP = rtP_Left;
    right.rtP.z_selPhaCurMeasABC = CurrentMeasBC;            // Right motor measured current phases = {iB, iC} -> do NOT change
    right.rtP.z_ctrlTypSel       = uint8_t(right.state.ctrlTyp);
    right.rtP.b_diagEna          = DIAG_ENA;
    right.rtP.i_max              = (right.state.iMotMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
    right.rtP.n_max              = right.state.nMotMax << 4;                       // fixdt(1,16,4)
    right.rtP.b_fieldWeakEna     = FIELD_WEAK_ENA;
    right.rtP.id_fieldWeakMax    = (right.state.fieldWeakMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
    right.rtP.a_phaAdvMax        = right.state.phaseAdvMax << 4;                   // fixdt(1,16,4)
    right.rtP.r_fieldWeakHi      = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
    right.rtP.r_fieldWeakLo      = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)
    right.rtM.defaultParam       = &right.rtP;
    right.rtM.dwork              = &right.rtDW;
    right.rtM.inputs             = &right.rtU;
    right.rtM.outputs            = &right.rtY;

    /* Initialize BLDC controllers */
    BLDC_controller_initialize(&left.rtM);
    BLDC_controller_initialize(&right.rtM);

    for (int i = 8; i >= 0; i--)
    {
        buzzer.state.freq = (uint8_t)i;
        HAL_Delay(50);
    }
    buzzer.state.freq = 0;

#define UART_DMA_CHANNEL DMA1_Channel7
    UART2_Init();
//#define UART_DMA_CHANNEL DMA1_Channel2
    //UART3_Init();

#ifdef MOTOR_TEST
    int pwm = 0;
    int8_t dir = 1;
#else
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, sizeof(command));
#endif

    for (;;) {
        HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

#ifndef MOTOR_TEST
        parseCommand();
#endif

        timeout = 0;

#ifdef MOTOR_TEST
        left.state.enable = true;
        left.state.ctrlMod = ControlMode::Voltage;
        left.state.ctrlTyp = ControlType::FieldOrientedControl;
        left.state.pwm = pwm;
        left.state.iMotMax = 2;

        right.state.enable = true;
        right.state.ctrlMod = ControlMode::Voltage;
        right.state.ctrlTyp = ControlType::FieldOrientedControl;
        right.state.pwm = pwm;
        right.state.iMotMax = 2;

        constexpr auto pwmMax = 500;

        pwm += dir;
        if (pwm > pwmMax) {
          pwm = pwmMax;
          dir = -1;
        } else if (pwm < -pwmMax) {
          pwm = -pwmMax;
          dir = 1;
        }
#endif

        // ####### CALC BOARD TEMPERATURE #######
        filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
        board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 20);  // convert fixed-point to integer
        board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

        sendFeedback();

#ifdef FEATURE_BUTTON
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN))
        {
            left.state.enable = right.state.enable = 0;           // disable motors

            while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // wait until button is released

            if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {               // do not power off after software reset (from a programmer/debugger)
                __HAL_RCC_CLEAR_RESET_FLAGS();                      // clear reset flags
            } else {
                poweroff();                                         // release power-latch
            }
        }
#endif

        main_loop_counter++;
        timeout++;
    }
}

namespace {
void updateMotors()
{
    DMA1->IFCR = DMA_IFCR_CTCIF1;

    if(offsetcount < 2000) {  // calibrate ADC offsets
        offsetcount++;
        offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
        offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
        offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
        offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
        offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
        offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
        return;
    }

    if (buzzer.timer % 1000 == 0) {  // because you get float rounding errors if it would run every time -> not any more, everything converted to fixed-point
        filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 20);  // convert fixed-point to integer
    }

    // Get Left motor currents
#ifdef PETERS_PLATINE
    int16_t curL_phaB = (int16_t)(offsetrl1 - adc_buffer.rl1)*2;
    int16_t curL_phaA = (int16_t)(offsetrl2 - adc_buffer.rl2)*2;
#else
    int16_t curL_phaA = (int16_t)(offsetrl1 - adc_buffer.rl1);
    int16_t curL_phaB = (int16_t)(offsetrl2 - adc_buffer.rl2);
#endif
    int16_t curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);

    // Get Right motor currents
#ifdef PETERS_PLATINE
    int16_t curR_phaB = (int16_t)(offsetrr1 - adc_buffer.rr1)*2;
    int16_t curR_phaC = (int16_t)(offsetrr2 - adc_buffer.rr2)*2;
#else
    int16_t curR_phaB = (int16_t)(offsetrr1 - adc_buffer.rr1);
    int16_t curR_phaC = (int16_t)(offsetrr2 - adc_buffer.rr2);
#endif
    int16_t curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

    const int8_t chopL = std::abs(curL_DC) > (left.state.iDcMax * A2BIT_CONV);
    if (chopL)
        left.chops++;

    const int8_t chopR = std::abs(curR_DC) > (right.state.iDcMax * A2BIT_CONV);
    if (chopR)
        right.chops++;

    // Disable PWM when current limit is reached (current chopping)
    // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
    if(chopL || timeout > TIMEOUT || left.state.enable == 0) {
      LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    } else {
      LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    }

    if(chopR || timeout > TIMEOUT || right.state.enable == 0) {
      RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    } else {
      RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
    }

    //create square wave for buzzer
    buzzer.timer++;
    if (buzzer.state.freq != 0 && (buzzer.timer / 5000) % (buzzer.state.pattern + 1) == 0) {
      if (buzzer.timer % buzzer.state.freq == 0) {
        HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
      }
    } else {
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    }

    // ############################### MOTOR CONTROL ###############################

    static boolean_T OverrunFlag = false;

    /* Check for overrun */
    if (OverrunFlag) {
      return;
    }
    OverrunFlag = true;

    constexpr int32_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000
    constexpr int32_t pwm_margin = 100;        /* This margin allows to always have a window in the PWM signal for proper Phase currents measurement */

    /* Make sure to stop BOTH motors in case of an error */

#ifdef FEATURE_IGNORE_OTHER_MOTOR
    constexpr bool ignoreOtherMotor = false;
#else
    constexpr bool ignoreOtherMotor = true;
#endif

    const bool enableLFin = left.state.enable && left.rtY.z_errCode == 0 && (right.rtY.z_errCode == 0 || ignoreOtherMotor);
    const bool enableRFin = right.state.enable && (left.rtY.z_errCode == 0 || ignoreOtherMotor) && right.rtY.z_errCode == 0;

    // ========================= LEFT MOTOR ============================
    // Get hall sensors values
    bool hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    bool hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    bool hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    /* Set motor inputs here */
    left.rtP.z_ctrlTypSel         = uint8_t(left.state.ctrlTyp);
    left.rtP.i_max                = (left.state.iMotMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
    left.rtP.n_max                = left.state.nMotMax << 4;                       // fixdt(1,16,4)
    left.rtP.id_fieldWeakMax      = (left.state.fieldWeakMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
    left.rtP.a_phaAdvMax          = left.state.phaseAdvMax << 4;                   // fixdt(1,16,4)

    left.rtU.b_motEna     = enableLFin;
    left.rtU.z_ctrlModReq = uint8_t(left.state.ctrlMod);
    left.rtU.r_inpTgt     = left.state.pwm;
    left.rtU.b_hallA      = hall_ul;
    left.rtU.b_hallB      = hall_vl;
    left.rtU.b_hallC      = hall_wl;
    left.rtU.i_phaAB      = curL_phaA;
    left.rtU.i_phaBC      = curL_phaB;
    left.rtU.i_DCLink     = curL_DC;

    /* Step the controller */
    BLDC_controller_step(&left.rtM);

    /* Get motor outputs here */
    int ul            = left.rtY.DC_phaA;
    int vl            = left.rtY.DC_phaB;
    int wl            = left.rtY.DC_phaC;

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)std::clamp(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)std::clamp(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)std::clamp(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // =================================================================


    // ========================= RIGHT MOTOR ===========================
    // Get hall sensors values
    bool hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    bool hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    bool hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    /* Set motor inputs here */
    right.rtP.z_ctrlTypSel         = uint8_t(right.state.ctrlTyp);
    right.rtP.i_max                = (right.state.iMotMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
    right.rtP.n_max                = right.state.nMotMax << 4;                       // fixdt(1,16,4)
    right.rtP.id_fieldWeakMax      = (right.state.fieldWeakMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
    right.rtP.a_phaAdvMax          = right.state.phaseAdvMax << 4;                   // fixdt(1,16,4)

    right.rtU.b_motEna      = enableRFin;
    right.rtU.z_ctrlModReq  = uint8_t(right.state.ctrlMod);
    right.rtU.r_inpTgt      = right.state.pwm;
    right.rtU.b_hallA       = hall_ur;
    right.rtU.b_hallB       = hall_vr;
    right.rtU.b_hallC       = hall_wr;
    right.rtU.i_phaAB       = curR_phaB;
    right.rtU.i_phaBC       = curR_phaC;
    right.rtU.i_DCLink      = curR_DC;

    /* Step the controller */
    BLDC_controller_step(&right.rtM);

    /* Get motor outputs here */
    int ur            = right.rtY.DC_phaA;
    int vr            = right.rtY.DC_phaB;
    int wr            = right.rtY.DC_phaC;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)std::clamp(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)std::clamp(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)std::clamp(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    // =================================================================

    /* Indicate task complete */
    OverrunFlag = false;
}

// ===========================================================
  /* Low pass filter fixed-point 32 bits: fixdt(1,32,20)
  * Max:  2047.9375
  * Min: -2048
  * Res:  0.0625
  *
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,32,20)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  *
  * Example:
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
  */
void filtLowPass32(int16_t u, uint16_t coef, int32_t *y)
{
    int tmp;

    tmp = (int16_t)(u << 4) - (*y >> 16);
    tmp = std::clamp(tmp, -32768, 32767);  // Overflow protection
    *y  = coef * tmp + (*y);
}

// ===========================================================

/** System Clock Configuration
*/
void SystemClock_Config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
    // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
    PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void UART2_Init()
{
    /* The code below is commented out - otwerwise Serial Receive does not work */
    // #ifdef CONTROL_SERIAL_USART2
    //   /* DMA1_Channel6_IRQn interrupt configuration */
    //   HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 6);
    //   HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    //   /* DMA1_Channel7_IRQn interrupt configuration */
    //   HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 7);
    //   HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    // #endif

    // Disable serial interrupt - it is not needed
    HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);    // Rx Channel
    HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);    // Tx Channel

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance           = USART2;
    huart2.Init.BaudRate      = USART2_BAUD;
    huart2.Init.WordLength    = USART2_WORDLENGTH;
    huart2.Init.StopBits      = UART_STOPBITS_1;
    huart2.Init.Parity        = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling  = UART_OVERSAMPLING_16;
    huart2.Init.Mode        = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);
    USART2->CR3 |= USART_CR3_DMAT;  // | USART_CR3_DMAR | USART_CR3_OVRDIS;

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Pull      = GPIO_PULLUP; //GPIO_NOPULL;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin     = GPIO_PIN_3;
    GPIO_InitStruct.Mode    = GPIO_MODE_INPUT; //GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    hdma_usart2_rx.Instance                 = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR; //DMA_NORMAL;
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    hdma_usart2_tx.Instance                   = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc                = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                  = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority              = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart2_tx);

      __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

      DMA1_Channel7->CPAR     = uint64_t(&(USART2->DR));
      DMA1_Channel7->CNDTR    = 0;
      DMA1->IFCR              = DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CGIF7;
}

//void UART3_Init() {

//  /* The code below is commented out - otwerwise Serial Receive does not work */
//  // #ifdef CONTROL_SERIAL_USART3
//  //   /* DMA1_Channel3_IRQn interrupt configuration */
//  //   HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 3);
//  //   HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
//  //   /* DMA1_Channel2_IRQn interrupt configuration */
//  //   HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 2);
//  //   HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
//  // #endif

//  // Disable serial interrupt - it is not needed
//  HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);  // Rx Channel
//  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);  // Tx Channel

//  __HAL_RCC_DMA1_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_USART3_CLK_ENABLE();

//  huart3.Instance             = USART3;
//  huart3.Init.BaudRate        = USART3_BAUD;
//  huart3.Init.WordLength      = USART3_WORDLENGTH;
//  huart3.Init.StopBits        = UART_STOPBITS_1;
//  huart3.Init.Parity          = UART_PARITY_NONE;
//  huart3.Init.HwFlowCtl       = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling    = UART_OVERSAMPLING_16;
//  #if defined(CONTROL_SERIAL_USART3)
//    huart3.Init.Mode          = UART_MODE_TX_RX;
//  #elif defined(DEBUG_SERIAL_USART3)
//    huart3.Init.Mode          = UART_MODE_TX;
//  #endif
//  HAL_UART_Init(&huart3);

//  #if defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
//    USART3->CR3 |= USART_CR3_DMAT;  // | USART_CR3_DMAR | USART_CR3_OVRDIS;
//  #endif

//  GPIO_InitTypeDef GPIO_InitStruct;
//  GPIO_InitStruct.Pin         = GPIO_PIN_10;
//  GPIO_InitStruct.Pull        = GPIO_PULLUP;
//  GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  #ifdef CONTROL_SERIAL_USART3
//    GPIO_InitStruct.Pin       = GPIO_PIN_11;
//    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//    /* Peripheral DMA init*/
//    hdma_usart3_rx.Instance                   = DMA1_Channel3;
//    hdma_usart3_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
//    hdma_usart3_rx.Init.PeriphInc             = DMA_PINC_DISABLE;
//    hdma_usart3_rx.Init.MemInc                = DMA_MINC_ENABLE;
//    hdma_usart3_rx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
//    hdma_usart3_rx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
//    hdma_usart3_rx.Init.Mode                  = DMA_CIRCULAR; //DMA_NORMAL;
//    hdma_usart3_rx.Init.Priority              = DMA_PRIORITY_LOW;
//    HAL_DMA_Init(&hdma_usart3_rx);
//    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
//  #endif

//  hdma_usart3_tx.Instance                     = DMA1_Channel2;
//  hdma_usart3_tx.Init.Direction               = DMA_MEMORY_TO_PERIPH;
//  hdma_usart3_tx.Init.PeriphInc               = DMA_PINC_DISABLE;
//  hdma_usart3_tx.Init.MemInc                  = DMA_MINC_ENABLE;
//  hdma_usart3_tx.Init.PeriphDataAlignment     = DMA_PDATAALIGN_BYTE;
//  hdma_usart3_tx.Init.MemDataAlignment        = DMA_MDATAALIGN_BYTE;
//  hdma_usart3_tx.Init.Mode                    = DMA_NORMAL;
//  hdma_usart3_tx.Init.Priority                = DMA_PRIORITY_LOW;
//  HAL_DMA_Init(&hdma_usart3_tx);

//  #ifdef CONTROL_SERIAL_USART3
//    __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);
//  #endif
//  #if defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
//    DMA1_Channel2->CPAR     = (uint32_t) & (USART3->DR);
//    DMA1_Channel2->CNDTR    = 0;
//    DMA1->IFCR              = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2;
//  #endif
//}

void MX_GPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = LEFT_HALL_U_PIN;
    HAL_GPIO_Init(LEFT_HALL_U_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_HALL_V_PIN;
    HAL_GPIO_Init(LEFT_HALL_V_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_HALL_W_PIN;
    HAL_GPIO_Init(LEFT_HALL_W_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_HALL_U_PIN;
    HAL_GPIO_Init(RIGHT_HALL_U_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_HALL_V_PIN;
    HAL_GPIO_Init(RIGHT_HALL_V_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_HALL_W_PIN;
    HAL_GPIO_Init(RIGHT_HALL_W_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CHARGER_PIN;
    HAL_GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_PIN;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);


    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUZZER_PIN;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OFF_PIN;
    HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);


    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pin = LEFT_DC_CUR_PIN;
    HAL_GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_U_CUR_PIN;
    HAL_GPIO_Init(LEFT_U_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_V_CUR_PIN;
    HAL_GPIO_Init(LEFT_V_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_DC_CUR_PIN;
    HAL_GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_U_CUR_PIN;
    HAL_GPIO_Init(RIGHT_U_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_V_CUR_PIN;
    HAL_GPIO_Init(RIGHT_V_CUR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCLINK_PIN;
    HAL_GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

    //Analog in
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pin = LEFT_TIM_UH_PIN;
    HAL_GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_TIM_VH_PIN;
    HAL_GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_TIM_WH_PIN;
    HAL_GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_TIM_UL_PIN;
    HAL_GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_TIM_VL_PIN;
    HAL_GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LEFT_TIM_WL_PIN;
    HAL_GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_UH_PIN;
    HAL_GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_VH_PIN;
    HAL_GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_WH_PIN;
    HAL_GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_UL_PIN;
    HAL_GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_VL_PIN;
    HAL_GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RIGHT_TIM_WL_PIN;
    HAL_GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
}

void MX_TIM_Init() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    TIM_SlaveConfigTypeDef sTimConfig;

    htim_right.Instance               = RIGHT_TIM;
    htim_right.Init.Prescaler         = 0;
    htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim_right.Init.Period            = 64000000 / 2 / PWM_FREQ;
    htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_right.Init.RepetitionCounter = 0;
    htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim_right);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
#ifdef PETERS_PLATINE
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
#else
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
#endif
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);

    htim_left.Instance               = LEFT_TIM;
    htim_left.Init.Prescaler         = 0;
    htim_left.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim_left.Init.Period            = 64000000 / 2 / PWM_FREQ;
    htim_left.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_left.Init.RepetitionCounter = 0;
    htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim_left);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim_left, &sMasterConfig);

    sTimConfig.InputTrigger = TIM_TS_ITR0;
    sTimConfig.SlaveMode    = TIM_SLAVEMODE_GATED;
    HAL_TIM_SlaveConfigSynchro(&htim_left, &sTimConfig);

    // Start counting >0 to effectively offset timers by the time it takes for one ADC conversion to complete.
    // This method allows that the Phase currents ADC measurements are properly aligned with LOW-FET ON region for both motors
    LEFT_TIM->CNT 		     = ADC_TOTAL_CONV_TIME;

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
#ifdef PETERS_PLATINE
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
#else
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
#endif
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_3);

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim_left, &sBreakDeadTimeConfig);

    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

    HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

    htim_left.Instance->RCR = 1;

    __HAL_TIM_ENABLE(&htim_right);
}

void MX_ADC1_Init() {
    ADC_MultiModeTypeDef multimode;
    ADC_ChannelConfTypeDef sConfig;

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 5;
    HAL_ADC_Init(&hadc1);
    /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
    __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

    /**Configure the ADC multi-mode
    */
    multimode.Mode = ADC_DUALMODE_REGSIMULT;
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.Channel = ADC_CHANNEL_11;  // pc1 left cur  ->  right
    sConfig.Rank    = 1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    sConfig.Channel = ADC_CHANNEL_0;  // pa0 right a   ->  left
    sConfig.Rank    = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_14;  // pc4 left b   -> right
    sConfig.Rank    = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_12;  // pc2 vbat
    sConfig.Rank    = 4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    //temperature requires at least 17.1uS sampling time
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;  // internal temp
    sConfig.Rank    = 5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_TSVREFE;

    __HAL_ADC_ENABLE(&hadc1);

    __HAL_RCC_DMA1_CLK_ENABLE();

    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CNDTR = 5;
    DMA1_Channel1->CPAR  = uint64_t(&(ADC1->DR));
    DMA1_Channel1->CMAR  = uint64_t(&adc_buffer);
    DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ADC2 init function */
void MX_ADC2_Init() {
    ADC_ChannelConfTypeDef sConfig;

    __HAL_RCC_ADC2_CLK_ENABLE();

    // HAL_ADC_DeInit(&hadc2);
    // hadc2.Instance->CR2 = 0;
    /**Common config
    */
    hadc2.Instance                   = ADC2;
    hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 5;
    HAL_ADC_Init(&hadc2);


    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.Channel = ADC_CHANNEL_10;  // pc0 right cur   -> left
    sConfig.Rank    = 1;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    sConfig.Channel = ADC_CHANNEL_13;  // pc3 right b   -> left
    sConfig.Rank    = 2;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    sConfig.Channel = ADC_CHANNEL_15;  // pc5 left c   -> right
    sConfig.Rank    = 3;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    sConfig.Channel = ADC_CHANNEL_2;  // pa2 uart-l-tx
    sConfig.Rank    = 4;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfig.Channel = ADC_CHANNEL_3;  // pa3 uart-l-rx
    sConfig.Rank    = 5;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    hadc2.Instance->CR2 |= ADC_CR2_DMA;
    __HAL_ADC_ENABLE(&hadc2);
}

void poweroff() {
  //  if (abs(speed) < 20) {  // wait for the speed to drop, then shut down -> this is commented out for SAFETY reasons
        buzzer.state.pattern = 0;
        left.state.enable = right.state.enable = 0;
        for (int i = 0; i < 8; i++) {
            buzzer.state.freq = (uint8_t)i;
            HAL_Delay(50);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
        for (int i = 0; i < 5; i++)
            HAL_Delay(1000);
  //  }
}

void parseCommand()
{
    bool any_parsed{false};

    for (int i = 0; i < 1; i++)
    {
        if (command.start != Command::VALID_HEADER)
            continue;

        uint16_t checksum = calculateChecksum(command);
        if (command.checksum != checksum)
            continue;

        left.state = command.left;
        right.state = command.right;

        buzzer.state = command.buzzer;

        if (command.poweroff)
            poweroff();

        HAL_GPIO_WritePin(LED_PORT, LED_PIN, command.led ? GPIO_PIN_RESET : GPIO_PIN_SET);

        command.start     = Command::INVALID_HEADER; // Change the Start Frame for timeout detection in the next cycle
        timeoutCntSerial  = 0;                       // Reset the timeout counter

        any_parsed = true;
        break;
    }

    if (!any_parsed)
    {
        if (timeoutCntSerial++ >= 100) // Timeout qualification
        {
            timeoutCntSerial  = 100; // Limit timout counter value

            left.state = right.state = {.enable=true};

            buzzer.state = { 24, 1 };

            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

            // Check periodically the received Start Frame. Try to re-sync by reseting the DMA
            if (main_loop_counter % 25 == 0)
            {
                HAL_UART_DMAStop(&huart2);
                HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, sizeof(command));
            }
        }
    }
}

void sendFeedback()
{
    if (main_loop_counter % 50 == 0) {    // Send data periodically
        if(UART_DMA_CHANNEL->CNDTR == 0) {
            feedback.start = Feedback::VALID_HEADER;

            feedback.left.angle = left.rtY.a_elecAngle;
            feedback.right.angle = right.rtY.a_elecAngle;

            feedback.left.speed = left.rtY.n_mot;
            feedback.right.speed = right.rtY.n_mot;

            feedback.left.error = left.rtY.z_errCode;
            feedback.right.error = right.rtY.z_errCode;

            feedback.left.current = left.rtU.i_DCLink;
            feedback.right.current = right.rtU.i_DCLink;

            feedback.left.chops = left.chops;
            feedback.right.chops = right.chops;
            left.chops = 0;
            right.chops = 0;

            feedback.left.hallA = left.rtU.b_hallA;
            feedback.left.hallB = left.rtU.b_hallB;
            feedback.left.hallC = left.rtU.b_hallC;
            feedback.right.hallA = right.rtU.b_hallA;
            feedback.right.hallB = right.rtU.b_hallB;
            feedback.right.hallC = right.rtU.b_hallC;

            feedback.batVoltage = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
            feedback.boardTemp = board_temp_deg_c;
            feedback.timeoutCntSerial = timeoutCntSerial;

            feedback.checksum = calculateChecksum(feedback);

            UART_DMA_CHANNEL->CCR    &= ~DMA_CCR_EN;
            UART_DMA_CHANNEL->CNDTR   = sizeof(feedback);
            UART_DMA_CHANNEL->CMAR    = uint64_t(&feedback);
            UART_DMA_CHANNEL->CCR    |= DMA_CCR_EN;
        }
    }
}

} // anonymous namespace

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
extern "C" void NMI_Handler() {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
extern "C" void HardFault_Handler() {
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while(1) {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
extern "C" void MemManage_Handler() {
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while(1) {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
extern "C" void BusFault_Handler() {
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while(1) {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
extern "C" void UsageFault_Handler() {
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while(1) {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
extern "C" void SVC_Handler() {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
extern "C" void DebugMon_Handler() {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
extern "C" void PendSV_Handler() {
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
#ifdef CONTROL_PPM
extern "C" void PPM_SysTick_Callback();
#endif

extern "C" void SysTick_Handler() {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
#ifdef CONTROL_PPM
  PPM_SysTick_Callback();
#endif
  /* USER CODE END SysTick_IRQn 1 */
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
extern "C" void DMA1_Channel1_IRQHandler()
{
    updateMotors();
}

extern "C" void DMA1_Channel6_IRQHandler()
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
extern "C" void DMA1_Channel7_IRQHandler()
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

#pragma once

#include "config.h"

namespace UVLO
{
static constexpr uint32_t NUM_MEASUREMENTS = 50;
static constexpr uint32_t MIN_VOLTAGE = 1500 * BAT_CALIB_ADC / BAT_CALIB_REAL_VOLTAGE;

static uint32_t measurements = 0;
static uint32_t sum = 0;
static bool locked_out = true;

static void update(uint16_t voltage)
{
    if (measurements < NUM_MEASUREMENTS)
    {
        sum += voltage;
        measurements++;

        if (measurements == NUM_MEASUREMENTS && sum / NUM_MEASUREMENTS >= MIN_VOLTAGE)
            locked_out = false;
    }
}

static bool isDone()
{
    return measurements >= NUM_MEASUREMENTS;
}

static bool isLockedOut()
{
    return locked_out;
}

} // namespace

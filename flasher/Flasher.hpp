#pragma once

#include <cstdint>
#include <cstring>

#include "stm32f1xx_hal.h"

#include "ab_boot/ab_boot.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"


namespace flasher {
    enum class State : uint8_t {
        Idle,
        Erasing,
        Waiting,
        Writing,
        Error
    };

    namespace {
        enum class FlashRegion {
            Invalid,
            Bootloader,
            AppA,
            AppB,
            Config
        };

        FlashRegion region_for_address(uintptr_t address) {
            using enum FlashRegion;

            if (address >= FLASH_START && address < APP_A_START) {
                return Bootloader;
            } else if (address >= APP_A_START && address < (APP_A_START + APP_SIZE)) {
                return AppA;
            } else if (address >= APP_B_START && address < (APP_B_START + APP_SIZE)) {
                return AppB;
            } else if (address >= CONFIG_START && address < (CONFIG_START + CONFIG_SIZE)) {
                return Config;
            }

            return Invalid;
        }

        size_t region_size(FlashRegion region) {
            using enum FlashRegion;

            switch (region) {
            case Bootloader:
                return AB_BOOT_SIZE;
            case AppA:
            case AppB:
                return APP_SIZE;
            case Config:
                return CONFIG_SIZE;
            default:
                return 0;
            }
        }

        bool is_valid_start_address(uint32_t address) {
            return address == FLASH_START || address == APP_A_START ||
                   address == APP_B_START || address == CONFIG_START;
        }

        static State state_;
        static FlashRegion region_;
        static uintptr_t address_;
        static uint8_t write_size_;
    }

    enum class Result : uint8_t {
        Success,
        RegionNotAllowed,
        InvalidParameter,
        InvalidState,
        WriteError,
        InProgress
    };

    void flasher_state_callback(State state, Result result, uint32_t arg);

    void init() {
        state_ = State::Idle;
        region_ = FlashRegion::Invalid;
        address_ = 0;
        write_size_ = 0;
        HAL_FLASH_Lock();
    }

    Result start(uintptr_t address) {
        if (state_ != State::Idle)
            return Result::InvalidState;

        if (!is_valid_start_address(address)) {
            return Result::InvalidParameter;
        }

        FlashRegion flashed_region = region_for_address(address);
        FlashRegion running_region = region_for_address((uintptr_t)&start);

        if (flashed_region == FlashRegion::Invalid) {
            return Result::InvalidParameter;
        }

        if (flashed_region == running_region) {
            // prohibit flashing the currently running app
            return Result::RegionNotAllowed;
        }

        size_t size = region_size(flashed_region);
        FLASH_EraseInitTypeDef ferase = {
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .PageAddress = address,
            .NbPages = size / FLASH_PAGE_SIZE
        };

        HAL_FLASH_Unlock();
        if (HAL_FLASHEx_Erase_IT(&ferase) != HAL_OK) {
            return Result::WriteError;
        }

        state_ = State::Erasing;
        region_ = flashed_region;
        address_ = address;

        return Result::InProgress;
    }

    Result write(uint8_t *data, uint8_t length) {
        if (state_ != State::Waiting)
            return Result::InvalidState;

        uint32_t program_type;
        switch (length) {
        case 2:
            program_type = FLASH_PROC_PROGRAMHALFWORD;
            break;
        case 4:
            program_type = FLASH_PROC_PROGRAMWORD;
            break;
        case 8:
            program_type = FLASH_PROC_PROGRAMDOUBLEWORD;
            break;
        default:
            return Result::InvalidParameter;
        }

        uint64_t data_int = 0;
        memcpy(&data_int, data, length);

        if (HAL_FLASH_Program_IT(program_type, address_, data_int) != HAL_OK)
            return Result::WriteError;

        state_ = State::Writing;
        write_size_ = length;

        return Result::InProgress;
    }

    State get_state() {
        return state_;
    }

    void flash_callback(bool success) {
        using enum State;

        // Ignore if we are in Idle state, could be the result of
        // a cancelled operation.
        if (state_ == Idle)
            return;

        if (success) {
            switch (state_) {
            case Writing:
            case Erasing:
                address_ += write_size_;
                flasher_state_callback(state_, Result::Success, address_);
                state_ = Waiting;
                write_size_ = 0;
                break;
            default:
                // Spurious callback
                HAL_FLASH_Lock();
                state_ = Error;
            }
        } else {
            switch (state_) {
            case Writing:
            case Erasing:
                flasher_state_callback(state_, Result::WriteError, address_);
            [[fallthrough]];
            default:
                // Spurious callback
                HAL_FLASH_Lock();
                state_ = Error;
            }
        }
    }
}

#include "comms/GamecubeBackend.hpp"

#include "core/InputSource.hpp"

#include "modes/MeleeLimits.hpp"

#include <Arduino.h>
#include <GamecubeConsole.hpp>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <hardware/gpio.h>
#include <pico/platform.h>
#include <string.h>

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

static constexpr uint32_t metadata_magic = 0x4842594d; // "MYBH"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t chunks;
    uint8_t data[gc_metadata_max_size];
} persisted_gc_metadata_t;

static_assert(
    sizeof(persisted_gc_metadata_t) <= FLASH_SECTOR_SIZE,
    "metadata persistence exceeds one flash sector"
);

static constexpr uint32_t metadata_flash_offset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static const persisted_gc_metadata_t *const metadata_flash =
    (const persisted_gc_metadata_t *)(XIP_BASE + metadata_flash_offset);

// Keep flash staging buffer out of stack. A 4KB stack allocation in the commit
// path can overflow on RP2040/Arduino and crash after a seemingly successful
// save.
static uint8_t metadata_sector_buf[FLASH_SECTOR_SIZE];

static void build_haybox_default_metadata(uint8_t *metadata, uint8_t &chunks) {
    memset(metadata, 0, gc_metadata_max_size);

    uint8_t *p = metadata;
    int idx = 0;
    const char *key = "firmware";
    const char *value = "HayBox";

    // UBJSON: { U <len_key> <key> S U <len_val> <value> }
    p[idx++] = 0x7B;
    p[idx++] = 'U';
    p[idx++] = (uint8_t)strlen(key);
    memcpy(&p[idx], key, strlen(key));
    idx += (int)strlen(key);

    p[idx++] = 'S';
    p[idx++] = 'U';
    p[idx++] = (uint8_t)strlen(value);
    memcpy(&p[idx], value, strlen(value));
    idx += (int)strlen(value);

    p[idx++] = 0x7D;
    chunks = 1;
}

static bool load_persisted_metadata(uint8_t *metadata, uint8_t &chunks) {
    if (metadata_flash->magic != metadata_magic) {
        return false;
    }

    if (metadata_flash->chunks == 0 || metadata_flash->chunks > gc_metadata_max_chunks) {
        return false;
    }

    chunks = metadata_flash->chunks;
    memcpy(metadata, metadata_flash->data, gc_metadata_max_size);
    return true;
}

static void __no_inline_not_in_flash_func(save_persisted_metadata)(
    const uint8_t *metadata,
    uint8_t chunks
) {
    memset(metadata_sector_buf, 0xFF, sizeof(metadata_sector_buf));

    persisted_gc_metadata_t persisted;
    persisted.magic = metadata_magic;
    persisted.chunks = chunks;
    memcpy(persisted.data, metadata, gc_metadata_max_size);
    memcpy(metadata_sector_buf, &persisted, sizeof(persisted));

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(metadata_flash_offset, FLASH_SECTOR_SIZE);
    flash_range_program(metadata_flash_offset, metadata_sector_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(interrupts);
}

//#define TIMINGDEBUG

GamecubeBackend::GamecubeBackend(
    InputSource **input_sources,
    size_t input_source_count,
    uint data_pin,
    bool nerfOn,
    PIO pio,
    int sm,
    int offset
)
    : CommunicationBackend(input_sources, input_source_count) {
    _gamecube = new GamecubeConsole(data_pin, pio, sm, offset);

    uint8_t metadata[gc_metadata_max_size];
    uint8_t chunks = 1;
    if (!load_persisted_metadata(metadata, chunks)) {
        build_haybox_default_metadata(metadata, chunks);
        save_persisted_metadata(metadata, chunks);
    }
    _gamecube->SetMetadata(metadata, chunks);
    _gamecube->SetMetadataWriteCallback(&GamecubeBackend::OnMetadataWrite, this);

    _report = default_gc_report;
    _nerfOn = nerfOn;
}

GamecubeBackend::~GamecubeBackend() {
    delete _gamecube;
}

void GamecubeBackend::SendReport() {
    // Update slower inputs before we start waiting for poll.
    //ScanInputs(InputScanSpeed::SLOW);
    //ScanInputs(InputScanSpeed::MEDIUM);
    //This fork won't support slower inputs

    static uint32_t minLoop = 16384;
    static uint loopCount = 0;
    static bool detect = true;//false means run
    static uint sampleCount = 1;
    static uint32_t sampleSpacing = 0;
    static uint32_t oldSampleTime = 0;
    static uint32_t newSampleTime = 0;
    static uint32_t loopTime = 0;
    //const uint fastestLoop = 950; //fastest possible loop for AVR
    const uint fastestLoop = 450; //fastest possible loop for rp2040

    oldSampleTime = newSampleTime;
    //YOU CANNOT USE MICROS ON AVR
    //You need to manually use the low level timers
    newSampleTime = micros();
    loopTime = newSampleTime - oldSampleTime;

    if(detect) {
        //run loop time detection procedure
#ifdef TIMINGDEBUG
        gpio_put(1, loopCount%2 == 0);
#endif
        loopCount++;
        if(loopCount > 5 && loopTime > 300) {//screen out implausibly fast samples; the limit is 2500 Hz (400 us)
            minLoop = min(minLoop, loopTime);
        }
        if(loopCount >= 100) {
            detect = false;
            sampleCount = 1;
            while(1000*sampleCount <= minLoop) {//we want [sampleCount] ms-spaced samples within the smallest possible loop
                sampleCount++;
            }
            sampleSpacing = minLoop / sampleCount;
            if(sampleSpacing < fastestLoop && sampleCount > 1) {
                sampleCount--;
                sampleSpacing = minLoop / sampleCount;
            }
        }
        // Make sure to respond while measuring.
        ScanInputs(InputScanSpeed::FAST);

        // Run gamemode logic.
        UpdateOutputs();

        _report.a = _outputs.a;
        _report.b = _outputs.b;
        _report.x = _outputs.x;
        _report.y = _outputs.y;
        _report.z = _outputs.buttonR;
        _report.l = _outputs.triggerLDigital;
        _report.r = _outputs.triggerRDigital;
        _report.start = _outputs.start;
        _report.dpad_left = _outputs.dpadLeft | _outputs.select;
        _report.dpad_right = _outputs.dpadRight | _outputs.home;
        _report.dpad_down = _outputs.dpadDown;
        _report.dpad_up = _outputs.dpadUp;

        // Analog outputs
        _report.stick_x = _outputs.leftStickX;
        _report.stick_y = _outputs.leftStickY;
        _report.cstick_x = _outputs.rightStickX;
        _report.cstick_y = _outputs.rightStickY;
        _report.l_analog = _outputs.triggerLAnalog;
        _report.r_analog = _outputs.triggerRAnalog;
    } else {
        //run the delay procedure based on samplespacing
        //in the stock arduino software, it samples 850 us after the end of the poll response
        //we want the last sample to begin [850 + extra computation time] before the beginning of the last poll to give room for the sample and the travel time+nerf computation
        //
        for (uint i = 0; i < sampleCount; i++) {
#ifdef TIMINGDEBUG
            gpio_put(1, 0);
#endif

            const int nerfTime = 0;
            const int computationTime = 250 + nerfTime;//*_nerfOn;//us; depends on the platform.
            const uint32_t targetTime = ((i+1)*sampleSpacing)-computationTime;
            int count = 0;
            while(micros() - newSampleTime < targetTime) {
                count++;//do something?
                //spinlock
            }
#ifdef TIMINGDEBUG
            gpio_put(1, count>0);
#endif

            ScanInputs(InputScanSpeed::FAST);

            // Run gamemode logic.
            UpdateOutputs();

            if(_gamemode->isMelee()) {
                //APPLY NERFS HERE
                OutputState nerfedOutputs;
                limitOutputs(sampleSpacing/4, _nerfOn ? AB_A : AB_B, _inputs, _outputs, nerfedOutputs);

                // Digital outputs
                _report.a = nerfedOutputs.a;
                _report.b = nerfedOutputs.b;
                _report.x = nerfedOutputs.x;
                _report.y = nerfedOutputs.y;
                _report.z = nerfedOutputs.buttonR;
                _report.l = nerfedOutputs.triggerLDigital;
                _report.r = nerfedOutputs.triggerRDigital;
                _report.start = nerfedOutputs.start;
                _report.dpad_left = nerfedOutputs.dpadLeft | nerfedOutputs.select;
                _report.dpad_right = nerfedOutputs.dpadRight | nerfedOutputs.home;
                _report.dpad_down = nerfedOutputs.dpadDown;
                _report.dpad_up = nerfedOutputs.dpadUp;

                // Analog outputs
                _report.stick_x = nerfedOutputs.leftStickX;
                _report.stick_y = nerfedOutputs.leftStickY;
                _report.cstick_x = nerfedOutputs.rightStickX;
                _report.cstick_y = nerfedOutputs.rightStickY;
                _report.l_analog = nerfedOutputs.triggerLAnalog;
                _report.r_analog = nerfedOutputs.triggerRAnalog;
            } else {
                // Digital outputs
                _report.a = _outputs.a;
                _report.b = _outputs.b;
                _report.x = _outputs.x;
                _report.y = _outputs.y;
                _report.z = _outputs.buttonR;
                _report.l = _outputs.triggerLDigital;
                _report.r = _outputs.triggerRDigital;
                _report.start = _outputs.start;
                _report.dpad_left = _outputs.dpadLeft | _outputs.select;
                _report.dpad_right = _outputs.dpadRight | _outputs.home;
                _report.dpad_down = _outputs.dpadDown;
                _report.dpad_up = _outputs.dpadUp;

                // Analog outputs
                _report.stick_x = _outputs.leftStickX;
                _report.stick_y = _outputs.leftStickY;
                _report.cstick_x = _outputs.rightStickX;
                _report.cstick_y = _outputs.rightStickY;
                _report.l_analog = _outputs.triggerLAnalog;
                _report.r_analog = _outputs.triggerRAnalog;
            }
        }
        if(loopTime > minLoop+(minLoop >> 1)) {//if the loop time is 50% longer than expected
            detect = true;//stop scanning inputs briefly and re-measure timings
            loopCount = 0;
            sampleCount = 1;
            sampleSpacing = 0;
            oldSampleTime = 0;
            newSampleTime = 0;
            loopTime = 0;
        }
    }

    _gamecube->WaitForPollStart();
#ifdef TIMINGDEBUG
    gpio_put(1, 1);
#endif

    // Send outputs to console unless poll command is invalid.
    if (_gamecube->WaitForPollEnd() != PollStatus::ERROR) {
        _gamecube->SendReport(&_report);
    }
}

void GamecubeBackend::OnMetadataWrite(void *context) {
    GamecubeBackend *self = static_cast<GamecubeBackend *>(context);

    // Called once the final 0xB0 chunk lands. The joybus layer advertises a
    // chunk count of 0 while a multi-chunk write is still in flight and the
    // real count only once the final chunk arrives, so reaching here means the
    // full object is present and we erase flash exactly once per write — never
    // between chunks. Metadata writes are infrequent, so the brief stall here
    // is acceptable.
    uint8_t metadata[gc_metadata_max_size];
    uint8_t chunks;
    self->_gamecube->GetMetadata(metadata, chunks);

    if (chunks == 0) {
        return;
    }

    // Skip the erase/program if flash already matches.
    if (metadata_flash->magic == metadata_magic && metadata_flash->chunks == chunks &&
        memcmp(metadata_flash->data, metadata, gc_metadata_max_size) == 0) {
        return;
    }

    // Park the other core for the flash erase/program window, then write.
    rp2040.idleOtherCore();
    save_persisted_metadata(metadata, chunks);
    rp2040.resumeOtherCore();
}

int GamecubeBackend::GetOffset() {
    return _gamecube->GetOffset();
}

//
// SPDX-FileCopyrightText: Copyright 2024 Arm Limited and/or its affiliates <open-source-office@arm.com>
// SPDX-License-Identifier: BSD-3-Clause
//

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#include "pico/pdm_microphone.h"

#include "usb_microphone.h"

#include "rnnoise.h"

#include "hardware/clocks.h"

const int noise_suppression_enabled_pin = 18;

// configuration
const struct pdm_microphone_config pdm_config = {
  .gpio_data = 21,
  .gpio_clk = 20,
  .pio = pio0,
  .pio_sm = 0,
  .sample_rate = SAMPLE_RATE,
  .sample_buffer_size = 480, // PDM driver internal buffer size
};

// variables
int16_t denoise_buffer[480];       // Buffer for raw PDM samples before processing
volatile bool new_samples = false; // Flag from PDM callback to core1

// Circular buffer for USB transmission
// It's effectively a double buffer, where each half is 480 samples.
#define AUDIO_BLOCK_SAMPLES 480
#define NUM_AUDIO_BLOCKS 2
volatile uint16_t sample_buffer[AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS];

// Volatile for shared access between core1 (writer) and core0/ISR (reader)
volatile int in_index = 0;  // Write index for sample_buffer (managed by core1)
volatile int out_index = 0; // Read index for sample_buffer (managed by on_usb_microphone_tx_ready)

absolute_time_t last_tx_time; // Timestamp of the last successful USB transmission

// callback functions
void on_pdm_samples_ready();
void on_usb_microphone_tx_ready();

uint32_t core1_stack[0xc000 / sizeof(uint32_t)];
void core1_entry();


int main(void)
{
  stdio_init_all();
  set_sys_clock_khz(160000, true);

  gpio_init(noise_suppression_enabled_pin);
  gpio_pull_up(noise_suppression_enabled_pin);
  gpio_set_dir(noise_suppression_enabled_pin, GPIO_IN);

  // Initialize last_tx_time before USB stack might call the callback
  last_tx_time = get_absolute_time();

  usb_microphone_init();
  usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

  multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof(core1_stack));

  while (1) {
    usb_microphone_task();
  }

  return 0;
}

void core1_entry()
{
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
  pwm_config pwn_cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&pwn_cfg, 4.0f);
  pwm_init(slice_num, &pwn_cfg, true);

  float x_f32[AUDIO_BLOCK_SAMPLES];
  DenoiseState* st = rnnoise_create(NULL);

  memset(x_f32, 0x00, sizeof(x_f32));
  rnnoise_process_frame(st, x_f32, x_f32);

  pdm_microphone_init(&pdm_config);
  pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);
  pdm_microphone_set_filter_gain(16); // Adjust gain as needed
  pdm_microphone_start();

  // Initialize in_index to point to the start of the second block,
  // assuming out_index starts at 0. This gives one block of buffer space initially.
  // Or simply start both at 0, and let in_index get ahead.
  // For simplicity, let's start both at 0. PDM will fill first.
  // in_index = AUDIO_BLOCK_SAMPLES; // Alternative: start in_index ahead

  while (1) {
    while(!new_samples) {
      tight_loop_contents();
    }
    new_samples = false;

    // --- Start of critical section for buffer check (conceptual) ---
    // Check if there's space to write a new block without overwriting data
    // that USB hasn't read yet.
    // `current_out_index` is a snapshot.
    int current_out_index = out_index;
    int samples_in_flight = (in_index - current_out_index + (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS)) % (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS);
    
    if (samples_in_flight + AUDIO_BLOCK_SAMPLES > (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS)) {
        // Overrun potential: USB is too slow, or core1 is too fast.
        // This means the buffer is full. We'd be overwriting unread data.
        // Simplest (but not ideal) is to drop new PDM samples.
        // Or, a more robust system might slightly adjust sample rate or have larger buffers.
        // For now, we'll allow overwrite as the original code did, but this is a known issue spot.
        // A more advanced fix would be to wait or skip, or use a counter for available blocks.
        // If this happens, the LED could signal it.
        // pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, 0xFFFF); // Signal overrun
        // continue; // Skip this frame to prevent overwrite
    }
    // --- End of critical section for buffer check ---


    float vad = 0.0;
    float* f32_ptr = x_f32;
    int16_t* i16_raw_ptr = denoise_buffer;

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      *f32_ptr++ = *i16_raw_ptr++;
    }

    if (gpio_get(noise_suppression_enabled_pin)) {
      vad = rnnoise_process_frame(st, x_f32, x_f32);
    }

    // Copy processed data to the circular USB buffer
    // Cast to non-volatile for the loop assignment, relying on volatile in_index for overall sync
    uint16_t* i16_target_ptr = (uint16_t*)&sample_buffer[in_index];
    f32_ptr = x_f32;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      // Apply gain if needed, rnnoise output is typically -32768 to 32767
      // If your gain was in pdm_microphone_set_filter_gain, it's already applied to denoise_buffer
      // If rnnoise changes scale, adjust here.
      float sample = *f32_ptr++;
      if (sample > 32767.0f) sample = 32767.0f;
      if (sample < -32768.0f) sample = -32768.0f;
      *i16_target_ptr++ = (int16_t)sample;
    }

    // Advance write pointer
    in_index = (in_index + AUDIO_BLOCK_SAMPLES) % (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS);

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, (uint16_t)(vad * 0xFFFF));
  }
}

void on_pdm_samples_ready()
{
  // Read PDM samples into a temporary buffer for core1 to process
  pdm_microphone_read(denoise_buffer, AUDIO_BLOCK_SAMPLES);
  new_samples = true;
}

#define USB_PACKET_SAMPLES 16 // Typically 1ms of data at 16kHz mono -> 16 samples
#define USB_PACKET_BYTES (USB_PACKET_SAMPLES * sizeof(int16_t))
#define HOST_TIMEOUT_US (100 * 1000) // 100 ms timeout

void on_usb_microphone_tx_ready()
{
  absolute_time_t now = get_absolute_time();
  if (absolute_time_diff_us(last_tx_time, now) > HOST_TIMEOUT_US) {
    // Host hasn't requested data for a while. This could be a stall or app closure.
    // To recover gracefully, we can try to resynchronize out_index.
    // A simple resync: assume the buffer is now stale, try to skip to "fresher" data.
    // Set out_index to be one block behind in_index.
    // This discards potentially old data and attempts to catch up.
    int current_in_index = in_index; // Volatile read
    out_index = (current_in_index - AUDIO_BLOCK_SAMPLES + (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS)) % (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS);
    // This ensures out_index points to the start of the block just before the one in_index might be writing to.
  }
  last_tx_time = now;

  int current_in_index = in_index; // Volatile read for consistent check
  if (current_in_index == out_index) {
    // Buffer is empty (underrun). This means PDM/core1 is not keeping up.
    // Send silence to fulfill USB host's request.
    // This should ideally not happen if PDM source is faster or equal to USB rate.
    static int16_t silence[USB_PACKET_SAMPLES] = {0};
    usb_microphone_write(silence, USB_PACKET_BYTES);
    // Note: We are not advancing out_index here on underrun to prevent it from
    // racing ahead if PDM is genuinely stalled. The host will get silence repeatedly.
    // If the requirement is to always advance, then out_index should be updated.
    // However, if PDM is working, in_index will soon advance, and this condition will clear.
  } else {
    // Send data from the circular buffer
    usb_microphone_write((const int16_t*)&sample_buffer[out_index], USB_PACKET_BYTES);
    out_index = (out_index + USB_PACKET_SAMPLES) % (AUDIO_BLOCK_SAMPLES * NUM_AUDIO_BLOCKS);
  }
}

#ifndef _BME_ROTARY_ENCODER_H_
#define _BME_ROTARY_ENCODER_H_

#include "driver/gpio.h"
#include "esp_event.h"

class RotaryEncoder
{
   private:
    gpio_num_t clkPin, dataPin, buttonPin;
    SemaphoreHandle_t lvgl_mux = NULL;
    uint8_t state = 0b1111;
    int8_t count = 0;
    int8_t pulseCount = 0;

   public:
    RotaryEncoder(gpio_num_t clkPin, gpio_num_t dataPin, gpio_num_t buttonPin);
    ~RotaryEncoder();

    void sampleEncoder();
    int buttonState();
    int8_t getAndResetCount();
};

ESP_EVENT_DECLARE_BASE(ROTARY_ENCODER_EVENTS);

enum {
    ROTARY_ENCODER_EVENT_LEFT,
    ROTARY_ENCODER_EVENT_RIGHT,
};

typedef union {
    RotaryEncoder *encoder;
} rotary_encoder_event_t;

#endif
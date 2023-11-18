#include "rotary_encoder.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ESP_EVENT_DEFINE_BASE(ROTARY_ENCODER_EVENTS);

#define TAG "rotary encoder"

#define ENCODER_CHECK(a, str, ret)                                             \
    if (!(a)) {                                                                \
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret);                                                          \
    }

IRAM_ATTR static void gpio_isr(void *args)
{
    auto *encoder = (RotaryEncoder *)args;

    encoder->sampleEncoder();
}

RotaryEncoder::RotaryEncoder(gpio_num_t clkPin, gpio_num_t dataPin,
                             gpio_num_t buttonPin)
    : clkPin(clkPin), dataPin(dataPin), buttonPin(buttonPin)
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((uint64_t)1 << clkPin) + ((uint64_t)1 << dataPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((uint64_t)1 << buttonPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    this->lvgl_mux = xSemaphoreCreateRecursiveMutex();

    gpio_isr_handler_add(clkPin, gpio_isr, this);
    gpio_isr_handler_add(dataPin, gpio_isr, this);
    gpio_intr_enable(clkPin);
    gpio_intr_enable(dataPin);

    esp_event_handler_instance_register(
        ROTARY_ENCODER_EVENTS, ESP_EVENT_ANY_ID,
        [](void *event_handler_arg, esp_event_base_t event_base,
           int32_t event_id, void *event_data) {
            auto *event = (rotary_encoder_event_t *)event_data;
            RotaryEncoder *thisEncoder = (RotaryEncoder *)event_handler_arg;

            if (event->encoder == thisEncoder) {
                if (event_id == ROTARY_ENCODER_EVENT_LEFT) {
                    xSemaphoreTakeRecursive(thisEncoder->lvgl_mux, -1);
                    thisEncoder->count--;
                    xSemaphoreGiveRecursive(thisEncoder->lvgl_mux);
                } else if (event_id == ROTARY_ENCODER_EVENT_RIGHT) {
                    xSemaphoreTakeRecursive(thisEncoder->lvgl_mux, -1);
                    thisEncoder->count++;
                    xSemaphoreGiveRecursive(thisEncoder->lvgl_mux);
                }
            }
        },
        this, nullptr);
}

RotaryEncoder::~RotaryEncoder()
{
    gpio_intr_disable(clkPin);
    gpio_intr_disable(dataPin);
    gpio_isr_handler_remove(this->clkPin);
    gpio_isr_handler_remove(this->dataPin);
}

int RotaryEncoder::buttonState()
{
    return gpio_get_level(this->buttonPin);
}

int8_t RotaryEncoder::getAndResetCount()
{
    int8_t result;
    xSemaphoreTakeRecursive(this->lvgl_mux, -1);
    result = this->count;
    this->count = 0;
    xSemaphoreGiveRecursive(this->lvgl_mux);

    return result;
}

void RotaryEncoder::sampleEncoder()
{
    uint8_t state = this->state;

    state = (state << 2) + gpio_get_level(this->clkPin) +
            gpio_get_level(this->dataPin) * 2;

    state &= 0x0f;

    this->state = state;

    switch (state) {
        case 13:
        case 2:
        case 4:
        case 11:
            this->pulseCount++;
            break;
        case 14:
        case 8:
        case 1:
        case 7:
            this->pulseCount--;
            break;
    }

    if ((state & 3) == 3) {
        if (this->pulseCount >= 3) {
            rotary_encoder_event_t event = {.encoder = this};

            ESP_ERROR_CHECK(esp_event_isr_post(ROTARY_ENCODER_EVENTS,
                                               ROTARY_ENCODER_EVENT_RIGHT,
                                               &event, sizeof(event), nullptr));
        } else if (this->pulseCount <= -3) {
            rotary_encoder_event_t event = {.encoder = this};

            ESP_ERROR_CHECK(esp_event_isr_post(ROTARY_ENCODER_EVENTS,
                                               ROTARY_ENCODER_EVENT_LEFT,
                                               &event, sizeof(event), nullptr));
        }
        this->pulseCount = 0;
    }
}

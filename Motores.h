#pragma once

#include <Arduino.h>
#include "driver/ledc.h"
#include "Seguidor.h"

namespace motores {

// Reserva explicitamente HIGH_SPEED / timer 2 para os motores.
// A camera usa LOW_SPEED / timer 0 / canal 0.
// Nao use analogWrite neste projeto.
constexpr ledc_mode_t MODE = LEDC_HIGH_SPEED_MODE;
constexpr ledc_timer_t TIMER = LEDC_TIMER_2;

const int pins[4] = {
    cfg::IN1,
    cfg::IN2,
    cfg::IN3,
    cfg::IN4
};

const ledc_channel_t channels[4] = {
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7
};

struct Mailbox {
  int left = 0;
  int right = 0;

  uint32_t at = 0;
  uint32_t generation = 0;
  uint32_t frameAt = 0;
  uint32_t holdMs = 0;

  bool fault = false;
  bool pulseOn = false;

  int actualL = 0;
  int actualR = 0;

  seg::Pace pace = seg::Pace::CONTINUOUS;
};

static Mailbox box;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t taskHandle = nullptr;

inline Mailbox snapshot() {
  portENTER_CRITICAL(&mux);
  Mailbox copy = box;
  portEXIT_CRITICAL(&mux);

  return copy;
}

inline void command(
    int left,
    int right,
    seg::Pace pace = seg::Pace::CONTINUOUS,
    uint32_t frameAt = 0,
    uint32_t holdMs = 0
) {
  portENTER_CRITICAL(&mux);

  box.left = (int)seg::limit(left, -cfg::PWM_MAX, cfg::PWM_MAX);
  box.right = (int)seg::limit(right, -cfg::PWM_MAX, cfg::PWM_MAX);
  box.at = millis();
  box.pace = pace;
  box.frameAt = frameAt;
  box.holdMs = holdMs;
  ++box.generation;

  portEXIT_CRITICAL(&mux);
}

inline void fault() {
  portENTER_CRITICAL(&mux);

  box.fault = true;
  box.left = 0;
  box.right = 0;

  portEXIT_CRITICAL(&mux);
}

inline void clear() {
  portENTER_CRITICAL(&mux);

  box.left = 0;
  box.right = 0;
  box.at = millis();
  box.fault = false;

  portEXIT_CRITICAL(&mux);
}

inline void duty(int index, int value) {
  ledc_set_duty(MODE, channels[index], value);
  ledc_update_duty(MODE, channels[index]);
}

inline void side(int a, int b, int value) {
  // Sempre desliga a entrada oposta antes de habilitar um novo sentido.
  if (value > 0) {
    duty(b, 0);
    duty(a, value);
  }
  else if (value < 0) {
    duty(a, 0);
    duty(b, -value);
  }
  else {
    duty(a, 0);
    duty(b, 0);
  }
}

inline void worker(void *) {
  seg::MotorRamp leftRamp;
  seg::MotorRamp rightRamp;
  seg::PulseGate pulse;

  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    const uint32_t now = millis();
    Mailbox current = snapshot();

    const bool expired =
        (current.left || current.right) &&
        seg::elapsed(now, current.at) > cfg::COMANDO_EXPIRA_MS;

    if (expired) {
      fault();
      current.fault = true;
    }

    bool enabled =
        !current.fault &&
        seg::commandWindowOpen(
            seg::elapsed(now, current.at),
            current.holdMs
        );

    if (seg::usesPulses(current.pace)) {
      enabled = pulse.tick(
          now,
          current.generation,
          seg::onTime(current.pace),
          seg::offTime(current.pace),
          enabled && (current.left || current.right),
          seg::elapsed(now, current.frameAt)
      );
    }
    else {
      pulse.reset();
    }

    const seg::PwmDrive targets = {
        enabled ? current.left : 0,
        enabled ? current.right : 0
    };

    const seg::PwmDrive kick = seg::startupForDrive(targets);

    const int left = leftRamp.tick(
        targets.left,
        now,
        current.fault,
        kick.left
    );

    const int right = rightRamp.tick(
        targets.right,
        now,
        current.fault,
        kick.right
    );

    side(0, 1, cfg::INVERTER_DIREITA ? -right : right);
    side(2, 3, cfg::INVERTER_ESQUERDA ? -left : left);

    portENTER_CRITICAL(&mux);
    box.actualL = left;
    box.actualR = right;
    box.pulseOn = enabled && (left || right);
    portEXIT_CRITICAL(&mux);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

inline bool begin() {
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  ledc_timer_config_t timer = {};
  timer.speed_mode = MODE;
  timer.timer_num = TIMER;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.freq_hz = cfg::PWM_HZ;
  timer.clk_cfg = LEDC_AUTO_CLK;

  if (ledc_timer_config(&timer) != ESP_OK) {
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    ledc_channel_config_t channel = {};
    channel.gpio_num = pins[i];
    channel.speed_mode = MODE;
    channel.channel = channels[i];
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = TIMER;
    channel.duty = 0;
    channel.hpoint = 0;

    if (ledc_channel_config(&channel) != ESP_OK) {
      for (int j = 0; j < i; ++j) {
        ledc_stop(MODE, channels[j], 0);
      }

      return false;
    }
  }

  clear();

  return xTaskCreatePinnedToCore(
      worker,
      "motores",
      3072,
      nullptr,
      4,
      &taskHandle,
      1
  ) == pdPASS;
}

}  // namespace motores

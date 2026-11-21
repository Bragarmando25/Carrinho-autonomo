#pragma once

#include <stdint.h>
#include "Config.h"

namespace partida {

enum class State {
  IDLE,
  LOOKING,
  WAITING,
  STARTED
};

// Cronometro independente do hardware.
// Nao bloqueia a captura da camera nem o controle dos motores.
class StartGate {
 private:
  State state_ = State::IDLE;

  uint32_t since_ = 0;
  uint32_t lastFrame_ = 0;
  uint32_t attempt_ = 0;

  int aligned_ = 0;
  bool hasFrame_ = false;

 public:
  State state() const {
    return state_;
  }

  uint32_t attempt() const {
    return attempt_;
  }

  int alignedFrames() const {
    return aligned_;
  }

  bool active() const {
    return state_ == State::LOOKING || state_ == State::WAITING;
  }

  // Sem historico ao ligar: observa parado; nao faz varredura motorizada.
  int searchDirection(uint32_t) const {
    return 0;
  }

  void cancel() {
    *this = StartGate();
  }

  void begin(uint32_t now) {
    cancel();

    state_ = State::LOOKING;
    since_ = now;
    attempt_ = 1;
  }

  uint32_t remainingMs(uint32_t now) const {
    uint32_t duration = 0;

    if (state_ == State::LOOKING) {
      duration = cfg::BUSCA_PARTIDA_MS;
    }
    else if (state_ == State::WAITING) {
      duration = cfg::PAUSA_TENTATIVA_MS;
    }

    const uint32_t elapsedMs = now - since_;

    return elapsedMs < duration
        ? duration - elapsedMs
        : 0;
  }

  bool observe(uint32_t now, uint32_t frameAt, bool aligned) {
    if (state_ == State::WAITING) {
      if (now - since_ >= cfg::PAUSA_TENTATIVA_MS) {
        state_ = State::LOOKING;
        since_ = now;
        ++attempt_;
        aligned_ = 0;
        hasFrame_ = false;
      }

      // Um quadro da pausa nao conta como procura da proxima tentativa.
      return false;
    }

    if (state_ != State::LOOKING) {
      return false;
    }

    if (now - since_ >= cfg::BUSCA_PARTIDA_MS) {
      state_ = State::WAITING;
      since_ = now;
      aligned_ = 0;
      hasFrame_ = false;

      return false;
    }

    // As comparacoes abaixo continuam corretas mesmo se millis() der overflow.
    if ((int32_t)(frameAt - since_) < 0 ||
        (int32_t)(frameAt - now) > 0) {
      return false;
    }

    if (hasFrame_ && (int32_t)(frameAt - lastFrame_) <= 0) {
      return false;
    }

    lastFrame_ = frameAt;
    hasFrame_ = true;
    aligned_ = aligned ? aligned_ + 1 : 0;

    if (aligned_ >= cfg::QUADROS_PARTIDA) {
      state_ = State::STARTED;
      return true;
    }

    return false;
  }
};

static_assert(
    cfg::BUSCA_PARTIDA_MS > 0 &&
    cfg::BUSCA_PARTIDA_MS < 0x80000000u,
    "Janela inicial invalida"
);

static_assert(
    cfg::PAUSA_TENTATIVA_MS > 0 &&
    cfg::PAUSA_TENTATIVA_MS < 0x80000000u,
    "Pausa inicial invalida"
);

static_assert(
    cfg::QUADROS_PARTIDA > 0,
    "Confirme pelo menos um quadro antes de partir"
);

}  // namespace partida

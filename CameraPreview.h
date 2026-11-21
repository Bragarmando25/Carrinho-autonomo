#pragma once

#include <stdint.h>
#include "Config.h"

namespace preview {

enum class Frame {
  USE,
  DROP,
  FAULT
};

// A transmissao PGM pode deixar uma imagem antiga no buffer unico da camera.
// Em bancada e com o carrinho parado, permite descartar no maximo um frame
// depois de uma transmissao de preview.
class Recovery {
 private:
  bool afterPreview = false;

 public:
  void sent() {
    afterPreview = true;
  }

  Frame inspect(
      bool layoutValid,
      int64_t frameUs,
      int64_t nowUs,
      bool parked
  ) {
    const bool mayDrop = afterPreview && parked;
    afterPreview = false;

    if (!layoutValid || frameUs < 0 || frameUs > nowUs) {
      return Frame::FAULT;
    }

    const int64_t maxAgeUs =
        (int64_t)cfg::FRAME_MAX_IDADE_MS * 1000;

    if (nowUs - frameUs <= maxAgeUs) {
      return Frame::USE;
    }

    return mayDrop
        ? Frame::DROP
        : Frame::FAULT;
  }
};

}  // namespace preview

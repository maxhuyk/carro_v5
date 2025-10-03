#pragma once
#include <Arduino.h>

namespace utils {

// Placeholder de media móvil simple
class MediaMovil {
public:
    bool begin(size_t ventana) {
        ventana_ = ventana == 0 ? 1 : ventana;
        valores_.resize(ventana_, 0.0f);
        idx_ = 0; llenos_ = 0; suma_ = 0.0f;
        return true;
    }

    float actualizar(float v) {
        if (valores_.empty()) return v;
        suma_ -= valores_[idx_];
        valores_[idx_] = v;
        suma_ += v;
        idx_ = (idx_ + 1) % ventana_;
        if (llenos_ < ventana_) llenos_++;
        return suma_ / (float)llenos_;
    }

private:
    size_t ventana_ = 1;
    size_t idx_ = 0;
    size_t llenos_ = 0;
    float suma_ = 0.0f;
    std::vector<float> valores_;
};

} // namespace utils

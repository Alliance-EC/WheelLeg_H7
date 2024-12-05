#pragma once
#include "string.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>
namespace tool::filter {
struct band_stop_filter {
    double filter_signal[5];
    double signal[5];
    double a[5];
    double b[5];
};

class BandStopFilter {
public:
    explicit BandStopFilter(
        uint32_t low_cutoff_freq, uint32_t high_cutoff_freq, uint32_t fs, uint32_t order = 3) {
        // switch (order) {
        // case 3: BandStopFilter_init_5rd_order(low_cutoff_freq, high_cutoff_freq,
        // fs); break; default:
        //     // Handle other orders if needed
        //     break;
        // }
        BandStopFilter_init_5rd_order(low_cutoff_freq, high_cutoff_freq, fs);
    }

    double update(double x) {
        if (std::isnan(x) || std::isinf(x)) {
            return 0;
        }
        // 更新原始信号
        _Czh.signal[4] = _Czh.signal[3];
        _Czh.signal[3] = _Czh.signal[2];
        _Czh.signal[2] = _Czh.signal[1];
        _Czh.signal[1] = _Czh.signal[0];
        _Czh.signal[0] = x;
        // 更新滤波信号
        _Czh.filter_signal[4] = _Czh.filter_signal[3];
        _Czh.filter_signal[3] = _Czh.filter_signal[2];
        _Czh.filter_signal[2] = _Czh.filter_signal[1];
        _Czh.filter_signal[1] = _Czh.filter_signal[0];
        _Czh.filter_signal[0] =
            _Czh.b[0] * _Czh.signal[0] + _Czh.b[1] * _Czh.signal[1] + _Czh.b[2] * _Czh.signal[2]
            + _Czh.b[3] * _Czh.signal[3] + _Czh.b[4] * _Czh.signal[4]
            - _Czh.a[1] * _Czh.filter_signal[1] - _Czh.a[2] * _Czh.filter_signal[2]
            - _Czh.a[3] * _Czh.filter_signal[3] - _Czh.a[4] * _Czh.filter_signal[4];
        // std::cout << _Czh.filter_signal[0] << std::endl;
        if (_Czh.filter_signal[0] < -15.0 || _Czh.filter_signal[0] > 15.0) {
            return 0;
        } else {
            return _Czh.filter_signal[0];
        }
    }

private:
    void BandStopFilter_init_5rd_order(
        uint32_t low_cutoff_freq, uint32_t high_cutoff_freq, uint32_t fs) {
        _Czh.a[0] = 1;
        _Czh.a[1] = -3.83173424960587;
        _Czh.a[2] = 5.54963588823307;
        _Czh.a[3] = -3.60044303496303;
        _Czh.a[4] = 0.88302608655344;

        _Czh.b[0] = 0.939692914565668;
        _Czh.b[1] = -3.71608864228446;
        _Czh.b[2] = 5.55327614565519;
        _Czh.b[3] = -3.71608864228446;
        _Czh.b[4] = 0.939692914565667;
        // memset(_Czh.filter_signal, 0, sizeof(_Czh.filter_signal));
        // memset(_Czh.signal, 0, sizeof(_Czh.signal));
    }
    band_stop_filter _Czh;
};
} // namespace tool::filter
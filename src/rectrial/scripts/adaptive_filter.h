// adaptive_filter.h

#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <vector>
#include <complex>

class AdaptiveFilter
{
public:
    /**
     * @brief Constructor for the adaptive FIR notch filter.
     * @param Fs Sampling frequency in Hz.
     * @param N Filter length (number of taps).
     * @param freqs A vector of frequencies (in Hz) to suppress.
     */
    AdaptiveFilter(double Fs, int N, const std::vector<double>& freqs);

    /**
     * @brief Processes a single new input sample and returns the filtered output.
     * @param xNew The new input sample.
     * @return The filtered output sample.
     */
    double measurement(double xNew);

private:
    // Filter parameters
    double m_Fs;
    int m_N;
    std::vector<double> m_freqs;

    // Internal state
    std::vector<std::complex<double>> m_weights; // FIR filter weights
    std::vector<double> m_xBuffer;               // Input buffer of the last N samples
};

#endif // ADAPTIVE_FILTER_H

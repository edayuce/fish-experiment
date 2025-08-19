// adaptive_filter.cpp

#include "adaptive_filter.h"
#include <stdexcept>
#include <numeric> // For std::inner_product


AdaptiveFilter::AdaptiveFilter(double Fs, int N, const std::vector<double>& freqs)
{
    // --- 1. Validate inputs ---
    if (Fs <= 0) throw std::invalid_argument("Sampling frequency (Fs) must be positive.");
    if (N < 1) throw std::invalid_argument("Filter length (N) must be at least 1.");
    
    // --- 2. Assign parameters ---
    m_Fs = Fs;
    m_N = N;
    m_freqs = freqs;
    m_weights.resize(m_N);
    m_xBuffer.assign(m_N, 0.0); // Initialize buffer with zeros

    // <<< FIX: Define the imaginary unit 'i' in a C++11 compatible way.
    const std::complex<double> i(0.0, 1.0);

    // --- 3. Calculate initial prototype filter weights (frequency-sampling method) ---
    for (int n = 0; n < m_N; ++n) {
        if (n == 0) {
            m_weights[n] = m_N;
        } else {
            std::complex<double> numerator = 1.0 - exp(-i * 2.0 * M_PI * static_cast<double>(n));
            std::complex<double> denominator = 1.0 - exp(-i * 2.0 * M_PI * (static_cast<double>(n) / m_N));
            m_weights[n] = numerator / denominator;
        }
    }

    // --- 4. Create notches by subtracting complex sinusoids ---
    for (double freq_to_suppress : m_freqs) {
        double k = freq_to_suppress / (m_Fs / m_N);

        for (int n = 0; n < m_N; ++n) {
            // <<< FIX: Use the C++11 compatible imaginary unit 'i'.
            m_weights[n] -= exp(-i * 2.0 * M_PI * k * (static_cast<double>(n) / m_N));
            m_weights[n] -= exp(-i * 2.0 * M_PI * (static_cast<double>(m_N) - k) * (static_cast<double>(n) / m_N));
        }
    }

    // --- 5. Normalize the final weights ---
    for (int n = 0; n < m_N; ++n) {
        m_weights[n] /= m_N;
    }
}

double AdaptiveFilter::measurement(double xNew)
{
    // --- 1. Shift the buffer and insert the new sample at the front ---
    for (int i = m_N - 1; i > 0; --i) {
        m_xBuffer[i] = m_xBuffer[i-1];
    }
    m_xBuffer[0] = xNew;

    // --- 2. Apply the FIR filter by calculating the dot product ---
    std::complex<double> y_complex = 0.0;
    for(int i = 0; i < m_N; ++i) {
        y_complex += m_weights[i] * m_xBuffer[i];
    }

    return y_complex.real();
}

void AdaptiveFilter::primeBuffer(double value)
{
    // This function fills the entire internal buffer with a single, constant value.
    // This "warms up" the filter so its output is immediately stable and meaningful.
    for (int i = 0; i < m_N; ++i) {
        m_xBuffer[i] = value;
    }
}

void AdaptiveFilter::updateFrequencies(const std::vector<double>& new_freqs)
{
    // Store the new frequencies
    m_freqs = new_freqs;

    // This logic is identical to the constructor. We are recalculating the
    // filter's properties based on the new frequency list.

    const std::complex<double> i(0.0, 1.0);

    // 1. Start with the base low-pass filter prototype
    for (int n = 0; n < m_N; ++n) {
        if (n == 0) {
            m_weights[n] = m_N;
        } else {
            std::complex<double> numerator = 1.0 - exp(-i * 2.0 * M_PI * static_cast<double>(n));
            std::complex<double> denominator = 1.0 - exp(-i * 2.0 * M_PI * (static_cast<double>(n) / m_N));
            m_weights[n] = numerator / denominator;
        }
    }

    // 2. Create the new notches by subtracting sinusoids for each new frequency
    for (double freq_to_suppress : m_freqs) {
        double k = freq_to_suppress / (m_Fs / m_N);
        for (int n = 0; n < m_N; ++n) {
            m_weights[n] -= exp(-i * 2.0 * M_PI * k * (static_cast<double>(n) / m_N));
            m_weights[n] -= exp(-i * 2.0 * M_PI * (static_cast<double>(m_N) - k) * (static_cast<double>(n) / m_N));
        }
    }

    // 3. Normalize the final weights
    for (int n = 0; n < m_N; ++n) {
        m_weights[n] /= m_N;
    }
}

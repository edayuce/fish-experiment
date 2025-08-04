// adaptive_filter.cpp

#include "adaptive_filter.h"
#include <stdexcept>
#include <numeric> // For std::inner_product
#include <complex>

// Define the imaginary unit 'i' for convenience
using namespace std;

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

    // --- 3. Calculate initial prototype filter weights (frequency-sampling method) ---
    // This creates a low-pass filter prototype.
    for (int n = 0; n < m_N; ++n) {
        if (n == 0) {
            // Handle the DC component separately to avoid division by zero
            m_weights[n] = m_N;
        } else {
            std::complex<double> numerator = 1.0 - exp(-1.0i * 2.0 * M_PI * static_cast<double>(n));
            std::complex<double> denominator = 1.0 - exp(-1.0i * 2.0 * M_PI * (static_cast<double>(n) / m_N));
            m_weights[n] = numerator / denominator;
        }
    }

    // --- 4. Create notches by subtracting complex sinusoids ---
    // For each frequency to suppress, we remove its component from the filter's response.
    for (double freq_to_suppress : m_freqs) {
        // Convert target frequency to normalized FFT bin index
        double k = freq_to_suppress / (m_Fs / m_N);

        for (int n = 0; n < m_N; ++n) {
            // Subtract the sinusoid at the target frequency
            m_weights[n] -= exp(-1.0i * 2.0 * M_PI * k * (static_cast<double>(n) / m_N));
            // Subtract its complex conjugate to keep the filter real-valued
            m_weights[n] -= exp(-1.0i * 2.0 * M_PI * (static_cast<double>(m_N) - k) * (static_cast<double>(n) / m_N));
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
    // This creates a sliding window of the most recent N samples.
    for (int i = m_N - 1; i > 0; --i) {
        m_xBuffer[i] = m_xBuffer[i-1];
    }
    m_xBuffer[0] = xNew;

    // --- 2. Apply the FIR filter by calculating the dot product ---
    // y = sum(weights[i] * buffer[i])
    std::complex<double> y_complex = 0.0;
    for(int i = 0; i < m_N; ++i) {
        y_complex += m_weights[i] * m_xBuffer[i];
    }

    // Since the input is real and the filter is designed to have a real impulse response,
    // the output should also be real. We return the real part.
    return y_complex.real();
}

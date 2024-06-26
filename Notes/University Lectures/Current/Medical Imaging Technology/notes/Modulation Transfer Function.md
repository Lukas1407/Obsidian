- The MTF quantifies the system’s ability to transmit different spatial frequencies.
- It is defined as the **ratio of the image contrast to the target contrast**, expressed as a function of spatial frequency: $$MTF(u) = \frac{C’(u)}{C(u)}$$
    - $C(u)$ represents the contrast in the target.
    - $C’(u)$ represents the corresponding contrast in the image.
- The MTF curve shows how well the system preserves spatial details at different frequencies.
- Higher MTF values indicate better resolution and contrast preservation.

## Calculation
- To obtain the MTF from the [[Point or Line Spread Function]]:
    - Take the **Fourier Transform** of the LSF to yield the MTF.
    - The MTF measures how spatial frequencies pass through the system.
    - It provides valuable information about the system’s imaging capabilities.
## Ideal Case
- In an ideal imaging system, the MTF would be constant across all spatial frequencies.
- This means that the system would transmit all spatial frequencies with the same efficiency, preserving the contrast of the original scene perfectly.
- The modulation, or contrast, of the output signal would not depend on the frequency; high-frequency details (fine details) and low-frequency details (broad features) would be equally well represented.
## Real Case
- In real imaging systems, the MTF varies with spatial frequency.
- Typically, the MTF starts high at low frequencies (good contrast for broad features) and decreases as the frequency increases (contrast for fine details diminishes).
- This decrease in MTF at higher frequencies is due to various factors such as lens aberrations, diffraction limits, sensor noise, and processing algorithms.
- The modulation of the output signal, therefore, depends on the frequency: as the spatial frequency increases, the ability of the system to preserve contrast decreases
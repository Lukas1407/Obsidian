> [!abstract] Definition
> Image noise refers to variations in pixel intensity. It affects the clarity of structures in the image.
Higher noise levels reduce the ability to perceive low-contrast structures.

^83b914


![[Pasted image 20240510084859.png#invert|400]]
- The left image is uniform, where every pixel takes on the same value (127) 
- In the image on the right, the value of each pixel has been randomly taken from a Gaussian distribution, where the mean value is 127, and the standard deviation is 20
### Gaussian Noise
- This type of noise arises during image acquisition <mark style="background: #FFB86CA6;">due to inherent sensor noise</mark>, <mark style="background: #FFB86CA6;">electronic circuitry</mark>, and <mark style="background: #FFB86CA6;">amplifier noise</mark>. 
- It follows a Gaussian distribution and is often present in dark areas of the image. 
### Salt-and-Pepper Noise
- Also known as spike noise, it appears as <mark style="background: #FFB86CA6;">dark pixels in bright regions and bright pixels in dark areas</mark>. 
- It can result from <mark style="background: #FFB86CA6;">analog-to-digital converter</mark> errors or <mark style="background: #FFB86CA6;">bit errors during transmission</mark>
### Temporal Noise (Random Noise)
- Completely random fluctuations occur <mark style="background: #FFB86CA6;">when converting incoming photons into electrons</mark>. 
- -> If you capture the same pixel multiple times, you’ll notice fluctuations in its digital value
### Spatial Noise (Pattern Noise)
- Non-uniformities between adjacent pixels cause spatial noise. 
- These variations are not random and can be observed even when the scene remains unchanged.
## Why does Noise reduce Perceived Contrast?
- Noise introduces random fluctuations in pixel values. 
- In noisy images, the difference between signal and background is blurred by this noise, making it harder to distinguish structures.
- High noise levels reduce the effective contrast, making subtle features less visible.
## Signal to Noise Ratio (SNR)
- SNR is a measure to compare the level of a desired signal to the level of background noise. -> It quantifies how much the signal stands out from the noise.
- SNR is defined as the ratio of signal to noise: $$\text{SNR}=\frac{\text{signal}}{\text{noise}}=\frac{\mu_{roi}}{\sigma_{b}}$$
- Improving SNR involves increasing signal strength, reducing noise, filtering, or using error correction techniques. 

### Scatter Noise
- Scatter noise occurs <mark style="background: #FFB86CA6;">due to interactions within the body</mark>, which <mark style="background: #FFB86CA6;">can cause photons to deviate from their original path</mark>
- This type of noise is related to radiation scatter, which is largely dependent on the energy of the beam, also known as beam quality. 
### Quantum Noise
- This noise is associated with <mark style="background: #FFB86CA6;">the number of photons that actually reach the detector</mark>. 
- It’s called quantum because it’s related to the inherent fluctuations in the number of photons detected from one point to another. 
- Even if there’s no object (absorber), quantum noise will still be present due to the random nature of photon emission and detection.
-> When the detector is exposed without an object, the resulting image will appear grainy rather than a uniform grey scale because of these fluctuations
### Poisson distribution
- The number of photons that reach a pixel on the detector follows a **Poisson distribution**. 
- For a Poisson distribution, if the average number of photons per pixel is $N$, then the standard deviation is $\sqrt{N}$.
- The probability of $k$ photons hitting a pixel is given by the formula:$$P(k) = \frac{N^k e^{-N}}{k!}$$
- The SNR is calculated as the ratio of the signal (average number of photons, $N$) to the noise (standard deviation, $\sqrt{N}$), which simplifies to:$$SNR = \frac{N}{\sqrt{N}} = \sqrt{N}$$
### Increase the SNR
- **Increase the current**: This means more photons are produced, which increases the signal.
- **Increase the exposure time**: This allows more photons to reach the detector, also increasing the signal.
However, both methods result in a higher dose to the patient, which is an important consideration in medical imaging. 
The goal is to achieve the highest SNR possible while minimizing the dose to the patient for safety reasons. 
## Contrast to Noise Ratio (CNR)
- CNR measures the contrast between the tissue of interest and the background (neighboring tissue)
- CNR is calculated by subtracting the average background signal ($S_{b}$) from the average roi signal ($S_{roi}$) and then dividing by the noise standard deviation:$$\text{CNR}=\frac{S_{roi}-S_{b}}{\sigma_{b}}$$
- A higher CNR indicates better visibility of structures and improved diagnostic accuracy.
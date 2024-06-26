- Is comprised of [[Image Resolution#Spectral Resolution|Spectral Resolution]], [[Image Resolution#Spatial Resolution|Spatial Resolution]], and [[Image Resolution#Radiometric Resolution|Radiometric Resolution]]
## Spatial Resolution
- **spatial resolution** is about <mark style="background: #FFB86CA6;">the clarity and detail in an image</mark> and <mark style="background: #FFB86CA6;">depends on the spatial sampling</mark>, which is the number of [[Pixel|pixels]] or [[Voxel|voxels]] in the Field of View (FOV).
- Key points include:
    - Smaller pixel or voxel sizes allow for better resolution of small details.
    - Poor spatial sampling can lead to **aliasing**, which manifests as jagged artifacts or moire fringes.
    - The **detector** or the **visualization screen** often limits the maximum spatial resolution achievable.
## Spectral Resolution
- **Spectral resolution** refers to a sensor’s <mark style="background: #FFB86CA6;">ability to distinguish between different frequencies of light</mark>.
- It is influenced by several factors:
    - The **frequency band** of the detector, which determines the range of wavelengths the sensor can detect.
    - The performance of the **Analog-to-Digital Converter (ADC)**, which affects how accurately the analog signal is converted into a digital one.
    - The **image reconstruction algorithms** used for spatial encoding, which can enhance or degrade the frequency content of the reconstructed image.
    - Other **instrumental limitations** specific to the imaging modality, such as noise and sensor sensitivity
## Radiometric Resolution
- **Radiometric resolution** refers to the sensitivity of a sensor to <mark style="background: #FFB86CA6;">detect slight differences in signal intensity</mark>.
- It is <mark style="background: #FFB86CA6;">determined by the quantization range</mark>, which is the number of bits used to represent the captured signal.
- Higher radiometric resolution means the sensor can distinguish more levels of intensity, resulting in finer gradations of shades in an image.
- For example, an 8-bit sensor can distinguish 256 levels of gray, while a 16-bit sensor can distinguish 65,536 levels
## Measuring Image Resolution
- [[Linear Time-Invariant (LTI) system]]
- [[Point or Line Spread Function]]
- [[Modulation Transfer Function]]
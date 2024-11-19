## CT Image Quality Factors
### Image Resolution
- Spatial resolution refers to the ability of the imaging system to resolve small, independent objects in close proximity to each other. It determines how finely details can be visualized in the image.
- In CT, spatial resolution is measured in line pairs per centimeter (lp/cm). It represents the number of line pairs that can be imaged as separate structures within one centimeter.
- There are two types of resolution in CT scanning:
	- [[Computed Tomography Image Quality#Transaxial resolution|Transaxial resolution]]
	- [[Computed Tomography Image Quality#Z-Sensitivity|Z-sensitivity]]
- Factors affecting spatial resolution include:
	- Focal spot size (smaller focal spots give higher resolution).
	- Detector size (smaller detectors give higher resolution).
	- Detector design properties (such as quarter ray detector offset).
	- Reconstruction filter (higher resolution kernels have better spatial resolution but may produce more noise).
	- Pixel size (determined by field of view and image matrix size).
### [[Image Noise|Noise]]
- ![[Image Noise#^83b914]]
- Factors influencing noise:
#### Slice Thickness
- Thin slices result in better image resolution because they capture finer details within the patient’s anatomy.
- Advantages of thin slices:
    - Improved spatial resolution: Smaller slice thickness allows better visualization of small structures.
    - Reduced scatter noise: Thinner slices encounter less scatter radiation, leading to clearer images.
    - Disadvantage:
        - More quantum noise: Thinner slices receive fewer photons, resulting in increased quantum noise (statistical fluctuations).
#### Tube Current 
- The tube current determines the number of x-ray photons produced during the scan.
- Direct proportionality: The number of photons used to create a CT image is directly proportional to the tube current.
- Increasing tube current:
    - Increases the number of photons.
    - Improves signal-to-noise ratio (SNR) by reducing quantum noise.
    - However, higher tube current also means higher radiation dose to the patient.
#### Energy (X-ray tube voltage)
- When the x-ray tube voltage (kVp) increases:
    - The number of x-rays produced also increases.
    - The relationship is nonlinear (greater than linear).
    - Doubling the tube voltage (kVp) can increase the total number of x-ray photons produced by about a factor of four (approximately proportional to kVp^2).
- Impact on noise and contrast:
    - Noise decreases due to increased photon count.
    - Relative contrast between tissues decreases because higher photon energy reduces X-ray attenuation (less differentiation between tissues).
#### Reconstruction Algorithms and Filters

### [[Image Signal#Contrast|Contrast]]:
- CT contrast refers to the difference in density between adjacent structures.
- It allows visualization of structures with varying attenuation coefficients (e.g., soft tissues, bones, and contrast-enhanced vessels).
- CT imaging is superior to conventional radiography in detecting low-contrast details. This is because CT can differentiate subtle differences in tissue densities, which may not be apparent in traditional x-ray images.
- In CT, the grey levels in an image represent different tissue densities and are quantified using [[Computed Tomography (CT)#Hounsfield scale|Hounsfield Units (HU)]], also known as CT numbers
- The grey levels in a CT image depend on the energy of the x-ray beam used. Different energies can affect the attenuation of x-rays by tissues, thus altering the contrast.
- Proper contrast enhances diagnostic information.
	- Good contrast is essential for distinguishing between different types of tissues, such as bone, soft tissue, and air. It allows radiologists to identify abnormalities and make accurate diagnoses.
#### Visualization of Contrast
- The visualization of contrast in a CT image is controlled by adjusting the window level (WL) and window width (WW).
- **Window Level (WL)**: The WL is the midpoint of the range of CT numbers displayed. Adjusting the WL can change the brightness of the image:
    - Decreasing WL makes the image brighter.
    - Increasing WL makes the image darker.
- **Window Width (WW)**: The WW determines the range of HU values that are displayed. A wider WW allows a larger range of HU values to be visualized, which can be useful for seeing both very dense and less dense structures in the same image.
### [[Image Artifacts]]
- Artifacts are unintended features in the image caused by various factors.
- Common artifacts in CT include:
	- Beam hardening: Results from differential attenuation of x-rays as they pass through different materials.
	- Metal artifacts: Caused by high-density objects (e.g., metal implants) that scatter and attenuate x-rays.
	- Motion artifacts: Arise from patient movement during scanning.
	- Ring artifacts: Due to detector malfunctions or calibration errors.
## Influencing Factors
### Beam Geometry
- The geometry of the x-ray beam affects image quality. Proper alignment and collimation are essential.
### Display Resolution
- The monitor’s resolution impacts how well the image is visualized. High-resolution displays are crucial for accurate interpretation.
### [[Image Reconstruction in Tomography|Reconstruction Algorithms]]
- The choice of reconstruction algorithm affects image quality. Different algorithms emphasize different aspects (e.g., bone, soft tissue, or vascular structures).
### Subject’s Transmissivity
- The attenuation properties of the patient (e.g., body composition, tissue density) impact image quality.
### Scatter
- Scatter radiation contributes to noise and affects image quality. Proper collimation minimizes scatter.
### Energy
- The energy of the x-ray beam influences image contrast and penetration. Higher energy beams penetrate tissues better but may increase scatter.
### Slice Thickness
- Thicker slices reduce spatial resolution but decrease noise. Thinner slices improve resolution but increase noise.

## Transaxial Resolution
- Transaxial resolution refers to the ability of the CT scanner to distinguish small structures in the transverse plane (i.e., the plane perpendicular to the patient’s body axis).
- This is the plane which we slice
![[Pasted image 20240513122100.jpg#invert|300]]
- Factors affecting transaxial resolution include:
### Focal Spot Size
- The focal spot is the area on the x-ray tube anode where electrons strike to produce x-rays. Smaller focal spots result in higher resolution because they create a narrower x-ray beam. 
- However, there’s a trade-off: smaller focal spots can handle less current before damaging the anode.
- There are usually two available focal spot sizes on CT scanners: 
	- Fine = 0.7 mm 
	- Broad = 1.2 mm
### Detector Size 
- The resolution of a CT image is partly determined by the size of the detectors. Smaller detectors can capture finer details because they can detect smaller changes in the x-ray beam’s intensity as it passes through different tissues.
- To increase resolution, you can pack more detectors into the same area. This means that for a given size of the detector array, having more, smaller detectors will improve the image resolution.
- However, each detector needs to be separated by a small amount of space, known as a partition or dead space. This space is necessary to electrically isolate each detector and to house the electronics needed for them to function.
- These partitions do not contribute to image formation and thus are considered “dead space.”
- As you increase the number of detectors to improve resolution, you also increase the amount of dead space. This can lead to a reduction in the overall detection efficiency because a larger proportion of the x-ray beam is hitting non-detecting areas (the partitions), rather than being captured by the detectors.
### Number of Projections
- The number of projections (views) acquired during a CT scan affects image quality.
- More projections lead to better resolution, but they also increase radiation dose.
### Reconstruction Algorithms and Filters
- Reconstruction algorithms convert raw projection data into an image.
- **Sharp reconstruction filters** (such as the ramp filter) emphasize high-frequency components, resulting in better spatial resolution. However, these filters do not average out noise.
- Different filters (e.g., soft tissue, bone) can be applied during reconstruction to enhance specific features.
### Matrix Size and Field of View (FOV)
- The pixel size (d) in millimeters is given by: $$d = \frac{{\text{{FOV}}}}{{\text{{Matrix Size}}}}$$
    - FOV represents the area of the patient being imaged.
    - Matrix size refers to the number of pixels in the reconstructed image (e.g., 512x512 pixels).
- To improve image resolution:
    - **Reduce the FOV**: Smaller FOV results in a smaller pixel size (d), which improves resolution.
    - **Increase the Matrix Size $n$**: Larger matrix size (more pixels) also leads to smaller pixel size $d$ and better resolution.
- The highest spatial frequency that can be obtained in the image is given by: $$f_{\text{max}} = \frac{1}{{2d}}$$
    - Smaller pixel size $d$ corresponds to higher spatial frequencies.
    - This formula comes from the Nyquist theorem, which states that to accurately capture a signal (in this case, the spatial variations in the image), you need to sample it at least twice per cycle of the highest frequency present.
	- If you decrease the pixel size $d$, you increase the value of$$\frac{1}{2d}$$

	    which means you can capture higher spatial frequencies.
		- Essentially, smaller pixels allow the CT scanner to detect finer changes in intensity across the image, leading to a higher resolution image with more detail.
## Z-Sensitivity
- This refers to resolution along the length of the patient in the z-direction (along the scan axis).
![[Pasted image 20240513122100.jpg#invert|300]]
- Factors affecting z-sensitivity 
### Collimation
- Collimation is the process of narrowing the x-ray beam to the desired slice thickness. Tighter collimation results in thinner slices, which improves z-sensitivity by reducing volume averaging (where tissues from above and below the slice are included in the image).
### Detector Size in the Z-axis
- The size of the detectors in the z-axis also affects z-sensitivity. Smaller detectors can capture finer details along the z-axis, but as mentioned earlier, smaller detectors can increase noise.
### Improving Z-Sensitivity
#### Overlapping Slices
- Acquiring data using overlapping slices can enhance z-sensitivity. This is because overlapping slices reduce the gaps between slices, allowing for better continuity and detail in the z-axis.
- Overlapping is achieved by using a low spiral pitch (pitch < 1), where the table moves slower relative to the speed of the x-ray tube rotation, resulting in more overlap of the x-ray beam paths.
#### Fine Focal Spot
- A fine focal spot size produces a sharper x-ray beam, which improves both the in-plane resolution and the z-sensitivity. This is because a finer beam can better differentiate structures along the z-axis without blurring.
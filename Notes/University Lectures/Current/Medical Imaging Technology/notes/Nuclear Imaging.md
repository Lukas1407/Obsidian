> [!abstract] Definition
> Nuclear imaging is a medical technique used to diagnose, treat, and monitor diseases. It involves the use of radioactive substances called [[Radiopharmaceuticals|radiopharmaceuticals]] or [[Radioactive Tracers|tracers]]. 
> -> Nuclear Imaging enables the imaging of metabolic and biological processes, which can massively enhance diagnostics

- Due to the continuous emission of radiation by radionuclides, patients who undergo nuclear imaging may need to be isolated in specific rooms to prevent unnecessary radiation exposure to others until the radiopharmaceutical is no longer radioactive
- [[Therapy Radiopharmaceuticals]]
- [[Diagnostic Radiopharmaceuticals]]
- The 3 imaging modalities in Nuclear Imaging are: 
	1. [[Positron Emission Tomography (PET)|PET]]
	2. [[Single Photon Emission Computed Tomography (SPECT)|SPECT]]
	3. [[Scintigraphy]]
## Differences Between Nuclear Imaging and X-ray Imaging
### Source and Detector Location
- In X-ray imaging, both the radiation source and the detector are external and their positions are known. In contrast, nuclear imaging involves introducing a radioactive tracer inside the patient’s body, making the patient the radiation source. 
- The detector’s position is known, but the source’s exact location within the body is determined by where the tracer localizes
- This is why Nuclear Imaging is known as emission-based in contrast to [[X-Ray Imaging]] which are transmission-based or [[Ultrasound Imaging]] which is reflection-based
### Duration of Radiation
- X-ray exposure is controlled and lasts only as long as the examination requires. However, the duration of radiation from radionuclides is dependent on the [[Half-Life Time]] of the radioactive compound and how quickly the body can eliminate it
### [[Photon]] Fluxes
- X-ray imaging uses relatively intense photon fluxes to create images, while nuclear imaging involves detecting much lower levels of photon fluxes. This allows for the recording of individual photons in Single Photon Emission Computed Tomography (SPECT) and the coincidence detection of two photon events in Positron Emission Tomography (PET)
## Considerations
### Emission-Based Imaging and Photon Counting
- Emission-based imaging techniques, rely on detecting photons emitted from radioactive isotopes (radiopharmaceuticals) within the body.
- The basic principle is to count the number of photons that reach the detector during a specific time interval.
- These emitted photons carry information about the distribution of radiopharmaceuticals in tissues and organs.
### Temporal Considerations:
- For functional or metabolic studies (e.g., dynamic PET scans), it is essential to acquire multiple images over time.
- In such cases, we aim to acquire approximately 10 images per second to capture dynamic processes (e.g., blood flow, glucose metabolism, receptor binding, etc.).
- -> The detector must be able to detect in 1/10 of second enough photons to build the image
### Energy Sensitivity
- The detector must be sensitive to the energy of the $\gamma$ radiation
- By being energy-sensitive, the detector can distinguish between primary emissions (directly related to the radiopharmaceutical distribution) and scatter emissions (resulting from interactions with other tissues).
- Scatter emissions can distort the image, so the detector’s sensitivity to energy helps minimize this effect.
### Isotropic Emission
- The $\gamma$ radiation emission is isotropic
- Isotropic means that the emission occurs uniformly in all directions from the radiopharmaceutical source.
- This property simplifies the imaging process because we don’t need to account for specific angles or orientations when detecting emitted photons.
## Clinical Applications
- Nuclear medicine is crucial for studying various physiological processes and diagnosing conditions. It is commonly used for assessing bone metabolism, myocardial (heart muscle) perfusion and viability, detecting lung embolisms, identifying tumors, evaluating thyroid function, and investigating neurological disorders
## Image  Quality
- Image quality in nuclear medicine is mainly measured in terms of:
	1. Spatial and temporal resolution 
	2. SNR 
	3. Temporal resolution 
	4. Artifacts
### Role of  the [[Gamma Camera#Collimator|Collimator]]
- The collimator ensures that only gamma rays traveling in certain directions reach the detector, which is crucial for creating a clear image.
- The distance $R$ represents the minimum separation at which two small sources of radioactivity can be detected as distinct from each other.
- The formula given for $R$ is: $$ R = \frac{d*(L_{eff}+z)}{L_{eff}}$$here, $d$ is a constant related to the intrinsic resolution of the detector, $z$ is the depth of the object within the body, and $L_{eff}$ is the effective length of the collimator holes, which is given by: $$L_{eff} = L - \frac{3}{\mu_{septa}}$$
![[Pasted image 20240528095207.png#invert|150]]
- The spatial resolution of an imaging system is measured as the Full Width at Half Maximum (FWHM) of the photon distribution.
- FWHM is the width of the curve at half the maximum value of the peak, which indicates how spread out the photons are at half of their maximum count rate.
- The closer the region of radioactivity is to the surface of the body, the higher the spatial resolution, meaning the image will be clearer and more detailed.
- Conversely, regions deeper in the body will have lower spatial resolution due to increased scattering and absorption of the gamma rays.
### Roll of the [[Gamma Camera#Scintillating Crystal|Scintillators]]
- The intrinsic spatial resolution of a gamma camera, denoted as $R_{\gamma}$, is a measure of the camera’s ability to define the exact location where light is produced in the scintillation crystal after gamma ray interaction.
- This resolution is influenced by the **thickness of the crystal** and the **resolution of the Anger position encoder**.
- A thicker crystal will result in a broader light spread function, leading to poorer spatial resolution. Conversely, a thinner crystal will have a narrower light spread, resulting in better resolution.
- The typical value for $R_{\gamma}$ ranges from **3 to 5 mm**.
- The overall spatial resolution of the system, $R_{system}$, takes into account both the resolution of the collimator (( R_{coll} )) and the intrinsic resolution of the gamma camera $R_{\gamma}$.
- It is calculated using the formula: $$R_{system} = \sqrt{R_{coll}^2 + R_{\gamma}^2}$$
- This formula combines the effects of the collimator and the scintillator to give a complete picture of the system’s resolution capabilities.
- To increase the SNR, the system must be capable of ignoring photons that arrive with lower energy, as these do not contribute useful information and can degrade the image quality.
- A thicker crystal can absorb more gamma rays, which increases the SNR because more interactions mean more signal relative to the background noise.
- The energy resolution of the gamma camera is defined as the Full Width at Half Maximum (FWHM) of the signal peak.
- It is typically about **10% of the nominal gamma photon energy**.
- This measure helps to distinguish between photons of different energies, which is important for accurate imaging.
### Roll of the [[Gamma Camera#Photomultiplier Tube (PMT)|PMT]]
- To increase efficiency and SNR PMTs are arranged in a close-packed array to cover the crystal surface. 
- The hexagonal cross section is preferred
### Role of the Digitization Matrix
![[Pasted image 20240528095656.png|400]]
### SNR
- Factors which affect the SNR include the following:
	- The radioactive dose administered 
	- The upper limit in the number of counts that can be recorded per unit time by the system 
	- The effectiveness of the radiotracer at targeting a specific organ 
	- The total time over which the image is acquired (limited by the biological half-life of the radiotracer). Longer time, higher SNR but worse temporal resolution and motion artifacts
$$\text{SNR}\propto \sqrt{\text{DOSE}\cdot\text{TIME}}$$
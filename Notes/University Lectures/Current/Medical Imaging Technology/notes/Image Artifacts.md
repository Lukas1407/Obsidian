- Any element that appears in the image but is not present in reality (FP)
- An element in reality that is not displayed in the image (FN)
![[Pasted image 20240510085806.png|700]]
- Understanding these artifacts is crucial for radiologists and technicians to ensure accurate diagnosis and to determine whether an abnormality is a true pathology or just an artifact. 
- In some cases, repeating the imaging with adjustments or additional testing may be necessary to clarify the nature of an observed abnormality
## Examples
1. **Aliasing**:
    - Aliasing occurs when the imaging system is unable to accurately represent a high-frequency signal due to insufficient sampling (the Nyquist theorem). For very small elements, this can result in misrepresentation or distortion. However, with modern imaging technology, aliasing is quite rare.
2. **Heel Effect**:
    - The heel effect is observed in X-ray imaging. It’s caused by the geometry of the X-ray tube, where the intensity of the X-rays is greater on the cathode side than on the anode side. This results in uneven image density, with one side appearing darker than the other.
3. **Variation of Focal Spot**:
    - The focal spot is the area on the anode of the X-ray tube where electrons hit and produce X-rays. Variations in the size of the focal spot can lead to changes in image sharpness and resolution.
4. **Motion**:
    - Patient or organ motion during image acquisition can cause blurring or streaking artifacts. This is common in MRI and CT scans where even slight movements can significantly affect image quality.
5. **Signal Dropout or Detector Mis-calibration**:
    - If part of the detector fails to capture the signal or is miscalibrated, it can result in areas of the image that are too dark or too bright. Regular calibration and maintenance of the detector are essential to prevent this.
6. **Radiopaque Elements on the Patient**:
    - Items such as jewelry or hair clips that are not removed before imaging can show up as bright spots on the image because they block X-rays. These are known as radiopaque artifacts.
7. **Overexposure**:
    - If the detector is exposed to too much radiation, it can become saturated, leading to an image that is too bright or washed out. This can obscure details and reduce the overall quality of the image.

## Motion Artifacts
![[Pasted image 20240513125006.png|200]]
- **Causes**:
    - **Patient Swallowing**: Movement of the throat during swallowing can introduce blurring.
    - **Breathing**: Involuntary chest or abdominal movement during breathing.
    - **Heart and Vessels Beating**: Cardiac motion can cause blurring in coronary CT angiography.
    - **Patient Moving**: Any patient movement during the scan.
- **Solutions**:
    - **Scan Parameters**: Optimize scan parameters (e.g., pitch, rotation time) to minimize motion effects.
	    - **Shorten Scan Time**: Faster scans reduce the chance of motion artifacts.
	    - **Spiral Scanning**: Continuous helical scanning minimizes motion during table movement.
	    - **ECG Gating**:
	        - **Prospective Gating**: Triggers image acquisition during a specific point in the cardiac cycle (e.g., end-diastole) when heart motion is minimal.
	        - **Retrospective Gating**: Reconstructs data from specific ECG phases to create motion-free images.
	- **Patient Parameters**:
		- Breath Hold: Instruct the patient to hold their breath during scanning to reduce respiratory motion.
		- Immobilization Systems: Use devices (e.g., foam pads, straps) to restrict patient movement.
## Ring Artifacts
![[Pasted image 20240513125133.jpg|200]]
- Cause:
	- Defective Detector Element: Malfunctioning detector elements can lead to ring-like patterns in the image.
	- Wrong Single Detector Calibration: Incorrect calibration of individual detectors within the array.
- Impact:
	- Ring artifacts appear as concentric circles or arcs in the reconstructed image.
	- They can degrade image quality and affect diagnostic accuracy.
- Solutions:
	- Detector Maintenance: Regularly check and maintain detector elements.
	- Calibration: Ensure proper calibration of individual detectors.
	- Artifact Reduction Algorithms: Some reconstruction algorithms can minimize ring artifacts.
## Beam Hardening
- Beam hardening occurs when a polychromatic x-ray beam (composed of photons with various energies) passes through an object, causing lower energy photons to be attenuated more than higher energy ones
- The remaining beam, now “hardened,” has a higher average energy.
### Artifact 
- The CT scanner may interpret this increased mean beam energy as passing through a less dense material, leading to an incorrect assignment of lower Hounsfield units (HU).
- As a result, areas that should have uniform density appear darker, creating artifacts.
### Cupping Artifacts
- In objects with a thick center, the beam hardens more as it passes through the middle than at the periphery.
- This results in the center being assigned lower HUs, creating a “cupped” appearance in the image.
- Correction: Modern scanners use beam hardening correction algorithms to adjust for this effect
## Photon Starvation
- Photon starvation happens when very dense materials (like metal implants) absorb most or all of the x-ray beam, preventing sufficient photons from reaching the detectors
- This leads to a higher noise level in the image because there’s a smaller signal-to-noise ratio.
### Artifact 
- The lack of photons causes streaking artifacts, which appear as dark bands between dense objects.
- Reduction: Increasing the tube current can help reduce photon starvation by providing more photons, thus decreasing noise
## Streak Artifacts
- Streak artifacts can occur due to both beam hardening and photon starvation.
- They manifest as dark streaks between high-density objects, such as metal implants or electrodes.
- These artifacts can obscure details and affect the diagnostic quality of the image.
## Partial volume artifacts
- These artifacts arise when an object of interest doesn’t fully occupy the entire thickness of the imaging slice.
- Instead, the object only partially fills the voxel, and the rest of the voxel includes other materials or air.
- The CT scanner assigns a Hounsfield Unit (HU) based on the average attenuation of all the materials within the voxel.
- If the voxel contains both high-density tissue (like bone) and low-density tissue (like muscle), the resulting HU will be an average of the two, which may not accurately represent either tissue.
- This can lead to misinterpretation, where the tissues appear less dense than they actually are.
- Partial volume artifacts can obscure true tissue borders and pathology, leading to diagnostic errors. For example, a small calcification within a vessel may be missed if it’s averaged with surrounding blood.
### Reducing
- **Smaller Slice Thickness**: Using thinner slices reduces the partial volume effect because the voxel represents a smaller volume, thus reducing the chance of averaging different tissues
- **High-Resolution Scans**: High-resolution scans with smaller voxels can also help in reducing these artifacts.
- **Overlap in Scanning**: Overlapping slices during scanning can improve z-axis resolution and reduce partial volume effects.
> [!abstract] Definition
>  PET stands for Positron Emission Tomography. It is a sophisticated medical imaging technique that utilizes radioactive substances known as [[Radioactive Tracers]] to visualize and measure changes in metabolic processes, and in other physiological activities such as blood flow, regional chemical composition, and absorption.

## How it works
### Accumulation in the Body
- Once inside the body, these radiotracers accumulate in areas of interest, such as tumors or regions of inflammation. 
- They can also bind to specific proteins within the body. 
- For instance, the [[Radioactive Tracers]] [[F-18 fluorodeoxyglucose (FDG)]] is similar to glucose and is commonly used because cancer cells, being more metabolically active, may absorb glucose at a higher rate
### [[Gamma Ray]] Emission
- A positron, that is emitted by a radiotracer, produces 2 [[Gamma Ray|gamma rays]] in opposite directions during the process of [[Annihilation]]
- Tissue with higher metabolic activity absorbs more of the radiotracer, this leads to a higher positron emission, which leads to more annihilation.
### Detection
- A special camera then detects gamma ray emissions from the radiotracers. 
	- This is similar to the [[Gamma Camera]]
	- No [[Gamma Camera#Collimator|collimator]]
	- The gamma photons interact with a [[Gamma Camera#Scintillating Crystal|scintillating crystal]] to produce a light signal
	- The **energy resolution** of a detector system refers to its ability to distinguish between different energy levels of detected radiation.
	- Ideally, a PET detector should only detect gamma photons with precisely 511 keV energy. However, due to practical limitations, some scatter occurs.
	- **Scatter gamma photons** are those that have lost some energy (less than 511 keV) due to interactions with other materials (e.g., the patient’s body or detector components).
	- If the energy resolution is not high enough, the detector may also register these scatter gamma photons. They are still scintillated in the crystal and produce a light signal, but their energy is lower than the ideal 511 keV.
	- -> the process is more efficient (less signal is lost)
- The PET scanner is designed to detect these two photons nearly simultaneously – this is known as a coincident event
	- Only if the two gamma photons are detected in a very short amount of time difference (Counting Window Period CWP), it is stated as valid and is so used for image reconstruction
### Combination with Other Imaging
- Often, PET is combined with [[Computed Tomography (CT)]] scans in a PET/CT scanner. This combination provides both metabolic and anatomic information, which can be crucial for accurate diagnosis and treatment planning
![[Pasted image 20240528085616.jpg#invert|500]]
## Instrumentation
![[Pasted image 20240529074916.png|300]]
- Looks similar to a [[Computed Tomography (CT)|ct scanner]] but the round part of the PET machine is deeper in the direction of the couch axis (where the patient lies) because of several rings of detectors.
- Unlike a CT scanner, there is **no gantry spinning around the patient** in a PET scanner -> he PET scanner itself remains stationary during the imaging process.
- The **couch** (where the patient lies) is responsible for movement.
	- It can **translate** (move) the patient inside the ring of detectors.
- Unlike a CT scanner, the PET scanner does **not have a collimator**.
### Detectors
- The PET scanner features **multiple rings of detectors** arranged in the **cranio-caudal direction** (along the axis of the patient couch)
![[Pasted image 20240529075829.png#invert|100]]

- Having many detectors leads to an **improved [[Temporal Resolution|temporal resolution]]**
	- In PET, this means accurately determining when coincident gamma photons are detected.
- Having many detectors also **increases the field of view (FOV)** of the PET scanner.
	- A larger FOV enables the scanner to capture images of **full-body scans** or larger portions of the patient.
#### Block Detector
![[Pasted image 20240529080441.png#invert|600]]
- The block detector is a coupled version of many individual detector elements.
- It consists of a large block made of scintillating crystals.
- These crystals are coupled to four PMTs (photomultipliers).
- The crystals are cut at different depths, depending on their position within the array.
- In the middle of the block, the cuts are less deep.
- At the corners, the cuts are deeper.
- The varying depths ensure that each detector element (each scintillator crystal) produces a unique combination of signal intensities in the four PMTs.
	- The depth of the crystal cut affects the **light collection efficiency** for gamma photons coming from different angles.
	- Crystals at different depths are more sensitive to gamma photons arriving from specific directions.
	- -> The varying depths allow the PET scanner to determine which part of the patient’s body emitted the gamma photons.
#### Desired Characteristics of Detectors
##### High Density and Linear Attenuation Coefficient
- A high attenuation coefficient reduces the probability that $\gamma$-photon pass the scintillator without depositing (its total) energy and potentially leaves undetected or are detected with a wrong energy value.
- These properties ensure that gamma photons from a real decay event are not missed by the crystal.
##### High Energy Resolution
- The ability of the detector to accurately determine the Energy of the incoming photon
- By being less sensitive to scatter (energies below 511 keV), the detector reduces noise, improving image quality.
- Energy resolution is expressed as a percentage of the energy peak. For instance, if the energy resolution is 10%, the detector would accept energies within a range of 51 keV above or below 511 keV, meaning between 460 and 562 keV $$511 keV \pm 10\%$$
### Scintillation Crystal
- The energy absorbed by the scintillation crystal depends on its interaction with the gamma photon.
- This interaction can result in two main effects:
    - **Photoelectric Effect**: The entire energy of the gamma photon is absorbed, and an electron is ejected from the crystal.
    - **Compton Interactions**: Part of the gamma photon’s energy is transferred to an electron, and the photon is scattered with reduced energy.
- The energy spectrum of absorbed gamma photons has a peak known as the **photopeak**.
- The photopeak corresponds to the maximum energy of the gamma photons, which is 511 keV in PET.
![[Pasted image 20240529083906.png#invert|300]]
- The energy resolution is a measure of the crystal’s ability to distinguish between different energy levels of the incoming photons.
- It is calculated using the formula:$$\text{Energy Resolution (\%)} = \frac{\text{FWHM}}{\text{Photopeak}} \times 100$$
- A lower Full Width Half Maximum (FWHM) indicates a sharper peak and better energy resolution.
#### Materials
- Usually BGO (bismuth germanate) is used in scintillator for PET
	- It has a wide energy resolution (20%), which is not too good because it has more noise, but a high density (7,13).  
- Also LSO (lutetium oxyorthosilicate) and LYSO (lutetium yttrium orthosilicate) are used. 
	- Both have a narrower energy resolution than BGO.
### Coincident Event
- Each detector in the PET scanner generates a **time pulse** when it detects a gamma photon.
- A second detector, on the opposite side of the ring, detects the other photon from the same annihilation event and also generates a time pulse.
- The time pulses from both detectors are overlaid to determine if they occurred simultaneously.
![[Pasted image 20240529081446.png#invert|500]]
- If the two peaks in the time pulse diagrams are aligned within an allowed CWT, it indicates a coincident event.
#### Field of View (FOV) and Detector Angles
- The connected detectors don't need to be exactly opposite of each other, but can be within an **Angle of Acceptance** 
![[Pasted image 20240529081950.png#invert|400]]
- This angle depends on the FOV we want to cover
- Any connection line not passing through the FOV is considered invalid for coincidence detection.
#### Line of Response (LOR)
- The **Line of Response (LOR)** is defined as the line along which the two gamma photons travel.
#### Types of Coincidence Events
![[Pasted image 20240529082358.png#invert|500]]
##### True Coincidence
- A true coincidence occurs when both gamma photons detected in a pair result from a single decay event (such as positron annihilation).
- These photons travel along the correct Line of Response (LOR), connecting the two detectors that measured them.
##### Scatter Coincidence
- In a scatter coincidence, one or both of the detected photons come from a single event but are deflected (scattered) before reaching the detectors.
- The LOR calculated based on these scattered photons is incorrect because it does not represent the true path of the original gamma photons.
- Scatter coincidences can introduce noise and reduce image quality.
- Scatter coincidence are rejected if we choose the right energy resolution (scintillator material).
##### Random Coincidence
- A random coincidence occurs when two gamma photons are detected simultaneously, but they are not from the same decay event.
- These photons are generated by different events (e.g., unrelated [[Annihilation|positron annihilations]]) and happen to coincide purely by chance.
- The LOR calculated for random coincidences is also incorrect because it connects unrelated events.
#### Discarding Coincidence Events
- **Single Photon Detection**: If only one photon is detected, it does not constitute a coincidence event. PET relies on detecting pairs of photons.
- **Multiple Photon Pairs Arriving Simultaneously**: If more than one pair of photons arrive at different modules (detector elements) simultaneously (within the coincidence time resolution), it becomes challenging to determine which pair corresponds to the same annihilation event. To avoid ambiguity, such events are discarded.
- **Two Photons Detected by the Same Module**: This condition is crucial. When two gamma photons are detected by the same module (within the scintillation decay interval), it indicates that they likely originated from the same annihilation event. However, if they are detected too close together (within the scintillation decay time), it becomes difficult to distinguish them as separate coincidences. Therefore, to maintain accuracy, these events are also discarded.
## Timing Importance in PET
- In PET, precise timing is crucial for accurate imaging.
- Two critical timing aspects are:
    - **Raising Time** $\tau_{r}$: This refers to how quickly the scintillation crystal emits photons after an energy deposit (e.g., due to gamma photon interaction).
    - **Decay Time** $\tau_{d}$: This indicates how fast the number of emitted photons decreases after reaching the peak.
![[Pasted image 20240529084751.png#invert|600]]
- $$N(t) = N_0 * e^\frac{-t}{\tau_{d}}$$describes this exponential decay, where $N(t)$ is the number of photons at time $t$, $N_0$ is the maximum number of emitted photons
### Fast Emission (Short Raising Time)
- A **fast emission** of photons ensures that the exact **time stamp** of the energy deposit is recorded in the scintillator.
- When a gamma photon interacts with the crystal, it generates a light signal (scintillation).
- Short raising time allows for precise timing of this scintillation process.
### Short Decay Time
- A **short decay time** is essential to avoid detecting multiple gamma photons from the same event.
- If the decay time were too long, multiple photons emitted during the decay process might be detected as one event.
- Determining the actual count of incident gamma photons would become difficult or impossible.
## Image Reconstruction
- The LORs are organised such that the lines that are more or less parallel are grouped 
![[Pasted image 20240529085801.jpg|500]]
- Then we can use [[Analytical Methods for Image Reconstruction in Tomography#Back Projection|Back Projection]] to reconstruct the image
## Time of Flight PET (TOF-PET)
- TOF PET is an advanced technique in PET imaging that significantly improves spatial resolution and image quality by incorporating precise timing information related to detected gamma photons.
### Traditional PET vs. TOF PET
- In traditional PET, image reconstruction relies on detecting pairs of gamma photons emitted from positron annihilation events.
- However, TOF PET takes advantage of the **time difference** between the arrival of these two photons.
- Recent detectors can measure this time difference with high precision (less than 0.5 ns).
![[Pasted image 20240529090725.png#invert|600]]
### Localization Using Time Difference
- When an annihilation event occurs, the two photons travel a distance at the speed of light.
- If the annihilation occurs at the midpoint between detectors, both photons will reach the detectors simultaneously.
- However, if annihilation occurs closer to one detector, one photon will arrive faster than the other.
- This time difference ($\Delta t$) can be used to **localize** where along the Line of Response (LOR) the annihilation occurred.
- The localization accuracy ($\Delta x$) in TOF PET is proportional to the time-of-flight (TOF) difference ($\Delta t$).
- The equation relating them is:$$t_{1}-t_{2}=\Delta t = \frac{2x}{c}\rightarrow\Delta x = c \cdot \frac{\Delta t}{2}$$
    - Where:
        - ∆x is the uncertainty in position along the LOR.
        - c is the speed of light (approximately 300 * 10^6 m/s).
        - ∆t is the time difference between photon arrivals.
- The time difference between the arrival of the two annihilation photons is used to create a probability distribution locating the recorded event
### Advantages
- If the TOF information is used in the image reconstruction, it can be shown that this will result in a gain in SNR in the reconstructed image.
## Image Quality
- PET imaging tends to have a lot of **noise** due to nuclear medicine methods.
- The **temporal resolution** is low because PET aims to image metabolic processes, which are linked to motion (resulting in motion artifacts).
### Spatial Resolution
- The spatial resolution in PET depends on several factors:
    - **Scintillator Type**: The material used in the scintillation crystal affects resolution.
    - **Detector Size**: The individual detector crystals are typically about 4 mm x 4 mm, limiting the overall detector resolution to about 2-3 mm.
    - **Reconstruction Algorithm**: The image reconstruction method impacts spatial resolution.
    - **Space Covered by the Positron**: The extent of positron annihilation contributes to resolution.
- The empirical expression to calculate the spatial resolution of PET is: $$R_{FWHM} = 1.25\sqrt{ \left( \frac{d}{2} \right)^2 + (0.0044R)^2 + s^2 + b^2 + \frac{(12.5r)2}{r^2 + R^2}}$$where:
	- $R_{FWHM}$ represents the full width at half maximum of the point spread function (PSF) of compact radioactive sources (spatial resolution).
	- $d$ is the size of the scintillation crystal.
	- $R$ is the radius of the PET scanner ring.
	- $s$ represents the positron annihilation stroke related to the isotope type.
	- $b$ accounts for error in crystal decoding.
	- $r$ is the distance from the Line of Response (LOR) to the center of the ring
### Attenuation Correction (AC)
- Without this correction, the reconstructed image can be distorted, affecting the accuracy of activity visualization.
- The impact of AC is evident when comparing images with and without correction.
![[Pasted image 20240529092121.png|400]]
- Each Line of Response (LOR) must account for the amount of attenuation it encounters.
- Areas deeper within the object or surrounded by dense structures experience more attenuation.
- AC involves applying a correction factor to the reconstructed PET signal.
- This factor can be determined using either:
#### Calculated AC
- Assumes a uniform attenuation coefficient throughout the imaged object. Mathematical models are used.
- Consider an example:
    - A positron-emitting source is located at depth $x$ within a medium.
    - The attenuation coefficient is (\mu), and the object has thickness $D$
    - $p_1$ represents the probability that photon 1 escapes the object without interaction.
    - $p_2$ is similar for photon 2.
    - The joint probability for a coincidence event is $$p_{coinc}=p_{1}p_{2}=e^{-\mu x}e^{-\mu(D-x)}=e^{-\mu D}$$
- The attenuation correction factor is given by the reciprocal of the joint probability: $$\text{Attenuation Correction Factor} = \frac{1}{p_{coinc}}$$ 
#### Measured AC
- Accounts for heterogeneous attenuation. Direct measurements are taken.
- It requires knowing the distribution of attenuation coefficients within the object.

## Exercise
![[Pasted image 20240529092843.png#invert|600]]
![[Pasted image 20240529092851.png#invert|600]]
![[Pasted image 20240529092901.png#invert|600]]
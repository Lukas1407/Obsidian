## Interactions with Matter
When electromagnetic (EM) radiation is in the range of X-ray wavelengths and energy, it has several unique interactions with matter due to its high energy and short wavelength. Here’s a detailed explanation of each point:
### Interactions at and Atomic Level
- X-rays have enough energy to interact with individual atoms. 
- They can interact with the inner shell electrons, which are tightly bound to the nucleus. 
- This interaction can cause various effects, such as the <mark style="background: #FFB86CA6;">ejection of electrons (ionization)</mark> or the <mark style="background: #FFB86CA6;">excitation of electrons to higher energy levels</mark>.
### Ionizing the Matter
- The energy of X-rays is sufficient to overcome the binding energy of electrons in an atom, thus removing them and creating ions. 
- This process is known as ionization. 
- Ionizing radiation, like X-rays, can break chemical bonds and cause changes in the atomic structure, which is why it’s used in medical imaging and cancer treatment.
### Penetrating the Matter
- Due to their high energy, X-rays can penetrate materials that are opaque to visible light. 
- Different materials will attenuate X-rays to varying degrees based on their density and atomic number. 
- This property allows X-rays to pass through soft tissues in the body but be absorbed or scattered by denser materials like bone, creating contrast in X-ray images.
### Attenuate Them Through the Matter
  - As X-rays travel through matter, their intensity decreases due to absorption and scattering. 
  - This process is known as attenuation. 
  - The degree of attenuation depends on the material’s properties, such as thickness, density, and atomic number. 
## Improvements over time
![[Pasted image 20240510090050.png|700]]
-> Correlates to ![[Medical Imaging Technology#^f54a09]]

## ![[X-Ray Tube]]
## X-ray production
### Beam Quality
- Refers to the ability of the X-ray beam to <mark style="background: #FFB86CA6;">penetrate an object or the energy of the beam</mark>. 
- It characterizes how well the X-rays can penetrate tissues and structures within the body. 
- We often describe beam quality by considering the <mark style="background: #FFB86CA6;">average energy of the X-ray spectrum</mark>.
### Beam Quantity
- Refers to the <mark style="background: #FFB86CA6;">number of X-ray photons in the beam</mark>. 
- It represents the total intensity of X-rays produced and is related to the <mark style="background: #FFB86CA6;">area under the X-ray spectrum curve</mark>.
### Manipulating the X-Ray spectrum
- To achieve desired X-ray characteristics, we can manipulate the X-ray spectrum using the following factors:
#### Increase the Tube Potential (kV):
- **Effect on Quantity**:
    - Increasing the tube potential (kV) leads to an <mark style="background: #FFB86CA6;">increase in the quantity of X-ray photons</mark> produced.
    - Higher kV results in <mark style="background: #FFB86CA6;">more electrons being accelerated</mark> across the X-ray tube, leading to increased X-ray production.
- **Effect on Energy**:
    - The <mark style="background: #FFB86CA6;">average energy of the X-ray spectrum also increases</mark> with higher kV.
    - Additionally, <mark style="background: #FFB86CA6;">the maximum energy of the X-ray photons (the peak of the spectrum) increases</mark>.
    - If the kV is sufficiently high, more **characteristic energy** X-rays are produced.
#### Increasing the Tube Current (mA):
- **Effect on Quantity**:
    - Increasing the tube current (mA) results in an <mark style="background: #FFB86CA6;">increased quantity of X-ray photons</mark>
    - Tube current controls the number of electrons passing through the X-ray tube per unit time.
- **Effect on Energy**:
    - However, tube current <mark style="background: #FFB86CA6;">does not alter</mark> the characteristic energy or average energy of the X-ray spectrum.
    - The minimum and maximum energies remain unchanged.
#### Increasing the Atomic Number of the Target Material:
- **Effect on Quantity**:
    - Using a target material with a higher atomic number (Z) <mark style="background: #FFB86CA6;">increases the quantity of X-ray photons</mark>.
    - Higher <mark style="background: #FFB86CA6;">Z materials interact more efficiently with electrons</mark>, resulting in more X-ray production.
- **Effect on Energy**:
    - The characteristic energy <mark style="background: #FFB86CA6;">remains specific to the anode material</mark> (e.g., tungsten).
#### Filtration:
- <mark style="background: #FFB86CA6;">Involves placing a filter in the path of the X-ray beam</mark>
- The purpose of filtration is to selectively <mark style="background: #FFB86CA6;">remove lower-energy (soft) X-rays from the beam</mark>, leaving behind higher-energy (hard) X-rays.
- -> <mark style="background: #FFB86CA6;">Reduces the number</mark> of low-energy X-ray photons (those with less penetration ability).
	- These low-energy photons contribute less to image formation and can be absorbed by superficial tissues.
- -> The <mark style="background: #FFB86CA6;">average energy</mark> of the remaining X-ray photons <mark style="background: #FFB86CA6;">increases</mark>.
	- This results in better tissue penetration and improved image quality.
- Since some X-ray photons are filtered out, the total number of photons reaching the detector decreases.
#### Power Supply Rectification:
- The power supply rectification process <mark style="background: #FFB86CA6;">impacts the X-ray tube voltage waveform</mark>
	- The voltage waveform represents the variation of voltage over time during an X-ray exposure.
- For <mark style="background: #FFB86CA6;">efficient X-ray production, a constant high voltage (kV) is desired</mark> across the X-ray tube.
	- -> This <mark style="background: #FFB86CA6;">ensures consistent acceleration</mark> of electrons and stable X-ray output.
- **Voltage Waveform Ripple**:
    - The ripple refers to the fluctuation in voltage during each cycle.
    - It is calculated as the difference between the maximum and minimum voltages per cycle, expressed as a percentage of the maximum voltage.
- **Types of Systems and Ripple**:
    - **Single-Phase Systems**:
        - These systems have **100% ripple** because the voltage waveform is a simple sine wave.
    - **Three-Phase 6-Pulse Systems**:
        - These systems have approximately **13% ripple**.
    - **Three-Phase 12-Pulse Systems**:
        - These systems have even lower ripple, around **4%**.
    - **High-Frequency Generators**:
        - High-frequency generators produce X-rays with ripple comparable to 12-pulse systems.
        - Their stable voltage waveform contributes to consistent image quality.
## Interactions of X-ray with tissue
### Attenuation
- Attenuation is the reduction in intensity of the X-ray beam as it passes through tissue. 
- It is quantified by the formula:$$dI = I' - I = -\mu I dx$$Here, $I$ is the initial intensity, $I'$ is the intensity after passing through a distance $dx$ in the tissue, and $\mu$ is the linear attenuation coefficient, which depends on the tissue type and the energy of the X-ray photons.
### Scattering
- In water photoelectric is dominant up to 26 keV, 
- In bone, it is dominant up to 45 keV. 
- Beyond those transition points Compton scatter occurs more often than photoelectric
#### Coherent Scatter (Rayleigh)
- <mark style="background: #FFB86CA6;">Occurs when low-energy X-rays interact with atoms, causing them to become excited without ionization</mark>.
	- Rayleigh scatter occurs when the <mark style="background: #FFB86CA6;">energy of the incident photon</mark> $E_{0}$ is <mark style="background: #FFB86CA6;">much less than the binding</mark> energy of the electrons in the tissue $E_{BE}$, typically in the range of <mark style="background: #FFB86CA6;">15-30 keV</mark>
- The photon is deflected by the atom, there is no energy loss to the atom.
	- As a result, there is no ionization, and the energy of the scattered photon remains the same as the incident photon $E_{0}$
- The atom releases this excess energy as a scattered photon with the same energy as the incident photon but in a different direction.
	- The probability of coherent scattering is inversely proportional to the square of the incident photon energy $$\frac{Z}{E_0^2}$$where $Z$ is the atomic number of the matter.
- This type of scatter <mark style="background: #FFB86CA6;">contributes minimally to image formation</mark> and <mark style="background: #FFB86CA6;">can degrade image quality</mark> due to increased noise.
- Significance in Imaging:
	- At lower energies, such as those used in [[Mammography]] (around 30 keV), Rayleigh scatter accounts for a larger proportion of measured photons, approximately 10%
#### Compton Scatter
- Involves the <mark style="background: #FFB86CA6;">interaction of X-ray photons with outer shell electrons</mark>.
- The <mark style="background: #FFB86CA6;">energy of the incident photon</mark> $E_0$ is <mark style="background: #FFB86CA6;">much greater than the binding energy</mark> of the outer-shell electrons $E_{BE}$
- This typically occurs in the diagnostic X-ray energy range (approximately 30 keV to 30 MeV).
- The incident photon transfers part of its energy to the electron, <mark style="background: #FFB86CA6;">ejecting it from the atom</mark>, and the photon is scattered with reduced energy.
- The photon loses energy (increases $\lambda$) and is deflected and it can still interact with other atoms:$$\lambda_{2}-\lambda_{1}=\frac{h}{m_{e}c}(1-\cos(\theta))$$
- Compton scatter is a <mark style="background: #FFB86CA6;">primary source of image contrast but can also contribute to image noise</mark>.
##### Interaction Process:
- The incident X-ray photon collides with the outer-shell electron.
- The collision results in ionization: the outer-shell electron is ejected from the atom.
- The scattered photon loses energy (reducing its frequency) and changes direction.
- Importantly, the scattered photon <mark style="background: #FFB86CA6;">can still interact with other atoms in the tissue</mark>.
##### Characteristics:
- Compton scatter is considered incoherent because it does not maintain phase coherence with the incident photon.
- The probability of Compton scatter is <mark style="background: #FFB86CA6;">directly proportional to the number of outer-shell electrons</mark> (i.e., the electron density) in the material.
- Compared to the photoelectric effect, Compton scatter is <mark style="background: #FFB86CA6;">weakly dependent on the atomic number</mark> $Z$ <mark style="background: #FFB86CA6;">and the energy</mark> of the incident photon $E_{0}$. It occurs regardless of the specific element.
- Compton scatter contributes to image noise and <mark style="background: #FFB86CA6;">can cause tissue damage due to ionization</mark>.
#### Photoelectric Effect
- Occurs when an X-ray photon with <mark style="background: #FFB86CA6;">energy greater than the binding energy of an inner-shell electron</mark> is absorbed, <mark style="background: #FFB86CA6;">ejecting</mark> the electron.
- When $E_0 \geq E_{BE}$, the photon interacts with an inner electron, typically from the K or L shell, and ejects it from the atom.
- <mark style="background: #FFB86CA6;">The ejected electron, known as a photoelectron</mark>, carries away kinetic energy equal to the difference between the photon’s energy and the binding energy $E_e = E_0 - E_{BE}$
- The vacancy is filled by an electron from a higher energy level, and the difference in energy levels is emitted as characteristic radiation.
- The photoelectric effect is <mark style="background: #FFB86CA6;">highly dependent on the atomic number of the tissue and the energy of the incident photon</mark>. It is a <mark style="background: #FFB86CA6;">significant contributor to image contrast</mark>, especially in bones and other dense tissues.
	- It is proportional to $Z^3$ and inversely proportional to $E_0^3$
	- This means that the higher the atomic number of the tissue and the lower the energy of the photon, the more likely the photoelectric effect is to occur
## Contrast in X-Ray images
- X-ray images <mark style="background: #FFB86CA6;">display variations in tissue density</mark>, which contribute to the contrast observed.
- The following general rules apply:
    - **Bones**: Bones appear white (high density) due to the presence of calcium.
    - **Air**: Air-filled spaces appear black (low density) because air does not attenuate X-rays significantly.
    - **Soft Tissue**: Soft tissues (e.g., muscles, organs) appear as shades of gray due to their intermediate density (between bones and air).
![[Pasted image 20240510125122.jpg|400]]
### Factors Impacting [[Image Noise#Contrast to Noise Ratio (CNR)|CNR]]
- The <mark style="background: #FFB86CA6;">Compton effect contributes to image blurring</mark>.
- Situations that enhance Compton scatter include:
	- High Tissue <mark style="background: #FFB86CA6;">Density</mark>: Greater tissue density <mark style="background: #FFB86CA6;">leads to more interactions and increased scatter</mark>.
	- <mark style="background: #FFB86CA6;">Thickness</mark> of Tissue: Thicker tissues <mark style="background: #FFB86CA6;">cause more interactions and scatter</mark>, affecting image clarity.
	- Large <mark style="background: #FFB86CA6;">Imaging Area</mark>: Capturing a large area increases scatter compared to focusing on a smaller region (e.g., a single bone).
### Strategies to Improve [[Image Noise#Contrast to Noise Ratio (CNR)|CNR]]
- Collimation: <mark style="background: #FFB86CA6;">Close collimation reduces scatter by limiting the X-ray beam to the area of interest</mark>.
- Grids: <mark style="background: #FFB86CA6;">Grids placed between the patient and the detector absorb scatter but also increase patient dose</mark>.
- Air Gap Technique: <mark style="background: #FFB86CA6;">Increasing the patient-film distance reduces scatter but may increase geometric un-sharpness</mark> (blur).
### Clinical Considerations
- Radiographers optimize techniques to balance contrast and noise.
- High kilovoltage (kV) techniques (e.g., chest X-rays) reduce contrast to better visualize lung markings.
- Understanding scatter and its impact helps improve image quality.
### Energy Dependency of Signal and Noise contributions
- The generation <mark style="background: #FFB86CA6;">voltage</mark> of the X-rays <mark style="background: #FFB86CA6;">must be selected appropriately</mark>. 
- We have learned that <mark style="background: #FFB86CA6;">above a certain voltage the Compton effect outweighs the photoelectric effect</mark>. 
- The tissue to be passed through must also be taken into account. 
- <mark style="background: #FFB86CA6;">Especially with thin tissue, lower voltage ensures a better X-ray image, as the CNR is better</mark>.
#### Photoelectric Effect and Signal Contribution
- The photoelectric effect is highly energy-dependent, especially in terms of the signal contribution to the X-ray image.
- <mark style="background: #FFB86CA6;">At low energies (below 120 keV), the photoelectric effect is more likely to occur because the energy of the incident photons is closer to the binding energy of the inner-shell electrons in the tissue</mark>.
- <mark style="background: #FFB86CA6;">When imaging a thin layer of tissue at low energies, the photoelectric effect contributes significantly to the signal</mark>, enhancing contrast as it is more likely to absorb low-energy photons, especially in tissues with high atomic numbers like bones.
- <mark style="background: #FFB86CA6;">At high energies (such as 120 keV), the probability of the photoelectric effect decreases</mark>, reducing its contribution to the overall signal. <mark style="background: #FFB86CA6;">This is because high-energy photons are more likely to pass through the tissue without interaction</mark>, leading to less absorption and, consequently, lower contrast in the image.
![[Pasted image 20240510125825.png|500]]
#### Compton Scatter and Noise Contribution:
- <mark style="background: #FFB86CA6;">Compton scatter contributes to the noise in an X-ray image</mark> and is <mark style="background: #FFB86CA6;">also dependent on the energy</mark> of the incident photons.
- <mark style="background: #FFB86CA6;">At 60 keV, Compton scatter is less prevalent, meaning the noise contribution to the image is lower. This results in a clearer image with better contrast.</mark>
- As the energy increases to <mark style="background: #FFB86CA6;">75 keV and 120 keV, the Compton scatter becomes more significant</mark>. <mark style="background: #FFB86CA6;">Higher energy photons are more likely to interact with outer-shell electrons, leading to increased scatter and noise</mark>.
- This noise manifests as a grainy appearance in the image and can obscure the details of the underlying tissue, reducing the clarity and contrast of the image.
![[Pasted image 20240510125745.png|500]]
## Contrast Media
- These agents enhance the visibility of specific structures during X-ray examinations.
![[Pasted image 20240510130132.png|500]]
### Iodine-Based Contrast Agents:
- Purpose: Iodine-based contrast agents are commonly used for vascular contrast studies.
- Administration: They are injected directly into the bloodstream.
- K-Edge: Iodine has a K-edge around 40 keV.
- Function: These agents <mark style="background: #FFB86CA6;">increase the absorption of X-rays in blood vessels, allowing better visualization of blood flow, vessel abnormalities, and other vascular structures.</mark>
### Barium Sulfate Contrast Agents:
- Purpose: Barium sulfate is used for gastrointestinal tract imaging.
- Administration: It is administered orally (usually as a drink or suspension).
- K-Edge: Barium also has a K-edge around 40 keV.
- Function: Barium sulfate enhances the absorption of X-rays <mark style="background: #FFB86CA6;">in the gastrointestinal tract</mark>, aiding in the visualization of the esophagus, stomach, intestines, and other digestive structures
## Instrumentation for Planar Imaging in X-Ray Systems
![[Pasted image 20240510130344.png#invert|600]]
### Filter
- A filter is used to remove low-energy X-rays from the beam spectrum.
- These low-energy X-rays would otherwise not significantly contribute to image quality but <mark style="background: #FFB86CA6;">would only add to patient dose (especially skin dose) and scatter</mark>.
- There are two types of filtration:
    - <mark style="background: #FFB86CA6;">Inherent Filtration</mark>: This comes <mark style="background: #FFB86CA6;">from components within the X-ray tube itself</mark>, such as the window, housing, cooling oil, and the anode target. Inherent filtration is equivalent to approximately 0.5-1.0 mm of aluminum.
    - <mark style="background: #FFB86CA6;">Added Filtration</mark>: Interchangeable metal sheets (such as aluminum, copper, or molybdenum) can be <mark style="background: #FFB86CA6;">added to the X-ray beam path</mark> based on the specific imaging application.
- Filtration reduces the X-ray intensity but <mark style="background: #FFB86CA6;">does not affect the maximum energy</mark> of the X-ray beam spectrum. However, it <mark style="background: #FFB86CA6;">increases the average X-ray energy</mark>, making the beam more penetrating.
- The change in the shape of the beam spectrum due to filtration is referred to as <mark style="background: #FFB86CA6;">beam hardening</mark>.
### Collimator
- The collimator shapes and <mark style="background: #FFB86CA6;">limits the X-ray beam to the area of interest</mark>.
- Proper collimation helps <mark style="background: #FFB86CA6;">reduce scatter radiation and improves image quality</mark> by focusing the X-rays only on the region of interest.
### Patient Anti-Scatter Grid:
#### Structure of Anti-Scatter Grids
- Anti-scatter grids consist of thin lead strips arranged parallel to each other to match the size of the detector.
- These lead strips are designed to <mark style="background: #FFB86CA6;">absorb scattered X-rays, which are X-rays that have deviated from their original path after interacting with the patient’s body</mark>.
#### Grid Ratio
- The grid ratio is a <mark style="background: #FFB86CA6;">measure of the effectiveness of a grid in rejecting scatter</mark>.
- It is defined as the ratio of the height of the lead strips $h$ to the distance between the lead strips $d$, which is the interspace.
- The formula for grid ratio is:$$\text{Grid Ratio}=\frac{d}{h}​$$
- A <mark style="background: #FFB86CA6;">higher grid ratio means the grid will reject scatter more effectively, but it also requires a higher dose to the patient to compensate for the increased absorption of primary X-rays</mark>.
#### Grid Frequency (Strip Line Density)
- Grid frequency refers to the number of grid lines per unit distance.
- It is calculated as:$$\text{Grid Frequency}=\frac{1}{d+t1}​$$Where $t$ is the thickness of the lead strips.
- The grid frequency is typically in the range of:
	- 40 - 50 lines/cm for low-frequency grids.
	- 50 - 60 lines/cm for medium-frequency grids.
	- 60 - 70+ lines/cm for high-frequency grids.
#### Impact on Image Quality
- By reducing scatter radiation, anti-scatter grids <mark style="background: #FFB86CA6;">enhance the contrast of the image, making it easier to distinguish between different types of tissues</mark>.
- However, the use of grids requires careful consideration of the <mark style="background: #FFB86CA6;">balance between image quality and patient dose</mark>
### Detector
- The detector <mark style="background: #FFB86CA6;">captures the X-rays after they pass through the patient</mark>.
- It <mark style="background: #FFB86CA6;">converts the X-ray energy into an electrical signal</mark>, which is then processed to create the final image.
![[Pasted image 20240510131038.png#invert|400]]
#### Detector classification
##### Analog Detectors:
- Film: Traditional [[X-Ray Film]] <mark style="background: #FFB86CA6;">captures images through a chemical reaction when exposed to X-rays</mark>.
##### Digital Detectors:
- [[Computed Radiography (CR)]]: Uses storage phosphors to capture X-ray images. The phosphors store the image data until stimulated by a laser to release the data in the form of light.
	- [[Direct Radiography (DR)]]: Converts X-ray energy directly into electrical signals using materials like amorphous selenium.
	- Indirect Conversion:
		- CCD/CMOS: Charge-Coupled Devices (CCD) or Complementary Metal-Oxide-Semiconductor (CMOS) sensors use a scintillator to convert X-rays into light, which is then turned into an electrical signal.
		- Indirect TFT: Thin-Film Transistor (TFT) arrays with a scintillator layer that converts X-rays to light before being converted to an electrical signal.
	- Direct Conversion:
		- Direct TFT: Uses a photoconductor layer that directly converts X-ray photons into an electrical charge collected by the TFT array.
##### Use of Cassettes:
- In CR systems, cassettes containing the phosphor plate are used to capture and then process the image.
##### [[Fluoroscopy#Image Intensifier|Image Intensifier]]:
- An [[Fluoroscopy#Image Intensifier|Image Intensifier]] is a device used in [[Fluoroscopy]] to <mark style="background: #FFB86CA6;">increase the brightness of the image produced by low-intensity X-rays</mark>.
##### Scintillators:
- Scintillators are materials that emit light when they absorb electromagnetic radiation, such as X-rays.
- Fluorescence vs. Phosphorescence:
	- Fluorescence: Occurs when the emission of light happens almost immediately after absorption (within about (10^{-8}) seconds). This is a fast process where the excited state is not stable.
	- Phosphorescence: Involves a delay between absorption and re-emission of light, ranging from milliseconds to hours. This happens when the excited state is metastable, meaning it has a longer lifetime before returning to the ground state.
## X-Ray Image Resolution
### [[Image Resolution#Spatial Resolution|Spatial Resolution]]
- One <mark style="background: #FFB86CA6;">factor that can limit spatial resolution is the detector size</mark>. 
	- <mark style="background: #FF5582A6;">Larger detectors</mark> can capture more information across a wider area, but they <mark style="background: #FF5582A6;">may sacrifice fine details</mark>. 
		- Imagine using a large canvas to paint a detailed picture—the overall scene is captured, but intricate features might be less distinct.
- The resolution of an image is influenced by three main factors: 
	- <mark style="background: #FFB86CA6;">Detector Resolution</mark>: This refers to the <mark style="background: #FFB86CA6;">number of pixels</mark> or elements in the detector array. Higher detector resolution (more pixels) allows finer details to be represented.
	- <mark style="background: #FFB86CA6;">Scintillator Properties</mark>: The scintillator material converts X-rays into visible light. A high-quality scintillator efficiently converts X-rays, enhancing image sharpness.
	- <mark style="background: #FFB86CA6;">Focal Spot Size</mark>: The focal spot is <mark style="background: #FFB86CA6;">the area on the anode where X-rays are generated</mark>. <mark style="background: #FFB86CA6;">Smaller focal spots result in better spatial resolution because they produce less blurring</mark>.
- The overall resolution of an imaging system combines these three factors:$$R = \sqrt{R_{\text{det}}^2 + R_{\text{sci}}^2 + R_{\text{spot}}^2}$$
#### Adjusting Focal Spot Size
- The focal spot size can be adjusted at the anode level using collimators. 
- However, there are limitations due to heat dissipation. 
- Smaller focal spots may limit the maximum tube current (mA).
### Sharpness
- is crucial for visualizing fine structures. 
- It <mark style="background: #FFB86CA6;">depends on both the scintillator material and the focal spot</mark>.
### Penumbra
- Refers to the <mark style="background: #FFB86CA6;">blurring or shadowing of edges</mark> in an image due to the finite size of the focal spot.
- Adjusting the source-to-patient (a) and patient-to-detector (b) distances affects penumbra. The equation is:$$\text{Penumbra (Ug)} = f \cdot \frac{b}{a}$$
    - Here, $f$ represents the diameter of the focal spot.
    - Increasing the distance between the source focal spot and the object (patient) results in a sharper image.
### Parts affecting Image Quality
#### Automatic Exposure Control (AEC)
- In medical imaging, particularly with X-rays, <mark style="background: #FFB86CA6;">AEC systems are used to ensure that the minimum number of detected X-rays required for good image quality is achieved</mark>.
- The AEC <mark style="background: #FFB86CA6;">adjusts the exposure</mark> time automatically so that this minimum number is reached before the X-ray source is turned off. 
- -> This <mark style="background: #FFB86CA6;">helps in maintaining consistent image quality</mark> across different scans.
#### Energy and Transmission
- <mark style="background: #FFB86CA6;">Higher energy X-rays</mark> have <mark style="background: #FFB86CA6;">higher transmission</mark> through tissue, which means they are <mark style="background: #FFB86CA6;">less likely to be absorbed</mark> and <mark style="background: #FFB86CA6;">more likely to reach the detector</mark>.
- This results in less [[Image Noise#Quantum Noise|quantum noise]], improving the [[Image Noise#Signal to Noise Ratio (SNR)|SNR]]. 
- However, <mark style="background: #FFB86CA6;">higher energy also means more Compton scatter</mark>, which can <mark style="background: #FFB86CA6;">degrade the</mark> [[Image Noise#Contrast to Noise Ratio (CNR)|CNR]], making it harder to distinguish between different tissues.
#### Filtering
- X-rays are often filtered to remove lower energy photons, which are more likely to be absorbed and contribute to patient dose without improving the image. 
- However, filtering also <mark style="background: #FFB86CA6;">reduces the overall number</mark> of photons reaching the detector, which <mark style="background: #FFB86CA6;">can lower the</mark> [[Image Noise#Signal to Noise Ratio (SNR)|SNR]]. 
#### Patient Size
- The <mark style="background: #FFB86CA6;">larger the patient, the thicker the tissue the X-rays must penetrate, resulting in greater attenuation</mark> (reduction in intensity) of the X-ray beam. 
- This leads to a <mark style="background: #FFB86CA6;">lower</mark> [[Image Noise#Signal to Noise Ratio (SNR)|SNR]] and [[Image Noise#Contrast to Noise Ratio (CNR)|CNR]]. 
- To compensate, the <mark style="background: #FFB86CA6;">exposure time may be increased</mark>, which unfortunately increases the X-ray dose to the patient.
#### Scintillation Crystal Thickness
- In digital detectors, the <mark style="background: #FFB86CA6;">scintillation crystal converts X-rays into light</mark>. 
- A <mark style="background: #FFB86CA6;">thicker crystal will absorb more X-rays and produce more light, thus increasing the</mark> [[Image Noise#Signal to Noise Ratio (SNR)|SNR]]. 
- However, <mark style="background: #FFB86CA6;">this can also reduce spatial resolution because the light can spread within the crystal before being detected, blurring the image</mark>.
#### Anti-Scatter Grid Ratio
- An anti-scatter grid is used to reduce the amount of Compton scatter reaching the detector. 
- A grid with a higher ratio will attenuate more scattered X-rays, which can reduce the [[Image Noise#Signal to Noise Ratio (SNR)|SNR]] because fewer overall photons are detected. 
- However, it can improve the [[Image Noise#Contrast to Noise Ratio (CNR)|CNR]] by reducing the amount of scatter that obscures the image contrast.
## Protection Devices 
- The human body can only handle a certain amount of X-radiation, until the absorbed photons cause serious tissue damage. 
- That’s why it is mandatory to wear protective gear when handling devices that use X-ray for imaging.
- It is also common for doctors to have a sensor on them, measuring the number of photons they’re exposed to and telling them, when to take a break from being around X-ray devices in order to not endanger their own health
### Radiation-Protective Gloves
- The hands are often closest to the radiation source and thus at high risk. Using gloves that provide 60-64% reduction in exposure can significantly protect the hands, especially when working with X-ray beams of 52–58 kV.
### Goggles
- The eyes are very sensitive to radiation and can develop cataracts as a result of prolonged exposure. Goggles with a lead equivalence of 0.15 mm can reduce the radiation exposure by about 70%, protecting the eyes from damage.
### Thyroid Collar
- The thyroid gland is also susceptible to radiation, and a collar can reduce scattered radiation by approximately 2.5 times. This is important because a significant percentage of papillary thyroid carcinomas are linked to radiation exposure.
### Lead Apron
- Wearing a lead apron can greatly decrease the amount of scattered radiation received. In the anteroposterior (AP) position, it can reduce exposure by 16 times, and in the lateral position, by 4 times. This is essential for protecting the body’s vital organs during fluoroscopic procedures.
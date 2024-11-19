> [!abstract] Definition
>  Computed Tomography (CT) is an advanced imaging method that <mark style="background: #FFB86CA6;">provides detailed cross-sectional images</mark> of the body, which can be used to create a three-dimensional representation of the patient’s internal structures.

- Planar images represent an attenuation map of x-rays travelling through tissue. This means that we lose information, because we only have a 2D-image. <mark style="background: #FF5582A6;">Because of overlaid tissue, it is hard to see separate structures</mark>.
	- -> Tomographic Imaging
## Volume Reconstruction
- A computed tomography consists of a lot of <mark style="background: #FFB86CA6;">2D slices that are stacked</mark> in the end to create a 3D image. 
- The slices are perpendicular cross sections to the three different patient axes. 
- They each have a <mark style="background: #FFB86CA6;">thickness of 0,2 to 10 mm</mark> and a spatial resolution of 512 x 512 pixels. 
	- The <mark style="background: #FFB86CA6;">slice thickness impacts the</mark> [[Image Resolution#Spatial Resolution|spatial resolution]] in the sagittal and coronal plane
	- The <mark style="background: #FFB86CA6;">slice thickness is determined by the width of the X-ray beam</mark>, which is <mark style="background: #FFB86CA6;">controlled by the collimator</mark>.
	- The choice of slice <mark style="background: #FFB86CA6;">thickness also considers the ALARA</mark> (As Low As Reasonably Achievable) principle, which aims to minimize the patient’s exposure to radiation while still obtaining the necessary diagnostic information.
- -> An <mark style="background: #FFB86CA6;">entire scan can take several minutes</mark> and consists of single slices which take 0.2 to 0.5 seconds each.
- This results in an overall image in 3D consisting of the single cross sections, which are shown in 2D. The previously covered structures can now be seen and the scan provides depth information.
### Temporal Resolution and Motion Artifacts
- <mark style="background: #FFB86CA6;">The speed of acquisition affects the</mark> [[Temporal Resolution]], which is the ability to capture moving objects within the body. 
- <mark style="background: #FFB86CA6;">Faster acquisition times can reduce motion artifacts</mark>, which are distortions in the image caused by patient movement during the scan
## Spatial Resolution
### Spatial resolution in axial plane
- The information obtained by scanning the body gets reconstructed into an image usually over a grid of 512 x 512 x 1 voxels. 
- For the final image the cuboid voxels get projected onto planar and square pixels, which leads to a pixel resolution of 512 x 512 pixels and results in a minimal loss of depth. 
- The final resolution of the image <mark style="background: #FFB86CA6;">is also influenced by the FOV</mark> (field of view) which is usually 500 x 500 mm, which leads to a spatial resolution of 0.98 x 0.98 mm, but <mark style="background: #FFB86CA6;">the FOV can also be reduced to focus on a smaller area and generate a higher resolution.</mark>

## Hounsfield scale
> [!abstract] Definition
>  The Hounsfield scale, named after Sir Godfrey Hounsfield, is a quantitative scale used in CT scans to <mark style="background: #FFB86CA6;">measure the radiodensity of tissues</mark>.

- CT images are quantitative because they provide a measurable map of how different tissues attenuate X-rays. This attenuation is represented by varying levels of gray in the images.
- **Hounsfield Units (HU)**: The scale is <mark style="background: #FFB86CA6;">calibrated around the</mark> [[Linear Attenuation Coefficient|linear attenuation coefficient]] <mark style="background: #FFB86CA6;">of water</mark>. The Hounsfield unit (HU) or CT number for a given tissue is calculated using the formula:$$HU(x,y,z) = \frac{\mu(x,y,z) - \mu_{water}}{\mu_{water}}$$here $\mu(x,y,z)$ is the [[Linear Attenuation Coefficient|linear attenuation coefficient]] of the tissue at a specific point, and $\mu_{water}$ is the linear attenuation coefficient of water.
- By setting the HU of water at 0 and air at -1000, all other tissues can be assigned a value on this scale. For example, fat typically has negative HU values, while bone has positive HU values, reflecting their relative densities compared to water.
- **Radiodensity Representation**:
    - **Air**: -1000 HU
    - **Fat**: -120 to -90 HU
    - **Soft Tissue**: +100 to +300 HU
    - **Bone**: +300 to +1900 HU
### Clinical Applications
- The Hounsfield scale is not only used for anatomical imaging but also provides:
	- A quantitative map of X-ray attenuation in tissue, which can help in identifying the type of tissue or substance.
	- A quantitative map of dose absorption from tissues, which is crucial for planning treatments like radiation therapy
## CT Scanner Components
### External
![[Pasted image 20240511121222.png#invert|300]]
#### [[CT Gantry]]  
![[CT Gantry#^4939b2]]
#### Bore  
- The patient transits through the bore during a scan. Bore diameters typically range from 70 to 85 cm.
#### Couch  
- The patient lies on the couch or table which is height adjustable and can travel
![[Pasted image 20240511121250.png#invert|300]]
#### Setup Laser  
- Prior to a scan, <mark style="background: #FFB86CA6;">the patient is roughly centered in the bore using setup lasers</mark> that are projected from it.
#### Control Panel  
- Used to perform essential tasks prior to a scan, like turning on the lasers or positioning the couch
### Internal
#### X-ray tube
- Produces X-rays with a peak potential (80 keVp - 140 keVp) and a current of 100 - 600 mA (to consider: higher current  -> less quantum noise, but more dose). A <mark style="background: #FFB86CA6;">cooling system</mark> is also included to avoid overheating
#### Filter
- Thin <mark style="background: #FFB86CA6;">aluminium plate</mark>: Low energy filtration
- <mark style="background: #FFB86CA6;">Bow Tie filter</mark>: <mark style="background: #FFB86CA6;">reduces the beam's lateral sections  -> less dose, uniform fluorescence pattern</mark> at the detector
#### Collimator
- The collimator determines the beam aperture (FAN angle) in the transverse plane and <mark style="background: #FFB86CA6;">the slice thickness</mark>, which prevents a penumbra caused of overbeaming. 
- In other words: The collimator is used to <mark style="background: #FFB86CA6;">determine the size of the area</mark> to be irradiated.
#### Detector
- The detector in a CT scanner consists of an <mark style="background: #FFB86CA6;">anti-scatter grid, a scintillator, a photodiode array and a readout ASCI</mark>. 
- The scintillator or scintillation crystal is a material that emits light when it interacts with ionizing radiation (like x-rays).
- There are two possible ways to position the detector:
	- <mark style="background: #FFB86CA6;">Detector array rotates opposite of the X-ray source</mark> on the gantry and collects data along multiple channels simultaneously
	- <mark style="background: #FFB86CA6;">Stationary ring detector that covers 360°</mark> -> only the x-ray tube rotates
![[Pasted image 20240511121450.png#invert|300]]
#### Slip ring
- The <mark style="background: #FFB86CA6;">spinning gantry is powered via the slip ring</mark>, which consists of a series of conductive rings that transmit <mark style="background: #FFB86CA6;">power by electromagnetic induction</mark>. 
- Additionally the slip ring allows the transmission of detected signal to the computer for image reconstruction.
#### Automatic Exposure Control (AEC)
- <mark style="background: #FFB86CA6;">The AEC adjusts the current</mark> at the X-ray tube <mark style="background: #FFB86CA6;">dependent on the thickness of the tissue</mark> to be penetrated.  
- The tissue's <mark style="background: #FFB86CA6;">thickness depends on the angle of the incoming X-rays and the physique of the patient</mark>. 
- The following applies: thicker tissue layer -> more current. 
- As a result, the patient <mark style="background: #FFB86CA6;">dosage is reduced</mark> and the <mark style="background: #FFB86CA6;">signal-to-noise ratio in the picture is nearly constant</mark>.
## CT slice acquisition
### Axial Mode
- This is a scanning technique where <mark style="background: #FFB86CA6;">the CT scanner acquires images slice by slice</mark>. 
- The <mark style="background: #FFB86CA6;">patient is moved forward</mark> along the longitudinal axis of the CT scanner, <mark style="background: #FFB86CA6;">pausing at intervals to allow a trans-axial image to be captured</mark> at each position along the axis
![[Pasted image 20240511121856.png#invert|300]]
- <mark style="background: #FFB86CA6;">When the beam is on, the gantry rotates 360 degrees around the patient</mark>. During this rotation, the X-ray tube emits X-rays, and detectors capture the data required to create an image of the patient’s body. The <mark style="background: #FFB86CA6;">couch remains still</mark> during this process.
- <mark style="background: #FFB86CA6;">After the gantry completes its rotation and the image for that slice is acquired</mark>, the beam is turned off. The <mark style="background: #FFB86CA6;">couch then moves a distance equal to the slice thickness</mark> (denoted as $z$) to position the patient for the next slice.
- This process is repeated—alternating between beam on and beam off—until the desired coverage along the cranio-caudal (CC) axis, which is the head-to-toe direction, is achieved.
#### Before slip ring
- In older CT scanners without slip ring technology, <mark style="background: #FFB86CA6;">cables were used to supply power to the X-ray tube</mark>. Because these cables could <mark style="background: #FFB86CA6;">become twisted as the gantry rotated</mark>, the direction of the gantry rotation had to be reversed periodically to prevent the cables from tangling. This reversal of rotation is referred to as “inverting the gantry rotation to unroll the cables.”
- The <mark style="background: #FFB86CA6;">need to reverse the gantry’s rotation to untwist cables introduced delays between scans</mark>, which decreased the temporal resolution of the CT scan. Temporal resolution is the ability of the scanner to capture rapidly changing images over time.
#### With slip ring
- With the advent of slip ring technology, continuous rotation of the gantry became possible, eliminating the need for cable untwisting and <mark style="background: #FFB86CA6;">improving temporal resolution</mark>. Slip rings allow the transfer of electrical power and signals to the rotating parts of the CT scanner without the need for cables that can twist, thus enabling continuous data acquisition and faster, more efficient scanning.
### Helical Mode
- In helical mode, the patient is moved through a rotating X-ray beam and detector set. From the patient’s perspective, <mark style="background: #FFB86CA6;">the X-ray beam traces a helical path as it scans the body</mark>.
![[Pasted image 20240512080704.png#invert|200]]
- The helical path results in a three-dimensional data set, which <mark style="background: #FFB86CA6;">can then be reconstructed into sequential images</mark> (slices) to form an image stack.
- Helical CT <mark style="background: #FFB86CA6;">allows a scan to be performed in a single breath-hold</mark>, making it <mark style="background: #FFB86CA6;">faster and more efficient</mark> than the step-wise method.
- Most modern CT protocols use helical acquisition due to its speed and reduced mis-registration caused by patient movement or breathing
#### Helical Pitch
- The pitch is a crucial parameter in helical CT. It is defined as the **table distance (d) traveled in one 360° gantry rotation**, divided by the **beam collimation (slice thickness, s)**.
- Mathematically, the pitch $p$ can be expressed as: $$p = \frac{d}{s}$$
- Let’s consider an example: If the table travels 5 mm in one rotation, and the beam collimation along the z-axis (slice thickness) is also 5 mm, then the pitch equals 1.0 (since ($5 \text{mm} /5 \text{mm} = 1.0)$).
![[Pasted image 20240512080745.png#invert|400]]
- Here’s how different pitch values affect the acquisition:
    - <mark style="background: #FFB86CA6;">(p = 1.0)</mark>: X-ray beams are contiguous for adjacent rotations. This <mark style="background: #FFB86CA6;">provides consistent image quality</mark>.
    - <mark style="background: #FFB86CA6;">(p > 1.0)</mark>: X-ray beams are not contiguous for adjacent rotations. While this <mark style="background: #FFB86CA6;">reduces dose to the patient and speeds up acquisition, it also leads to decreased image quality</mark> (fewer projections obtained, resulting in lower Signal-to-Noise Ratio, SNR).
    - (p < 1.0): There is X-ray beam overlap; i.e., a volume of tissue is irradiated more than once per scan. This results in <mark style="background: #FFB86CA6;">better image quality but at the expense of a higher patient dose</mark>
- When the pitch is set to **0**, it means that the table movement is synchronized with the gantry rotation, resulting in a **pure axial acquisition** (no helical path). However, this is not commonly used in practice
#### Advantages
- No need to pause between scans for table movement 
- Pitches greater than 1 possible (reduced dose) 
- Longer scan lengths possible within single breath-hold 
- Reduced patient movement artifacts 
- Reconstruction can be done at any position of z 
- 'Overlapping’ (p<1)reconstructions allow best z-axis resolution
#### Data Reconstruction
- In helical mode, we acquire a **spiral pattern of data** as the patient moves through the rotating beam.
##### Single Slice
- To reconstruct one slice, we use data from a broader beam profile compared to axial slices.
	- Because the patient is moving through the scanner, the X-ray beam must cover a larger area to ensure that the entire desired slice thickness is imaged.
	- This necessitates a broader beam profile to capture sufficient data for image reconstruction.
- The broader beam profile in helical scanning allows for faster data acquisition and can reduce motion artifacts.
##### 360° Z-Position Interpolation
- In this method, data from a full 360° rotation of the X-ray source around the patient is used.
- The data collected over this rotation is then mathematically manipulated to fit into the desired slice plane, which is the ( z )-position.
- This technique helps to create a complete image by using data from all angles around the patient.
- However, because the data is collected over a longer period, there’s a higher chance of motion artifacts, especially if the patient moves during the scan.
- To mitigate these artifacts, data from a 720° span (360° before and after the selected Z-position) can be interpolated onto the Z-plane.
- This technique is known as 360° Z-position interpolation.
##### 180° Z-Position Interpolation
- This interpolation technique takes advantage of the fact that X-ray attenuation is symmetrical; that is, a photon will see the same attenuation whether it passes through tissue ‘forward’ or ‘backward’.
- Therefore, for any given point in the image, data from the beam passing through the patient at an angle and its opposite angle (180° apart) can be used.
- It generates a second, complementary spiral of data for interpolation, reducing the time distance between data points.
- The benefit of this approach is that it reduces the time between data acquisition points, which can decrease motion artifacts.
- However, because the data is collected over a shorter arc, there may be more quantum noise in the image.
## Multislice CT (MSCT)
> [!abstract] Definition
> **Multislice CT (MSCT) scanners**, which are advanced versions of CT scanners capable of <mark style="background: #FFB86CA6;">acquiring multiple slices or in a single rotation of the X-ray gantry</mark>. 
- MSCT scanners provide a <mark style="background: #FFB86CA6;">more efficient</mark> and flexible way to acquire detailed images with <mark style="background: #FFB86CA6;">less patient dose</mark> compared to SSCT scanners. 
### Functionality
- MSCT scanners <mark style="background: #FFB86CA6;">have detectors arranged in multiple rows along the z-axis</mark> (the axis running head-to-toe through the body). These rows allow for the simultaneous acquisition of multiple slices, which can vary in thickness but typically range from **0.5mm to 1mm**.
- **Image Reconstruction**: The data from these multiple slices can be reconstructed in various configurations. For instance, four 2.5mm thick detectors can produce either four individual slices at 2.5mm each or two slices at 5mm, offering flexibility in image resolution and detail
### Dose Efficiency
- **Absorption Efficiency**: This refers to how effectively the scanner’s detectors capture X-rays. MSCT scanners are comparable to single-slice CT (SSCT) scanners in absorption efficiency when the total detector size is considered as the slice width.
- **Geometric Efficiency**: <mark style="background: #FFB86CA6;">MSCT scanners excel in geometric efficiency, which measures the scanner’s ability to utilize radiation effectively to create detailed images</mark>. A larger detector array allows for a wider X-ray beam collimation, meaning more radiation is used to create the image, and less is wasted. This efficiency is crucial for reducing patient exposure while maintaining image quality.
- **Optimal X-Ray Beam Width**: <mark style="background: #FFB86CA6;">To maximize the use of multiple detectors, the X-ray beam is slightly wider than the total detector width. This ensures that no slice receives the less focused</mark>, peripheral parts of the beam, which could result in blurrier images due to lower radiation doses at the edges
> [!abstract] Definition
>  Ultrasound imaging, also known as sonography, is a diagnostic technique that uses high-frequency sound waves to produce images of structures within the body. The process involves an [[Ultrasound Imaging#Probe|ultrasound probe]], called a transducer, which emits sound waves that are above the threshold of human hearing (above 20 kHz). These sound waves travel through the body and are reflected back to the transducer by boundaries between tissues, such as the boundary between fluid and soft tissue or tissue and bone. The echoes generate electrical signals that are then used to create images

![[Pasted image 20240525102938.png|400]]


## Advantages
- Non-invasive: No radiation or magnetic force is used, making it safer for patients, including pregnant women
- Cost-effective: Ultrasound is generally less expensive than other imaging modalities like [[Computed Tomography (CT)]] scans or [[Magnetic Resonance Imaging (MRI)]]
- Patient preparation: Preparing for an ultrasound is typically simpler and more comfortable for patients
- Temporal resolution: Ultrasound can capture real-time movements within the body, such as fetal movements or blood flow
- Spatial resolution: It provides high-resolution images that can help in diagnosing and monitoring various conditions
- Doppler effect: Ultrasound can measure fluid velocity and tissue elasticity indirectly, which is useful in assessing blood flow and organ function
## Disadvantages
- Limited penetration: Bone and air can limit the penetration of ultrasound waves, making it difficult to image certain areas like the brain
- Penetration depth: The depth of penetration is frequency-dependent and generally ranges from 2 to 30 cm
- Contrast: Soft tissues often have similar echogenicity, leading to low contrast in images, although this can be improved with contrast agents
- Noise: The presence of noise can affect the signal-to-noise ratio (SNR), due to the heterogeneous nature of tissues
- Field of View (FOV): The FOV is limited by the probe size and the penetration of ultrasound waves
- Operator dependency: The quality of the ultrasound images is highly dependent on the skill of the operator in manipulating the probe, adjusting settings as well as the pressure
	- -> this is why US imaging is not very reproducible
	- -> Using robots can improve reproducibility

## Physical Principles
Ultrasound imaging is based on the physical principles of acoustics, specifically the behavior of sound waves as they travel through a medium.
### Acoustic Waves
- Acoustic waves are a form of mechanical energy that propagates through a medium by alternately compressing and rarefying the molecules within that medium. 
- Unlike electromagnetic (EM) waves, which can travel through a vacuum, acoustic waves require a material medium to travel. 
	- This is because they are not self-propagating energy forms like EM waves.
#### Propagation through Elastic Medium
- The medium through which ultrasound waves travel must be elastic, meaning it can deform when subjected to a force and then return to its original shape when that force is removed. 
- This elasticity allows the medium to absorb and then release the energy carried by the sound waves.
#### Generation of Pressure
- When a mechanical force is applied to a medium, it creates pressure (measured in newtons per square meter ($\frac{N}{m^{2}})$, or pascals). 
- This force causes a displacement of the medium’s elements, doing work and thus transferring energy. 
- The energy is transferred longitudinally, meaning that while the molecules of the medium oscillate back and forth, they do not travel with the wave; instead, they pass the energy along to neighboring molecules.
#### Compression and Rarefaction
- As the sound waves travel through the medium, they create regions of compression, where the molecules are pushed closer together, and rarefaction, where the molecules are spread further apart. 
- These regions represent localized changes in pressure within the medium.
![[Pasted image 20240525103721.png#invert|300]]
#### Properties of Acoustic Waves
- The acoustic pressure $p$ evolves as a function of position $x$ and time $t$, described by the wave equation:$$\nabla^2 p - \frac{1}{c^2} \frac{\partial^2 p}{\partial t^2} = 0$$where $c$ is the speed of sound in the medium.
	- $c$ is determined by the medium's elastic property (bulk modulus $B$) and density $\rho$
	- In a uniform medium it is given by: $$c=\sqrt\frac{B}{\rho}$$
	- Bulk Modulus: Reflects how much a material will compress under external pressure. A stiffer medium (higher $B$) allows waves to travel faster.
	- Density: Heavier media (higher $\rho$) slow down the wave. However, the bulk modulus has a more significant impact on the speed of sound than density.
- For plane waves, the equation simplifies and describes how pressure changes locally over time and space: $$\frac{\partial^2p}{\partial x^{2}}-\frac{1}{c^{2}}\frac{\partial^2 p}{\partial t^2} = 0$$
- The frequency $f$ of the wave is the number of times a wave’s cycle occurs per second, and the wavelength $\lambda$ is the distance between successive regions of compression or rarefaction. They are related by the equation: $$f=\frac{c}{\lambda} \rightarrow c=f\cdot\lambda$$
![[Pasted image 20240525104454.png#invert|400]]
- The speed of sound $c$ varies depending on the medium, affecting the wavelength $\lambda$. For medical imaging, frequencies above 2 MHz are typically used.
	- Frequency $f$ is what we control and stays constant (!!!), velocity $c$ depends on the medium, wavelength $\lambda$ is the compensating mechanism that links frequency and speed
	- Different tissues have different sound speeds, affecting how ultrasound waves propagate and are reflected. For example, sound travels at 330 m/s in air and 4080 m/s in bone.
		- US are mainly used to image soft tissues and we assume an “average speed” of 1540 m/s
	- -> The wavelength changes at the boundaries of two different media due to the change in sound speed
- Acoustic Impedance $Z$: Describes a material’s reaction to mechanical vibrations and is calculated as:$$
Z=\rho⋅c$$It increases with higher density, stiffness, and propagation speed, and is independent of frequency.
- Large differences in acoustic impedance between two adjacent media result in significant reflections, which form the basis of contrast in ultrasound imaging.
##### Peak Pressure Levels
- Most diagnostic ultrasound beams deliver peak pressure levels of approximately **1 MPa** (megapascal).
- This pressure level exceeds Earth’s atmospheric pressure by **10 times**.
- The mechanical force generated by ultrasound waves also produces energy, which is transferred at a certain rate.
##### Power and Intensity
- **Power** represents how much energy is transferred per unit time (measured in watts, W).
- The relationship between pressure and intensity:
    - **Intensity $I$** is the amount of power carried by the wave per unit area:$$I=\frac{\text{Power}}{\text{Area}}=\frac{\text{Pressure}^{2}}{2 \cdot Z}$$
    - Increasing power (pressure change) leads to an increase in intensity.
    - As we spread the power over a larger area, the intensity decreases.
    - Intensity $I$ is proportional to the **pressure amplitude squared** ($I \propto P^2$). Doubling the pressure amplitude results in a **quadruple increase** in intensity.
##### Relative Intensity and Sound Level (dB)
- Absolute intensity measurement at a specific region is challenging.
- Instead, we use **relative intensity** expressed in a logarithmic scale to represent sound level $L$
- The formula for sound level in decibels (dB): $$L = 10 \log_{10}\left(\frac{I}{I_0}\right)$$ 
    - $I$ is the actual intensity.
    - $I_0$ is the threshold intensity at **1000 Hz** of human hearing (corresponding to **absolute silence**).
    - Increasing intensity by a factor of 10 corresponds to a **10 dB increase** in sound level (approximately twice the perceived loudness in humans).
    - Doubling the intensity results in a **3 dB increase**.

## Types of Tissue Interactions
![[Pasted image 20240525110651.jpg#invert|400]]
### Reflection 
- When an ultrasound wave encounters a tissue boundary (interface), a fraction of its intensity is reflected back.
- This reflection generates an **echo**, which is the signal received by the ultrasound system.
- Echoes provide information about tissue structures and help create ultrasound images.
### Transmission
- Some of the ultrasound energy passes through the tissue boundary and continues to propagate deeper into the body.
- Transmission allows imaging of structures beyond the initial tissue layer.
### Refraction 
- Refraction occurs when the ultrasound wave changes direction as it crosses a tissue boundary due to differences in sound speed between the two tissues.
- Refraction can affect the accuracy of image localization and alignment.
- Refraction is the bending of the ultrasound wave as it passes from one medium to another with different propagation speeds.
#### Snell's Law
- Snell’s law describes the relationship between the angles of incidence and refraction and the speeds of sound in the two media:$$\frac{\sin(\theta_2)}{\sin(\theta_1)} = \frac{c_2}{c_1}$$where $\theta_1$ and $\theta_2$ are the angles of incidence and refraction, and $c_1$ and $c_2$ are the speeds of sound in the respective media.
	- If $c_2 < c_1$, then $\theta_2$ decreases.
	- Refraction is relevant for imaging as it affects the path of the ultrasound wave and can influence the accuracy of the image.$$T=\frac{I_{t}}{I_{i}}=\frac{4Z_{1}Z_{2}\cos^{2}(\theta_i)}{(Z_{2}\cos(\theta_{i})+Z_{1}\cos(\theta_{t}))^2}$$
### Diffusion (Scatter) 
- Diffusion, also known as scattering, happens when ultrasound waves encounter small structures or irregularities within tissues.
- These structures scatter the ultrasound energy in various directions.
- Diffusion contributes to the overall intensity distribution and affects image quality.
- Caused by tissue elements smaller than the wavelength $\lambda$ of the ultrasound
- **Factors Increasing Scatter**:
    - **Density $\rho$**: Higher density leads to more scatter.
    - **Acoustic Impedance $Z$**: Greater impedance increases scatter.
    - **Element Size**: Larger elements cause more scatter.
    - **Frequency $f$**: Higher frequency results in increased scatter.
- **Echogenicity**:
    - **Hyper echogenic**: Tissues with more scatter appear brighter on the image.
    - **Hypo echogenic**: Tissues with less scatter appear darker.
- **Contrast Agents**:
    - Injecting micro bubbles enhances scatter, increasing tissue echogenicity and image contrast.
- **Attenuation**:
    - The amplitude of the ultrasound wave attenuates due to tissue interactions and heat generation.
    - Attenuation depends on the tissue’s attenuation coefficient $\mu$, frequency $f$, and depth $x$, measured in decibels (dB):$$I_{2}=I_{1}e^{-\mu f x}$$
### Absorption
- Absorption refers to the conversion of ultrasound energy into heat within the tissue.
- Tissues absorb some of the ultrasound energy, leading to a gradual decrease in intensity as the wave penetrates deeper.
- Absorption is essential for safety, as excessive energy absorption can cause tissue damage.
### Range Equation
- The range equation relates the depth of tissue penetration $d$ to the time of flight $t$ and the average speed of sound $c$ assumed for soft tissue: $$d = \frac{c \cdot t}{2}$$
### Perpendicular Reflection
![[Pasted image 20240525111033.png#invert|200]]
- In perpendicular reflection, the reflection coefficient $R$ and the transmission coefficient $T$ represent the relative intensity at a tissue boundary.
- The reflection coefficient is the ratio of reflected intensity to incident intensity: $$R=\frac{I_{r}}{I_{i}}=\left(\frac{Z_{2}-Z_{1}}{Z_{2}+Z_{1}}\right)^{2}$$
- The transmission coefficient is the ratio of transmitted intensity to incident intensity:$$T=\frac{I_{t}}{I_{i}}=\frac{4Z_{1}Z_{2}}{(Z_{2}+Z_{1})^2}$$
#### Example
- Reflection coefficient: (R = 1.71)
- Transmission coefficient: (T = 1.34)
We can calculate the relative intensity changes: $$R = \frac{1.34 - 1.71}{1.34 + 1.71} = 0.0147$$ $$T = \frac{4R}{1 - R} = 0.9853$$ 
Therefore, doubling the pressure amplitude leads to a quadruple increase in intensity, and the relative intensity changes are reflected in the resulting ultrasound images. edge browser The user has the page open in a Microsoft Edge browser window whose metadata is:
### Specular Refraction
![[Pasted image 20240525111615.png#invert|200]]
- Occurs when the ultrasound wave encounters a smooth and regular surface.
- The angle of incidence $\theta_{i}$ is equal to the angle of reflection $\theta_r$
- Specular reflection is not useful for imaging because most biological tissues have irregular surfaces, which do not reflect sound waves uniformly.
### Non-Specular Reflection
![[Pasted image 20240525111720.png#invert|200]]
- Happens when the ultrasound wave encounters an irregular surface, such as most tissues and organs.
- The reflected signal is scattered in many directions, resulting in a noisy signal.
- Despite the low signal-to-noise ratio (SNR), non-specular reflection is useful for imaging because it provides information about the tissue’s structure.

## Instrumentation
### Probe
#### Probe Variations
- Frequency: Higher frequency probes are used for surface applications, while lower frequencies allow deeper penetration but with reduced image quality.
- Shape: Probes can be long and pointy for cavity insertion. Phased array probes electronically steer and focus the ultrasound beam, offering better image quality and a wider field of view (FOV).
- FOV: Linear probes are used for small parts, whereas convex-shaped probes provide a larger FOV.
#### Probe Internal Design 
![[Pasted image 20240525113245.png#invert|300]]
- **Plastic Case**: Houses all internal components, offering protection and insulation.
- **Acoustic Absorber**: Reduces noise and improves signal clarity.
- **Wiring**: Transmits electrical signals and enables beam focusing and steering.
##### Damping Block 
- Absorbs excess energy, leading to shorter spatial pulse length and better axial resolution.
- Its purpose is to dampen resonant vibrations within the piezoelectric element.
- By reducing resonant vibrations, the damping block creates a shorter spatial pulse length.
- This shorter pulse length has two important benefits:
    1. **Better Axial Resolution**: Axial resolution refers to the ability to distinguish closely spaced structures along the ultrasound beam’s direction. A shorter pulse length improves this resolution.
    2. **Time to Receive Reflected Echoes**: The shorter pulse allows more time for the probe to receive and process reflected echoes from deeper tissues, contributing to better image quality.
##### [[Piezoelectric Effect|Piezoelectric Material]] 
- Converts electrical signals into ultrasound waves and vice versa, essential for image generation.
- Piezoelectric materials are crystals (such as quartz, lead zirconate titanate, or polyvinylidene fluoride) that exhibit a unique property: they can convert electrical energy into mechanical vibrations (and vice versa).
- When an alternating voltage (AC) is applied to a piezoelectric element, it undergoes alternating dimensional changes (expansion and contraction).
- These dimensional changes result in alternating pressure changes within the material.
- The pressure generated by the piezoelectric element propagates as a sound wave (ultrasound wave) into the surrounding medium (such as human tissue).
##### Matching Layer
- Ensures efficient transmission of ultrasound waves into the body with a middle acoustic impedance $Z$ between the piezo material and body impedance.
- The matching layer has an acoustic impedance $Z$ that sits between the impedance of the piezoelectric material $Z_{piezo}$ and the impedance of the body tissue $Z_{tissue}$.
- By having a middle impedance, the matching layer helps minimize the reflection of ultrasound waves at the probe-tissue interface.
- Gel is often used to couple the probe to the skin, preventing air pockets and ensuring smooth contact for better signal transmission.
- The matching layer also plays a role in allowing specific beam focusing and steering.
- By optimizing the impedance match, the ultrasound beam can be directed more precisely toward the desired imaging area.
- Focusing and steering enhance image quality and allow visualization of specific structures within the body.
### Time Gain Compensation (TGC) Amplifier
- The intensity of the returned signal depends on the type of tissue <mark style="background: #FF5582A6;">but also the depth</mark> because of all the interactions along the way
- We need to compensate for this in order to correctly identify the tissue and its location
	- -> Use an amplifier
- A normal amplifier simply amplifies all incoming signals by e.i. 10
- TGC amplifies the signal based on the time!
![[Pasted image 20240527084413.jpg#invert|]]
- This ensures that structures far from the probe are as clearly visible in the ultrasound image as those closer to the probe
### Echo Generation
- When the ultrasound wave encounters tissue boundaries or structures, some of it gets reflected back (echo).
- The returning echo wave compresses the piezoelectric element again.
- This compression generates an AC voltage proportional to the intensity of the reflected sound wave.
- Essentially, the piezoelectric material converts the mechanical energy of the echo into an electrical signal that can be processed and used to create an ultrasound image.
### Frequency Determination
- The frequency of the ultrasound wave generated by each crystal within the piezoelectric material depends on:
- The speed at which sound travels in that specific material.
- The thickness of the piezoelectric material.
- The relationship between frequency $f$, speed of sound $c$, and wavelength $\lambda$ is given by: $$f = \frac{c}{\lambda} = \frac{c}{2t_h}$$ where:
	- $c$ is the speed of sound in the material.
	- $t_h$ is the thickness of the piezoelectric material.

## Pulsed Echo 
- Continuous pulse are oscillations for a long period at a certain frequency which is the resonance frequency of the piezo element and it depends on the element thickness
- Using a continuous wave won't allow us to separate the emitted from the received waves
	- -> There are always waves coming in and going out
- This is why US uses very short pulses (pulsed echo) with transition time $t_{t}$
- In pulsed echo we dampen the piezo using the [[Ultrasound Imaging#Damping Block|damping block]]to stop the oscillation and we obtain a larger frequency BW around the central frequency
- While the piezo is not fired to produce waves it can be used as a detector to receive reflected US with receive Time $t_{r}$
- Every pulse contains 2-3 cycles (here 2):
![[Pasted image 20240527081520.png#invert|200]]
### Depth and Timing
- To visualize deeper structures, we need to send ultrasound pulses deeper into the body.
- However, if we send a second pulse before the echoes from the first pulse have returned, we encounter a problem: **interference**.
- Interference occurs when the echoes from the first pulse overlap with the echoes from the second pulse.
- This overlapping creates confusion in interpreting the returning signals, leading to distorted images.
### Pulse Duration 
- Time during which US are generated 
- $$PD = \#cycles\cdot T=\frac{\#cycles}{f}$$ with:
	- $T$: time of the cycle
	- $f$: frequency of the sound (fixed by the specific piezo-element)
- Once we chose the probe, we cannot change the PD, because the frequency is set and we do not have control on the medium
### Spatial Pulse Length 
- Distance in space traveled by the US during one pulse
- $$SPL=\#cycles\cdot\lambda$$
- Depends on source and medium because $\lambda$ depends on it
- As the wavelength $\lambda$ increases, the SPL increases 
- It affects spatial resolution
- Once we chose the probe, we cannot change the SPL, because the frequency is set and we do not have control on the medium
### Pulse Repetition Period 
- Number of pulses per unit time (1 second)
### Pulse Repetition Period 
- Amount of time between the start of one pulse and the start of the next pulse 
- $$PRP=\frac{1}{PRF}$$
![[Pasted image 20240527082530.png#invert|300]]
### Duty Cycle
- Percentage of time the machine is emitting pulses over a repetition time 
- $$DF=\frac{PD}{PRP} \cdot 100$$
## Visualization Modes 
### A-Mode
- Amplitude mode 
- A-mode ultrasound presents data in a **one-dimensional** graphical format:
![[Pasted image 20240527082959.png#invert|300]]
- This is similar to how sonar works, where a sound wave is sent out, bounces off an object, and the reflected wave is received back. 
- Only one line is scanned
- By analyzing the time it takes for the echoes to return and the amplitude of the echoes, A-mode can measure the **distance between** or the **thickness of tissues**.
- The distance is determined by the time delay between the sent pulse and the received echo.
- The thickness can be inferred from the spacing between spikes on the graph.
- An example is examining the cornea, lens and chambers of the eye 
### B-Mode
- Brightness mode 
- Each peak seen in A-mode is depicted as pixel of a varying grey scale.
![[Pasted image 20240527083202.png|400]]
- The brightness of each dot depends on the amplitude of the returning echo 
- The position on the screen is based on the time taken for each echo to return to the transducer 
- Multiple lines are scanned sequentially
- We can only visualize the change in tissue using this
	- -> The gray level within the tissue on the image is determined by the scatter, which is essentially the “background noise” generated by the tissue
#### Real Time Imaging
- B-mode ultrasound creates images by scanning lines sequentially across a plane within the body.
- To perceive these lines as a continuous image, the scanning must occur rapidly—at least 24 times per second
- This frequency allows the human eye to integrate the sequential lines into a single coherent image, much like how a film projector works.
### M-Mode
- Motion mode
- M-mode utilizes the depth information from A-mode and the temporal information from B-mode.
- It focuses on a single line (like A-mode) but displays how the echoes change over time (like B-mode).
- M-mode is particularly useful for examining tissues that move, such as the heart’s chambers and valves.
- It can measure the distance between tissue planes and track their movement throughout the cardiac cycle
### Doppler Mode
- The goal of Doppler US is to determine flows (directions, velocities, acceleration) of liquids within the body. 
- Therefore the physical principle of [[Doppler Effect]] is used.
- When an ultrasound wave encounters a moving target (such as blood cells), the frequency of the reflected wave changes due to the [[Doppler Effect]]
- The observed frequency shift $\Delta f$ is related to the relative velocity $\Delta v$ between the source (probe) and the target (moving blood cells):$$\Delta f = 2 \frac{\Delta v}{c} f_0$$ where
	- $c$ is the speed of sound in the medium (usually tissue).
	- $f_0$ is the original frequency emitted by the probe.
	- The factor of 2 accounts for both the incident and reflected waves.
#### Inclination Angle and Doppler Angle
- The probe’s inclination angle relative to the direction of blood flow affects the measured Doppler shift.
- The Doppler angle (( \alpha )) is the angle between the ultrasound beam and the direction of blood flow.
- The measured Doppler shift depends on the cosine of the Doppler angle:$$
    v_t = c \left( 2f \cos(\alpha) \right) f_0 $$
    - $v_t$ represents the true velocity of blood flow.
    - The dependence on the cosine of the angle means that the measured velocity is highest when the Doppler angle is close to 0° (parallel to blood flow).
- If the device is adjusted for a Doppler angle of 0° (i.e.,  $\cos(\alpha) = 1$ ), the measured velocity is accurate.
- However, as the Doppler angle increases, the measured velocity overestimates the true velocity.
#### Continuous Wave (CW) Doppler Ultrasound
- In CW Doppler, the system uses **two separate transducer elements**.
- One element **continuously transmits** ultrasound waves, while the other **continuously receives** the reflected waves.
- The **flow** of blood or fluid is detected within the **overlap** of the transmit and receive sound beams.
- This overlapping area is where the movement of blood cells causes a change in the frequency of the received waves.
- The received wave has a **varying frequency** due to the motion of the blood cells within the vessels.
- This variation is what allows the system to measure the velocity of blood flow.
- The system detects the **Doppler frequency shift** by subtracting the frequency of the transmitted signal from that of the received signal.
- The typical shift is around **10/1000 KHz**, which falls within the range of audible sound.
- The frequency shift can be **audibly presented** through a speaker.
#### Duplex Doppler Ultrasound
- Duplex Doppler ultrasound combines two essential components:
    - **Two-dimensional (2D) imaging**: Provides a visual image of the vessel under examination.
    - **Doppler ultrasound**: Evaluates the velocity and direction of blood flow within the vessel.
- The combination of these two modes allows for a comprehensive assessment of both the vessel’s structure and its blood flow characteristics.
##### Pulsed Mode Imaging (2D)
- One sector of the probe is used for pulsed mode imaging.
- In this mode, the ultrasound system emits short pulses of sound waves and receives their echoes.
- The reflected waves create a 2D image of the vessel’s walls and lumen.
##### Continuous Mode (Doppler)
- Two other sectors (external to the same probe) are used for continuous mode Doppler.
- In continuous mode, the ultrasound system continuously transmits and receives sound waves
- These waves interact with moving blood cells, resulting in a Doppler shift.
- The Doppler shift provides information about blood flow velocity and direction.
- The continuous mode is used to measure blood flow velocities throughout the cardiac cycle.
##### Inclination Angle and Doppler Shift
- To calculate blood flow velocity accurately, we need to estimate the inclination angle (angle between the ultrasound beam and blood flow direction).
- The Doppler shift formula accounts for this angle:$$\Delta f = 2v_t \cos(\alpha) / c f_0 $$
	- $v_t$ represents the true velocity of blood flow.
	- The cosine of the angle $\cos(\alpha)$ adjusts the measured velocity based on the angle.
	- The Doppler shift is detected across the entire active region of the ultrasound beam.
##### Challenges with Opposite Velocities
- A limitation arises when opposite velocities cancel each other out.
- For example, if two blood vessels overlap, their opposite velocities may interfere, leading to inaccurate measurements.
- Clinicians must be aware of this potential issue and interpret the results carefully.
#### Color Doppler
- Color Doppler provides a visual representation of blood flow velocities and directions superimposed on a regular 2D ultrasound image.
- **Red**: Indicates blood flow **toward** the probe (positive velocity).
- **Blue**: Indicates blood flow **away** from the probe (negative velocity).
- **Black**: Represents **no motion** (stationary tissue or no flow).
![[Pasted image 20240527110324.png|400]]
#### Spectral Doppler
- With Spectral Doppler it is possible to not only show the direction of flow (like with color doppler), it is also possible to show phases or acceleration of blood flow. The reason is that the frequency of the sound waves returned to the transducer is a band with of heterogeneous Doppler shifts yielded by each red blood cell in motion and not only one frequency.
- Therefore Fourier analysis is used to average frequencies over time period, e.g. 5 milliseconds. Frequencies are converted with Doppler equation into velocities, but as a spectrum. The spectrum is the outcome, it is displayed in a graphics.
- Y-axis shows the direction and velocity of the flow. X-axis shows the flow over time. The gradient of the wave forms show the acceleration. 
- At spectral doppler there is also selected a small window in the image to do the analysis in this specific area.
![[Pasted image 20240527110418.png|400]]
## Beam Geometry
- The geometry of the ultrasound beam is a critical factor in determining the quality and detail of the ultrasound image.
- It involves the shape and focus of the beam and how these characteristics influence image resolution and depth.
### Unfocused vs. Focused Beams
- An unfocused beam has a uniform cross-section, whereas a focused beam converges at a point, enhancing spatial resolution and intensity at the focal point.
- Focused beams have a focal zone with improved resolution.
![[Pasted image 20240527084606.jpg#invert|400]]
### Beam Zones
The ultrasound beam is divided into two main zones:
- Fresnel Zone (Near Field): The region close to the transducer where the beam is more uniform and less divergent.
- Fraunhofer Zone (Far Field): The region beyond the near field where the beam starts to diverge
![[Pasted image 20240527084642.jpg#invert|600]]
- If we want to look deeper we need a further focal zone, this can be achieved by:
	- Changing the frequency $f$ (using a different probe) 
	- Changing the diameter $d$, this is done using the ball by the operator
### Transducer Types
![[Pasted image 20240527085828.jpg#invert|600]]
#### Single-Element
![[Pasted image 20240527085110.png#invert|200]]
- These probes consist of a **single piezoelectric element**.
- The focal depth (the depth at which the ultrasound beam is optimally focused) for single-element transducers is **fixed**.
- The focal depth is determined by the design of the transducer:
    - Some single-element probes have an **acoustic lens** in front of the element. The lens helps focus the ultrasound beam.
    - Others achieve focus using a **concave piezoelectric element** itself
#### Linear Array
![[Pasted image 20240527085357.png#invert|200]]
- Linear arrays consist of multiple piezoelectric elements arranged in a **linear configuration** (side by side).
- These arrays can have **up to 512 elements**.
- Unlike single-element probes, linear arrays allow for **adjustable focus**:
    - By selectively firing different elements, the system can focus the ultrasound beam at different depths.
    - Changing the number of fired elements dynamically alters the focal point.
![[Pasted image 20240527085439.png#invert|300]]
#### Phased Array
- Phased arrays also use multiple elements, but they are arranged in a **curved or circular pattern**.
- The key feature of phased arrays is **electronic focusing**:
    - By adjusting the **timing (phase)** of when each element emits its pulse, the system can steer and focus the beam electronically.
    - This dynamic control allows for rapid adjustments during real-time imaging.
### Beam Steering
- Beam steering involves changing the direction of the ultrasound beam without physically moving the probe.
- This is achieved by adding a time delay to the firing sequence of the piezoelectric elements in the transducer
- By controlling the timing, the beam can be angled to insonate (expose to ultrasound) a point in the tissue from multiple directions, enhancing image quality.
![[Pasted image 20240527090115.png#invert|100]]
### Beam Focusing
- Focusing refers to converging the ultrasound beam at different depths to create a narrow point called the focal point.
- At the focal point, the lateral resolution of the beam is the greatest, which is crucial for detailed imaging
- Advanced transducers can dynamically focus the beam by adjusting the time delay in the firing of the piezoelectric elements.
- The **outermost elements** are fired first, and the **center-most element** is fired last.
- The ultrasound pulses from these elements constructively interact to form a composite pulse that converges at the focal point.
- The ability to steer and focus the beam electronically means that even small probes can capture a larger area than if they were only transmitting in a linear fashion.
## Image Resolution

| Piezo Transducer | Axial        | Lateral      | Elevational  | Hints!                                                                       |
| ---------------- | ------------ | ------------ | ------------ | ---------------------------------------------------------------------------- |
| Thickness        | $\checkmark$ |              |              | $SPL=\#cycles\cdot\lambda\rightarrow f\rightarrow \text{thickness}=c_{p}/2f$ |
| Width            |              | $\checkmark$ |              | Width of beam in lateral plane                                               |
| Height           |              |              | $\checkmark$ | Width of beam in elevational plane                                           |

### Axial Resolution
![[Pasted image 20240527090332.png#invert|200]]
- Axial resolution in ultrasound imaging is a measure of the system’s ability to distinguish between two points that are close together along the beam’s path, which is the longitudinal axis
![[Pasted image 20240527090511.png#invert|]]
- In the picture you can see an ultrasound device on the right and left that emits waves (blue). These waves first hit one layer and then another and reflect the signals (orange and green). The distance between the layers is only half as large on the right-hand side. If the distance between the layers is not large enough, the reflected signals can no longer be distinguished. If the distance is half the SPL, we obtain a continuous wave consisting of the reflections of the two different layers.
- ->$$\text{Axial resolution} = \frac{1}{2} \text{SPL} $$
- To get the best limit of axial resolution, the SPL has to be as small as possible. 
- That can be achieved if we use dampening to reduce the number of cycles and have a high frequency to shorten the wavelength. 
- But keep in mind that the frequency depends on the choice of source and probe.
- That means that higher frequency US have better axial resolution and the axial resolution does not change with depth.
### Lateral Resolution
![[Pasted image 20240527090746.png#invert|200]]
- Lateral resolution refers to the system’s capability to distinguish between different elements (reflectors) that lie at the **same depth but in different lateral planes**.
- If the beam is **wide**, it may not be able to distinguish two closely spaced elements, and they appear as a single entity.
- Conversely, if the beam is **narrow**, each element is seen distinctly in its own lateral position.
- Proper **focusing of the beam** is crucial for achieving good lateral resolution.
- Focusing ensures that the ultrasound beam is narrow enough to differentiate adjacent structures.
- The **best lateral resolution** occurs within the **focal zone** of the beam.
- As you move away from the focal zone, lateral resolution decreases.
- The limit of lateral resolution is determined by the **beam width**.
- If the beam is too wide, adjacent structures cannot be resolved separately.
### Elevational Resolution
![[Pasted image 20240527090912.png#invert|300]]
- Elevational resolution refers to the system’s ability to distinguish between different elements at the **same depth but in different elevational planes** (similar to slice thickness).
- Elevational resolution depends not only on beam focusing but also on the **beam height** (the dimension perpendicular to the imaging plane) and, consequently, the **element height**.
- The narrower the beam in the elevational direction, the better the resolution.
#### Using Acoustic Lenses
- To improve elevational resolution at a specific depth, an acoustic lens can be used to focus the beam in the elevational plane.
- Similar to lateral resolution, optimizing the beam width enhances elevational resolution.
#### 1.5D Transducer Arrays
- For even better elevational resolution, 1.5D transducer arrays (multiple linear array transducers with five to seven rows) can be used.
- These arrays allow dynamic steering and focusing of the beam in the elevational dimension by activating different rows of elements.
- However, this approach may result in a decrease in temporal resolution due to the increased complexity.
#### 3D Imaging
- Full 2D transducer arrays with enhanced computational power enable 3D imaging.
- 3D imaging provides more uniform resolution throughout the image volume by combining information from different elevational planes.
### Temporal Resolution
- From an imaging or guidance point of view the US is the fastest imaging modality for real time investigation.
- The frame rate describes the number of frames we can acquire in one second:$$\text{frame rate}=\frac{\#\text{frames}}{\text{second}}=\frac{1}{T_\text{frame}}$$
- The time to acquire one frame ($T_{frame})$ depends on the number of scanned lines ($N_{lines}$​) and the time to scan one line ($T_{line}$​):$$T_{frame} = N_{lines} \times T_{line} $$
- $T_line$​ is equivalent to the PRP, which is the time it takes for an ultrasound pulse to travel to the desired depth and back:$$T_{line} = PRP = \frac{2 \times \text{desiredDepth}}{c} = 13\mu s \times \text{desiredDepth}$$where $c$ is the speed of sound in tissue.
- As the desired depth increases, the PRP and $T_{line}$​ increase, leading to a decrease in frame rate and temporal resolution.
- Reducing the number of scanned lines can increase temporal resolution but may decrease lateral resolution.
- Narrowing the field of view reduces the number of lines to acquire, which can be a compromise between temporal resolution and depth.
## 3D/4D Imaging
### 3D Imaging
- 3D ultrasound imaging involves volumetric reconstruction of the scanned area, creating a three-dimensional image of the structures
### 4D Imaging
- 4D imaging adds the dimension of time to 3D imaging, resulting in a dynamic sequence (like a video) that shows movement within the three-dimensional space
#### Transducer Design
- The transducer in 3D/4D imaging is almost spherical with a squared array inside. It emits waves in multiple planes—axial, lateral, and elevational—unlike traditional phased arrays that only send in one plane.
#### Image Generation
- By moving the probe over the point of interest, the transducer sends out waves in various directions, and the underlying software compiles these into a 3D image.
## Artifacts
- Ultrasound artifacts are discrepancies between the ultrasound image and the actual anatomy or physiology, often due to the inherent limitations of the technology or user error.
### General Assumptions in Ultrasound
- Ultrasound imaging simplifies complex physical principles to be practically useful.
- Assumptions like constant sound speed across different media and straight-line travel of waves are made for this purpose.
- The imaging process is operator-dependent, meaning the technique and pressure applied by the sonographer can affect the image quality.
### Artifact due to Refraction Angle
- Refraction occurs when ultrasound waves pass through boundaries between media with different densities, bending the waves.
- This can cause objects to be missed or mis-mapped in the image, creating a shadow effect where structures are obscured.
- To mitigate this, images from multiple orientations can be taken.
![[Pasted image 20240527100105.png|400]]
### Reverberation Artifact
- Occurs when there are multiple reflections between two strong reflectors, such as an air bubble and the transducer.
- This results in reverberation, where echoes bounce back and forth, creating equally spaced signals with decreasing amplitude.
- Using ultrasound gel and avoiding air bubbles is crucial to prevent this artifact.
![[Pasted image 20240527100126.png|400]]
### Mirror Artifact
- A highly reflective surface, like the diaphragm, can cause nearly total reflection.
- This leads to a “second” image of objects in front of the reflective surface, as if there were a mirror.
![[Pasted image 20240527100140.png|400]]
### Side Lobes Effects
- The transducer emits ultrasound waves primarily along the central axis, but some waves (side lobes) can travel in other directions.
- This can cause objects to appear multiple times or extended in the direction of the beam movement.
![[Pasted image 20240527100157.png#invert|400]]
### Artifacts due to Constant Sound Speed Assumption
- Assuming a constant sound speed in soft tissue can lead to displacements in the image behind objects with different sound speeds.
- Variations in tissue density and composition can affect the speed of sound, leading to inaccuracies in the displayed image.
![[Pasted image 20240527100210.png#invert|400]]
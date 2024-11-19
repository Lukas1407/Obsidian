> [!abstract] Definition
> Magnetic Gradient Fields are additional magnetic fields that vary linearly across the body in each of the three dimensions (x, y, and z). 

## Equation
$$\omega=\gamma(B_{0}+\Delta B)$$
- **Static Field ($B0$):** The main uniform magnetic field of the MRI scanner.
- **Gradient Field ($G$):** The additional field applied to create a spatial variation in the magnetic field. This is measured in Tesla per meter (T/m).
- **Magnetic Field Variations ($\Delta B_x, \Delta B_y, \Delta B_z$):** Changes in the magnetic field strength across each axis, calculated as $\Delta B_x = G \times x$, $\Delta B_y = G \times y$, $\Delta B_z = G \times z$.
## Effect on Nuclear Spins
The presence of gradient fields modifies the local magnetic field in each voxel. Since the frequency of nuclear spin precession (the Larmor frequency) is directly proportional to the magnetic field strength, varying the field strength changes the precession frequency of the spins in each voxel:
- **Linear Variation in Precession:** In the presence of a gradient, spins in different locations will precess at different frequencies. This difference allows the MRI system to map the origin of the signals based on frequency variations.
- **Directionality of Gradient Fields:** Gradient fields can be applied in any direction (x, y, z), and their polarity can be either positive (increasing the Larmor frequency) or negative (decreasing the Larmor frequency), depending on the direction and strength of the field.
## Pulse Sequence with Gradients
![[Pasted image 20240703092222.png#invert|200]]
### 1. **RF Pulse (90°)**
The RF pulse, shown at the top, is used to excite the hydrogen nuclei by flipping their spins from their equilibrium position (aligned with the main magnetic field, $B_0$ to a perpendicular orientation. This is typically a 90° flip, which maximizes signal detection as the spins start to precess around $B_0$
### 2. **Slice Selection Gradient ($G_s$)**
- **Purpose:** To excite only a specific slice of the body.
- **Activation:** It is turned on simultaneously with the RF pulse.
- **Function:** By applying this gradient, the magnetic field strength varies linearly along one axis (usually the z-axis), and only spins in a specific range of magnetic field strengths will resonate with the frequency of the RF pulse. This restricts the RF pulse's effect to a particular slice.
#### Relation Between Pulse Shape and Frequency Spectrum
![[Pasted image 20240703092730.png#invert|300]]
- A wide waveform RF pulse, as shown in your image, corresponds to a narrow frequency bandwidth. This means that it can excite spins very selectively within a narrow range of frequencies.
![[Pasted image 20240703092749.png#invert|300]]
- Conversely, a narrow waveform RF pulse corresponds to a wide frequency bandwidth. This means it can excite spins across a broader range of frequencies.
- For slice selection, the RF pulse needs to have a bandwidth that covers the range of frequencies corresponding to the slice thickness. This range ($\Delta F$) is given by $$\Delta F = \gamma G_s \Delta z$$where $\Delta z$ is the thickness of the slice and $G_s$​ is the strength of the slice selection gradient.
- Ideally, a sinc-shaped RF pulse, which has a rectangular frequency spectrum, would be used to precisely excite a specific bandwidth (slice) without affecting neighboring frequencies. However, a perfect sinc pulse requires an infinite duration, which is impractical.
- In practice, the sinc pulse is truncated to a finite duration, which leads to a sinc-shaped frequency spectrum with side lobes. These side lobes can cause some excitation outside the desired frequency range, leading to artifacts in the image.
### 3. **Phase Encoding Gradient ($G_{\phi}$)**
- **Purpose:** To encode spatial information along one axis of the image (usually the y-axis).
- **Activation:** It is switched on briefly right after the RF pulse and before the FID (Free Induction Decay) signal is read.
- **Function:** This gradient is applied briefly and varies linearly across the selected slice. Each row of spins acquires a different phase due to this gradient, which is crucial for reconstructing the image later. The phase accumulated by the spins depends on their location along the y-axis.
- -> This technique allows the MRI to distinguish signals along the $x$ and $y$ axes within the XY plane of the slice.
- After the slice has been selected using the slice selection gradient and the RF pulse, all the nuclei within this slice are precessing in phase at their respective Larmor frequencies if they are located along the gradient direction. However, to generate a 2D image of this slice, the MRI must be able to distinguish signals not just from different depths ($z$ axis, managed by slice selection) but also across the width and height ($x$ and $y$ axes) of the slice.
#### Application of Phase Gradient
- The phase encoding gradient is applied perpendicular to the slice selection gradient, typically along the $y$-axis.
- This gradient is briefly switched on for a very short period after the RF pulse and before the detection of the Free Induction Decay (FID) signal. It's also before the application of the frequency encoding gradient.
- When this gradient is applied, it causes a linear variation in the phase of the precessing nuclei across the $y$-axis. Nuclei at different positions along this axis accumulate phase at different rates proportional to their position. This variation in phase is temporary and only exists while the gradient is active.
- During the short duration when the phase encoding gradient is active, each row of nuclei across the $y$-axis accumulates a phase shift proportional to its position.
### 4. **Frequency Encoding Gradient ($G_f$)**
- **Purpose:** To encode spatial information along another axis of the image (usually the x-axis).
- **Activation:** It is switched on during the reading of the signal (FID).
- **Function:** While this gradient is on, the precession frequency of the spins varies linearly across the selected slice. Since the frequency of the emitted signal directly depends on the local magnetic field strength, this gradient allows each column of spins to emit signals at distinct frequencies. This frequency variation is crucial for determining the position of the signals along the x-axis.
#### Application of the Gradient
- Unlike the phase encoding gradient which is applied briefly and turned off before FID acquisition, the frequency encoding gradient is continuously applied while the FID signal is being read.
- When this gradient is applied, it causes a linear variation in the magnetic field strength across the slice along the direction of the gradient. This variation results in a corresponding linear change in the Larmor frequency of the hydrogen nuclei because the precession frequency of these nuclei is directly proportional to the local magnetic field strength.
- As the frequency encoding gradient increases the magnetic field linearly from one side of the slice to the other, nuclei located at different positions along this gradient will resonate at different frequencies. This is because their Larmor frequency ($\omega$) is given by $\omega = \gamma B$, where $\gamma$ is the gyromagnetic ratio and $B$ is the local magnetic field strength.
- Therefore, nuclei at one end of the slice (where the magnetic field is stronger due to the gradient) will precess faster (higher frequency), and those at the opposite end will precess slower (lower frequency).
### 5. **Signal**
The last line shows the signal received by the MRI's detectors. This signal, the FID, contains all the spatial information encoded by the gradients and is processed to reconstruct the image.


The explanation you're asking about pertains to how MRI pulse sequences manipulate the behavior of magnetic spins in tissue to create images with different contrasts, specifically focusing on T1-weighted imaging. Let's break down the details:

### 2.4 Pulse Sequence Overview
### 2.4.1 The Role of TR for T1
#### RF Pulse Application
- **Initial Condition:** When a 90° RF pulse is applied, it flips the net magnetization vector from its alignment along the z-axis (aligned with the main magnetic field \(B_0\)) to the xy-plane. This results in $M_{xy} = M_0$ (where \(M_0\) is the initial magnetization) and \(M_z = 0\), since all the magnetization is now transverse, and none is longitudinal.
#### Between RF Pulses
- **Relaxation:** Once the RF pulse is turned off, the spins begin to relax back toward their equilibrium state along the z-axis. This relaxation is not instantaneous and is characterized by the T1 relaxation time, which describes how quickly spins realign with the magnetic field \(B_0\).
- **Signal Reception:** The signal detected in MRI is from the transverse magnetization (\(M_{xy}\)). Immediately after the RF pulse, no information about the longitudinal relaxation (T1) is available because \(M_z\) has been zeroed out.
#### Timing and Reapplication of the RF Pulse
- **Choosing TR:** The repetition time (TR) is the interval between successive RF pulses. It is crucial in T1-weighted imaging because it controls how much longitudinal relaxation occurs between pulses. The equation \(M_z = M_0(1 - e^{-\frac{TR}{T1}})\) describes the recovery of the longitudinal magnetization.
  - If TR is short compared to T1, \(M_z\) does not fully recover between pulses. This enhances T1 contrast because differences in T1 times between tissues will result in significantly different \(M_z\) values at the time of the next pulse.
  - If TR is long (much longer than T1), \(M_z\) approaches \(M_0\) for all tissues, reducing T1 contrast as most tissues fully recover their longitudinal magnetization.
- **Effect of Successive RF Pulses:** When another 90° RF pulse is applied at time \(t = TR\), the amount of magnetization flipped into the xy-plane (\(M_{xy}\)) will be less than \(M_0\) if \(M_z\) has not fully recovered. This is expressed as \(M_{xy} < M_0\), reflecting incomplete T1 recovery.
#### Demonstration
![[Pasted image 20240625084218.png#invert|400]]
- **First RF Pulse**: A 90-degree RF pulse is applied, which flips the net magnetization vector $M_{0}$​ from the longitudinal axis (aligned with $B_{0}$​) to the transverse plane. At this moment, both water and fat have their magnetization vectors lying entirely in the transverse plane ($M_{xy} = M_0$ and $M_{z}​=0$).
- **Relaxation between Pulses**: After the first RF pulse, the magnetization vectors begin to relax back towards the longitudinal axis (z-axis). Fat, having a shorter T1 relaxation time, relaxes faster than water. This means that by the time the second RF pulse is applied, the magnetization vector for fat is more aligned with $B_0$​ compared to that of water.
- **Second RF Pulse**: When the second 90-degree RF pulse is applied after a time $TR$ (repetition time), the vectors are flipped again. Because fat's vector has realigned more towards the longitudinal axis than water's, fat's transverse magnetization $M_{xy}$​ after this pulse will be relatively greater than that of water.
The formula that describes the measured value is:$$

M_{xy} = M_0 (1 - e^{-\frac{TR}{T1}}) e^{-\frac{t}{T2}}$$
- **($1 - e^{-\frac{TR}{T1}}$)**: This part describes the recovery of longitudinal magnetization MzM_zMz​ during the time TRTRTR between pulses. It shows how much of M0M_0M0​ has recovered towards the longitudinal axis based on the tissue’s T1 value. For fat (short T1), this value recovers more significantly than for water (long T1).
- **$e^{-\frac{t}{T2}}$​**: This represents the exponential decay of the transverse magnetization due to T2 relaxation processes after the magnetization vector has been flipped back into the transverse plane by the second RF pulse.
### The Role of TE for T2
As we saw in the transverse magnetization over time graph, a delayed signal detection increases the contrast, since the gap between short T2 (fat) and long T2 (water) increases. This also increases the SNR. The FID is therefore calculated to be:$$ M_{xy} = M_0 \left(1 - e^{-\frac{TR}{T1}}\right) \cos(\omega_0 t) e^{-\frac{TE}{T2^*}} $$
- **$M_0$**: The initial magnetization in the longitudinal axis before any RF pulse is applied.
- **$1 - e^{-\frac{TR}{T1}}$**: This component describes the recovery of longitudinal magnetization $M_z$ during the repetition time $TR$. It reflects how much of the magnetization has returned to the longitudinal (z-axis) direction after a certain period governed by the tissue’s T1 relaxation time.
- **$\cos(\omega_0 t)$**: Represents the oscillatory behavior of the transverse magnetization due to the Larmor precession around the magnetic field $B_0$. $\omega_0$ is the Larmor frequency, which depends on the magnetic field strength and the gyromagnetic ratio of the nuclei.
- **$e^{-\frac{TE}{T2^*}}$**: This exponential term accounts for the decay of the transverse magnetization due to both intrinsic T2 relaxation and additional dephasing from magnetic field inhomogeneities (represented by $T2^*$ instead of pure T2).
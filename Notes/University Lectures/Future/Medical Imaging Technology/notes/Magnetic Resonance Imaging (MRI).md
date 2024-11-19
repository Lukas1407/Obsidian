> [!abstract] Definition
> Magnetic resonance imaging (MRI) is an imaging technique used primarily in medical settings to produce high quality images of the inside of the human body. MRI is based on the principles of [[Nuclear Magnetic Resonance (NMR)]], a spectroscopic technique used by scientists to obtain microscopic chemical and physical information about molecules.

- Useful links which also include questions: 
	- https://mri-q.com/index.html
	- https://www.cis.rit.edu/htbooks/mri/

## Advantages
- **Non-Ionizing Radiation**: MRI uses electromagnetic waves that are non-ionizing, meaning they do not damage tissues or DNA, unlike X-rays or CT scans.
- **Multi-Planar Imaging**: It can acquire images in any three-dimensional orientation, including oblique scans, providing comprehensive views of the anatomy.
- **High Soft Tissue Contrast**: MRI offers very high soft tissue contrast, making it excellent for visualizing organs, muscles, and the brain.
- **High Spatial Resolution**: It can produce high-resolution images with details finer than **1 mm**, allowing for the detection of small abnormalities.
- **Depth-Independent Resolution**: The image resolution and signal intensity are consistent regardless of the tissue’s depth within the body, avoiding penetration effects.
## Disadvantages
- **Patient Comfort**: The MRI environment can be uncomfortable, with long scanning times in a closed space potentially causing claustrophobia. The loud background noise during scans also requires ear protection.
- **Safety Concerns with Metal**: Patients with certain medical implants or metal in their bodies may not be able to undergo an MRI due to safety concerns.
- **Infrastructure Requirements**: MRI systems require expensive siting, including an RF-shielded room, substantial electrical power, and strict temperature and humidity control, along with a constant source of chilled water.
- **High Costs**: The cost of MRI scanners is relatively high. A typical clinical **1.5 Tesla (T)** whole-body imager costs around **$1–1.5 million**, and a **3 T** scanner can cost between **$2–3 million**.
## History
### Nuclear Magnetic Resonance (NMR) Origins
- **1922**: Physicists **Otto Stern** and **Walther Gerlach** demonstrated the existence of the magnetic moment of the atom.
- **1937**: **Isaac Rabi**, another physicist, discovered the phenomenon of nuclear magnetic resonance (NMR). Rabi described how nuclei could be induced to flip their principal magnetic orientation by an oscillating magnetic field. His group demonstrated NMR using a molecular beam in a vacuum, where individual nuclei were isolated from each other and their environment.
### Felix Bloch and Edward Mills Purcell
- **1946**: Independently, physicists **Felix Bloch** and **Edward Mills Purcell** demonstrated NMR in condensed matter.
- They placed a substance in a magnetic field, aligning the nuclei of its atoms.
- By bombarding it with radio frequency waves, they forced the nuclei out of alignment.
- As the nuclei returned to alignment, the atoms emitted unique electromagnetic signals.
- These signals could be analyzed to determine the chemical composition and molecular structure of the material.
- Both Bloch and Purcell paved the way for **Magnetic Resonance Spectroscopy (MRS)**.
### Challenges and Innovations
- Initially, MRS was slow in the time domain and produced weak signals, rendering it impractical for real-world applications.
- **1966**: **Richard R. Ernst**, a physical chemist, made a groundbreaking discovery.
- He replaced the slow, sweeping radio waves traditionally used in NMR spectroscopy with short, intense pulses.
- Ernst applied **Fourier transformation** to interpret the NMR signal measured as a function of time (called **FID-free induction decay**).
- By converting it to the signal intensity as a function of frequency, he dramatically increased the sensitivity of NMR techniques.
### Paul C. Lauterbur and Peter Mansfield
- **1971**: Paul C. Lauterbur developed a mechanism to encode spatial information into an NMR signal using magnetic field gradients.
- He published the theory behind it in **March 1973**, effectively inventing MR imaging.
- Around the same time, **Peter Mansfield** enhanced the technique by rapidly scanning the body and producing clearer, more precise images.
- Their combined efforts transformed MRI from an experimental tool into a clinical powerhouse.
### Clinical Adoption and Widespread Use
- The first clinical MRI scanners were installed in the early 1980s.
- Significant development followed in subsequent decades, leading to MRI’s widespread use in medicine today.

## How it works - An Overview
- It is based on the absorption and emission of energy in the radio frequency of the [[Electromagnetic (EM) Waves|electromagnetic spectrum]]
	- -> Non-ionizing
- Many scientists were thought that you can not image objects smaller than the wavelength of the energy being used to image. -> That's why MRI took so long to develop
- MRI gets around this limitation by producing images based on spatial variations in the phase and frequency of the radio frequency energy being absorbed and emitted by the imaged object. 
- The human body is primarily fat and water. 
- Fat and water have many hydrogen atoms which make the human body approximately 63% hydrogen atoms. 
- Hydrogen nuclei have an [[Nuclear Magnetic Resonance (NMR)|NMR]] signal. For these reasons magnetic resonance imaging primarily images the NMR signal from the hydrogen nuclei.
- When placed in a strong magnetic field, these hydrogen nuclei align themselves with the field.
- The MRI machine then applies radiofrequency (RF) pulses to perturb this alignment.
### Steps in Examination
1. The patient is placed in a magnet
2. A radio wave is sent in
3. The radio wave is turned off
4. The patient emits a signal which is detected
5. The image is reconstructed based on that signal
### Resonance and Relaxation
- **Resonance**: The RF pulse excites the hydrogen nuclei, causing them to absorb energy and move away from their aligned state.
- After the RF pulse is turned off, the nuclei return to their original alignment, releasing energy.
- This process is called **nuclear magnetic resonance (NMR)**.
#### Relaxation Times
- **T1 (Spin-Lattice Relaxation)**: The time it takes for hydrogen nuclei to return to their aligned state after excitation. Tissues with longer T1 values appear brighter in T1-weighted images.
- **T2 (Spin-Spin Relaxation)**: The time it takes for the nuclei to lose phase coherence (i.e., stop emitting a coherent signal). Tissues with longer T2 values appear brighter in T2-weighted images.
### Signal Detection and Tissue Contrast
- The emitted energy (NMR signal) is detected by the MRI machine’s receiver coils.
- The signal intensity depends on various tissue properties, leading to different contrasts in the resulting images.
### Factors Influencing Signal Intensity
- **Hydrogen Density**: The number of hydrogen nuclei within a voxel (a 3D volume element) affects the signal intensity. More hydrogen nuclei lead to a stronger signal.
- **Tissue Viscosity**: Viscous tissues (e.g., cartilage) alter the relaxation times of hydrogen nuclei, affecting signal intensity.
- **Diffusion**: Water molecules diffuse differently in different tissues. Fast diffusion results in higher signal intensity.
- **Blood Flow and Perfusion**: Blood flow affects the signal. For example, contrast agents can enhance blood vessels’ visibility.
## Physical Principles
- It is based on the phenomenon of Nuclear Magnetic Resonance produced by “spins” when interacting with RF waves (that produce an oscillating magnetic field)

### Spin
- Protons (positively charged) can be seen as little planets, that constantly spin around an axis
![[Pasted image 20240609094130.png|200]]
- -> Protons possess a spin
- The positive electrical charge, being attached to the proton, naturally spins around with it.
- And what is a moving electrical charge? -> an electrical current
- Where there is an electrical current there is also an electrical field! Physics
- Thus the proton has its own magnetic field
- When we put the protons in an external magnetic field, they align themselves with that field! (like a compass needle in the magnetic field of the earth)
- There are 2 ways the protons can align themselvs!
	- The protons may align with their South and North poles in the direction of the external field, parallel to it. 
	- Or they may point exactly in the complete opposite direction, anti-parallel.
- These types of alignments are on different energy levels
- Aligning anti-parallel requires obviously more energy (higher energy level)
- Naturally the preferred state of alignment is the one that needs less energy. So more protons are on the lower energy level
- The difference in number is, however, very small and depends on the strength of the applied magnetic field.
#### Precession
- We will see that the protons do not just lay there, aligned parallel or anti-parallel to the magnetic field lines. 
- Instead, they move around in a certain way. The type of movement is called precession
![[Pasted image 20240609095044.png#invert|600]]
- For reasons we will learn below, it is important to know how fast the protons precess. This speed can be measured as precession frequency $\omega_{0}$
- This precession frequency is not constant. It depends upon the strength of the magnetic field $B_{0}$ in which the proton is placed
	- The stronger the magnetic field, the faster the precession rate and the higher the precession frequency, given by the Larmor equation:$$\omega_{o}=\gamma B_{0}$$
	- The strength of the external magnetic file $B_{0}$ is given in [[Tesla]] $T$. $\gamma$ is the so called gyro-magnetic ratio
	- Precession does not require energy 
	- This gyro-magnetic ratio is different for different material
![[Pasted image 20240609095823.png|600]]
- There are millions of protons in our body, so its easy to imagine that at a certain moment there may be one proton pointing exactly in the opposite direction to another proton
	- -> the magnetic forces in the opposing directions cancel each other out
	- But as we have read: there are more protons pointing up than down, and the magnetic forces of these protons are not cancelled by others. So we are left with some protons (4 in our example) pointing up
- However, not only magnetic forces pointing up and down can cancel or neutralize each other. As the protons that are left pointing up precess, there may be one pointing to the right, when there is another one pointing to the left
- What we end up with in effect is a magnetic vector in the direction of the external magnetic field and this vector is a sum vector made up by adding the magnetic vectors of the protons pointing upwards.
- Now - what does this mean? This means that by placing a patient in the magnet of the MR unit , the patient himself becomes a magnet, i.e. has his own magnetic field. Why? Because the vectors of the protons, that do not cancel each other out, add up 
- Because it is longitudinal however, it cannot be measured directly.
#### Why don't all Protons are in lower Energy States?
- At non-zero temperatures, nuclei are in constant motion due to thermal energy.
- These movements result in collisions and interactions that can cause transitions between energy states.
- Thermal collisions provide energy that can excite spins from the lower energy state to the higher energy state.
- The likelihood of these transitions depends on the thermal energy relative to the energy difference between the spin states.
- The Boltzmann distribution describes the distribution of particles among different energy states in thermal equilibrium
- The probability $P(E_{i})$ of finding a particle in a state with energy $E_{i}$ is given by: $$P(E_i) \propto e^{-\frac{E_i}{k_B T}}$$where:
	- $k_{B}$​ is the Boltzmann constant.
	- $T$ is the absolute temperature.
- For two states with energies $E_{\text{low}}$ and $E_{\text{high}}$​, the ratio of the populations $N_{\text{high}}$​ and $N_{\text{low}}$ is: $$\frac{N_{\text{high}}}{N_{\text{low}}} = \frac{e^{-\frac{E_{\text{high}}}{k_B T}}}{e^{-\frac{E_{\text{low}}}{k_B T}}} = e^{-\frac{\Delta E}{k_B T}}$$
	- $\Delta E$ is the energy difference between the two states.
- At higher temperatures, the population difference between states decreases because thermal energy is sufficient to populate higher energy states more significantly.
- The number of protons in the high energy state is very low compared to the number of protons in the low energy state
![[Pasted image 20240610094428.png#invert|400]]
- -> Net Magnetization $M_0$ is very small (in the order of $\mu T$ )
- This is very small as compared to the main magnetic field and technically infeasible to be measured in equilibrium
#### RF Pulse
- The purpose of this RF pulse is to disturb the protons, which are peacefully precessing in alignment with the external magnetic field
- We need a certain RF pulse that can exchange energy with the protons to change their alignment.
##### Excitation
- The Larmor equation gives us the necessary frequency of the RF pulse to send in. Only when the RF pulse and the protons have the same frequency, can protons pick up some energy from the radio wave, a phenomenon called resonance
	- A proton in the state E↑ can switch to the state E↓ by absorbing a photon with energy equal to:$$\hbar\gamma B_{0}=\hbar\omega_{0}$$
	- -> Some protons pick up energy, and go from a lower to a higher energy level -> this effects the patients magnetization
	- Resonance is an energy transition that occurs when an object is excited by a frequency the same as its own
- In NMR, an external magnetic field $B_0$​ is used to align the nuclear spins of the sample. To study these spins, they must be excited from their equilibrium state, which is achieved using a radiofrequency (RF) field, denoted as $B_{1}(t)$
- $B_{1}​(t)$ must be perpendicular to the static magnetic field $B_{0}$​, typically oriented along the z-axis. This perpendicular orientation ensures that $B_{1}(t)$ can effectively interact with the spins and cause transitions between energy states.
- $B_{1}​(t)$ should have components oscillating near the resonant frequency $\omega_0$​. This matching frequency ensures resonance, where energy transfer between the RF field and the nuclear spins is most efficient.
- The $B_{1}$ field is produced by driving alternate electrical currents through specialized RFtransmit coils
##### Resulting Effects
- Another effect that happens is: the protons do not point in random directions any more, but move in step, in sync - they are "in phase" (transversal magnetization).
	- -> They now point in the same direction at the same time, and thus their magnetic vectors add up in this direction (longitudinal magnetization)
![[Pasted image 20240609101347.png|600]]
- This newly established magnetic vector naturally does not stand still, but moves in line with the precessing protons, and thus with the precession frequency
- In summary: the RF pulse causes longitudinal magnetization to decrease, and establishes a new transversal magnetization
- As the transversal magnetic vector moves around with the precessing protons, it induces an electric current
- As soon as the RF pulse is switched off, the whole system, which was disturbed by the RF pulse, goes back to its original quiet, peaceful state, it relaxes. The newly established transverse magnetization starts to disappear (a process called transversal relaxation), and the longitudinal magnetization grows back to its original size (a process called longitudinal relaxation).
- Not all protons switch to their original energy levels at once 
	- Their received energy is emitted 
- The time that it takes for the longitudinal magnetization to recover, to go back to its original value, is described by the longitudinal relaxation time, also called $T1$
- If one plots the longitudinal magnetization vs . time after the RF pulse was switched off, one get s a so called $T_1$-curve:
![[Pasted image 20240609102853.png|400]]
- $T_1$ was defined as the time when about 63% of the original longitudinal magnetization is reached ($63\% = 1-1/e$)
-  radio frequency pulse that has the same frequency as the precessing protons, can cause resonance, transfer energy to the protons. This results in more protons being anti-parallel and thus neutralizing/cancelling more protons in the opposite direction. Consequence: the longitudinal magnetization decreases
- when the RF pulse is switched off - longitudinal magnetization increases again; this longitudinal relaxation is described by a time constant $T_1$, the longitudinal relaxation time
- $1/T_1$ is also called longitudinal relaxation rate
#### Different Precessing Frequencies
- The field of the MR magnet, in which the patient is placed, is not totally uniform, not totally homogeneous, but varies a little, thus causing different precession frequencies
- Each proton is influenced by the small magnetic fields from neighboring nuclei, that are also not distributed evenly, thus causing different precession frequencies too.
- -> These internal magnetic field variations are somehow characteristic for a tissue!

- So after the RF pulse is switched off, the protons are no longer forced to stay in step -in phase-; and as they have different precession frequencies, they will be soon out of phase.
![[Pasted image 20240610072721.png|400]]

#### Transversal Magnetization
- The $T_{2}$ curve:
![[Pasted image 20240610073151.png|400]]

- $T_2$ is the time when transversal magnetization decreased to 37% of the original value ($37\% = 1/e$)
- when the RF pulse is switched off transversal magnetization decreases and disappears; this transversal relaxation is described by a time constant T2, the transversal relaxation time.
- $1/T_2$ is called the transversal relaxation rate
#### 90° Pulse
![[Pasted image 20240610075222.png|400]]
- We have 2 protons pointing up (all other are canceling each other out)
- Now let us send in an RF pulse, which has just the correct strength and duration, that one of the two protons picks up energy, to go into the higher state of energy
- Th e longitudinal magnetization (up to now resulting from two protons pointing up ) will decrease , in our example to zero (one pointing up is neutralized by one pointing down). 
- But: as both protons are in phase, now there is a transversal magnetization which had not been there before.
- This can be looked at - i n effect - that a longitudinal magnetic vector is tilted 90° to the side
- -> An RF pulse which "tilts " the magnetization 90 ° is called a 90° pulse
- Naturally, other RF pulses are also possible, and are named accordingly, e.g . 180° pulse .
- Flip angle, also called tip angle, is the amount of rotation the net magnetization $M$ experiences during application of a radiofrequency (RF) pulse
- A rectangular RF pulse is characterized by a constant amplitude $B_1$​ and a specific duration $t_p$​.
- The flip angle $\alpha$ caused by a rectangular pulse can be approximated using the formula: $$\alpha = \gamma B_1 t_p​$$ where:
    - $\gamma$ is the gyromagnetic ratio of the nucleus (e.g., protons).
    - $B_1$​ is the amplitude of the RF field.
    - $t_p$​ is the duration of the RF pulse.
- For more complex RF pulses, where the amplitude $B_1$​ may vary over time, the flip angle $\alpha$ can be calculated using the integral of the product of $\gamma$ and $B_1$​ over the pulse duration: $$\alpha = \int_0^t \gamma B_1(\tau) \, d\tau$$
#### Tissue Characteristics
- Previously it was believed that measuring the relaxation times, would give tissue characteristic results, and thus enable exact tissue typing. 
- This, however, proved to be wrong, as there is quite some overlap of time ranges; and also $T_1$ is dependent on the magnetic field strength used for the examination
	- As we heard in the beginning, the precession frequency depends on magnetic field strength, a relationship described by the Larmor equation. If we have a stronger magnetic field, then the protons precess faster. And when they precess faster, they have more problems handing down their energy to a lattice (the surrounding) with more slowly fluctuating magnetic fields.
- Water/liquids have a long $T_1$ and a long $T_2$
- Compared to liquids/water fat has a short $T_1$ and a short $T_2$
##### What influences $T_1$
- $T_1$ depends on tissue composition, structure and surroundings
- $T_1$-relaxation has something to do with the exchange of thermal energy, which is handed over from the protons to the surroundings, the lattice
- The precessing protons have a magnetic field, that constantly changes directions, that constantly fluctuates with the Larmor frequency. 
- The lattice also has its own magnetic fields. 
- The protons now want to hand energy over to the lattice to relax. 
- This can be done very effectively, when the fluctuations of the magnetic fields in the lattice occur with a frequency that is near the Larmor frequency
- When the lattice consists of pure liquid/water, it is difficult for the protons to get rid of their energy, as the small water molecules move too rapidly. And as the protons (which are on the higher energy level) cannot hand their energy over to the lattice quickly, they will only slowly go back to their lower energy level, their longitudinal alignment.
- When the lattice consists of medium-size molecules (most body tissues can be looked at as liquids containing various sized molecules, kind of like a soup), that move and have fluctuating magnetic fields near the Larmor frequency of the precessing protons, energy can be transferred much faster, thus $T_1$ is short.
##### What influences $T_2$
- $T_2$-relaxation comes about when protons get out of phase, which - as we already know - has two causes: inhomogeneities of the external magnetic field, and inhomogeneities of the local magnetic fields within the tissues
- As water molecules move around very fast, their local magnetic fields fluctuate fast, and thus kind of average each other out, so there are no big net differences in internal magnetic fields from place to place. And if there are no big differences in magnetic field strength inside of a tissue, the protons stay in step for a long time, and so $T_2$ is longer.
- With impure liquids, e.g. those containing some larger molecules, there are bigger variations in the local magnetic fields. The larger molecules do not move around as fast, so their local magnetic fields do not cancel each other out as much. These larger differences in local magnetic fields consequently cause larger differences in precession frequencies, thus protons get out of phase faster, $T_2$ is shorter.

- ->All these processes influence how your MR picture will finally look!
##### Total Magnetic Moment of the Tissue
- The sum of the transversal magnetization vector and the longitudinal magnetization vector
- Our magnetic sum vector during relaxation goes back to a longitudinal direction, in the end equaling the longitudinal magnetization
- What we have to remember is that this whole system actually is precessing, including the sum magnetic vector/moment. And thus the sum vector will actually perform a spiraling motion
![[MergedImages (1).png]]
- In this illustration only the longitudinal and transversal magnetization vectors are depicted. 
- In (a) - before the RF pulse - there is only longitudinal magnetization. 
- Immediately after the 90° RF pulse there is no longitudinal but new transversal magnetization (b), and this transversal magnetization vector is spinning around. 
- With time this transversal magnetization decreases, while longitudinal magnetization increases (c-d), until the starting point with no transversal but full longitudinal magnetization is reached again (e). 
- Transversal and longitudinal magnetization vectors add up to a sum vector (pink). This sum vector performs a spiraling motion (f) when it changes its direction from being in the transversal (x-y) plane (no longitudinal magnetization) to its final position along the z-axis (no transversal magnetization).
- This change in magnetic field induces an electrical current which we can measure
	- This type of signal is called a FID signal, from free induction decay. It is easy to imagine, that you get a very good strong signal directly after the 90° RF pulse
	- ![[Pasted image 20240610081154.png|300]]
- Immediately after the RF pulse relaxation begins; transversal magnetization starts to disappear and longitudinal relaxation begins to reappear. The sum magnetic vector goes back to its original longitudinal alignment, the signal disappears.
##### Bloch Equation
- Represents the net magnetic moment per unit volume of the sample, comprising longitudinal ($M_z$​) and transverse ($M_x$​, $M_y$​) components.
- $T_1$​ (Spin-Lattice Relaxation Time): Describes the rate at which $M_z$​ returns to its equilibrium value.
- $T_2$​ (Spin-Spin Relaxation Time): Describes the rate at which $M_x$​ and $M_y$​ decay due to interactions among spins, leading to de-phasing.
- The $M_{z}$​ component returns to its equilibrium value $M_{0z}$ with a characteristic time $T_1$$​.$
	- The differential equation for $M_z$​ is: $$\frac{dM_z}{dt} = -\frac{M_z - M_{0z}}{T_1}​​$$
	- Solving this gives: $$M_z(t) = M_{0z} \left(1 - e^{-\frac{t}{T_1}}\right)$$
- The $M_z$​ and $M_y$​ components decay due to spin-spin interactions with a characteristic time $T_2$​.
	- The differential equation for $M_x$​ and $M_y$​ is: $$\frac{dM_{x,y}}{dt} = -\frac{M_{x,y}}{T_2}$$
	- Solving these gives: $$M_{x,y}(t) = M_{x,y}(0) e^{-\frac{t}{T_2}}$$
- [BLock Simulator](https://www.drcmr.dk/BlochSimulator/)
#### Pulse Sequence
- When you use more than one RF pulse - a succession of RF pulses - you use a so-called pulse sequence. 
- As you can use different pulses, e.g. 90° or 180° pulses, and the time intervals between successive pulses can be different, there can be many different pulse sequences. 
- The choice of a pulse sequence will determine what kind of signal you get out of a tissue. 
- So it is necessary to carefully chose and also describe the pulse sequence for a specific study.
- $TR$ = time to repeat
- The $T_1$-curves for brain and for cerebrospinal fluid (CSF):
	- ![[Pasted image 20240610082001.png|300]]
	- At the time 0 we have no longitudinal magnetization at all, and this can be the time immediately after our first 90° pulse. 
	- When we wait a long time before we repeat the 90° pulse ($TR_{\text{long}}$), longitudinal magnetization has pretty much recovered. The longitudinal magnetic vectors that will be "tilted" 90° differ only in a small amount, so there will be only a small difference in signal intensity, i.e. tissue contrast between brain and CSF. 
	- If we, however, send in the second pulse after the shorter $TR_{\text{short}}$ the difference in longitudinal magnetization is rather large, so there will be a better tissue contrast.
	- Why are the signals after a very long time TR between pulses not identical?
		- Because it depends on other parameters than $T_{1}$ for example the proton density
		- $T_1$- and proton density-weighted images

#### Stern and Gerlach Experiment
- They observed that when elementary particles (such as electrons, protons, and nuclei) interact with a magnetic field, they exhibit a specific behavior.
- The experiment involved passing a beam of silver atoms through an inhomogeneous magnetic field.
- Contrary to classical expectations, the silver atoms did not align smoothly along the magnetic field.
- Instead, they split into distinct lines on a detector screen.
- The lines corresponded to specific orientations of the atom’s intrinsic property—**spin**.
##### Conclusion
- Spin is analogous to the classical concept of angular momentum but exists at the quantum level.
- It represents the orientation of the particle in space.
- Importantly, spin interacts with external magnetic fields.
![[Pasted image 20240610082821.png#invert|700]]


### Spin in Quantum Mechanics
- The particle itself is not spinning, it just behaves like it does.
- It is an intrinsic property of subatomic particles like electrons and protons. It is not due to any physical spinning motion but is a fundamental characteristic of the particle.
- Unlike the continuous range of values for angular momentum in classical mechanics, spin angular momentum and its associated magnetic moment are quantized.
- This means that spin can only take on a limited set of discrete values. For example, an electron can have a spin of $+1/2$ or $-1/2$, but not any value in between
![[Wave-Particle Duality]]
- This wave-like character is expressed in terms of [[The Wave Function]], $\Psi(t)$, which is a time-dependent solution to the Schrödinger equation
![[Schrödinger Equation]]
![[Eigenstates and Eigenvalues]]
#### Spin States of a Hydrogen Nucleus
- A hydrogen nucleus (a proton) has spin of $\frac{1}{2}$, so there are two principal spin states
	- **Spin Up**: $\left| +\frac{1}{2} \right\rangle$
	- **Spin Down**: $\left|-\frac{1}{2} \right\rangle​$
	- These states are often represented using the "ket" notation from Dirac notation.
	- This splitting of energy levels in a magnetic field is known as the [[Zeeman Effect]]
- The wave function for a proton can be expressed as a linear combination of these states:$$\Psi(t) = a |+\frac{1}{2}\rangle + b |-\frac{1}{2}\rangle$$
	- $a(t)$ and $b(t)$ are complex numbers, known as quantum amplitudes, which can vary over time.
	- These amplitudes describe the probability amplitudes of the proton being in either of the 2 spin states
	- The probabilities of the proton being in each state are given by $|a(t)|^2$ and $|b(t)|^2$, respectively.
#### Energy Levels and Frequency
- The energy of the proton in the magnetic field depends on the spin state: $$E=-\mu\cdot B$$
	- where $\mu$ is the magnetic moment of the proton. The magnetic moment is related to the proton’s spin and the external magnetic field $B_{0}$​.
##### Energy States
- For $\left| +\frac{1}{2} \right\rangle$​ (spin up): $$E_+ = -\frac{1}{2} \hbar \gamma B_0$$
- For $\left| -\frac{1}{2} \right\rangle$ (spin down): $$E_- = \frac{1}{2} \hbar \gamma B_0$$
- Here, $\gamma$ is the gyromagnetic ratio of the proton.
- The difference in energy $\Delta E$ is then given by: $$\Delta E = E_{+}-E_{-}=\hbar \gamma B_0=\hbar\omega_0$$
##### Frequency
- The frequency associated with the $\left| +\frac{1}{2} \right\rangle$ state is $$\frac{1}{2}\omega_{0}$$
- The frequency associated with the $\left| -\frac{1}{2} \right\rangle$ state is $$-\frac{1}{2}\omega_{0}$$
- With the Larmor frequency $\omega_0$
#### Number of Spin States
- The number of [[Eigenstates and Eigenvalues|eigenstates]] (or pure spin states) for a nucleus with spin $I$ is given by $2I + 1$.
- Here, $I$ represents the nuclear spin [[Quantum Number|quantum number]] (also called spin)
##### Odd Atomic Mass
- The nuclear spin quantum number $I$ is a fraction of 2 (e.g., $I = 1/2, 3/2, 5/2$).
##### Even Atomic Mass and Even Atomic Number
- The nuclear spin quantum number $I$ is 0 (no NMR signal).
##### Even Atomic Mass and Odd Atomic Number
- The nuclear spin quantum number $I$ is an integer (e.g., $I = 1, 2, 3, 4$).





#### Quantization of Nuclear Spin Angular Momentum 
- The magnitude of the total nuclear angular momentum $J$ combines the nuclear spin $I$ and any other relevant angular momentum contributions, such as orbital angular momentum in more complex systems.
- It can be quantized as:$$J = \frac{h}{2\pi}\sqrt{I(I+1)} = \hbar\sqrt{I(I+1)}$$
	- where $\hbar = \frac{h}{2\pi}$​ is the reduced Planck constant
#### Measurement and Collapse of Spin State
- When a nucleus is placed in an external magnetic field $B_0$​, the spin component along the direction of the magnetic field (usually chosen as the z-axis) is quantized.
- The z-component of the nuclear spin$I_{z}$ takes on discrete values: $$I_z = m \hbar$$
	- where $m$ is the magnetic quantum number, and it ranges from $-I$ to $+I$ in integer steps.
##### Wave Function Before Measurement
- Before measuring the spin state, the wave function of the nucleus can be in a superposition of different spin states.
- This superposition is described by a combination of eigenstates of $I_z$​, each with its respective probability amplitude
##### Measurement of Spin State
- When we measure the spin state along the z-axis, we obtain a specific value $I_z = m \hbar$
- The act of measurement forces the wave function to collapse into one of these possible eigenstates
##### Collapse to a Specific State
- Upon measurement, the wave function collapses to a specific eigenstate $|m\rangle$ corresponding to the observed value of $i_{Z}$​.
- This collapse is instantaneous, and the system is found in one particular state, not a superposition

## More Parameters
### Echo Time $TE$
- **Definition:** TE is the Echo Time, which represents the time interval from the center (or start, depending on the definition in literature) of the RF (Radio Frequency) pulse to the peak of the echo signal received.
- **Role in MRI:** The choice of TE affects how much the signal has decayed due to T2 relaxation by the time it is measured. A longer TE allows more T2 decay, making the MRI more sensitive to differences in the T2 relaxation properties of different tissues. This is crucial for generating contrast based on T2 or T2* properties.
### Repetition Time $TR$
- **Definition:** TR is the Repetition Time, defining the duration between the starts of consecutive RF pulses in a sequence.
- **Role in MRI:** TR determines how much longitudinal relaxation (recovery) occurs between successive pulse sequences. A shorter TR minimizes recovery time, leading to T1 weighting of the image, as tissues with different T1 relaxation times will recover at different rates. A longer TR allows full recovery, reducing T1 effects and providing more proton density or T2 weighting.
![[Pasted image 20240625080241.png#invert|400]]
- **Top Row (TR):** Each segment between RF pulses signifies the TR period. This period determines how much time is left for tissues to return to their equilibrium state (full recovery or partial, depending on the length of TR).
- **Bottom Row (TE and Signal):** The time between the start or center of the RF pulse and the peak of the echo signal (depending on definition) is shown as TE. The amplitude of the signal received (indicated by the "signal" label and a downward arrow in the diagram) represents how much magnetization remains in the transverse plane, affected by T2 decay.
### T2 vs T2*
- T2* is an MRI parameter that represents the observed decay rate of the transverse magnetization due to a combination of inherent magnetic properties of tissues and external magnetic field inhomogeneities.
#### Components of T2* Decay
1. **Intrinsic T2 Relaxation:**
    - This is the natural exponential decay of the transverse magnetization due to interactions at the molecular level within a tissue, specifically spin-spin relaxation. It is independent of any external factors and is a fundamental property of the tissue itself.
2. **Magnetic Field Inhomogeneities:**
    - These are variations in the external magnetic field, which can be caused by imperfections in the MRI magnet, the presence of metal in the patient, or variations in the magnetic susceptibility of different tissues. These inhomogeneities lead to dephasing of the spins faster than what would be caused by pure T2 relaxation.
![[Pasted image 20240625080607.png#invert|200]]
- The observed decay rate of transverse magnetization, described by T2*, is shorter than the true T2 relaxation time due to these additional dephasing effects. The relationship between T2* and T2 can be described by the equation: $$\frac{1}{T2^*} = \frac{1}{T2} + \gamma \Delta$$Where $\gamma$ represents the gyromagnetic ratio, and $\Delta B$ represents the magnetic field inhomogeneity.
#### Effects of $B0$ Field Inhomogeneities
![[Pasted image 20240625080820.png#invert|400]]

- **Central Circle ($B0$):** Represents an area with a uniform magnetic field, where the precession frequency is ideal and leads to expected T2 decay without additional dephasing.
- **Ellipses ($B0$ ± ab):** These represent areas where the magnetic field strength deviates slightly from the uniform field (either stronger or weaker). These deviations cause the spins in these areas to precess at slightly different frequencies (either faster or slower than at $B0$).
#### Impact on Signal and Precession
The changes in magnetic field strength result in variations in the Larmor frequency (ω), the frequency at which magnetic resonance occurs. This can be expressed as: $$\omega^* = \omega_0 \pm \gamma \Delta B_0$$ where:
- **$\omega$*** is the modified precession frequency due to field inhomogeneity,
- **$\omega_{0}$** is the original Larmor frequency in a perfectly homogeneous field,
- **$\gamma$** is the gyromagnetic ratio, and
- **$\Delta B_{0}$** represents the deviation in the magnetic field from its ideal value.
As spins move through these areas of inhomogeneity, their phase changes due to the altered precession frequency. This leads to a rapid dephasing of the net magnetization vector across the sample, enhancing the T2* decay effect. Since this dephasing occurs due to external factors rather than intrinsic molecular interactions, it happens before T2 can manifest fully, leading to the observed difference in decay rates as shown in your first diagram.
![[MRI - Free Induction Decay (FID)]]
## Signal interpretation
Tissues with high $M_{x,y}$ results in a strong signal (white), whereas tissues with low $M_{x,y}$ result in weak signals (black). Air is always black due to low proton density and bone has very short T2, so $M_{x,y}$ decreases quickly.
### Explanation of MRI Image Types:
![[Pasted image 20240625083231.png|250]]
1. **Proton Density (PD) Image:**
    - **Characteristics:** PD images primarily reflect the density of hydrogen protons in the tissue. These images are generated with relatively short TE (echo time) and long TR (repetition time) to minimize T1 and T2 relaxation effects.
    - **Interpretation:** Areas with high proton density, like fluid-filled spaces, appear bright (white), while areas with low proton density, like dense fibrous tissue or air, appear dark (black). In your image, the brain tissue shows moderate intensity, reflecting the mixed composition of the tissue.#
    - Here, both fat and water have enough time to recover their longitudinal magnetization fully. Since TE is still short, the T2 decay effect is minimal. This setting highlights the inherent proton density of different tissues without significant weighting from T1 or T2 decay.
    - With a long TR, the contrast based on T1 differences diminishes, emphasizing proton density.
2. **T1-Weighted Image:**
    - **Characteristics:** T1 images are produced using short TR and short TE. This sequence highlights the longitudinal relaxation time—the time it takes for spun protons to realign with the magnetic field after the RF pulse is turned off.
    - **Interpretation:** Tissues with fast T1 relaxation times (like fat) appear bright, while those with slow T1 relaxation (like fluid) appear darker. In the provided image, the cerebrospinal fluid (CSF) in the ventricles appears dark, whereas the white matter appears brighter than in the PD image.
    - Short TE minimizes the influence of T2 relaxation, ensuring that the signal decay due to T2 is not significant. Short TR ensures that there isn't enough time for tissues with different T1 values to fully recover their longitudinal magnetization between pulses. This setting emphasizes differences in T1 relaxation rates between tissues, making T1 contrasts prominent.
    - In T1-weighted settings (short TE and TR), fat recovers faster and is brighter relative to water.
3. **T2-Weighted Image:**
    - **Characteristics:** T2 images use long TR and long TE, emphasizing the transverse relaxation time—the rate at which the spins lose phase coherence in the transverse plane.
    - **Interpretation:** Tissues with long T2 relaxation times, like fluids, appear very bright. For instance, the CSF in the ventricles is bright white, indicating a high fluid content.
    - Long TE allows significant dephasing of the transverse magnetization components due to T2 decay. The long TR allows for complete recovery of longitudinal magnetization, making the image more sensitive to differences in T2 decay times.
    - Tissues like fat and water show significant contrast differences because their T2 relaxation times are distinctly different, with water retaining signal longer than fat.
4. **Magnetic Resonance Angiography (MRA):**
    - **Characteristics:** MRA images are used to visualize blood vessels and are often enhanced by the use of contrast agents that affect the magnetic properties of blood. Techniques can vary but often involve sequences that are sensitive to flow or that suppress signals from static tissue.
    - **Interpretation:** In your image, the blood vessels appear bright against a suppressed background, highlighting the vascular architecture within the brain.
![[Pasted image 20240625085421.png#invert|]]
## Contrast
- Different tissue has different T1 and T2 values, resulting in a faster or slower decay depending on the tissue. 
![[Pasted image 20240625083646.png#invert|400]]
### 1. Longitudinal Magnetization ($M_z$)
In the first graph (Longitudinal Magnetization), you can see two curves for fat and water:
- **Fat** has a shorter T1 relaxation time. This means it returns to equilibrium (realigns with the magnetic field $B_0$​) more quickly after the RF pulse is turned off. As a result, fat reaches its maximum magnetization sooner than water, making it appear brighter in T1-weighted images when measured at optimal times.
- **Water**, on the other hand, has a longer T1 relaxation time and hence realigns more slowly, making it appear darker in comparison to fat at shorter measurement intervals.
### 2. Transverse Magnetization ($M_{x,y}$)
In the second graph (Transverse Magnetization), the relaxation behavior is depicted differently:
- **Fat** shows faster decay of its transverse magnetization due to a shorter T2 relaxation time. It loses phase coherence quickly, resulting in a rapid decrease in signal.
- **Water** retains its transverse magnetization longer because of its longer T2 relaxation time, keeping the signal stronger for a longer period and thus appearing brighter in T2-weighted images.
### Contrast and Repetition Time (TR)
![[Pasted image 20240625083759.png#invert|400]]
- **Short TR**: Here, the contrast between fat and water is more pronounced because fat recovers its longitudinal magnetization much faster than water. Using short TR in T1-weighted imaging exploits the rapid recovery of fat, maximizing the contrast at this interval.
- **Long TR**: As TR increases, both fat and water have more time to recover toward their full longitudinal magnetization, reducing the contrast between them. At long TRs, differences in T1 values become less significant, and the image becomes more uniform in terms of signal intensity.
### Poor Contrast
- Long TE and Short TR
- This combination is generally non-ideal for most diagnostic purposes because short TR does not allow enough time for significant recovery of longitudinal magnetization, and long TE causes extensive T2 decay, reducing overall signal intensity and contrast.
- The chart indicating poor contrast with long TE and short TR illustrates that neither T1 nor T2 characteristics are optimally displayed, leading to images with low diagnostic value.
![[Pasted image 20240625085552.png|300]]
## Pulse Sequence

![[MRI - Pulse Sequence]]
## Weighting
$$ M_{xy} = \kappa \rho (1 - e^{-\frac{TR}{T1}}) \cos(\omega_0 t) e^{-\frac{TE}{T2^*}} $$
This formula expresses the transverse magnetization $M_{xy}$ as a function of several factors:
- **$\kappa \rho$**: Represents the initial magnetization proportional to the proton density $\rho$ of the tissue. Here, $\kappa$ is a proportionality constant which includes factors like the gyromagnetic ratio and the strength of the main magnetic field $B_0$.
- **$1 - e^{-\frac{TR}{T1}}$**: Describes the recovery of longitudinal magnetization due to T1 relaxation during the time between successive RF pulses (TR).
- **$\cos(\omega_0 t)$**: Reflects the precessional motion of the magnetization around the magnetic field $B_0$, where $\omega_0$ is the Larmor frequency.
- **$e^{-\frac{TE}{T2^*}}$**: Accounts for the decay of the transverse magnetization due to T2* relaxation (which includes T2 relaxation and dephasing from field inhomogeneities) over the echo time TE.
## Spin Echo 
![[MRI - Spin Echo]]
## Exercise
![[Pasted image 20240625090007.png#invert|600]]
![[Pasted image 20240625090036.png#invert|600]]
### Solution
- [[Exercice Lecture 8_9 sulution.pdf]]
## Spatial Encoding
The detected signals represent the sum of responses from all excited tissues within the MRI scanner's field. To create a detailed image, the MRI system must determine the origin of these signals within the body. This process is known as spatial encoding. It requires a method to link (encode) the detected signals to specific locations or volumes (voxels) within the body.
- To achieve spatial encoding, MRI uses [[Magnetic Gradient Field]]
## Instrumentation
![[Pasted image 20240703094245.png#invert|400]]
### Main Components Inside the Scanner:
#### Magnet:
  - This is the core component of an MRI scanner. It creates a powerful and uniform magnetic field, typically using superconducting coils that are cooled with liquid helium. The main magnetic field, denoted as $B_0$, aligns the magnetic moments of the nuclei in the body.
##### Classification by Shape
MRI magnets can primarily be classified by their shape, which affects the design of the scanner and the patient experience.
- **Cylindrical-Closed Bore:**
    - **Description:** These are the most common type of MRI scanners. They feature a long, narrow tube or bore where the patient lies during the scan. The magnet has a cylindrical shape with the magnetic field oriented along the head-to-feet direction of the patient.
    - **Advantages:** Offers high field strength and uniformity, which results in higher resolution images and faster scan times.
    - **Considerations:** Some patients may experience claustrophobia due to the enclosed space.
- **Open Bore:**
    - **Description:** Open MRI scanners are designed with larger or open sides, making them less claustrophobic. The magnetic field in open MRI systems is typically oriented perpendicular to the head-feet direction, running horizontally through the body.
    - **Advantages:** More comfortable for patients, particularly those who are claustrophobic, obese, or children who may require a parent's presence.
    - **Considerations:** Typically have lower field strength than closed bore systems, which can result in longer scan times and potentially less detailed images.
##### Classification by How the Field is Generated
The technology used to generate the magnetic field also classifies MRI systems:
- **[[Superconductor|Superconducting]] Coils:**
    - **Description:** These are the most widely used systems in clinical settings. The coils are made from superconducting materials that, when cooled to very low temperatures (using liquid helium), can carry electricity without resistance, allowing for very high and stable magnetic fields.
    - **Field Strengths:** Common clinical scanners operate at 1.5 to 3 Tesla. Research scanners can go up to 7 Tesla and are now being introduced into clinical settings. There are also specialized ultra-high-field MRI systems, such as 12 Tesla, mainly used for research and small animal studies.
    - **Advantages:** High field strength allows for very high-resolution images and efficient scanning.
- **Permanent Magnets:**
    - **Description:** Some open MRI systems use permanent magnets, which do not require electricity to maintain the magnetic field. These are less common in clinical settings.
    - **Field Strengths:** Generally lower than those of superconducting systems.
    - **Advantages:** Lower operational costs since there's no need for cooling to superconducting temperatures.
    - **Considerations:** The lower field strength limits the resolution and speed of the MRI scans.
- **Resistive Coils (Obsolete):**
    - **Description:** Older systems used resistive electromagnets, which require a continuous supply of electricity to maintain the magnetic field.
    - **Considerations:** These are largely obsolete in modern MRI applications due to their inefficiency and the high costs associated with the power and cooling required.
#### Gradient Coils:
  - These coils are used to superimpose variable magnetic fields on top of the main magnetic field at different strengths and directions. This variation allows for spatial encoding of the MRI signals, enabling the machine to create images that can differentiate between different locations within the body.
#### RF Coil:
  - Radio Frequency (RF) coils are used to transmit RF pulses into the body to disturb the alignment of the magnetized nuclei and to receive the signals emitted by the nuclei as they return to their equilibrium state. There are different types of RF coils for different types of scans, some designed for transmitting, others for receiving, and some do both.
##### RF Coils as Transmitters
- **Function:** When used as transmitters, RF coils generate an oscillating or rotating magnetic field, known as $B_1$​, which is perpendicular to the static main magnetic field ($B_0$​). This perpendicular arrangement is crucial for effectively manipulating the magnetic moments of the nuclei in the body.
- **Mechanism:** The $B_1$​ field is applied at the Larmor frequency, which corresponds to the frequency at which the nuclei precess in the $B_0$​ field. By broadcasting at this frequency, RF coils can effectively flip or tilt the spins of the nuclei, moving them out of alignment with $B_0$​. This displacement is what allows the nuclei to emit signals as they realign with $B_0$​, which are then detected to create MRI images.
- [[Linearly Polarized (LP) Coils]]
- [[Circularly Polarized (CP) Coils]]
##### RF Coils as Receivers
- **Function:** As receivers, RF coils detect the MR signals emitted by the nuclei as they relax back into alignment with the $B_0$​ field after the RF pulse is switched off. The signals detected are essentially small electromagnetic waves emitted by the precessing nuclei.
- **Sensitivity:** The design and positioning of the receiver coils are critical for capturing these signals effectively. Receiver coils are often placed as close as possible to the anatomy being imaged to enhance signal detection, which improves the signal-to-noise ratio (SNR) of the resulting image.
- [[Frequency Synthesizer]]
- [[Pulse Modulator]]
- The orientation and placement of the coil in the xy-plane affect how the signal is detected. Coils can be aligned along the x or y axes to detect linearly polarized waves.
- **Using Two Coils in Quadrature ([[Circularly Polarized (CP) Coils|CP]]):** By placing two coils, each on the x and y axes, and receiving signals in quadrature (90 degrees phase shifted relative to each other), the setup captures more comprehensive data. This technique effectively increases the received signal strength by a factor of $\sqrt{2}$​ because it harnesses the signal components from two perpendicular directions.
- [[Quadrature Detection]]
##### RF Coils as Both
- Some coils are designed to function both as transmitters and receivers. These are known as transmit-receive (TR) coils. In smaller, specialized scanners (like those used for extremities), a single coil can efficiently manage both transmitting and receiving, simplifying the system and potentially increasing the sensitivity.
- **Separation of Coils:** In modern MRI scanners, transmitting and receiving functions are often separated into different coils. This separation optimizes each coil's function—transmit coils are optimized for homogeneous B1B_1B1​ field distribution, while receive coils are optimized for sensitivity and signal-to-noise ratio.
- **Site-Specific Coils:** These are designed not only to capture the MR signal efficiently but also to immobilize the part of the body being scanned, thereby reducing motion artifacts. Examples include head coils, knee coils, and breast coils.
#### Shield:
  - The shield is designed to contain the electromagnetic fields within the scanner and prevent external RF signals from interfering with the MRI signals.
#### Patient Table:
  - This is where the patient lies during the MRI scan. It can be moved into and out of the central hole of the magnet where the scanning takes place.
### System Control and Data Acquisition Components:
#### RF Detector:
  - This component detects the RF signals emitted by the nuclei in the body during the scan. These signals are then sent to a digitizer.
#### Digitizer:
  - Converts the analog RF signals received from the RF detector into digital form so that they can be processed by the computer.
#### RF Amp (RF Amplifier):
  - Amplifies the RF signals to be strong enough to excite the nuclei in the body.
#### RF Source:
  - Generates the RF waves that are necessary to excite the magnetic moments of the nuclei.
#### Gradient Amp:
  - Provides the necessary power to the gradient coils, allowing them to generate the gradient magnetic fields required for spatial encoding.
#### Gradient Pulse Programmer:
  - Controls the timing and shape of the pulses sent to the gradient coils.
#### Pulse Programmer:
  - Manages the timing and sequence of both the RF and gradient pulses according to the specifics of the scan being performed.
#### Computer:
  - The central processing unit that controls all other components, processes the received signals, and reconstructs images based on the MRI data. It runs software that allows technicians to set scanning parameters, view the images, and perform analyses.
### Output:
#### Film or Digital Display:
  - The processed images can be viewed on digital displays or printed on film for diagnostic purposes.

> [!abstract] Definition
>  Radioactive Decay is a spontaneous process in which an unstable atomic nucleus loses energy by emitting radiation. This results in the transformation of the original nucleus (parent) into a new nucleus (daughter), which may be a different element.

- The decay process is largely unaffected by the chemical composition or the physical state (solid, liquid, gas) of the atom. The nucleus decays independently of the electron cloud and the atomic bonds.
## Decay Modes
- [[Alpha Decay]]
- [[Beta Decay]]
- [[Gamma Decay]]
- [[Electron Capture (EC)]]
- [[Spontaneous Fission]]
### Factors Affecting Decay Mode
#### Neutron-to-Proton Ratio
- **Stability**: The stability of a nucleus is largely determined by the ratio of neutrons to protons. Nuclei with too many or too few neutrons compared to protons are unstable and will decay to achieve a more stable configuration.
- **Decay Modes**:
    - **Neutron-rich Nuclei**: Tend to undergo beta decay, where a neutron is converted into a proton, an electron, and an antineutrino.
    - **Proton-rich Nuclei**: Tend to undergo positron emission or electron capture to convert a proton into a neutron.
#### Mass-Energy Relationship
- **Energy Considerations**: The decay process is driven by the difference in mass-energy between the parent nucleus and the products (daughter nucleus and emitted particles). The total energy of the products must be less than the energy of the parent for the decay to be spontaneous.
- **Types of Particles Emitted**:
    - **Alpha Particles** (α\alphaα): Helium nuclei emitted by heavy elements.
    - **Beta Particles** (β\betaβ): Electrons or positrons emitted during beta decay.
    - **Gamma Rays** (γ\gammaγ): High-energy photons emitted during nuclear de-excitation. Mass-Energy Relationship:
## Emission of Photons (Gamma Radiation)
- Gamma rays are often emitted when the nucleus transitions from an excited state to a lower energy state after a decay process.
- Gamma radiation carries away excess energy and does not change the number of protons or neutrons in the nucleus.
## Decay Chains
- If the daughter nucleus produced in the decay process is also unstable, it will decay further, leading to a series of decays called a decay chain.
- **Example**: Uranium-238 decays to thorium-234, which decays to protactinium-234, and so on until a stable isotope is formed.
$$P→D+d1​+d2​+⋯$$
- $P$: Parent nucleus (original unstable nucleus).
- $D$: Daughter nucleus (resulting nucleus after decay).
- $d_{1},d_{2}$: Light particles emitted during the decay process, such as:
    - **Alpha particles** ($\alpha$)
    - **Beta particles** ($\beta$)
    - **Gamma rays** ($\gamma$)

## Formula
- Radioactive decay is an exponential decay
- The rate of change of the number of atoms $N$ depends on the number of atoms and a material dependent factor $-\alpha$:$$\frac{dN(t)}{dt}=-\alpha N(t)$$
- Separate $t$ and $N(t)$:$$dN(t)=-\alpha N(t)dt$$
- Integrate both sides: $$\int \frac{1}{N(t)}dN(t)=\int -\alpha dt$$
- $\int \frac{1}{x}dx=log(x)+C$:  $$log(N(t))=-\alpha t+C$$
- $exp(a+b)=exp(a)*exp(b)$:$$N(t)=exp(-\alpha t+C)$$
- $D=exp(C)$:$$N(t)=D*exp(-\alpha t)$$
- Initial condition: $N(0)=N_{0}$: $$N(0)=N_{0}=D*exp(-\alpha *0)$$
- $exp(0)=1$:$$\textcolor{orange}{N(t)=N_{0}*exp(-\alpha t)}$$
## Radioactive Decay in Medical Imaging
![[Pasted image 20240506133927.png#invert|600]]

## Kinetics
- **Statistical Process:** Radioactive decay is inherently random at the atomic level. Although we can predict the behavior of a large number of atoms, we cannot precisely predict when a single atom will decay.
- **Independent Nuclei:** The decay of one nucleus does not affect another. Each nucleus decays independently.
- **Property of Nuclide:** The probability of decay, characterized by the decay constant (\(\lambda\)), is a specific property of each type of nuclide and does not depend on external conditions like temperature or pressure.
- **Unpredictable Event:** The exact time when a single atom will decay is unpredictable.
### Exponential Decay Law
- **Formulas:**
  - $N = N_0 \cdot e^{-\lambda t}$: This formula represents the number of undecayed atoms at any time $t$. $N_0$ is the initial number of atoms, "$\lambda$"is the decay constant, and $e$ is the base of natural logarithms.
  - $A = A_0 \cdot e^{-\lambda t}$: This formula represents the activity of the nuclide, where $A_0$ is the initial activity. Activity is defined as the rate of decay or the number of disintegrations per second, measured in Becquerels ($Bq$).
### Key Concepts
- **Decay Constant ($\lambda$)**: This constant represents the probability per unit time that an atom will decay. It is specific to each nuclide.
- **Activity ($A$)**: It is the measure of how many atoms in a sample decay per second. Higher activity means more atoms are decaying in each second.
### Half-Life and Mean Lifetime
- **Half-Life ($T$)**: The time required for half of the radioactive nuclides in a sample to decay. It is calculated using the formula $T = \frac{\ln(2)}{\lambda}$, where $\ln(2)$ is approximately 0.693.
- **Mean Lifetime $\tau$**: The average lifespan of a radioactive nuclide before it decays. It's calculated as $\tau = \frac{1}{\lambda}$.
### Beer Forth Decay
- **Exponential Decay Law for Beer Froth:** The formula $h = h_0 \cdot e^{-\lambda t}$ represents the height of beer froth over time. $h_0$ is the initial height of the froth, $\lambda$ is the decay constant (specific to the froth’s rate of dissipation), and $t$ is the time elapsed. This formula illustrates that the froth's height decreases exponentially, similar to the decay of radioactive substances.
### Radioactive Decay – Kinetics
- **Activity:** This is the term used to describe the rate at which radioactive decay events occur, specifically the average number of decays per second. It is measured in Becquerels (Bq), where 1 Bq equals 1 decay per second.
- **Historical Measurement (Curie):** The Curie (Ci) is a historical unit of radioactivity defined as the activity of one gram of Radium-226, which equals approximately $3.7 \times 10^{10}$ Bq. This unit gives a sense of how activity was measured based on a specific and well-known radioactive substance.
### Further Radioactivity Measurements
- **Activity Concentration:** This measures the activity per unit volume, useful in environmental and safety assessments to determine the amount of radioactivity in a given volume of material, air, or water. It's commonly expressed in Bq/m³ or Bq/L.
- **Surface Contamination:** This refers to the amount of radioactive material present on a surface area, measured in Bq/m². It is crucial for safety regulations in environments where radioactive contamination can occur.
- **Specific Activity:** This measures the radioactivity per unit mass of a substance, expressed in Bq/g. It indicates how much radioactivity is concentrated in a material, important for understanding the potential hazard of the material and for its use in medical and industrial applications.



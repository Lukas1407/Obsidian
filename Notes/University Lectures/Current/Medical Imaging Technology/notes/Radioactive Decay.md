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

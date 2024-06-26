## Size
- The size of the nucleus can be estimated using empirical formulas derived from scattering experiments, where particles are scattered off nuclei to probe their structure.
- $$R\approx R_{0} \cdot A^{1 / 3}$$ with: 
	- $R$: Radius of the nucleus.
	- $R_0$​: Constant (approximately $1.2 \times 10^{-15}$ meters or 1.2 femtometers).
	- $A$: Mass number (total number of nucleons).
## Mass Density
- Mass density refers to the mass per unit volume.
- The density of atomic nuclei is remarkably consistent across different elements.
- **Value**: Approximately $2.3 \times 10^{17} \, \text{kg/m}^3$
    - This is extremely dense, indicating that nuclear matter is tightly packed.
### Why Constant?
- **Uniform Composition**: Nuclei are composed of nucleons (protons and neutrons) that have similar mass and are tightly bound.
- **Size and Mass Relationship**: The size of a nucleus is roughly proportional to the cube root of the number of nucleons, leading to a relatively constant density across different nuclei.
## Liquid Drop Model
- The nucleus is modeled as a drop of an incompressible liquid, where nucleons are analogous to molecules in a liquid drop.
- It helps in understanding the binding energy of nucleons within the nucleus and explains why large nuclei can become unstable and undergo fission.
- $$E_{B}\approx a_{v}A+a_{s}A^{\frac{2}{3}}-a_{c} \frac{Z^{2}}{A^{\frac{1}{3}}}-a_{a} \frac{(A-2Z)^{2}}{A}+\delta(A,Z)$$

- **Binding Energy ($E_B$​)**: The energy required to disassemble the nucleus into its individual protons and neutrons.
- **Terms in the Formula**:
    - **Volume Term ($a_{v}A$)**: Proportional to the number of nucleons $A$. It represents the bulk binding energy contributed by all nucleons, assuming they are uniformly packed.
    - **Surface Term ($a_{s}A^{\frac{2}{3}}$)**: Accounts for the lower binding energy of nucleons at the surface of the nucleus. It scales with the surface area of the nucleus.
    - **Coulomb Term ($a_{c} \frac{Z^{2}}{A^{\frac{1}{3}}}$​)**: Represents the electrostatic repulsion between protons. The more protons present, the higher the repulsion, reducing the overall binding energy.
    - **Asymmetry Term ($a_{a} \frac{(A-2Z)^{2}}{A}$​)**: Accounts for the preference of nuclei to have nearly equal numbers of protons and neutrons. Large deviations increase the energy.
    - **Pairing Term ($\delta(A,Z$)**: Corrects for the binding energy based on whether the nucleus has an even or odd number of protons and neutrons.
        - **Pairing Energy**: Typically positive for nuclei with even numbers of both protons and neutrons, negative for odd numbers, and zero for odd $A$.
- The constants $a_v$​, $a_s$​, $a_c$​, $a_a$​, and $\delta$ are empirically determined to best fit the known data for nuclear masses and binding energies.
### Relation to Mass Defect
- **Mass Defect**: The difference in mass between the nucleus and the total mass of its constituent protons and neutrons when they are free (not bound in the nucleus).
- When nucleons (protons and neutrons) come together to form a nucleus, the total mass of the nucleus is less than the sum of the individual masses of the nucleons. This difference is known as the mass defect ($\Delta m$).
- The binding energy ($E_B$​) is directly related to the mass defect and can be calculated using Einstein’s mass-energy equivalence principle:$$E_{B}=\Delta m \cdot c^{2}$$
- The binding energy is the energy released when nucleons bind together to form a nucleus.
- It is also the energy needed to break the nucleus apart into individual protons and neutrons.
- For most nuclei, the average binding energy per nucleon is around 6 to 8 MeV.
### Example Calculation: $\alpha$-Particle
- An $\alpha$-particle is a helium-4 nucleus ($\text{He}^{4}_{2}$), consisting of 2 protons and 2 neutrons.
1. **Determine the Masses:**
	- Mass of proton ($m_p$​):$1.0073u$
	- Mass of neutron ($m_n$​): $1.0087u$
	- Mass of helium nucleus ($m_{He}$​): $4.0015u$
2. **Calculate Total Mass of Free Nucleons:**
	- $$\begin{align} \text{Total mass of free nucleons}=2\cdot m_{p}+2\cdot m_{n} \\ =2\cdot 1.0073u+2\cdot 1.0087u \\ =4.0320u\end{align}$$
3. **Find Mass Defect ($\Delta m$):**
	- $$\begin{align} \Delta m = \text{Total mass of free nucleons - Mass of the nucleus} \\ =4.0320u-4.0014u \\ = 0.0305u\end{align}$$
4. **Convert Mass Defect to Energy:**
	- Use Einstein’s mass-energy equivalence principle $E=\Delta m \cdot c^{2}$
	- Convert atomic mass units (u) to energy in MeV $1u = 931.5MeV/u$
	- $$\begin{align} E_B=0.0305u\cdot 931.5MeV/u \\= 28.4MeV \end{align}$$
### Fusion and Fission
![[Pasted image 20240611093842.png#invert|400]]
#### Fusion
- **Occurs in Light Nuclei**: Elements on the left side of the graph (e.g., hydrogen, helium) have relatively low binding energy per nucleon.
- **Fusion Process**: Light nuclei combine to form heavier nuclei.
    - Example: Hydrogen nuclei fuse to form helium in stars.
- **Energy Release**: Fusion increases the binding energy per nucleon, releasing energy as the new nucleus is more stable.
- **Graph Explanation**: As light nuclei fuse and move toward higher binding energies per nucleon, the process is exothermic (releases energy).
#### Fission
- **Occurs in Heavy Nuclei**: Elements on the right side of the graph (e.g., uranium).
- **Fission Process**: Heavy nuclei split into lighter nuclei.
    - Example: Uranium-235 fission in nuclear reactors.
- **Energy Release**: Fission fragments typically have higher binding energy per nucleon than the original heavy nucleus, releasing energy.
- **Graph Explanation**: As heavy nuclei split and the resulting fragments move toward higher binding energies per nucleon, the process is exothermic.
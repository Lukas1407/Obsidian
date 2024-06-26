> [!abstract] Definition
>  **Electron binding energy**, also known as **ionization energy**, refers to the **minimum energy required to remove an electron** from an atom.
>  Negatively charged electrons are held in place around the positively charged nucleus due to the electrostatic attraction between them.
This energy is measured in electronvolts (eV), where 1 eV = 1.6 x 10^-19 J.

- The magnitude of the electron binding energy depends on the [[Atom Energy Levels|shell and sub-shell]]:
	- <mark style="background: #FFB86CA6;">Inner shell electrons have greater binding energy</mark> because they experience a stronger electrostatic pull from the nucleus.
	- <mark style="background: #FFB86CA6;">Outer shell electrons have lower binding energy</mark> due to weaker attraction.
- The electron binding energy is <mark style="background: #FFB86CA6;">directly proportional to the atomic number</mark> $Z$:
    - As Z increases (more protons in the nucleus), the electrostatic attraction between the nucleus and electrons strengthens, requiring more energy to remove an electron.
    - Conversely, as Z decreases, the binding energy decreases.
- The binding energy is <mark style="background: #FFB86CA6;">inversely proportional to the distance from the nucleus</mark>:
    - Inner shell electrons experience a stronger pull because they are closer to the nucleus.
    - Outer shell electrons are farther away and experience weaker attraction.

## Electron Removal
-> An electron can only be removed from an atom if the applied energy is greater than its electron binding energy
- <mark style="background: #FFB86CA6;">When an inner shell electron is ejected, it's place will be filled by an electron from an outer shell</mark>
	- Doesn't have to be the next outer shell, can be any outer shell
- When an electron moves from an outer shell to an inner shell, it loses energy because it moves to a lower energy state.
- The energy lost must be released by the atom in some form, either by [[Electron Binding Energy#Electron Removal#Emission of Characteristic X-Ray|Emission of Characteristic X-Ray]] or [[Electron Binding Energy#Emission of Auger Electron|Emission of Auger Electron]]
### Emission of Characteristic X-Ray
- When an electron in an atom transitions from a higher energy level (E2) to a lower energy level (E1), <mark style="background: #FFB86CA6;">the energy difference</mark> between these two levels <mark style="background: #FFB86CA6;">is released in the form of electromagnetic radiation</mark>. 
	1. **Quantized Energy Levels**:
	    - Atoms have discrete, or quantized, energy levels. Electrons can occupy these levels, but not the spaces in between.
	    - <mark style="background: #FFB86CA6;">When an electron moves from one level to another, it must absorb or emit a precise amount of energy equal to the difference between these levels</mark>.
	2. **Energy of the [[Photon]]**:
	    - The energy of the emitted electromagnetic radiation <mark style="background: #FFB86CA6;">is exactly equal to the energy difference</mark> between the two levels: $E2 - E1$ 
	    - This energy is carried away by a photon, which is a particle representing a quantum of light.
	3. **Planck’s Equation**:
	    - The relationship between the energy of the photon and its frequency $v$ is given by Planck’s equation: $E = h \cdot v$, where $h$is Planck’s constant.
	    - Therefore, the energy of the photon emitted when an electron transitions between two energy levels is given by: $$E_{photon} = h \cdot v = E2 - E1$$
#### Parent and Daughter Nuclei
![[Pasted image 20240611094357.png#invert|200]]
- **∗**: Indicates that the nucleus is in an excited state, meaning it has more energy than its ground state.
- **No ∗**: Indicates the nucleus is in its lowest energy state.
- **$\gamma$**: Represents the emission of a gamma photon.
- **Transition**: The nucleus releases excess energy by emitting a gamma photon, transitioning from the excited state to the ground state.
- The excited state is often called the "parent," and the ground state is called the "daughter," but the photon is emitted by the same nucleus in different energy states.
	- The term “parent” can be misleading since the nucleus itself is not changing identity, only its energy state is changing.
- The time it takes for a nucleus to emit a gamma photon and transition to the ground state is usually very short, around $10^{-10}$ seconds.
### Emission of Auger Electron
- Instead of emitting a photon, the energy can be transferred to another electron in the atom, usually one in an outer shell.
- The outer electron absorbs this energy and is ejected from the atom.
	- **Auger Electron**: This ejected electron is called an Auger electron.
	- **Non-Radiative Transition**: This process does not involve the emission of a photon but rather an electron.
- The probability of Auger electron emission is higher in materials with a lower atomic number ($Z$).
	- In lighter elements (low-$Z$), the energy difference between shells is smaller, making it more likely for the energy to be transferred to another electron (Auger process) rather than emitting an X-ray photon.
### Fluorescence Yield
- The fluorescence yield is a measure of the number of photons emitted per vacancy in a shell.
![[Pasted image 20240611084140.png#invert|200]]
- **High Fluorescence Yield**: Indicates that a large proportion of vacancies result in [[Electron Binding Energy#Electron Removal#Emission of Characteristic X-Ray|photon emission]] (common in high-Z materials).
- **Low Fluorescence Yield**: Indicates that many vacancies result in the emission of [[Electron Binding Energy#Emission of Auger Electron|Auger electrons]] rather than photons (common in low-Z materials).
> [!tldr] Definition
> Fourier transform is a mathematical operation which maps any function (also aperiodic) in the time domain to the frequency domain.
> It decomposes a signal (or function) into oscillatory functions with a certain frequency.

![[Pasted image 20240508104740.png#invert|500]]
- The <mark style="background: #FFB86CA6;">high frequencies</mark> contribute to the fast varying parts of the signal (sharp transitions)
	- They carry the information
- The <mark style="background: #FFB86CA6;">low frequencies</mark> contribute to the slow variations of the signal in the time domain
	- They carry the base line signal

## 2D Fourier Transform
- A 2D signal can be also broken up into 2D-sinusoidal
## Discrete Fourier Transform
- The DFT is used to analyze the **frequency content** of discrete signals.
- It transforms a sequence of complex numbers from the **time domain** into the **frequency domain**.
- The DFT provides a **frequency domain representation** of the original input sequence.
- It’s particularly useful because many signals can be more easily analyzed and manipulated in the frequency domain than in the time domain.

### Fourier Transform and Function Types
1. **Fourier Transform Basics**:
   - The Fourier Transform $F(x(t)) = X(\omega)$ of a function $x(t)$ is given by:
     $$
     F(x(t)) = \int_{-\infty}^{\infty} x(t) e^{-j\omega t} dt
     $$
   - This transform can be broken down into its components involving cosine and sine functions, representing the even and odd parts of $e^{-j\omega t}$ respectively:
     $$
     F(x(t)) = \int_{-\infty}^{\infty} x(t) (\cos(\omega t) - j\sin(\omega t)) dt
     $$

2. **Properties of Even and Odd Functions**:
   - **Integral of Product of Odd and Even Functions**: When integrating the product of an odd function and an even function over symmetric limits (like $[-a, a]$ or $[-∞, ∞]$), the result is zero. This is due to the symmetry properties where the contributions from the negative side cancel out those from the positive side.
   - **Fourier Transform of Even and Odd Functions**:
     - An **even function** $f(x) = f(-x)$ has a Fourier transform that is also even.
     - An **odd function** $f(x) = -f(-x)$ has a Fourier transform that is odd.
   - **Fourier Transform of Real Functions**:
     - A real **even function** has a Fourier transform that is **real and even**.
     - A real **odd function** has a Fourier transform that is **purely imaginary and odd**.

3. **Decomposition into Even and Odd Functions**:
   - Any function $x(t)$ can be expressed as a sum of an even function $x_e(t)$ and an odd function $x_o(t)$:
     $$
     x(t) = x_e(t) + x_o(t)
     $$
   - Where:
     $$
     x_e(t) = \frac{x(t) + x(-t)}{2} \quad \text{(even component)}
     $$
     $$
     x_o(t) = \frac{x(t) - x(-t)}{2} \quad \text{(odd component)}
     $$

### Application to MRI and RF Coils
In the context of MRI, the Fourier Transform is crucial for transforming the time-domain MR signals (which are typically complex-valued, reflecting both even and odd symmetries) into frequency-domain images. The RF transmit coils in MRI generate a $B_1$ field that can be modeled by sinusoidal functions, contributing both even (cosine) and odd (sine) components to the signal. Understanding these properties allows for:
- **Optimal Coil Design**: Designing coils that effectively create fields with the desired symmetry properties to maximize signal reception and minimize losses.
- **Signal Processing**: Efficiently processing MRI data by leveraging the properties of even and odd functions to filter or enhance specific signal components.
This fundamental understanding of the Fourier Transform and its properties is essential for the effective manipulation and analysis of signals in many scientific and engineering fields, including medical imaging.


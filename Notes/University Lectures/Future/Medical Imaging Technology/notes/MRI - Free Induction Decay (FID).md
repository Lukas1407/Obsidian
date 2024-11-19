> [!abstract] Definition
> FID is the decaying signal observed after the spins have been disturbed by an RF pulse and are returning to their equilibrium state. 

- **Free**: After the $B1$ field (oscillating magnetic field used to generate the RF pulse) is turned off, the magnetization vector is no longer influenced by it and begins to freely precess around the static magnetic field $B_0$​. This is termed "free precession."
- **Inducing**: The precessing magnetization vector in the transverse plane induces a current in the receiver coil (also oriented in the xy plane). This happens because the changing magnetic field associated with the moving magnetization cuts through the coil, generating an electromotive force (EMF) by electromagnetic induction.
- **Decay**: The signal detected in the coil decays over time due to the combined effects of intrinsic and extrinsic dephasing of the spins, represented by $T2^{*}$. This loss of signal coherence is what is observed as the FID.

## Mathematical Expression
$$ M_{xy}(t) |_{\alpha=90^\circ} = M_0 \cos(\omega t) e^{-\frac{t}{T2^*}} $$
1. **$M_0$:** This represents the initial magnitude of the net magnetization vector in the transverse plane immediately after the application of the RF pulse. $M_0$ is typically the result of a 90-degree pulse flipping the longitudinal magnetization ($M_z$) into the transverse plane.
2. **$\cos(\omega t)$:** This term represents the precession of the magnetization vector around the main magnetic field $B_0$ at the Larmor frequency $\omega$. The cosine function indicates that the magnetization vector oscillates back and forth in the xy-plane, which is perpendicular to $B_0$.
3. **$e^{-\frac{t}{T2^*}}$:** This exponential decay factor accounts for the loss of signal due to dephasing of the spins. The decay constant $T2^*$ includes both the intrinsic T2 relaxation (spin-spin interactions) and additional dephasing due to inhomogeneities in the magnetic field $B_0$.
### Graphical Representation
![[Pasted image 20240625081917.png#invert|400]]

The graph illustrates how the FID signal behaves over time:
- The **cosine wave** (solid line) depicts the oscillating component of the signal, representing the coherent precession of the magnetization.
- The **exponential envelope** (dashed line) shows the decay of the signal amplitude due to dephasing. The envelope follows the $e^{-\frac{t}{T2^*}}$ decay, reducing the amplitude of the cosine oscillations over time.

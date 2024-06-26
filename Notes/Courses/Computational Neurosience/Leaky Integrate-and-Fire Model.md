The LIF model simulates [[Neuron|neurons]] by modeling the [[Membrane potential]] using differential equations.
- it emits [[Action Potential|spikes]] at a rate that increases with injected current
When the [[Membrane potential]] reaches a threshold $V_{thr}$, a spike is emitted and the membrane potential is to the resting value.
![[Pasted image 20240206133124.png]]
## Equation
$$
C_{m} = \frac{dV_{m}}{dt} = I(t) - \frac{V_{m}(t)}{R_{m}} = G_{L}(E_{L}- V_{M}) + I(t)
$$
With $C_{m}$ being the [[Capacitance]] of the [[Zellmembran]], $V_{m}$ the [[Voltage]], $I_{t}$ the input [[Current]], $R_{m}$ the resistance.
- -> The conductance term causes charge to leak out and the voltage to decay with a time constant of $C_{m}/G_{l}$. 

For constant input, the minimum input to reach the threshold is _$I_{th} = V_{thr} / R_{m}$. Assuming a reset to zero, the firing frequency thus looks like:
$$
f(I) = 
\begin{cases}
    0\\
    [t_{ref} - R_{m}C_{m}log(1-\frac{V_{thr}}{IR_{m}})]^{-1}
  \end{cases}
$$


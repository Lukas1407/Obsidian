
## Characteristics
### Linearity
- The [[System|system]] obeys the principle of superposition. If $x_1(t)$ and $x_2(t)$ are input signals, then the output corresponding to $$x_1(t) + x_2(t)$$
- Characterizing the complete input-output properties of a system by exhaustive measurement is usually impossible. 
- When a system qualifies as a linear system, it is possible to use the responses to a small set of inputs to predict the response to any possible input.
### Time Invariante
- The system’s behavior does not change over time. 
- If an input signal is delayed or advanced, the output is similarly delayed or advanced.
## Impulse Response
- <mark style="background: #FFB86CA6;">Any LTI system can be characterized entirely by a single function called the system's impulse response</mark>
- It characterizes <mark style="background: #FFB86CA6;">how the system responds when it encounters an instantaneous impulse input</mark> (such as a brief spike in energy).
- Mathematically, the impulse response is denoted as $$h=H(\delta(t))$$
- The output of the LTI system $y(t)$ is obtained by convolving the input signal $x(t)$ with the impulse response: $$y(t) = x(t) * h(t)$$
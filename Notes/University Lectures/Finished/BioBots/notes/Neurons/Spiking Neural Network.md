Spiking Neural Networks (SNNs) aim to model a biological neural network more closely by modeling the [[Membrane potential|membrane potential]] and incorporating the concept of time into the model.
The idea is that neurons in the SNN do not transmit information at each time step (as it happens with typical multi-layer perceptron networks), but rather transmit information only when a [[Membrane potential|membrane potential]] reaches a specific value, called the threshold.
When the [[Membrane potential|membrane potential]] reaches the threshold, the neuron fires, and generates a signal that travels to other neurons which, in turn, increase or decrease their potentials in response to this signal.

## Different Models 
1. [[Hodgkin-Huxley Model]]
2. [[Leaky Integrate-and-Fire Model]]

## Comparison to ANN

|          ANN          |             SNN             |
|:---------------------:|:---------------------------:|
|        static         |           dynamic           |
|       position        |        time-position        |
| no knowledge of time  |       time dependent        |
| direct weight updates | difficult to update weights |

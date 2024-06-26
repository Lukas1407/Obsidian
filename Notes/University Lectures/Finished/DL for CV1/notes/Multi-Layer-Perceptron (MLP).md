> [!summary] Definition
>  A multi-layer perceptron (MLP) is a name for a modern feed-forward [[Artificial Neuronal Network (ANN)]] , consisting of fully connected [[Artificial Neuron|neurons]] with a nonlinear [[Activation Function|activation function]], organized in at least three layers.

![[Pasted image 20240303120604.png#invert|]]
- The softmax function leads produces [[Uncertainty#Why not use the softmax probability?|pseudo-probabilities]] that sum up to 1. 
- Learning occurs in the perceptron by changing connection weights after each piece of data is processed, based on the amount of error in the output compared to the expected result. This is an example of supervised learning, and is carried out through [[Backpropagation]]

- The MLP is a static network:![[Artificial Neuronal Network (ANN)#^60d405|static network]]
- Every Neuron has global Effect!
## Sate of  a Neuron
$$o_{i}^{(v)}=w_{i0}^{(v)}+\sum_{j}w_{ij}^{(v)}\cdot o_{j}^{(v-1)}$$, with $o$ being the state of the [[Artificial Neuron]], $v$ the layer, and $w_{i0}^{(v)}$ the bias
## Output of a Neuron
$$z_{i}^{(v)}=f_{i}^{(v)}(o_{i}^{(v)})$$




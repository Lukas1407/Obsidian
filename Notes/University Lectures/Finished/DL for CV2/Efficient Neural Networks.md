There is an increasing discrepancy between the hardware the model is trained on and the hardware we want to run inference on.
Especially for mobile devices there are additional constraints like:
- Size of the model
- Energy consumption
- Heat generation
- Runtime

## Problems of normal convolutions
- Normal ($3\times3$) convolutions have high computational complexity of: $$h\times w\times D_{in}\times H_{out}\times W_{out} \times D_{out}$$ and high memory consumption of: $$h\times w\times D_{in}\times D_{out}$$![[Pasted image 20240226095957.png]]
- $1\times 1$ convolutions reduce this complexity, as $h=1, w=1$, but they lack the ability to capture spacial dependencies -> a NN with only $1\times 1$ convolutions would not perform well

## Efficient Models
1. [[SqueezeNet]]
2. [[Grouped Convolutions]]
	1. [[Depthwise Separable Convolution]]
	2. [[ShuffleNet]]
3. Downsample input, usually done by the first 2 layers (stem cells)

## Efficient Training and Inference
1. [[Mixed Precision]]
2. [[Parameter Pruning]]
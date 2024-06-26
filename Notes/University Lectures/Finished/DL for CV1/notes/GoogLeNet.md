- Deep model with 22 layers
- Computational efficient due to the [[GoogLeNet#Inception Module|inception module]]
- No fully-connected layer -> reduces number of parameters greatly
	- A fc layer after a 256x256 feature map has 256 $*$ 256 parameters

## Architecture
### Inception Module
- Small network inside the model
- Extracts information from different spatial sizes by using different size convolutions, which are the concatenated
	- 1x1, 3x3, and 5x5
![[Pasted image 20240306085237.png#invert|]]
- This solves the problem of needing to find the best convolution
- (a) has the problem of increased parameter count, which is solved by the dimension downsampling if (b) before applying the convolutions
### Global Average Pooling
- Used instead of a fully-connected at the end
- Averages the last feature map to a 1x1 value, like the fc layer would do
- Then a linear layer is applied followed by softmax for classification
- Advantages
	- Better performance than fc layer
	- Zero learned parameters
### Auxiliary Loss Layers
- Helps with the problem of [[Vanishing Gradient]] when training such deep networks
- It injects additional loss into lower layers
- Only used during training
Problems with RNNs: Response at the current position is dependent on all previous ones.
Problems with [[Convolutional Neural Network|CNNs]]: It's difficult to capture long-range dependencies with convolutions; they need to be applied multiple times, which is computationally expensive.

Solution: attention mechanism

## Attention
- Is both efficient and can capture long-range dependencies
$$Attention(Q,K,V)=softmax(\frac{QK^T}{\sqrt{d_{k}}})V,$$ where $d_{k}$ is the key dimension.
- For self attention $Q, K$ and $V$ are calculated based on the same input.
-  $Q, K$ and $V$ are learnable linear projection matrices of dimension $B\times seq \times dim$
-  **Query**: A vector that represents the input token that is seeking to attend to other tokens. It is computed by multiplying the input embedding by a weight matrix 𝑾𝑸.
-  **Key**: A vector that represents the input token that is being attended to by other tokens. It is computed by multiplying the input embedding by a weight matrix 𝑾𝑲.
-  **Value**: A vector that contains the information of the input token that is being attended to. It is computed by multiplying the input embedding by a weight matrix 𝑾𝑽.
The attention calculation is then performed by taking the dot product of the query and the key, scaling it by the square root of the key dimension, applying a softmax function, and multiplying it by the value. This produces a weighted sum of the values, which is the output of the attention layer.

## Multi-Head Attention
- Uses multiple heads that allow to focus on different parts of the sequence $$MHA(Q,K,V)=concat(head_{1}, ...,head_{h})W^O,$$with $$head_{i}=Attention(QW_i^Q,KW_{i}^K,VW_{i}^V)$$

## Architecture
- Encoder-Decoder architecture
- ![[Pasted image 20240224150904.png|400]]
### Encoder
- Input is embedded either with a learnable embedding or a fixed sinusoidal embedding
- Then position encoding is added, because the transformer is position equivariant
### Decoder
- Input is shifted to the right such that the token we want to predict is not given as the input
- Masked MHA is used, which masks out the future tokens of the input sequence
	- This is done by setting $Q,K$ to $- \infty,$ this way when calculating the softmax, those tokens get 0 weight  

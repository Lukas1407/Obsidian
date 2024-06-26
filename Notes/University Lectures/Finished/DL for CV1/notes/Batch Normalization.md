> [!summary] Definition
> Batch normalization is a technique that standardizes the inputs to a layer for each batch, which makes the training of deep neural networks faster and more stable. 

## Idea
- The main idea of batch normalization is to normalize the inputs to a layer by subtracting the batch mean and dividing by the batch standard deviation. 
- This ensures that the inputs have zero mean and unit variance, which reduces the problem of <mark style="background: #FF5582A6;">internal covariate shift</mark>
- <mark style="background: #FF5582A6;">Internal covariate shift</mark> refers to the change in the distribution of inputs to a layer due to the updates of the preceding layers. This can slow down the learning process and make the network sensitive to the initial weights and the learning rate.
- Batch normalization can be applied to either the inputs of a layer or the outputs of the activation function. It is usually placed before the activation function, such as ReLU, to avoid the saturation of the non-linearity.

## Formula
$$\hat x=\frac{x-\mu_x}{\sqrt{\sigma^2_x+\epsilon}},$$
where $x$ is the input vector, $\hat x$ is the normalized output vector, $\mu_x$ is the batch mean, $\sigma^2_x$ is the batch std, and $ϵ$ is a small constant to avoid division by zero.

- However, simply normalizing the inputs may change the representation of the layer and reduce its expressive power
- Therefore, batch normalization also introduces two learnable parameters, $γ$ and $β$, which scale and shift the normalized inputs. This allows the network to recover the original representation if needed. 
- The formula for batch normalization with scaling and shifting is:
$$y=γ\hat x+β,$$
where $y$ is the final output vector, $γ$ is the scaling parameter, and $β$ is the shifting parameter.

## Advantages
- It speeds up the training by allowing higher learning rates and reducing the dependence on the initialization.
- It provides some regularization effect by adding noise to the inputs of each layer, which reduces overfitting.
- It smoothens the loss function and avoids the vanishing or exploding gradient problem.

## Disadvantages
- It adds computational complexity and memory overhead to the network, which may slow down the inference time.
- It depends on the batch size, which may affect the performance if the mini-batch size is too small or varies across batches.
- It introduces hyperparameters, such as $ϵ$, $γ$, and $β$, which need to be tuned for optimal results

## Batch Norm at Test Time
- At test time $\mu_x$ and $\sigma_x$ are not calculated based on the batch
- A fixed mean and std is used which was estimated during training with running averages
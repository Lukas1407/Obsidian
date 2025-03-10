## Motivation
**Key Concepts:**
1. **Principled Idea of Linear & Logistic Regression:**
   - These models are computationally efficient and have solid analytical properties.
   - They rely on pre-defined features or "basis functions" to make predictions.
2. **Challenge with Large-Scale Problems:**
   - Manually defining good feature functions becomes infeasible as the problem's dimensionality increases (known as the "curse of dimensionality").
3. **Solution - Learnable Feature Functions:**
   - Instead of manually choosing feature functions, we parameterize them and learn these parameters alongside the model weights.
   - The mathematical shift:
     $$
     y(\mathbf{x}, \mathbf{w}) = f\left(\sum_{j=1}^M w_j \phi_j(\mathbf{x})\right) \rightarrow y(\mathbf{x}, \mathbf{w}_1, \mathbf{w}_2) = f\left(\sum_{j=1}^M w_j^1 \phi_j(\mathbf{x}, \mathbf{w}_2)\right)
     $$
   - This change introduces an additional set of parameters, enabling the model to learn the feature representations.
### Logistic Regression Recap
**Logistic Regression Overview:**
1. **Likelihood Model:**
   - The probability of the target $t_i$ given input $x_i$ and weights $w$ is modeled using a Bernoulli distribution for binary classification ($t_i \in \{0,1\}$).

2. **Bernoulli Distribution:**
   - The probability expression:
     $$
     p(t_i \mid x_i, \mathbf{w}) = y_i^{t_i}(1-y_i)^{1-t_i}
     $$
   - $y_i = \sigma(\mathbf{w}^T \phi_i)$ where $\sigma$ is the sigmoid function, squashing outputs to the range $[0, 1]$.

3. **Error Function - Cross Entropy:**
   - The negative log-likelihood is:
     $$
     E(\mathbf{w}) = -\sum_{i=1}^N \left[t_i \log y_i + (1-t_i) \log (1-y_i)\right]
     $$
   - This is also known as cross-entropy loss, which is standard for classification tasks.

4. **Gradient of the Error:**
   - The gradient with respect to the weights is needed for optimization:
     $$
     \nabla E(\mathbf{w}) = \sum_{i=1}^N (\sigma(\mathbf{w}^T \phi_i) - t_i)\phi_i
     $$
### Visualization of Multi-Output Computation
![[Pasted image 20250310093246.png#invert|400]]
**Neural Network Interpretation:**
1. **Feature Function $\phi_i = \phi(\mathbf{x_i})$:**
   - A feature representation of the input $x_i$.

2. **Neural Network Structure:**
   - The diagram illustrates how different weights lead to different outputs using the same input features.
   - Pre-activations ($a$) and activations ($h$) are introduced:
     $$
     h = \sigma(W \phi_i)
     $$
   - This is a precursor to understanding hidden layers in neural networks.
Slide 4: Stacking Layers

**Deep Learning Concept:**
1. **Layer Concatenation:**
   - The output of one layer can serve as the input for another layer, allowing for deeper architectures.
   - The feature functions themselves can be learned:
     $$
     \phi_j(\mathbf{x}, \mathbf{w}_2) = \sigma(\mathbf{w}_2^T \mathbf{x})
     $$
   - This results in a multi-layer perceptron (MLP), where each layer extracts increasingly abstract features.

2. **Visualizing the Network:**
   - The neural network now includes multiple layers of transformation, with each layer applying weights and non-linear activation functions.
### Neural Networks as Nested Logistic Regression
![[Pasted image 20250310093304.png#invert|400]]
**Nested Logistic Regression:**
1. **Interpreting Neural Networks:**
   - A neural network with sigmoid activation can be seen as a "nested" logistic regression.
   - For a three-layer network:
     $$
     y_i = \sigma(W_2 \sigma(W_1 \sigma(W_0 \phi_i)))
     $$
   - The last layer essentially performs logistic regression on transformed inputs from previous layers.

2. **Key Takeaway:**
   - The network learns hierarchical representations, which are particularly powerful in tasks requiring complex feature extraction.
### Weight Updates
**Optimization Techniques:**
1. **Cross-Entropy Loss:**
   - The same cross-entropy function is used to calculate the loss for neural networks.

2. **Gradient Descent vs. Newton-Raphson:**
   - While Newton-Raphson could theoretically be applied, it is only feasible for small networks due to the computational cost of calculating the Hessian (second-order derivatives).
   - Instead, gradient descent is typically used:
     $$
     \mathbf{w}_{i+1} = \mathbf{w}_i - \eta \nabla E(\mathbf{w}_i)
     $$
   - Here, $\eta$ is the learning rate, and $\nabla E(\mathbf{w}_i)$ is the gradient of the error function.
### Computing the Gradient

**Numerical vs. Analytical Differentiation:**
1. **Numerical Differentiation:**
   - Uses finite differences to approximate the gradient.
   - Advantages: Simple and useful for verification.
   - Disadvantages: Requires many forward passes (one per weight).

2. **Analytical Differentiation:**
   - Computes gradients directly from a closed-form expression of the error function.
   - More precise and efficient, especially for large networks.
   - Requires differentiation rules and is the foundation for backpropagation in neural networks.
### Numerical Differentiation (Finite Differences)
**Gradient Computation:**
1. **Finite Difference Formula:**
   $$
   \frac{\partial E(\mathbf{W}, \phi_i)}{\partial \mathbf{W}} = \lim_{h \to 0} \frac{E(\mathbf{W}+h, \phi_i) - E(\mathbf{W}, \phi_i)}{h}
   $$
2. **Advantages:**
   - Linear complexity concerning the number of weights.
   - Intuitive and easy to implement.

3. **Disadvantages:**
   - Inefficient for large networks as it requires a separate forward pass for each weight.
   - This makes it impractical for deep learning, where the number of weights is large.

Sure! Here is a detailed explanation of each of the new slides:
### Analytical Differentiation (Backpropagation)
#### **Key Concepts:**
1. **Error Function (Cross-Entropy):**
   - The error function $E(\mathbf{w})$ is defined using cross-entropy for a batch of inputs:
   $$
   E(\mathbf{w}) = -\sum_{i=1}^N \left[t_i \log y_i + (1-t_i) \log (1-y_i)\right]
   $$
   - This measures how well the predicted probabilities $y_i$ match the actual targets $t_i$.

2. **Backpropagation Method:**
   - Backpropagation is a technique to compute gradients of the error function with respect to the weights efficiently.
   - It uses the chain rule of differentiation to propagate the error from the output layer back to the input layer.

3. **Steps in Backpropagation:**
   1. **Save Activations:** During the forward pass, save all activations $h$ for use in gradient computation.
   2. **Gradient Computation:** Use the chain rule to compute gradients for weights and activations, starting from the output layer and moving backwards.
   3. **Average Gradients:** When using a batch of inputs, compute the average of the gradients.

4. **Advantages:**
   - Only one forward and one backward pass are needed per training step.
   - Memory requirements grow linearly with the network size, which is manageable for large networks.
#### The Backpropagation Algorithm
##### **Detailed Algorithm:**
1. **Initialize:**
   - Set the initial input $h_0 = \mathbf{x}$.

2. **Forward Pass:**
   - For each layer $i$ from 1 to $l$ (where $l$ is the total number of layers):
     - Compute pre-activations: $a_i = W_i h_{i-1}$.
     - Compute activations: $h_i = \sigma(a_i)$ using an activation function like sigmoid or ReLU.

3. **Loss Derivative:**
   - Compute the initial gradient of the loss with respect to the output: $D h_l = \frac{\partial L(y, z)}{\partial z}$ evaluated at $z = h_l$.

4. **Backward Pass:**
   - For each layer $i$ from $l$ back to 1:
     - Compute the gradient of the loss with respect to the pre-activation: $g_i = D h_i \circ \sigma'(a_i)$ (Hadamard product with the derivative of the activation function).
     - Calculate the gradient with respect to the weights: $DW_i = g_i h_{i-1}^T$.
     - Backpropagate the error to the previous layer: $D h_{i-1} = W_i^T g_i$.

5. **Output:**
   - The final output is the concatenated vector of all weight gradients $DW = [\text{vec}(DW_1) \cdots \text{vec}(DW_l)]$.
### Activation Functions
![[Pasted image 20250310094132.png#invert|400]]
#### **Common Activation Functions:**
1. **Sigmoid Function:**
   - Formula: $\sigma(z) = \frac{1}{1+e^{-z}}$.
   - Squashes input to the range (0, 1).
   - **Problem:** The derivative of the sigmoid is small when inputs are far from zero, leading to the **vanishing gradient problem**, which makes training deep networks difficult.

2. **ReLU (Rectified Linear Unit):**
   - Formula: $R(z) = \max(0, z)$.
   - Allows gradients to flow when $z > 0$, preventing the vanishing gradient problem.
   - Simple and computationally efficient.
#### Rectified Linear Unit (ReLU)
##### **Advantages and Disadvantages:**
1. **Advantages:**
   - **Cheap to Compute:** The function is linear for positive inputs and zero otherwise.
   - **No Gradient Saturation:** Unlike sigmoid, the gradient does not diminish as the input grows.
   - **Simplicity as Non-Linearity:** The "kink" at 0 is sufficient to introduce non-linearity into the model.

2. **Disadvantages:**
   - **"Dying ReLU" Problem:** Up to 40% of neurons can "die," meaning they output zero for any input and stop learning because their gradients are zero.
   - **Solution:** Variants like Leaky ReLU introduce a small slope for negative inputs to keep gradients flowing.
### Gradient Descent
#### **Gradient Descent Formula:**
$$
W_j^{t+1} = W_j^t - \eta \nabla E(\mathbf{w})
$$
- $\eta$ is the learning rate, controlling how large each update step is.
- The gradient $\nabla E(\mathbf{w})$ indicates the direction and magnitude of the weight updates.

#### **Variants:**
1. **Gradient Descent (GD):** Averages gradients over the entire dataset.
2. **Stochastic Gradient Descent (SGD):** Averages gradients over a mini-batch, providing more frequent updates and enabling faster convergence.
### Optimizers
#### **Comparison of Optimizers:**
1. **SGD:**
   - Can get stuck in local minima.
   - Less stable with small batch sizes.

2. **SGD + Momentum:**
   - Adds inertia to the gradients, smoothing the updates.
   - Performs well on tasks like image classification when well-tuned.

3. **Adam (Adaptive Moment Estimation):**
   - Combines ideas from Momentum and RMSProp optimizers.
   - Computes individual learning rates for each weight.
   - Uses moving averages of past gradients and their squares to adapt the learning rate.
### Moving Averages

#### **Concept:**
1. **Main Idea:** Smoother estimation of the gradient by incorporating previous gradient estimates.
2. **Averaging Methods:**
   - **Simple Averaging:** Considers only the last $k$ samples.
   - **Cumulative Averaging:** Averages all samples up to the current step.
   - **Weighted Averaging:** Applies decaying weights, akin to convolution.
   - **Exponential Weighted Averaging:** Gives exponentially decreasing importance to older gradients.

3. **Formula:**
$$
\mathbf{m}_t = \begin{cases} 
\mathbf{g}_0 & t = 0 \\
\beta \mathbf{m}_{t-1} + (1-\beta) \mathbf{g}_t & t > 0 
\end{cases}
$$
- $\beta$ is a decay rate, typically close to 1.
### Mean and Variance

#### **Extended Moving Average:**
1. **Variance Calculation:**
$$
v_t = \frac{1}{t} \sum_{i=1}^t (m_t - x_i)^2
$$
- Computes the variance of the gradients, useful for normalizing updates.

1. **Gradient Normalization:**
$$
\hat{g}_t = \frac{g_t - m_t}{\sqrt{v_t}}
$$
- Helps to "correct" the gradient distribution, stabilizing training.
### Optimizer Comparison
#### **Adam Optimizer:**
1. **Strengths:**
   - Good choice for minimizing training loss.
   - Stable performance across a variety of tasks.

2. **Caution:**
   - The goal is not just to minimize training loss but to minimize **validation loss**.
   - Overfitting may occur if training accuracy reaches 100% while validation performance stagnates or worsens.

3. **Regularization Techniques:**
   - **Data Augmentation:** Increases dataset variability, helping generalization.
   - **Dropout:** Randomly "drops" neurons during training, preventing co-adaptation of features.
   - **Weight Decay:** Adds a penalty to large weights, reducing model complexity.
   - **Others:** Methods like batch normalization or early stopping can also help regularize the model.
### Weight Decay vs. L2 Regularization

**Concept:**
- **L2 Regularization** adds a penalty term to the loss function to prevent the model weights from becoming too large, which helps in avoiding overfitting. 
- The modified error function becomes:
  $$
  \bar{E}(\mathbf{w}) = -\log p(\mathbf{t} \mid \mathbf{x}, \mathbf{w}) + \frac{\lambda}{2} \|\mathbf{w}\|_2^2
  $$
  - The first term represents the standard loss (negative log-likelihood).
  - The second term is the L2 penalty, controlled by the hyperparameter $\lambda$.

**Gradient Calculation:**
- The gradient of this error function is:
  $$
  \nabla \bar{E}(\mathbf{w}) = \nabla E(\mathbf{w}) + \lambda \mathbf{w}
  $$
  - The penalty term $\lambda \mathbf{w}$ directly adds to the standard gradient.

**Weight Update Rule:**
- Incorporating the penalty into the gradient descent update:
  $$
  \mathbf{w}_{t+1} = \mathbf{w}_t - \eta (\nabla E(\mathbf{w}) + \lambda \mathbf{w}_t)
  $$
- Rearranging terms gives:
  $$
  \mathbf{w}_{t+1} = (1 - \eta \lambda) \mathbf{w}_t - \eta \nabla E(\mathbf{w})
  $$
  - The term $(1 - \eta \lambda)$ shows that each weight decays slightly in every update step, which is why L2 regularization is often referred to as "weight decay".
### Multi-class Logistic Regression

**Concept:**
- For **K > 2** classes, logistic regression is extended using the **softmax function** to handle multiple classes:
  $$
  p(y = k_i \mid \mathbf{x}) = \frac{\exp(a_i)}{\sum_{j=1}^K \exp(a_j)}
  $$
  - This normalizes outputs into probabilities across $K$ classes.

**Class Scores:**
- Define $a_i = \mathbf{w}_j^T \phi(\mathbf{x_i})$, where $\mathbf{w}_j$ represents the weight vector for the $j$-th class.

**One-Hot Encoding:**
- Labels $t_i$ are represented using **1-of-K (one-hot) encoding**:
  $$
  t_i = (0, \dots, 0, 1, 0, \dots, 0)
  $$
  - A 1 at the $k$-th position indicates the true class.

**Benefits:**
- This approach allows neural networks to handle multi-class classification problems naturally.
### Initialization (Random Normal, Xavier, …)

**Initial Weights:**
- Proper initialization prevents gradients from **exploding** or **vanishing** during training.

**Naive Initialization:**
- Randomly initializes weights using uniform or normal distributions. However, this can cause unstable gradients, especially in deep networks.

**Xavier (Glorot) Initialization:**
- Designed to maintain variance through layers:
  $$
  \text{Var}[h_i] = \text{Var}[h_j], \quad \forall i, j \text{ in layers with input } X \sim \mathcal{N}(0, 1)
  $$
- Ensures that the input and output variances of each layer remain consistent, making training more stable.

**Why Xavier?**
- Prevents the gradients from diminishing or amplifying across layers.
- A default choice in many deep learning frameworks due to its effectiveness.
### Batch Normalization

**Purpose:**
- Normalizes the distribution of activations across mini-batches to stabilize and accelerate training.

**Benefits:**
1. **Consistent Distributions:**
   - Ensures that activations fed into each layer have similar distributions, improving convergence and allowing larger batch sizes.

2. **Deeper Networks:**
   - Helps mitigate the **vanishing/exploding gradient problem**, enabling the training of deeper networks effectively.

3. **Handles Mean-Shift:**
   - Prevents issues caused by asymmetric activation functions (like ReLU).

**Process:**
1. For a mini-batch of inputs $x_1, \dots, x_m$:
   - Compute mean $\mu_B$ and variance $\sigma_B^2$.
   - Normalize each input $x_i$ to $\hat{x}_i$.
   - Scale and shift normalized inputs using learnable parameters $\gamma, \beta$:
     $$
     y_i = \gamma \hat{x}_i + \beta
     $$

**Why Use Batch Norm?**
- Reduces internal covariate shift, stabilizing training.
- Empirical evidence shows faster convergence and improved generalization.

## Motivation

**Why Bayesian Neural Networks?**
- **Predictive Uncertainty:**
  - Traditional neural networks provide a single prediction without any measure of uncertainty, making them less reliable for critical applications.
  - Bayesian reasoning, on the other hand, allows us to quantify uncertainty in predictions, which is crucial in fields like medicine, autonomous driving, and finance.

- **Limitations of Standard Neural Networks:**
  - Deep learning models, while powerful, are deterministic in nature and do not inherently express uncertainty about their predictions.
  - In real-world applications, having a measure of uncertainty can help in decision-making, model interpretability, and reliability.

- **The Idea:**
  - By applying Bayesian principles to neural networks, we can transform them into **Bayesian Neural Networks** that not only predict outcomes but also quantify the uncertainty of these predictions.

- **Other Techniques for Uncertainty Estimation:**
  - Besides Bayesian methods, there are several alternative techniques:
    - **Monte-Carlo Dropout:** Uses dropout at inference time to simulate an ensemble of networks and estimate uncertainty.
    - **Deep Ensembles:** Combines predictions from multiple models to capture both epistemic (model) and aleatoric (data) uncertainty.
    - **Calibration:** Techniques to adjust confidence estimates to reflect true prediction accuracy better.
### Sources of Predictive Uncertainty

**Types of Uncertainty:**
1. **Data Uncertainty (Aleatoric Uncertainty):**
   - Arises due to inherent noise or ambiguity in the data.
   - Example (Regression): Uncertain data points scattered around a trend line.
   - Example (Classification): Overlapping data points between classes.

2. **Model Uncertainty (Epistemic Uncertainty):**
   - Due to limited data or knowledge about the model parameters.
   - Example: Uncertainty in regions far from the training data or when multiple plausible models fit the data.

3. **Out-of-Distribution Uncertainty:**
   - Occurs when the model encounters inputs significantly different from the training data.
   - Detecting such uncertainty is vital for safe deployment in real-world applications.
### Model Uncertainty vs. Data Uncertainty

**Key Distinction:**
- **Epistemic Uncertainty (Model Uncertainty):**
  - Captures uncertainty about the model parameters due to limited or sparse training data.
  - Can be reduced by gathering more training data.

- **Aleatoric Uncertainty (Data Uncertainty):**
  - Captures noise inherent to the data itself, which cannot be reduced even with more data.

**Illustration:**
- Left diagram: A single classifier separates training data without expressing uncertainty.
- Right diagram: A range of classifiers expresses uncertainty, particularly in regions away from the training data.
### Model Uncertainty

**Standard Approaches:**
- Most traditional models (like neural networks and SVMs) seek a single optimal set of parameters using:
  - **Maximum Likelihood Estimation (MLE):** Finds parameters maximizing the likelihood of the observed data.
  - **Maximum A Posteriori (MAP):** Similar to MLE but incorporates a prior distribution over parameters.

**Bayesian Approach:**
- Instead of finding a single optimal set of parameters, Bayesian methods infer a **distribution** over possible parameter values.
- Predictive distribution is then obtained by averaging over all possible parameter configurations:
  $$
  p(y^* \mid X, y, x^*) = \int p(y^* \mid x^*, w) p(w \mid X, y) \, dw
  $$
- This approach explicitly represents model uncertainty, providing more reliable predictions.
### The Problem of Overconfidence

**Issue:**
- Standard neural networks tend to be **overconfident** in their predictions, particularly for out-of-distribution samples.
- Ideally, the predicted confidence should match the true likelihood of being correct.

**Expected Calibration Error (ECE):**
- Measures the difference between predicted confidence and actual accuracy:
  $$
  \text{ECE} = \mathbb{E}\left[\left| p(y^* = y \mid c^* = c) - c \right|\right]
  $$
- High ECE indicates poor calibration, meaning the model's confidence estimates are not reliable.

**Key Insight:**
- Bayesian Neural Networks, by capturing uncertainty in model parameters, can mitigate overconfidence.

### Classification with Neural Networks

**Standard Pipeline:**
1. **Input:** An image (e.g., a tennis ball).
2. **Convolutional Layers:** Extract features.
3. **Fully Connected Layers:** Map features to class scores.
4. **Softmax Layer:** Converts scores to probabilities.
5. **Loss Function:** Cross-entropy to measure prediction error.

**Key Challenge:**
- Standard neural networks provide no measure of model uncertainty, leading to unreliable predictions for unfamiliar or out-of-distribution inputs.

### The Predictive Distribution

**Predictive Entropy:**
- Measures uncertainty in predictions:
  $$
  H = -\sum_{k=1}^K p_k \log p_k
  $$
  - High entropy indicates high uncertainty, while low entropy suggests confident predictions.

**MC-Dropout in Inference:**
- A practical approximation to Bayesian inference:
  - Apply dropout during inference and perform multiple forward passes.
  - Use the variability in predictions to estimate uncertainty.

**Laplace Approximation:**
- Uses a Gaussian approximation around the MAP solution to represent uncertainty in the parameters.

**Deep Ensembles:**
- Train multiple models with different initializations to capture a distribution over predictions.

**Classifier Calibration:**
- Adjusts the predicted probabilities to better reflect actual accuracies.
Here is a detailed explanation of the slides focusing on model uncertainty in Bayesian Neural Networks.

### Model Uncertainty in Neural Networks

**Key Idea:**
To capture model uncertainty, we need to infer a distribution over the weights $\mathbf{w}$ rather than finding a single point estimate. The uncertainty in the weights can be represented by a **covariance matrix** $\Sigma_w$, which quantifies how much the weights can vary. 

**Laplace Approximation:**
- The posterior distribution of the weights is approximated as a Gaussian centered at the maximum a posteriori (MAP) estimate $\mathbf{w}^*$.
- Covariance is approximated using the inverse Hessian $H^{-1}$ of the negative log-posterior:
  $$
  \Sigma_w \approx H^{-1}
  $$
- Log-posterior approximation:
  $$
  \log p(\mathbf{w} \mid X, y) \approx \log p(\mathbf{w}^* \mid X, y) + \frac{1}{2} (\mathbf{w} - \mathbf{w}^*)^T H (\mathbf{w} - \mathbf{w}^*)
  $$
  - This is a second-order Taylor expansion around $\mathbf{w}^*$.

**Hessian Matrix $H$:**
- Defined as:
  $$
  [H]_{ij} = \frac{\partial^2 \log p(\mathbf{w} \mid X, y)}{\partial w_i \partial w_j}
  $$
- The Hessian captures the curvature of the log-posterior, indicating how confident we are about the MAP estimate.

---

### Approximating the Posterior Distribution

**Exponentiating the Approximation:**
- By taking the exponential of the log-posterior approximation:
  $$
  p(\mathbf{w} \mid X, y) \approx p(\mathbf{w}^* \mid X, y) \exp\left(-\frac{1}{2} (\mathbf{w} - \mathbf{w}^*)^T H (\mathbf{w} - \mathbf{w}^*)\right)
  $$
- This leads to a **Gaussian distribution**:
  $$
  p(\mathbf{w} \mid X, y) \approx \mathcal{N}(\mathbf{w}^*, H^{-1})
  $$
- The normalization term $Z$ becomes:
  $$
  Z = p(\mathbf{w}^* \mid X, y) \sqrt{(2\pi)^D \det H}
  $$

---

### Approximations

**Generalised Gauss-Newton Matrix (GNN):**
- For piece-wise linear activation functions like ReLU:
  - The **GNN** matrix is equivalent to the Hessian.
  - This simplifies computations significantly.

**Fisher Information Matrix:**
- When the loss function is from the exponential family (e.g., cross-entropy):
  - The GNN is equivalent to the **Fisher Information Matrix** $F$:
    $$
    F = \mathbb{E}[\delta \theta \delta \theta^T]
    $$
- This equivalence is useful because computing the Fisher matrix is often easier than directly computing the Hessian.

---

### Kronecker Factorisation

**Key Insight:**
- Directly computing and storing the full covariance matrix $\Sigma_w$ is computationally infeasible for large networks.
- **Kronecker Factorisation:**
  - Approximates the Fisher matrix $F$ using Kronecker products of smaller matrices:
    $$
    F_i = A_{i-1} \otimes G_i
    $$
  - Where $A$ represents input correlations and $G$ represents gradients.

**Benefits:**
- Reduces computational complexity by leveraging the structure of neural networks.
- Allows for efficient storage and inversion of the Fisher matrix.

---

### Eigenvalue Decomposition

**Decomposition Approach:**
- The Fisher matrix $F$ is decomposed into eigenvalues $\Lambda$ and eigenvectors $V$:
  $$
  F = V \Lambda V^T
  $$
- For the Kronecker factorisation approach:
  $$
  F \approx (U_A \otimes U_G) \Lambda (U_A \otimes U_G)^T
  $$
  - This expresses $F$ in terms of simpler, smaller matrices.

**Purpose:**
- Makes it feasible to invert the Fisher matrix efficiently.
- Helps in computing uncertainty estimates with reduced computational cost.

---

### The Predictive Distribution

**Goal:**
- Compute the predictive distribution:
  $$
  p(y^* \mid X, y, x^*) = \int p(y^* \mid \mathbf{w}, x^*) p(\mathbf{w} \mid X, y) \, d\mathbf{w}
  $$
- **Monte-Carlo Integration:** Approximates the integral by sampling weights $\mathbf{w}$ from the posterior distribution:
  $$
  \approx \frac{1}{M} \sum_{i=1}^M p(y^* \mid \mathbf{w}_i)
  $$
  - Where $\mathbf{w}_i \sim \mathcal{N}(\mu_w, \Sigma_w)$.

**Low-Rank Approximation:**
- Efficiently inverts the Fisher matrix using a low-rank approximation:
  $$
  F \approx (U_A \otimes U_G) \Lambda (U_A \otimes U_G)^T + D
  $$
- Reduces both memory and computational requirements.

**Key Insight:**
- This approach allows Bayesian Neural Networks to provide uncertainty estimates efficiently, making them practical for real-world applications.

Here is a detailed explanation of the slides focusing on ensembles and classifier calibration in Bayesian Neural Networks.

### Ensembles

**Concept:**
Ensemble methods improve prediction accuracy and robustness by combining multiple models rather than relying on a single model. They reduce variance, bias, or both, depending on the technique used.

**Types of Ensembles:**
1. **Bagging (Bootstrap Aggregating):**
   - Uses multiple sets of training data, created by sampling with replacement.
   - Each model in the ensemble is trained independently in parallel.
   - Predictions are averaged to reduce variance.
   - **Example:** Random Forest.

2. **Boosting:**
   - Combines multiple weak learners sequentially, where each model corrects the errors of the previous one.
   - Focuses on reducing bias.
   - **Example:** AdaBoost.

3. **Deep Ensembles:**
   - Extends the idea of ensembles to deep neural networks by training multiple networks with different initializations and combining their predictions.
   - Captures both aleatoric (data) and epistemic (model) uncertainty.

**Key Insight:**
- Ensembles mitigate overfitting and improve robustness by balancing bias and variance.

---

### Three Elements of Deep Ensemble

**1. Density Network for Mean and Variance:**
   - The last layer outputs mean and variance:
     $$
     \sigma^2 = s^2 = \exp(Z')
     $$
     - $Z'$ is the output of the last layer.
   - Trained with a Gaussian likelihood to capture aleatoric uncertainty.

**2. Proper Scoring Rule as Loss Function:**
   - Uses a scoring function $S(\theta, q)$ to ensure proper uncertainty quantification.
   - A scoring rule is proper if it satisfies:
     $$
     S(p, q) \leq S(q, q)
     $$
   - Loss function becomes:
     $$
     L(\theta) = -S(p_\theta, q)
     $$
   - Ensures the model is penalized correctly for poor uncertainty estimates.

**3. Adversarial Training for Robustness:**
   - Perturbs input data to generate adversarial examples:
     $$
     x' = x + \epsilon \cdot \text{sign}(\nabla_x L(\theta, x, y))
     $$
   - Improves robustness by making the model less sensitive to small input variations.

---

### Training Deep Ensemble

**Key Insight:**
- **Random Initialization:** 
  - Initializing neural networks randomly and shuffling data is sufficient to train diverse ensemble members effectively.
  - Diversity in initialization leads to capturing different aspects of uncertainty.

**Training Procedure:**
1. Randomly initialize parameters for each model.
2. Train each model independently on shuffled data.
3. Use adversarial examples to enhance robustness (optional).
4. Aggregate predictions to form the final ensemble output.

**Hyperparameter Tuning:**
- Different architectures and hyperparameters can be used for each model in the ensemble to increase diversity and improve performance.

---

### Predictions with Deep Ensemble

**Predictive Distribution:**
- The final prediction is obtained by averaging predictions from each model:
  $$
  p(y \mid x) = \frac{1}{M} \sum_{m=1}^M p(y \mid x, \theta_m)
  $$
- **Output Mean:**
  $$
  \mu^*(x) = \frac{1}{M} \sum_{m=1}^M h_{\theta_m}(x)
  $$
- **Output Variance:**
  $$
  \sigma^{2*}(x) = \frac{1}{M} \sum_{m=1}^M (\sigma_{\theta_m}^2(x) + h_{\theta_m}^2(x)) - \mu^*(x)^2
  $$
- Captures both aleatoric and epistemic uncertainty effectively.

---

### Deep Ensemble of Bayesian Neural Networks
![[Pasted image 20250310095817.png#invert|600]]
**Concept:**
- Combines the strengths of **variational methods** (which capture local uncertainty) and **deep ensembles** (which capture global uncertainty).
- Uses a mixture of Gaussian posteriors:
  $$
  p(\theta \mid D) \approx \frac{1}{M} \sum_{m=1}^M p(\theta_m \mid D)
  $$
- Each member is trained with different initializations and approximates a different mode in the parameter space.

**Key Insight:**
- Effective for uncertainty estimation, as demonstrated in competitive settings like Kaggle and NeurIPS.

---

### Classifier Calibration

**Purpose:**
- Ensures that predicted probabilities reflect true likelihoods.
- For example, if a model predicts a class with 80% probability, it should be correct 80% of the time.

**Advantages:**
- **Independent of training algorithm:** Can be applied to any classifier, even if it’s not a neural network.
- **Faster and simpler:** Easier than training multiple models or using complex Bayesian methods.

**Drawbacks:**
- Requires an additional **calibration dataset**.
- Needs a well-defined **calibration function**.

**Insight for Neural Networks:**
- Training on the **negative log-likelihood (NLL)** leads to overfitting on NLL but not on accuracy.
- Calibration helps adjust probabilities to reflect true confidence.

---

### Classifier Calibration Methods

**Binary Classification:**
1. **Platt Scaling:** Fits a logistic regression model to the outputs.
2. **Isotonic Regression:** A non-parametric method that preserves ordering.
3. **Beta Calibration:** Extends Platt scaling using a Beta distribution.
4. **Bayesian Binning into Quantiles:** Uses bins to approximate probability distributions.

**Multiclass Classification:**
1. **One-vs-All:** Extends binary methods to multiclass problems.
2. **Temperature Scaling:** Adjusts softmax outputs using a temperature parameter $T$:
   $$
   v(z) = \sigma\left(\frac{z}{T}\right)
   $$
   - $T > 1$ reduces confidence, while $T < 1$ increases it.

**Key Insight:**
- Simple and effective for improving calibration without retraining the entire model.

---

### Non-Parametric Classifier Calibration

**Main Idea:**
- Models the latent function $g$ using a **Gaussian Process (GP)**:
  $$
  g \sim \mathcal{GP}(\mu(\cdot), k(\cdot, \cdot \mid \theta))
  $$
- Trains the GP on a **calibration dataset** to learn a mapping from classifier scores to true probabilities.

**Efficiency:**
- Uses **inducing points** to reduce computational cost:
  - Inducing points act as a summary of the dataset, making GP inference feasible for large datasets.

**Key Insight:**
- Captures complex calibration functions effectively, improving reliability for critical applications.


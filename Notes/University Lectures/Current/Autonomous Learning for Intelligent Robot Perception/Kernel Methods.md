### **Classical Learning Algorithms and Feature Functions**
1. **Feature Representation Assumption**: 
   - Traditional algorithms assume that input data can be transformed into a **feature vector** of a finite and fixed length using a function $\phi(x)$. 
2. **Challenges with Feature Vectors**:
   - Some data types or objects are hard to being represented by fixed-length feature vectors:
     - **Text Documents**: Their length can vary, and the relationship between terms is not inherently captured by simple feature vectors.
     - **Molecular Structures**: Complex relationships between atoms and bonds may not be easily represented by fixed-length features.
     - **Evolutionary Trees**: These are hierarchical structures, and compressing them into a finite-length vector loses information.
### **Kernel Functions as an Alternative**
1. **Similarity-Based Approach**:
   - Instead of explicitly constructing feature vectors, kernel functions rely on **measuring the similarity** between two objects (data points) directly.
   - This eliminates the need to explicitly compute or store high-dimensional feature vectors.
2. **Examples of Kernels**:
   - For **text documents**, you might use an **edit distance kernel**:
     - The "edit distance" measures how many changes (insertions, deletions, substitutions) are required to transform one string into another.
     - This similarity measure can act as a kernel.
   - For **molecular structures**, you could define a kernel that compares bond types, connectivity, or other chemical properties.
3. **Kernel Trick**:
   - Kernel functions compute the dot product of the feature vectors in the transformed space without explicitly transforming the data:
     $$
     k(x_i, x_j) = \phi(x_i)^T \phi(x_j)
     $$
   - This allows working in very high-dimensional (or even infinite-dimensional) spaces efficiently.
## Dual Representation
The "Dual Representation" in the slides reformulates the Ridge Regression problem to express the solution in terms of **dot products** between training data points. 
### Ridge Regression Objective
The original Ridge Regression objective is:

$$
E(w) = \frac{1}{2} \sum_{i=1}^N \left(w^T \phi(x_i) - y_i \right)^2 + \frac{\lambda}{2} w^T w
$$

- $w$: Weight vector of length $M$, where $M$ is the dimensionality of the feature space.
- $\phi(x_i)$: Feature vector representation of input $x_i$, of size $M$.
- $y_i$: Target value for the $i$-th data point.
- $\lambda$: Regularization parameter controlling overfitting.
- $N$: Number of data points.
The sum of squared residuals and regularization term can be written in matrix form:

$$
E(w) = \frac{1}{2} w^T \Phi^T \Phi w - y^T \Phi w + \frac{1}{2} y^T y + \frac{\lambda}{2} w^T w
$$

Here:
- $\Phi \in \mathbb{R}^{N \times M}$: Design matrix, where each row is $\phi(x_i)^T$.
- $y \in \mathbb{R}^N$: Vector of target values.
By taking the gradient of $E(w)$ with respect to $w$ and setting it to zero, we obtain the optimal solution:

$$
w^* = \left(\Phi^T \Phi + \lambda I_M\right)^{-1} \Phi^T y
$$

Where:
- $I_M$ is the $M \times M$ identity matrix.
- This requires inverting an $M \times M$ matrix, which can be computationally expensive if $M$ (the feature dimension) is large. 
Applying the [[Matrix Inversion Lemma]] yields:
- $$w^{*} = \Phi^{T}\underbracket{(\Phi\Phi^{T}+\lambda I_{N})^{-1}}_{=:a}y$$
### Dual Representation
The Dual Representation expresses the weight vector $w$ as a linear combination of the training data's feature vectors:
$$
w = \Phi^T a
$$
- $a$ is a vector of coefficients that indicate how much influence each training data point has on $w$.
Instead of solving the problem in terms of $w$, we express $w$ as a linear combination of the training data's feature vectors:

$$
w = \Phi^T a
$$

Here:
- $a \in \mathbb{R}^N$: Coefficient vector for the linear combination of $\Phi^T$.

Substitute $w = \Phi^T a$ into the Ridge Regression objective:

$$
E(a) = \frac{1}{2} a^T \Phi \Phi^T \Phi \Phi^T a - a^T \Phi \Phi^T y + \frac{\lambda}{2} a^T \Phi \Phi^T a + \frac{1}{2} y^T y
$$

Simplify using the kernel matrix $K = \Phi \Phi^T \in \mathbb{R}^{N \times N}$:

$$
E(a) = \frac{1}{2} a^T K K a - a^T K y + \frac{\lambda}{2} a^T K a + \frac{1}{2} y^T y
$$

Where:
- $K_{ij} = \phi(x_i)^T \phi(x_j)$ is the kernel function (a dot product in feature space) which is a comparison of our data points in feature space (a comparison between each 2 data points) $$   K = 
   \begin{bmatrix}
   \phi(x_1)^T \phi(x_1) & \phi(x_1)^T \phi(x_2) & \cdots & \phi(x_1)^T \phi(x_N) \\
   \phi(x_2)^T \phi(x_1) & \phi(x_2)^T \phi(x_2) & \cdots & \phi(x_2)^T \phi(x_N) \\
   \vdots & \vdots & \ddots & \vdots \\
   \phi(x_N)^T \phi(x_1) & \phi(x_N)^T \phi(x_2) & \cdots & \phi(x_N)^T \phi(x_N) \\
   \end{bmatrix}
   $$
- $K$ is symmetric and of size $N \times N$, which depends only on the number of data points. Also $K^{T}= K$
### Dual Solution
The dual problem is now solved with respect to $a$. The gradient of $E(a)$ with respect to $a$ is set to zero:

$$
\frac{\partial E(a)}{\partial a} = K K a - K y + \lambda K a = 0
$$

Factorizing $K$, we get:

$$
(K + \lambda I_N) a = y
$$

Solve for $a$:

$$
a^* = (K + \lambda I_N)^{-1} y
$$
### Predictions Using the Dual Representation
Predictions for a new data point $x^*$ are computed as:

$$
f(x^*) = \phi(x^*)^T w
$$

Substitute $w = \Phi^T a$:

$$
f(x^*) = \phi(x^*)^T \Phi^T a
$$

Using the kernel matrix $K$:

$$
f(x^*) = k(x^*)^T a
$$

Where:
- $k(x^*) \in \mathbb{R}^N$: The kernel vector, where $k(x^*)_i = \phi(x_i)^T \phi(x^*)$.

Finally, substitute $a^*$ into the prediction:

$$
f(x^*) = k(x^*)^T (K + \lambda I_N)^{-1} y
$$

### The Kernel Trick
The **Kernel Trick** is a method to compute the dot product $\phi(x_i)^T \phi(x_j)$ directly, without ever knowing $\phi(x)$. Instead, we use a kernel function $k(x_i, x_j)$ to compute the similarity in feature space:
$$
k(x_i, x_j) = \phi(x_i)^T \phi(x_j)
$$
#### Why is this powerful?
1. **Implicit Feature Mapping**:
   - The kernel function defines a similarity measure directly in the input space.
   - Even if $\phi(x)$ maps to a very high-dimensional or infinite-dimensional space, we can compute $k(x_i, x_j)$ efficiently.
2. **Flexibility**:
   - You can design custom kernel functions tailored to your data, such as:
     - **Gaussian Kernel**: Measures similarity based on distance.
     - **Polynomial Kernel**: Captures polynomial relationships.
     - **Edit Distance Kernel**: Measures similarity for strings or sequences.
3. **No Explicit Features Needed**:
   - Instead of designing or computing features, you only need to define a kernel that captures the relationships you care about.
The key advantage of the dual representation is that it expresses $f(x^*)$ entirely in terms of **dot products** between training points ($K$) and between training points and the new input ($k(x^*)$). This enables the use of kernel functions:

$$
k(x_i, x_j) = \phi(x_i)^T \phi(x_j)
$$

Without explicitly computing $\phi(x)$, we can use kernel functions to work in **very high-dimensional or infinite-dimensional spaces** efficiently.
###  Summary
- The dual representation reformulates Ridge Regression to work in terms of **coefficients $a$** and the kernel matrix $K$, which depends only on pairwise dot products of the data.
- The dual solution requires inverting an $N \times N$ matrix (where $N$ is the number of data points), making it computationally preferable if $N \ll M$.
- The kernel trick allows working in high-dimensional spaces without explicitly computing the feature vectors.

### Predictions in Dual Form
Once the Dual Representation is solved, predictions for a new point $x^*$ are made using:
$$
f(x^*) = \sum_{i=1}^N a_i k(x^*, x_i)
$$
This means:
- The prediction depends on the similarity of the new point $x^*$ to each training point $x_i$.
- The training data points effectively "vote" on the prediction, weighted by the coefficients $a_i$.
### Example: Intuition for Kernels
#### Linear Kernel:
For a linear kernel, $k(x_i, x_j) = x_i^T x_j$. This corresponds to standard Ridge Regression in the input space.

#### Gaussian Kernel:
For a Gaussian kernel:
$$
k(x_i, x_j) = \exp\left(-\frac{\|x_i - x_j\|^2}{2\sigma^2}\right)
$$
- It measures similarity based on distance. Points closer together have higher similarity.
- This allows modeling highly non-linear relationships in the input space.
#### Edit Distance Kernel:
For text data, the kernel could be based on the edit distance (number of insertions, deletions, or substitutions to make one string equal to another). This avoids defining explicit features for text and instead measures similarity directly.
## Constructing Kernels
A kernel $k(x_i, x_j)$ is a function that computes the **similarity** between two data points $x_i$ and $x_j$ in a higher-dimensional feature space $\mathcal{H}$. This is equivalent to the dot product of their feature representations:
$$
k(x_i, x_j) = \phi(x_i)^T \phi(x_j)
$$
- $\phi(x)$: A feature map that maps data into a (possibly infinite-dimensional) feature space.
- The kernel function allows us to compute this similarity **without explicitly knowing or calculating $\phi(x)$**.
### Properties of a Valid Kernel
For $k(x_i, x_j)$ to be a valid kernel, it must satisfy the following mathematical properties:
1. **Symmetry**:
   $$
   k(x_i, x_j) = k(x_j, x_i)
   $$
   This means the similarity between $x_i$ and $x_j$ is the same as the similarity between $x_j$ and $x_i$.

2. **Linearity in $\mathcal{H}$** (Inner Product Structure):
   $$
   k(ax + bz, x_j) = a k(x, x_j) + b k(z, x_j)
   $$
   This ensures that the kernel behaves like a dot product in $\mathcal{H}$.

3. **Positive Semi-Definiteness**:
   The Gram matrix $K$, defined as:
   $$
   K = 
   \begin{bmatrix}
   k(x_1, x_1) & k(x_1, x_2) & \cdots & k(x_1, x_N) \\
   k(x_2, x_1) & k(x_2, x_2) & \cdots & k(x_2, x_N) \\
   \vdots & \vdots & \ddots & \vdots \\
   k(x_N, x_1) & k(x_N, x_2) & \cdots & k(x_N, x_N) \\
   \end{bmatrix}
   $$
   must be positive semi-definite, i.e., for any vector $v$,
   $$
   v^T K v \geq 0
   $$
   This ensures that the kernel represents a valid similarity measure.
### Mercer’s Theorem 
Mercer’s theorem guarantees that if a function $k(x_i, x_j)$ satisfies symmetry and positive semi-definiteness, then there exists a feature map $\phi(x)$ such that:
$$
k(x_i, x_j) = \phi(x_i)^T \phi(x_j)
$$
**Implications**:
- We do not need to explicitly find $\phi(x)$; we can directly work with $k(x_i, x_j)$ (this is the **Kernel Trick**).
- Any valid kernel implicitly corresponds to some feature space.
### Constructing Kernels
Creating valid kernels from scratch can be challenging, but there are several rules to combine or modify existing kernels to produce new valid kernels.
#### Rules:
1. **Scaling**:
   $$
   k(x_1, x_2) = c k_1(x_1, x_2), \quad c > 0
   $$
   Scaling a valid kernel by a positive constant $c$ results in a valid kernel.

2. **Addition**:
   $$
   k(x_1, x_2) = k_1(x_1, x_2) + k_2(x_1, x_2)
   $$
   The sum of two valid kernels is also a valid kernel.

3. **Multiplication**:
   $$
   k(x_1, x_2) = k_1(x_1, x_2) k_2(x_1, x_2)
   $$
   The product of two valid kernels is valid.

4. **Exponentiation**:
   $$
   k(x_1, x_2) = \exp(k_1(x_1, x_2))
   $$

5. **Affine Transformation**:
   $$
   k(x_1, x_2) = x_1^T A x_2, \quad A \text{ is positive semi-definite}
   $$

**Intuition**: These rules allow flexibility in designing kernels for specific applications by combining simpler kernels in meaningful ways.
### Examples of Valid Kernels
#### (a) **Polynomial Kernel**:
$$
k(x_i, x_j) = (x_i^T x_j + c)^d, \quad c > 0, \, d \in \mathbb{N}
$$
- Captures polynomial relationships of degree $d$ between data points.
- For example, $d=2$ allows the kernel to capture interactions like $x_1^2, x_2^2, x_1x_2$.

#### (b) **Gaussian (RBF) Kernel**:
$$
k(x_i, x_j) = \exp\left(-\frac{\|x_i - x_j\|^2}{2\sigma^2}\right)
$$
- Measures similarity based on the distance between points.
- Small $\sigma$: Focuses on local similarity.
- Large $\sigma$: Considers more global relationships.

#### (c) **Kernel for Sets**:
$$
k(A_1, A_2) = 2^{|A_1 \cap A_2|}
$$
- Designed for set inputs; similarity depends on the size of the intersection.

#### (d) **Matern Kernel**:
$$
k(r) = \frac{2^{1-\nu}}{\Gamma(\nu)} \left(\frac{\sqrt{2\nu} r}{l}\right)^\nu K_\nu\left(\frac{\sqrt{2\nu} r}{l}\right)
$$
- Used in Gaussian Processes for flexible smoothness control.
### A Simple Example
Consider the kernel:
$$
k(x, x') = (x^T x')^2, \quad x, x' \in \mathbb{R}^2
$$
This can be expanded as:
$$
(x_1x_1' + x_2x_2')^2 = x_1^2x_1'^2 + x_2^2x_2'^2 + 2x_1x_2x_1'x_2'
$$
This is equivalent to a dot product in the transformed space:
$$
\phi(x) = [x_1^2, x_2^2, \sqrt{2}x_1x_2]^T
$$
- In this case, $\phi(x)$ maps the input into a higher-dimensional space where the decision boundary (originally non-linear) becomes linear.
   - With a suitable kernel function, you can model non-linear relationships between data points even though the learning algorithm itself (e.g., Ridge Regression) operates linearly in the feature space.
### Visualization of Example
- The original decision boundary (in the input space) is an **ellipse** -> not an linear separator
- After applying the kernel (i.e., transforming to the feature space), the decision boundary becomes a **hyperplane**. This is the key idea behind kernels: <mark style="background: #FFB86CA6;">making complex problems linear in a higher-dimensional space</mark>.
![[Pasted image 20241121142426.png#invert|400]]
### Applications of Kernels
Kernels are used in:
- **Density Estimation**: Estimate probability distributions in unsupervised learning.
- **Regression**: Kernel Ridge Regression.
- **Classification**: Support Vector Machines (SVMs).
- **Dimensionality Reduction**: Principal Component Analysis (PCA) with kernels.
- **Gaussian Processes**: Probabilistic regression models.
## Kernelization
- **Kernelization** is the process of converting an existing algorithm into a **kernel-based algorithm**.
- The key idea:
  - Express similarities between data points in terms of **inner products**.
  - Replace these inner products with a **kernel function** $k(x_i, x_j)$.
- This replacement is known as the **kernel trick**, which allows computations in high-dimensional (or infinite-dimensional) feature spaces without explicitly constructing the feature vectors.
### **Nearest Neighbor Example**
#### Original Nearest Neighbor Algorithm:
- The nearest neighbor classifier uses **Euclidean distance** to find the closest data point and assigns its label to the new data point.
$$
\|x_i - x_j\|^2 = x_i^T x_i + x_j^T x_j - 2x_i^T x_j
$$
#### Kernelized Nearest Neighbor:
- Replace the dot products $x_i^T x_j$ in the distance formula with a **kernel function**:
$$
d(x_i, x_j)^2 = k(x_i, x_i) + k(x_j, x_j) - 2k(x_i, x_j)
$$
- Now, the distance is computed directly in terms of the **kernel** without explicitly constructing feature vectors.
**Intuition**:
- This enables the nearest neighbor classifier to operate in a **higher-dimensional feature space**, making it more effective for complex, non-linear data distributions. 
### Kernel Regression
1. **Predictive Distribution:**
   - Kernel regression involves predicting a value $t^*$ for a new input $x^*$ based on given data $(X, t)$, where $X$ is the set of input data points and $t$ represents the corresponding target values.
   - The predictive distribution is expressed as:
     $$
     p(t^* \mid x^*, X, t) = \frac{p(t^*, x^*, X, t)}{p(x^*, X, t)} = \frac{p(t^*, x^*, X, t)}{\int p(t^*, x^*, X, t) dt^*}
     $$
     This shows that the predictive probability is the ratio of the joint distribution of $t^*$ and $x^*$ to the marginal distribution over $x^*$.
2. **Key Idea:**
   - Use **Kernel Density Estimation** to approximate the joint distribution $p(t^*, x^*, X, t)$:
     $$
     p(t^*, x^*, X, t) \approx \frac{1}{N} \sum_{i=1}^N f(x^* - x_i, t^* - t_i)
     $$
     Here, $f(\cdot)$ is a kernel function that determines the similarity or closeness of the new point $x^*$ to the data points $x_i$ and their corresponding targets $t_i$.
3. **Predictive Mean:**
   - Instead of the full predictive distribution, we are only interested in the **predictive mean**, $\mu^*$, which is computed as:
     $$
     \mu^* = \int t^* p(t^* \mid x^*, X, t) dt^*
     $$
     This integral gives a single predicted value $\mu^*$ for the target $t^*$ based on the kernel-weighted contribution of the data points.

#### Predictive Mean in Kernel Regression
1. **Formula for Predictive Mean:**
   - By substituting the kernel density approximation, the predictive mean simplifies to:
     $$
     \mu^*(x^*) = \frac{\sum_{i=1}^N g(x^* - x_i)t_i}{\sum_{i=1}^N g(x^* - x_i)}
     $$
     - $g(\cdot)$ is the kernel function that determines how much each data point $x_i$ contributes to the prediction.
     - Intuitively, $t_i$ values are weighted by their similarity to $x^*$, as determined by $g(x^* - x_i)$.
2. **Symmetric Kernels:**
   - A symmetric kernel function $k(x^*, x_i)$ is commonly used, leading to:
     $$
     \mu^*(x^*) = \sum_{i=1}^N k(x^*, x_i)t_i
     $$
     Here, $k(x^*, x_i)$ acts as a normalized weight, ensuring all contributions sum up to 1.
3. **Nadaraya-Watson Model:**
   - The resulting model is called the **Nadaraya-Watson kernel regression model**.
   - It computes the prediction as a weighted average of the target values $t_i$, with the weights determined by the kernel function.
4. **Non-Bayesian Approach:**
   - Note: This method is not Bayesian, as it does not explicitly model a posterior distribution over $t^*$. Instead, it uses kernel density estimation to approximate the predictive mean.
### **Bayesian Linear Regression**
Bayesian Linear Regression reformulates standard regression with a probabilistic approach:
- **Prior Distribution**: The weights $w$ are assumed to have a prior distribution:
  $$
  p(w) = \mathcal{N}(w; 0, \Sigma_{\text{pr}})
  $$
  This reflects our initial belief about $w$ (e.g., zero mean, specific covariance).

- **Likelihood**: The observed data likelihood, given $w$, is:
  $$
  p(y \mid w, X) = \mathcal{N}(y; \Phi w, \Sigma_{ll})
  $$
  This assumes the data $y$ is normally distributed around the predictions $\Phi w$.

- **Posterior Distribution**: Using Bayes’ theorem, the posterior distribution over $w$ combines the prior and likelihood:
  $$
  p(w \mid y, X) = \mathcal{N}(w; \mu_{\text{post}}, \Sigma_{\text{post}})
  $$
  Where:
  $$
  \mu_{\text{post}} = \Sigma_{\text{post}} \Phi^T \Sigma_{ll}^{-1} y
  $$
  $$
  \Sigma_{\text{post}} = (\Sigma_{\text{pr}}^{-1} + \Phi^T \Sigma_{ll}^{-1} \Phi)^{-1}
  $$
### **Solution to Bayesian Regression**
- The **maximum a posteriori (MAP)** estimate of $w$ is:
$$
w^* = (\Phi^T \Phi + \lambda I)^{-1} \Phi^T t
$$
This is equivalent to the solution of Ridge Regression, as it balances data fitting with regularization based on the prior.
### **A Different Way of Looking at This**
In Bayesian Linear Regression:
1. Each sample from $p(w)$ induces a **different function** $f(x)$. This gives us a **distribution over functions**.
2. The predictions $y$ are distributed as:
   - Mean:
     $$
     E[y] = \Phi E[w]
     $$
     Since $E[w] = 0$ (zero-mean prior):
     $$
     E[y] = 0
     $$
   - Covariance:
     $$
     \text{cov}[y] = \Phi \text{cov}[w] \Phi^T = \Phi \Sigma_{\text{post}} \Phi^T
     $$
     This covariance reflects the uncertainty in the predictions.

3. After training:
   - The distribution over $y$ is a **Gaussian Process** with mean zero and covariance $\Phi \Sigma_{\text{post}} \Phi^T$.
## **Gaussian Process (GP) Explanation**
### **Definition**
A **Gaussian Process (GP)** is a collection of random variables, any finite number of which have a joint Gaussian distribution. GPs are a **distribution over functions**, meaning:
- Instead of specifying a fixed function, a GP represents a **prior over the space of possible functions** that could describe the data.
- For a GP, the function's behavior is fully defined by:
  - A **mean function** $m(x)$:
    $$
    m(x) = \mathbb{E}[y(x)]
    $$
  - A **covariance function** (kernel) $k(x_1, x_2)$:
    $$
    k(x_1, x_2) = \mathbb{E}[(y(x_1) - m(x_1))(y(x_2) - m(x_2))]
    $$

This enables GPs to model distributions over infinitely many random variables, where only finite subsets are observed or queried.
### **Example Visualization**
![[Pasted image 20241121143002.png#invert|300]]
The plot shows:
- **Green Line**: True function generating the data (sinusoidal with Gaussian noise).
- **Blue Circles**: Observed noisy data points.
- **Red Line**: The **mean function** of the Gaussian process (posterior mean prediction).
- **Shaded Red Area**: A **confidence interval** around the prediction, representing the uncertainty (±2 standard deviations, $2\sigma$).

**Key Intuition**:
- The GP "learns" the function from observed points (blue circles), while uncertainty increases in regions with no data (as shown by wider confidence intervals).
### **Handling Infinite Variables**
Since a GP deals with an infinite number of random variables (the entire function space):
- We split the random variables into two parts:
  - **Finite Part** ($x_f$): Observed data points or specific predictions.
  - **Infinite Part** ($x_i$): The remaining unobserved variables.
From the **marginalization property of multivariate Gaussians**, we compute the marginal distribution of the finite part:
$$
p(x_f) = \int p(x_f, x_i) dx_i
$$
This reduces the infinite dimensionality to a finite problem, allowing us to work with finite covariance matrices.
### **Covariance Function (Kernel)**
The covariance function $k(x_p, x_q)$ determines the structure of the functions modeled by the GP.
#### **Squared Exponential Kernel**
The most common kernel is the **squared exponential kernel (RBF kernel)**:
$$
k(x_p, x_q) = \sigma_f^2 \exp\left(-\frac{1}{2\ell^2} (x_p - x_q)^2\right) + \sigma_n^2 \delta_{pq}
$$
Where:
- $\sigma_f^2$: **Signal variance**, controls the scale of the function values.
- $\ell$: **Length scale**, controls how quickly the correlation between points decreases (smoothness of the function).
- $\sigma_n^2 \delta_{pq}$: Noise variance, accounts for observation noise (only added when $x_p = x_q$).

#### Other Kernels:
- **Exponential Kernel**:
  $$
  k(x_p, x_q) = \exp(-\theta |x_p - x_q|)
  $$
  Used for processes like the Ornstein-Uhlenbeck process (less smooth functions).
### **Sampling from a GP**
A GP is a distribution over functions, so we can generate **samples of functions** from it.
#### Steps:
1. Choose input points $x_1^*, \ldots, x_M^*$.
2. Compute the covariance matrix $K$, where $K_{ij} = k(x_i^*, x_j^*)$.
3. Sample a random Gaussian vector from:
   $$
   y^* \sim \mathcal{N}(0, K)
   $$
4. Plot $y^*$ vs. $x^*$.

**Result**:
Each sample represents a different possible function consistent with the prior distribution.
### **Prediction with a GP**
For **prediction**, the goal is to compute the posterior distribution of the function at test points $x_*$, given:
- Training data $(x, y)$,
- Test inputs $x_*$.
#### Joint Gaussian Distribution
The training data $y$ and test outputs $y_*$ have a joint Gaussian distribution:
$$
\begin{pmatrix}
y \\
y_*
\end{pmatrix}
\sim \mathcal{N}\left(0, \begin{pmatrix}
K(X, X) & K(X, X_*) \\
K(X_*, X) & K(X_*, X_*)
\end{pmatrix}\right)
$$
Where:
- $K(X, X)$: Covariance matrix of training points.
- $K(X, X_*)$: Cross-covariance between training and test points.
- $K(X_*, X_*)$: Covariance matrix of test points.
#### Conditional Distribution
Using the properties of Gaussian conditionals, the posterior distribution at the test points is:
$$
p(y_* \mid x_*, X, y) = \mathcal{N}(\mu_*, \Sigma_*)
$$
Where:
- Mean:
  $$
  \mu_* = K_*^T K^{-1} y
  $$
- Covariance:
  $$
  \Sigma_* = K(X_*, X_*) - K_*^T K^{-1} K_*
  $$

**Key Intuition**:
- The posterior mean $\mu_*$ represents the predicted function values.
- The posterior covariance $\Sigma_*$ represents uncertainty in the predictions.
### **Varying the Hyperparameters**
The **squared exponential kernel** can be generalized:
$$
k(x_p, x_q) = \sigma_f^2 \exp\left(-\frac{1}{2} (x_p - x_q)^T M (x_p - x_q)\right) + \sigma_n^2 \delta_{pq}
$$
Where $M$ determines how the input space is scaled:
1. $M = \ell^{-2} I$: Isotropic kernel with a single length scale $\ell$.
2. $M = \text{diag}(\ell_1, \ldots, \ell_D)^{-2}$: Anisotropic kernel, each dimension has its own length scale.
3. $M = \Lambda^T \Lambda + \text{diag}(\ell_1, \ldots, \ell_D)^{-2}$: Adds correlations between dimensions.

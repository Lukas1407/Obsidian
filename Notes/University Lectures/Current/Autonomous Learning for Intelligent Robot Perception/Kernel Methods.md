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
## **Bayesian Linear Regression vs. Gaussian Processes**
1. **Question:**
   - How does the predictive distribution in Bayesian Linear Regression relate to Gaussian Process Regression?
2. **Answer:**
   - Bayesian Linear Regression is a **special case** of Gaussian Process Regression (GPR).
   - In GPR, the kernel function $k(x_i, x_j)$ represents the covariance between the outputs for inputs $x_i$ and $x_j$. For Bayesian Linear Regression, the kernel function is:
     $$
     k(x_i, x_j) = \sigma_1^2 \phi(x_i)^T \phi(x_j) + \sigma_2^2 \delta_{ij}
     $$
     - $\phi(x)$: Feature mapping function (basis function).
     - $\delta_{ij}$: Kronecker delta (handles noise by adding variance $\sigma_2^2$ to the diagonal).
     - The covariance depends on the dot product of features and additive noise.
### **GP Implementation**
![[Pasted image 20241128130936.png#invert|500]]
1. **Formulas:**
   - GPR predicts a mean $\mu_*$ and variance $\Sigma_*$ at a test point $x_*$:
     $$
     \mu_* = k_*^T K^{-1} y
     $$
     $$
     \Sigma_* = k(x_*, x_*) - k_*^T K^{-1} k_*
     $$
     - $k_*$: Covariance vector between $x_*$ and training points.
     - $K$: Covariance matrix of the training data.
     - $y$: Observed outputs.
2. **Algorithm Steps:**
   - Precompute during training:
     - Covariance matrix $K_{ij} = k(x_i, x_j)$.
     - Add noise $\sigma_n^2 I$.
     - Use **Cholesky decomposition** for efficient inversion of $K$.
   - Test phase:
     - Compute predictive mean and variance using $\mu_*$ and $\Sigma_*$.
3. **Notes:**
   - Cholesky decomposition is preferred for numerical stability.
   - It is efficient for computing the inverse of covariance matrices.
### **Estimating Hyperparameters**
1. **Marginal Likelihood:**
   - To find optimal hyperparameters (e.g., kernel length scales $l$, variance $\sigma_f^2$, noise $\sigma_n^2$), maximize the marginal likelihood:
     $$
     p(y \mid X) = \int p(y \mid f, X) p(f \mid X) df
     $$
     - This integrates out the latent function $f$ (GP assumption).
2. **Optimization:**
   - Take the logarithm of the marginal likelihood.
   - Compute its derivative and set it to zero (closed-form solution due to Gaussian assumptions).
   - This process forms the **training step**.
### Hyperparameter Challenges
1. **Log Marginal Likelihood:**
   - The log marginal likelihood is **not necessarily concave**, meaning it can have **local maxima**.
   - This complicates optimization and can lead to suboptimal hyperparameter values.
2. **Illustrations:**
   - Contour plots show the interplay of noise variance and length scale in the kernel.
   - Predicted outputs can vary significantly depending on hyperparameters.
### **Automatic Relevance Determination (ARD)**
1. **ARD Concept:**
   - The covariance function can include weights for each input dimension:
     $$
     k(x, x') = \sigma_f^2 \exp\left(-\frac{1}{2} \sum_{i=1}^D \eta_i (x_i - x'_i)^2\right)
     $$
     - $\eta_i$: Determines the importance of each dimension.
![[Pasted image 20241128131021.png#invert|300]]
2. **Relevance of Dimensions:**
   - If $\eta_i$ (or equivalently the length scale $l_i = 1/\eta_i$) is large, that dimension is **less relevant**.
   - ARD automatically identifies which input features matter during training.
### **Application of GPs in Terrain Mapping**
1. **Problem Statement:**
   - Robotics terrain mapping faces challenges:
     - Lack of unique landmarks.
     - Difficulty in "closing the loop" when returning to the same location.
     - Need for uncertainty quantification.
2. **Observations:**
   - The environment can often be treated as **2D** (e.g., along gravity) instead of fully 3D.
3. **Proposed Solution:**
   - Use **Gaussian Process Regression** with an **efficient, sparse kernel** to model the map and capture uncertainties effectively.
### **Sparse Kernel Interpolation**
1. **Core Idea:**
   - **Inducing Points $\mathbf{u}_i$:**
     - Instead of using all data points (which is computationally expensive), select a small number of **inducing points** $\mathbf{u}_i$ on a grid structure.
   - **Interpolation:**
     - Interpolate between these grid points using **weights**. This reduces the computational burden while maintaining a good approximation of the original kernel function.
![[Pasted image 20241128131159.png#invert|300]]
1. **Mathematical Explanation:**
   - **Interpolated Kernel:**
     - The interpolated kernel is defined as:
       $$
       \tilde{k}(x_i, u_j) = w_i k(u_a, u_j) + (1 - w_i) k(u_b, u_j)
       $$
       - $k(u_a, u_j)$ and $k(u_b, u_j)$: Kernel values between grid points.
       - $w_i$: Interpolation weights based on the location of the point $x_i$ relative to the grid.

   - **Matrix Approximation:**
     - The kernel matrix $K_{X,X}$ (between all input points) is approximated using the grid-induced kernel:
       $$
       K_{X,X} \approx WK_{U,U}W^T
       $$
       - $W$: Weight matrix for interpolation.
       - $K_{U,U}$: Kernel matrix between inducing points.
       - This formulation reduces complexity by working with a smaller matrix $K_{U,U}$.
3. **Visualization:**
   - The grid structure (black dots) and the original data points (green crosses) illustrate the interpolation. Inducing points serve as a simplified representation of the data.
### **Gaussian Process Gradient Maps (GPGMaps)**
1. **Overview:**
   - **GPGMaps** are a specific application of Gaussian Processes to generate visual representations (maps) of terrain, point clouds, or environments. They include:
     - A **gradient map**: Highlights changes in terrain.
     - An **elevation image**: Shows height variations.
     - A **variance image**: Quantifies uncertainty in the predictions.
2. **Key Features:**
   - **Sparse Kernels:**
     - Sparse kernels are used for faster inference, which is crucial for real-time applications like robotics or planetary mapping.
   - **Visual Cues:**
     - These maps enhance registration (alignment) of point clouds that might otherwise lack sufficient structure.
3. **Benefits:**
   - **Detecting Loop Closures:**
     - In Simultaneous Localization and Mapping (SLAM), loop closures help correct drift by identifying when the robot revisits a location.
   - **Robust Registration:**
     - The maps provide strong features for aligning datasets, even in environments lacking geometric features (e.g., rocks in sloped terrain).
   - **Higher Loop Closure Count:**
     - Gradient and variance maps enable better performance in challenging terrains like slopes
## Logistic Regression
Logistic Regression is a method for **binary classification** where we predict the probability of a data point belonging to one of two classes. Let's break down the concepts from the slides step by step:
### Converting Regression into Classification
1. **Regression to Classification:**
   - Logistic Regression converts a regression problem into a classification problem using the **sigmoid function** $\sigma(a)$:
     $$
     \sigma(a) = \frac{1}{1 + e^{-a}}
     $$
     - The sigmoid function maps any input value $a$ to a range between 0 and 1, which can be interpreted as a **classification probability**.
2. **Linear Model and Probability:**
   - We use a linear model $f(x, w) = w^T \phi(x)$, where:
     - $\phi(x)$: Feature vector for input $x$.
     - $w$: Weight vector.
   - The predicted probability of belonging to the positive (foreground) class is:
     $$
     y_i = \sigma(w^T \phi(x_i))
     $$
   - This gives a two-class (binary) classification model.
### **Why Use the Logistic Sigmoid?**
1. **Generative Model Connection:**
   - Logistic regression can be derived from Bayes' rule:
     $$
     p(y = k \mid x) = \frac{p(x \mid y = k) p(y = k)}{p(x)}
     $$
   - For two classes ($k_1$ and $k_2$):
     $$
     p(y = k_1 \mid x) = \frac{p(x \mid y = k_1)p(y = k_1)}{p(x \mid y = k_1)p(y = k_1) + p(x \mid y = k_2)p(y = k_2)}
     $$
2. **Simplifying to Sigmoid:**
   - Let $a$ represent the **log-odds**:
     $$
     a = \log \frac{p(x \mid y = k_1)p(y = k_1)}{p(x \mid y = k_2)p(y = k_2)}
     $$
   - The probability $p(y = k_1 \mid x)$ becomes:
     $$
     p(y = k_1 \mid x) = \frac{1}{1 + e^{-a}} = \sigma(a)
     $$
3. **Key Idea:**
   - Logistic regression predicts $a = w^T \phi(x)$ and applies the **sigmoid function** to obtain probabilities.
### **Properties of the Sigmoid Function**
1. **Point Symmetry:**
   - $\sigma(-a) = 1 - \sigma(a)$, reflecting symmetry around 0.5.

2. **Inverse Function:**
   - The inverse of $\sigma(a)$ is:
     $$
     \sigma^{-1}(b) = \log \frac{b}{1 - b}, \, b \in (0, 1)
     $$
   - This computes the log-odds from the probability.
3. **Derivative:**
   - The derivative of $\sigma(a)$ is:
     $$
     \sigma'(a) = \sigma(a)(1 - \sigma(a))
     $$
   - This is useful for optimization during training.
### **Likelihood for Logistic Regression**
1. **Likelihood of the Data:**
   - For binary classification, the class label $t_i \in \{0, 1\}$:
     $$
     p(t \mid x, w) = \prod_{i=1}^N p(t_i \mid x_i, w)
     $$
   - Each $p(t_i \mid x_i, w)$ is modeled using the **Bernoulli distribution**:
     $$
     p(t_i \mid x_i, w) = y_i^{t_i} (1 - y_i)^{1 - t_i}, \, y_i = \sigma(w^T \phi(x_i))
     $$

2. **Log-Likelihood:**
   - To train the model, we maximize the (log)-likelihood:
     $$
     \arg \max_w \, \log p(t \mid x, w)
     $$
### Training Logistic Regression
1. **Objective Function:**
   - Minimize the **negative log-likelihood** (equivalent to maximizing the likelihood):
     $$
     E(w) = -\sum_{i=1}^N \left( t_i \log y_i + (1 - t_i) \log (1 - y_i) \right)
     $$
     - This is also called the **cross-entropy loss**.

2. **Gradient for Optimization:**
   - Compute the gradient of $E(w)$ with respect to $w$:
     $$
     \nabla E(w) = \sum_{i=1}^N \left( \sigma(w^T \phi(x_i)) - t_i \right) \phi(x_i)
     $$
   - This gradient is used in optimization algorithms (e.g., Gradient Descent) to find the best $w$.
### Explanation of Minimization and IRLS (Iterative Reweighted Least Squares)

### Newton-Raphson Algorithm
1. **Purpose:**
   - The Newton-Raphson method is a popular optimization technique for finding the minimum of a function. Here, it's used to minimize the **negative log-likelihood** $E(w)$ in logistic regression.
2. **Steps:**
   - Start with an initial guess for the weights $w_0$.
   - At each iteration:
     1. Compute the **gradient** of $E(w)$ at the current weights $w_i$.
     2. Compute the **Hessian** (second derivative) of $E(w)$ at $w_i$.
     3. Update $w$ by moving in the opposite direction of the gradient, scaled by the inverse of the Hessian:
        $$
        w_{i+1} = w_i - H^{-1} \nabla E(w_i)
        $$
   - Repeat until convergence.
3. **Visualization:**
   - The plot shows how Newton-Raphson refines the estimate by considering both the gradient ($\nabla E(w)$) and the curvature (Hessian) to make larger, more accurate steps towards the minimum.
### **Gradient and Hessian in Logistic Regression**
1. **Gradient of $E(w)$:**
   - The gradient $\nabla E(w)$ is:
     $$
     \nabla E(w) = \sum_{i=1}^N \left( \sigma(w^T \phi_i) - t_i \right) \phi_i
     $$
     - $\sigma(w^T \phi_i)$: Predicted probability (using the sigmoid function).
     - $t_i$: True label.
     - This represents the direction of steepest ascent in $E(w)$.
2. **Hessian (Second Derivative):**
   - The Hessian $H$ is the matrix of second derivatives:
     $$
     H = \nabla^2 E(w) = \sum_{i=1}^N \phi_i \sigma(w^T \phi_i)(1 - \sigma(w^T \phi_i)) \phi_i^T
     $$
     - The term $\sigma(w^T \phi_i)(1 - \sigma(w^T \phi_i))$ is the variance of the prediction for $x_i$, often denoted $r_i$.
   - The Hessian can be written compactly as:
     $$
     H = \Phi^T R \Phi
     $$
     - $\Phi$: Matrix of all feature vectors $\phi_i$.
     - $R$: Diagonal matrix of variances $r_i = \sigma(w^T \phi_i)(1 - \sigma(w^T \phi_i))$.
### **Iterative Reweighted Least Squares (IRLS)**

1. **Update Rule:**
   - Using the gradient and Hessian, the weights are updated as:
     $$
     w_{\text{new}} = w_{\text{old}} - ( \Phi^T R \Phi )^{-1} \Phi^T (y - t)
     $$
     - $\Phi^T (y - t)$: Residuals (difference between predicted probabilities and true labels).
     - $\Phi^T R \Phi$: Weighted least squares adjustment.

2. **Why "Iterative" and "Reweighted"?**
   - **Iterative:**
     - The update depends on $R$, which is recomputed at each step based on the current weights $w$.
   - **Reweighted:**
     - $R$ represents weights applied to each data point, reflecting the confidence in the prediction for each point.

3. **Algorithm Name:**
   - This iterative approach is known as **Iterative Reweighted Least Squares (IRLS)** because it applies weights (via $R$) to the least squares solution at every step.

4. **Key Property:**
   - IRLS converges faster than simple gradient descent because it uses the second-order information (Hessian).
## **Bayesian Logistic Regression**
1. **Bayesian Approach:**
   - Unlike standard logistic regression, Bayesian logistic regression places a **prior distribution** on the weights $w$:
     $$
     p(w) = \mathcal{N}(w; 0, \sigma_w^2 I)
     $$
     - This is a Gaussian prior with mean 0 and variance $\sigma_w^2$, reflecting initial beliefs about $w$.
2. **Likelihood:**
   - The likelihood of the observed data is the product of Bernoulli probabilities (as in standard logistic regression):
     $$
     p(t \mid x, w) = \prod_{i=1}^N y_i^{t_i} (1 - y_i)^{1 - t_i}, \quad y_i = \sigma(w^T \phi(x_i))
     $$
     - $t_i \in \{0, 1\}$: True labels.
3. **Posterior:**
   - The posterior distribution combines the prior and likelihood using Bayes’ rule:
     $$
     p(w \mid x, t) \propto p(t \mid x, w) p(w)
     $$
   - **Problem:**
     - The likelihood $p(t \mid x, w)$ is non-Gaussian, so the posterior does not have a closed form. This necessitates approximation methods like the **Laplace Approximation**.
## **Laplace Approximation (Overview)**
1. **Objective:**
   - Approximate a complex posterior $p(z)$ (or $p(w \mid x, t)$ in this case) with a simpler Gaussian distribution.

2. **Key Idea:**
   - Find a Gaussian $q(z) = \mathcal{N}(z; \mu, \sigma^2)$ that closely approximates $p(z)$:
     $$
     p(z) \approx q(z) = \mathcal{N}(z; \mu, \sigma^2)
     $$
     - The mean $\mu$ is chosen to be the **mode** of $p(z)$ (the value where $p(z)$ is maximized).

3. **Steps:**
   - Compute the **mode** of $p(z)$ using optimization methods.
   - Use the curvature of $\log p(z)$ at the mode to estimate the variance $\sigma^2$.
### Mathematical Derivation
1. **Finding the Mode:**
   - By definition, the mode of $p(z)$ is where the first derivative of the log-posterior is zero:
     $$
     \frac{d}{dz} \log f(z) \bigg|_{z=\mu} = 0
     $$
     - This is the maximum point of $f(z)$.

2. **Taylor Expansion:**
   - Around the mode $\mu$, approximate $\log f(z)$ using a second-order Taylor expansion:
     $$
     \log f(z) \approx \log f(\mu) + \frac{1}{2} \frac{d^2}{dz^2} \log f(z) \bigg|_{z=\mu} \cdot (z - \mu)^2
     $$

3. **Second Derivative:**
   - Denote the second derivative at $\mu$ as $A$:
     $$
     A = \frac{d^2}{dz^2} \log f(z) \bigg|_{z=\mu}
     $$
### Gaussian Approximation
1. **Approximation:**
   - Using the Taylor expansion:
     $$
     \log f(z) \approx \log f(\mu) + \frac{1}{2} A (z - \mu)^2
     $$
     - Exponentiating this gives:
       $$
       f(z) \propto \exp\left(\frac{1}{2} A (z - \mu)^2\right)
       $$
       - This is proportional to a Gaussian distribution.

2. **Posterior as Gaussian:**
   - The posterior $p(z)$ can now be approximated as:
     $$
     p(z) \approx \mathcal{N}(z; \mu, \sigma^2), \quad \sigma^2 = -A^{-1}
     $$
     - Variance $\sigma^2$ is the negative inverse of the second derivative (curvature).

3. **Multi-Dimensional Case:**
   - For higher dimensions, $A$ becomes the **Hessian matrix**:
     $$
     A = \nabla^2 \log f(z)
     $$
     - The covariance matrix is $\Sigma = -A^{-1}$.

### **Summary and Visualization**
![[Pasted image 20241128132435.png#invert|400]]
1. **Summary of Steps:**
   - **Step 1:** Compute the mode $\mu$ of the posterior $p(w \mid x, t)$ using optimization methods like Newton-Raphson or IRLS.
   - **Step 2:** Compute the Hessian $A$ (second derivative of the log-posterior).
   - **Step 3:** Approximate the posterior as a Gaussian:
     $$
     p(w \mid x, t) \approx \mathcal{N}(w; \mu, \Sigma), \quad \Sigma = -A^{-1}
     $$

2. **Visualization:**
   - The yellow area represents the original (non-Gaussian) posterior.
   - The red curve shows the Gaussian approximation centered at the mode, with variance derived from the second derivative.

## Gaussian Processes for Classification
1. **Using Sigmoid Functions for Classification:**
   - A GP provides a distribution over functions, not just a single function.
   - The first plot shows a function sampled from a GP.
   - To turn this into a classification problem, apply a **sigmoid function** (e.g., logistic or cumulative Gaussian) to map the function values to probabilities between 0 and 1 (as shown in the second plot).

2. **Cumulative Gaussian Sigmoid:**
   - Another commonly used sigmoid function is the **cumulative Gaussian**:
     $$
     \Phi(z) = \int_{-\infty}^z \mathcal{N}(x; 0, 1) dx
     $$
     - This maps function values into probabilities while maintaining symmetry.
### Class Prediction with a GP
1. **Goal:**
   - Predict the probability of a test point $x_*$ belonging to a class $y_* = +1$:
     $$
     p(y_* = +1 \mid X, y, x_*)
     $$

2. **Marginalization over Latent Variables:**
   - The prediction involves marginalizing (integrating out) the latent function values $f$:
     $$
     p(y_* = +1 \mid X, y, x_*) = \int p(y_* \mid f_*) p(f_* \mid X, y) df_*
     $$
   - The posterior over latent functions $p(f \mid X, y)$ is required to make this computation.

3. **Posterior Distribution:**
   - The posterior over $f$ is computed using Bayes’ rule:
     $$
     p(f \mid X, y) = \frac{p(y \mid f) p(f \mid X)}{p(y \mid X)}
     $$
     - $p(y \mid f)$: Likelihood (often a sigmoid function).
     - $p(f \mid X)$: GP prior.
     - $p(y \mid X)$: Normalizer.
### **A Simple Example**
1. **Red Points:**
   - Represent the two-class training data.
2. **Green Line:**
   - Shows the mean function of the posterior $p(f \mid X, y)$, capturing the decision boundary's uncertainty.
3. **Light Blue Line:**
   - Represents the sigmoid of the mean function, mapping the mean function into probabilities for classification.
### **The Problem with Non-Gaussian Likelihoods**

1. **Challenge:**
   - The likelihood $p(y \mid f)$ (e.g., from the sigmoid function) is **not Gaussian**.
   - This makes the posterior $p(f \mid X, y)$ analytically intractable.

2. **Solutions:**
   - Approximate the posterior using methods like:
     - **Laplace Approximation:** Approximates the posterior with a Gaussian near the mode.
     - **Expectation Propagation (EP):** Matches moments iteratively.
     - **Variational Methods:** Optimizes a bound on the marginal likelihood.
### **Predictions with GPs**
1. **Predictive Distribution:**
   - Once the posterior $p(f \mid X, y)$ is approximated, the predictive distribution $p(f_* \mid X, y, x_*)$ is computed as:
     $$
     p(f_* \mid X, y, x_*) = \mathcal{N}(f_*; \mu_*, \Sigma_*)
     $$
     - Mean $\mu_*$:
       $$
       \mu_* = k_*^T K^{-1} f
       $$
     - Variance $\Sigma_*$:
       $$
       \Sigma_* = k(x_*, x_*) - k_*^T K^{-1} k_*
       $$

2. **Class Probabilities:**
   - Finally, compute $p(y_* = +1 \mid X, y, x_*)$ by integrating over the latent variables:
     $$
     p(y_* = +1 \mid X, y, x_*) = \int p(y_* \mid f_*) p(f_* \mid X, y) df_*
     $$
     - This requires either analytical approximations or sampling.
### **Predictive Distribution**
1. **Expected Value and Variance:**
   - The predictive mean and variance of the function $f_*$ are:
     $$
     \mathbb{E}[f_* \mid X, y, x_*] = k(x_*)^T K^{-1} \hat{f}
     $$
     $$
     \text{Var}[f_* \mid X, y, x_*] = k(x_*, x_*) - k_*^T (K + W^{-1})^{-1} k_*
     $$
     - $W$: Diagonal matrix of weights from the likelihood.

2. **Final Prediction:**
   - Compute $p(y_* = +1 \mid X, y, x_*)$ using:
     - Closed-form integration for the cumulative Gaussian sigmoid.
     - Sampling or approximation for logistic sigmoid.
### **Example of Classification**
1. **Two-Class Problem:**
   - Training data (red and blue points) belong to two classes.
2. **Decision Boundaries:**
   - The **green line** is the optimal decision boundary.
   - The **black line** is the GP classifier's decision boundary, incorporating uncertainty.
3. **Posterior Probability:**
   - The right plot shows the probability map of class membership based on the GP.

## The Learning Cube
![[University Lectures/Current/Autonomous Learning for Intelligent Robot Perception/images/Untitled 5.png#invert|500]]
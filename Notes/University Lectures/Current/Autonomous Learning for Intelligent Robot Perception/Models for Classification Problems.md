## Generative Models
   - **Class-conditional probabilities**: The generative model first calculates the probability of the data $x$ given a specific class label $k$, i.e., $p(x | y = k)$. This means the model tries to learn how the data is generated based on a class.
   - **Prior probability**: It also requires the prior probability $p(y = k)$, which represents how likely each class is to occur (i.e., without looking at the input data).
   - **Posterior probability**: Using **Bayes' Rule**, the generative model then combines these two pieces of information to calculate the posterior probability $p(y = k | x)$, which is the probability of the class given the data. 
     - **Bayes' Rule**: 
       $$
       p(y = k | x) = \frac{p(x | y = k) \cdot p(y = k)}{p(x)}
       $$
     - Generative models explicitly model the joint distribution $p(x, y)$, which can be useful for tasks like generating new data.
### Example
   - We start with some data points $x_1, x_2, \ldots, x_N$ sampled from an unknown distribution $p(x)$.
   - In this case, these data points are represented along the x-axis, and the goal is to understand the underlying probability distribution.
   - We consider a small region $R$ around a specific point $x$.
   - This small region allows us to estimate the probability of $x$ by looking at the frequency of points within $R$.
![[Pasted image 20241026093430.png#invert|400]]
   - The probability that a data point falls into region $R$ can be approximated by the integral $\int_R p(x) \, dx \approx p(x) V$, where $V$ is the volume of $R$.
![[Pasted image 20241026093454.png#invert|400]]
   - To estimate the density at point $x$, we look at the ratio $\frac{K}{N}$, where $K$ is the number of points in region $R$ and $N$ is the total number of points.
   - This gives the density estimate as $p(x) \approx \frac{K}{N V}$, which can help in understanding the distribution of the data.
#### Nearest-Neighbor Classification:
   - Given labeled data points (indicated by different colors or shapes), each new data point is assigned to the class of its nearest neighbor in the feature space.
   - The process involves:
     - Mapping the new data point into the feature space.
     - Computing the distances to all existing data points.
     - Assigning the label of the nearest neighbor to the new point.
1. **Generative Model Assumptions:**
   - We assume that the data points come from different classes, each associated with a certain probability density. For a new point $\mathbf{x}$, we want to determine the probability that it belongs to class $k$, denoted $p(y = k | \mathbf{x})$.
2. **Likelihood $p(\mathbf{x} | y = k)$:**
   - To estimate the likelihood, we consider a small sphere (or neighborhood) around $\mathbf{x}$ with a fixed volume $V$.
   - The likelihood $p(\mathbf{x} | y = k)$ is then proportional to the number of points $K_k$ within the sphere that belong to class $k$ divided by $N_k \cdot V$, where $N_k$ is the total number of points in class $k$.
   - This gives:
     $$
     p(\mathbf{x} | y = k) = \frac{K_k}{N_k \cdot V}
     $$
3. **Marginal Probability $p(\mathbf{x})$:**
   - The marginal probability $p(\mathbf{x})$ represents the total density of points in the neighborhood of $\mathbf{x}$, regardless of class. This is calculated by dividing the total number of points $K$ within the sphere by $N \cdot V$, where $N$ is the total number of all points.
   - So, we have:
     $$
     p(\mathbf{x}) = \frac{K}{N \cdot V}
     $$
4. **Applying Bayes' Rule:**
   - With Bayes’ theorem, we can now calculate the posterior probability $p(y = k | \mathbf{x})$ as follows:
     $$
     p(y = k | \mathbf{x}) = \frac{p(\mathbf{x} | y = k) p(y = k)}{p(\mathbf{x})}
     $$
   - Given our estimates of $p(\mathbf{x} | y = k)$ and $p(\mathbf{x})$, this reduces to:
     $$
     p(y = k | \mathbf{x}) = \frac{K_k}{K}
     $$
   - This result simplifies the computation, as it only requires counting the points in each class within the neighborhood.

5. **Maximum A Posteriori (MAP) Classification:**
   - To classify $\mathbf{x}$, we compute the posterior $p(y = k | \mathbf{x})$ for each class $k$, then assign $\mathbf{x}$ the label of the class that maximizes this posterior probability:
     $$
     t := \arg \max_k p(y = k | \mathbf{x})
     $$
## **Discriminative Model**
   - Unlike generative models, **discriminative models** don’t concern themselves with how the data is generated. Instead, they focus directly on the **posterior probability** $p(y = k | x)$, i.e., the probability of the class $k$ given the input data $x$. 
   - These models aim to distinguish between different classes without modeling the underlying data distribution. Examples include logistic regression and support vector machines (SVM).
   - Discriminative models usually perform better than generative models in tasks focused purely on classification.
## Discriminative Function
   - This approach involves learning a direct function $f(x)$ that maps input features $x$ to output labels $y$.
   - Instead of dealing with probabilities (as in the previous two models), discriminative functions try to find a direct relationship between input and output. In this case, the goal is to classify the data points by learning a function $f(x)$ that best separates or maps inputs to the correct class labels. Neural networks are an example of discriminative function-based models.
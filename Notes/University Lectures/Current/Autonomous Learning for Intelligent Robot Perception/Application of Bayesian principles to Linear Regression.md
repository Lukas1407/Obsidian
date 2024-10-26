   - We are trying to find a function $f(x, w)$ that maps inputs $x$ (such as data points) to outputs $y$, using parameters $w$. This function represents the relationship between $x$ and $y$.
   - The outputs $y$ are observed with **Gaussian noise**, meaning the observations have some randomness added to them.
   - The model for the observations is:
     $$
     y = f(x, w) + \epsilon
     $$
     where $\epsilon \sim \mathcal{N}(0, \sigma^2)$ is Gaussian noise with mean 0 and variance $\sigma^2$.

1. **Likelihood**:
   - The likelihood function expresses how probable the observed data $y$ is, given the parameters $w$ and input $x$. 
   - Since we assume Gaussian noise, the likelihood follows a normal distribution:
     $$
     p(y | x, w, \sigma) = \mathcal{N}(y; f(x, w), \sigma^2)
     $$
![[Untitled 1.png#invert|500]]
   - The red circles in the graph represent noisy observations, while the green line represents the true function $f(x, w)$ we are trying to estimate.
## Basis Functions in Linear Regression
   - The model we are fitting is **linear** in terms of the parameters $w$, but not necessarily in terms of the inputs $x$.
   - We transform the inputs $x$ using **basis functions** $\phi(x)$. These basis functions allow us to represent more complex relationships between $x$ and $y$.
   - The linear model becomes:
     $$
     f(x, w) = w^T \phi(x)
     $$
   - Here, $\phi(x)$ is a vector of transformed inputs (basis functions), and $w^T \phi(x)$ represents the linear combination of these basis functions weighted by $w$.
## Gaussian Prior on Parameters
1. **Prior on Model Parameters**:
   - In Bayesian regression, we place a **prior distribution** on the parameters $w$. This prior expresses our beliefs about $w$ before seeing any data.
   - We assume that $w$ follows a **Gaussian distribution** with mean 0 and variance $\sigma_p^2$:
     $$
     w \sim \mathcal{N}(0, \sigma_p^2 I)
     $$
   - This prior expresses that, before observing any data, we expect the parameters to be centered around zero with some spread controlled by $\sigma_p^2$.-
## Bayesian Linear Regression
1. **Posterior Distribution**:
   - After observing data, we update our beliefs about the parameters $w$ using **Bayes' Rule**.
   - The posterior distribution of the parameters $w$, given the data $x$ and $y$, is computed as:
     $$
     p(w | x, y) = \frac{p(y | x, w) p(w)}{p(y | x)}
     $$
   - This combines:
     - The **likelihood** $p(y | x, w)$, which represents how well the model $f(x, w)$ fits the data.
     - The **prior** $p(w)$, which represents our prior belief about $w$.
     - The denominator $p(y | x)$ normalizes the posterior to ensure the total probability sums to 1.
2. **Interpretation**:
   - The **posterior** represents our updated beliefs about the parameters $w$ after observing the data. It combines our prior knowledge (through the prior distribution) with the new evidence provided by the data (through the likelihood).
## Conclusion
1. **Posterior is Gaussian**:
   - It turns out that the **posterior distribution** for $w$ in Bayesian linear regression is also a **Gaussian distribution**. This makes the calculations tractable and allows us to compute the posterior in **closed form**.
   - This is a significant advantage of Bayesian linear regression: the posterior distribution can be analytically computed.
## In General: When Bayes meets Gauss

### Prior Distribution:
   - We start with a **prior** distribution on $x$:
     $$
     p(x) = \mathcal{N}(x; \mu, \Sigma_p) = \eta_p \exp \left( -\frac{1}{2} (x - \mu)^T \Sigma_p^{-1} (x - \mu) \right)

     $$
     where $x$ has a mean $\mu$ and covariance $\Sigma_p$. 
### Likelihood Function:
   - The **likelihood** of observing $y$ given $x$ is also Gaussian:
     $$
     p(y | x) = \mathcal{N}(y; Ax + b, \Sigma_l) = \eta_l \exp \left( -\frac{1}{2} (y - Ax - b)^T \Sigma_l^{-1} (y - Ax - b) \right)
     $$
     Here, $A$ and $b$ are parameters (or known transformations) that linearly relate $x$ to $y$ with Gaussian noise, having covariance $\Sigma_l$.
### Joint Distribution:
   - The **joint distribution** $p(x, y)$ is obtained by multiplying the prior and the likelihood:
     $$
     p(x, y) = p(y | x) p(x) 
     $$
     $$
     p(x, y) = \eta_l \eta_p \exp \left( -\frac{1}{2} (x - \mu)^T \Sigma_p^{-1} (x - \mu) \right) \exp \left( -\frac{1}{2} (y - Ax - b)^T \Sigma_l^{-1} (y - Ax - b) \right)
$$
$$
p(x, y) = \eta_l \eta_p \exp \left( -\frac{1}{2} (x - \mu)^T \Sigma_p^{-1} (x - \mu) - \frac{1}{2} (y - Ax - b)^T \Sigma_l^{-1} (y - Ax - b) \right)
$$
   - where $\eta_p$ is the normalizing constant for the Gaussian prior and $\eta_l$ is the normalizing constant for the Gaussian likelihood.
   - This joint distribution is Gaussian because the product of two Gaussian functions (prior and likelihood) results in another Gaussian.
### Quadratic Form:
   - The joint distribution can be rewritten in a **quadratic form** by expanding the terms involving $x$ and $y$.
   - This form separates the joint distribution into parts:
     - **Quadratic term**: Involving $x$ and $y$ as a vector (shown in the red box).
     - **Linear term**: Involving linear combinations of $x$ and $y$ (shown in the blue box).
     - **Constant term**: Independent of $x$ and $y$ (shown in the green box).
### Matrix Form:
   - The decomposition allows us to express the joint distribution compactly as:
     $$
     p(x, y) = \mathcal{N} \left( \begin{pmatrix} x \\ y \end{pmatrix}; \mu_j, \Sigma_j \right)
     $$
     where $\mu_j$ and $\Sigma_j$ represent the combined mean and covariance of $x$ and $y$.
### Combined Mean and Covariance:
   - By representing $z = \begin{pmatrix} x \\ y \end{pmatrix}$, we can derive the **mean vector** $\mu_j$ and the **covariance matrix** $\Sigma_j$:
     $$
     \mu_j = \begin{pmatrix} \mu \\ A\mu + b \end{pmatrix}, \quad \Sigma_j = \begin{pmatrix} \Sigma_p & \Sigma_p A^T \\ A \Sigma_p & \Sigma_l + A \Sigma_p A^T \end{pmatrix}
     $$
   - This form provides a compact representation for the joint distribution $p(x, y)$.
$$
	p(x,y)=p(z)=\mathcal{N}(z;\mu_{j},\Sigma_{j})
$$
### Marginal Distribution $p(y)$
- To find $p(y)$, we integrate out $x$:
$$
p(y) = \int p(x, y) \, dx = \int p(y | x) p(x) \, dx
$$

- Since $p(x)$ and $p(y | x)$ are both Gaussians, the integral of the product of two Gaussians (over $x$) results in another Gaussian. However, we’ll need to determine the mean and covariance of this resulting Gaussian.
##### Use the Formula for the Marginal of a Linear Gaussian Model
For a Gaussian prior $x \sim \mathcal{N}(\mu, \Sigma_p)$ and a Gaussian likelihood $y | x \sim \mathcal{N}(Ax + b, \Sigma_l)$, the marginal distribution $p(y)$ is also Gaussian with:
- **Mean**: $A \mu + b$
- **Covariance**: $\Sigma_l + A \Sigma_p A^T$
So, we have:
$$
p(y) = \mathcal{N}(y; A \mu + b, \Sigma_l + A \Sigma_p A^T)
$$
#### Intuition:
- The mean $A \mu + b$ reflects how the linear transformation $A$ affects the mean $\mu$ of $x$, plus the offset $b$.
- The covariance $\Sigma_l + A \Sigma_p A^T$ includes both:
  - The noise $\Sigma_l$ in the observation $y$,
  - The transformed uncertainty $A \Sigma_p A^T$ from the prior $x$.
### Posterior Distribution $p(x | y)$
- The posterior distribution $p(x | y)$ describes our updated belief about $x$ after observing $y$.
$$
p(x | y) = \frac{p(y | x) p(x)}{p(y)}
$$

- Since $p(y)$ is just a normalizing constant, we can focus on the product $p(y | x) p(x)$, which is part of the joint distribution $p(x, y)$.
- Given $p(y | x) = \mathcal{N}(y; Ax + b, \Sigma_l)$ and $p(x) = \mathcal{N}(x; \mu, \Sigma_p)$, the posterior $p(x | y)$ is Gaussian with:
	- **Mean**: $\Sigma \left( A^T \Sigma_l^{-1} (y - b) + \Sigma_p^{-1} \mu \right)$
	- **Covariance**: $\Sigma = \left( \Sigma_p^{-1} + A^T \Sigma_l^{-1} A \right)^{-1}$
- So we get:
$$
p(x | y) = \mathcal{N} \left( x; \Sigma \left( A^T \Sigma_l^{-1} (y - b) + \Sigma_p^{-1} \mu \right), \Sigma \right)
$$
### Summary and Insights
1. **Gaussian Prior + Gaussian Likelihood**:
   - The combination of a **Gaussian prior** and a **Gaussian likelihood** results in a **Gaussian posterior**. This is due to the conjugate prior property, where the Gaussian distribution serves as the conjugate prior for the mean of another Gaussian distribution.
2. **Linear Gaussian Model**:
   - This framework is known as a **Linear Gaussian Model**:
     - The posterior mean and covariance can be computed in closed form.
     - The Gaussian structure is preserved through Bayesian updating, which is computationally efficient and tractable.
## Bayesian Linear Regression
In Bayesian linear regression, given data points $X$ and their corresponding outputs $y$, the goal is to make a **prediction** for a new data point $x^*$ to estimate its target $y^*$. 
![[Pasted image 20241026085105.png#invert|500]]
#### Predictive Distribution:
To predict $y^*$ for $x^*$, we compute the **predictive distribution** $p(y^* | x^*, X, y)$. This distribution incorporates information from:
- The prior knowledge of $x$.
- The observed data $(X, y)$.
- The new data point $x^*$.
The predictive distribution helps in Bayesian regression by not only giving an estimate of $y^*$ but also providing uncertainty in that prediction.
##### Predictive Distribution Equation
   - The **predictive distribution** is calculated by integrating over all possible values of the model parameters $w$, weighted by the posterior distribution over $w$ given previous data:
     $$
     p(y^* | x^*, X, y) = \int p(y^* | x^*, w) p(w | X, y) \, dw
     $$
   - Here:
     - $p(y^* | x^*, w)$ is the **new data likelihood**, which models how likely the new target $y^*$ is for a given model parameter $w$.
     - $p(w | X, y)$ is the **posterior distribution** over the parameters $w$, given the old data $X$ and $y$.
   - **Key Insight**: The predictive distribution averages over all likely values of $w$, reflecting our uncertainty about the exact value of $w$.
##### Old Data Posterior (Gaussian)
   - From prior training data $(X, y)$, we obtain a posterior over $w$, typically a Gaussian:
     $$
     p(w | X, y) = \mathcal{N}(w | \mu_N, \Sigma_N)
     $$
   - This is the result of the Bayesian update after seeing previous data.
##### New Data Likelihood
   - The new data likelihood, assuming a Gaussian noise model, is:
     $$
     p(y^* | x^*, w, \sigma) = \mathcal{N}(y^* | \phi(x^*)^T w, \sigma^2)
     $$
   - This represents the likelihood of observing $y^*$ given a specific $x^*$ and a model parameter $w$.
##### Predictive Mean and Variance
   - By applying our previous Bayesian linear regression results, we derive the predictive distribution as Gaussian:
     $$
     p(y^* | X, y, x^*) = \mathcal{N}(y^* | \hat{\mu}, \hat{\sigma}^2)
     $$
   - With:
     - **Predictive mean** $\hat{\mu} = \phi(x^*)^T \mu_N$
     - **Predictive variance** $\hat{\sigma}^2 = \sigma^2 + \phi(x^*)^T \Sigma_N \phi(x^*)$
   - This accounts for both the uncertainty from the noise and the model parameters.
##### Example with Visuals (Sinusoidal Data)
![[Pasted image 20241026093002.png#invert|400]]
1. **1 Data Point**
   - **Predictive Distribution**: The red shaded area represents high uncertainty in the model’s predictions, as we have seen only one data point.
   - **Samples from Posterior**: The red curves are samples of possible functions given the single data point, showing considerable variation.
![[Pasted image 20241026093014.png#invert|400]]
2. **2 Data Points**
   - **Predictive Distribution**: With two data points, the model becomes more confident in certain regions, as indicated by the narrowing of the red shaded area.
   - **Samples from Posterior**: The sample functions begin to fit both data points, though there's still significant variability elsewhere.
![[Pasted image 20241026093032.png#invert|400]]
3. **4 Data Points**
   - **Predictive Distribution**: The shaded area further narrows, indicating that the model’s predictions are becoming more certain where data has been observed.
   - **Samples from Posterior**: The functions now closely fit all observed points, with reduced variability between them.
![[Pasted image 20241026093047.png#invert]]
4. **25 Data Points**
### Offline vs. Online Processing
#### Offline Processing:
   - All data $X$ and corresponding targets $y$ are available beforehand.
   - We fit a single model using the entire dataset and make predictions in a batch or "offline" fashion.
   - [[Maximum A Posteriori (MAP) Regression]]
#### Online (Sequential) Processing:
   - Data arrives sequentially, and we update our beliefs after each new observation.
   - In this case, each posterior update becomes the prior for the next observation, making it a **sequential updating process**.
   - [[Bayesian Linear Regression with Sequential Data]]

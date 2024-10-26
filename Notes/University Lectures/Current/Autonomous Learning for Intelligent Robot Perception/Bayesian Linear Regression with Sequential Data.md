This process involves updating our estimate of the regression parameters $w$ as new data points $(x_i, y_i)$ arrive. This sequential (or online) learning approach is particularly useful when we want to process data in real-time or when computational resources are limited, and we can't store all data points at once.
### Initialization and Sequential Update Steps
1. **Initialization**:
   - We start with a **prior** on $w$:
     $$
     p_0(w) = \mathcal{N}(w; \mu_0, \Sigma_0)
     $$
   - $\mu_0$ is the prior mean, and $\Sigma_0$ is the prior covariance matrix, representing our initial beliefs about $w$.

2. **Observing a Data Point**:
   - For each incoming data point $(x_i, y_i)$, we use it to update our estimate of $w$.
   
3. **Formulate the Likelihood**:
   - Given the data point $(x_i, y_i)$, we assume a Gaussian **likelihood** for $y_i$ as a function of $w$:
     $$
     p(y_i | x_i, w) = \mathcal{N}(y_i; w^T \phi(x_i), \sigma^2)
     $$
   - Here:
     - $\phi(x_i)$ represents the basis function applied to $x_i$.
     - $\sigma^2$ is the noise variance.

4. **Update the Prior with the Likelihood**:
   - We multiply the prior $p(w)$ with the likelihood $p(y_i | x_i, w)$ and **normalize** to obtain a new Gaussian distribution for $w$.
   - This gives us the **posterior distribution** for $w$ after observing $(x_i, y_i)$:
     $$
     p(w | x_0, \dots, x_i, y_0, \dots, y_i) = \mathcal{N}(w; \mu_{i+1}, \Sigma_{i+1})
     $$
   - The mean $\mu_{i+1}$ and covariance $\Sigma_{i+1}$ are updated based on the new information.

5. **Repeat the Process**:
   - Set $i = i + 1$ and continue updating as new data points arrive.
### Applying the Update Equations
This slide applies the general rules derived earlier (from the conjugate Gaussian updates) to update the mean $\mu_{i+1}$ and covariance $\Sigma_{i+1}$ of $w$ after each observation.

1. **Updating the Mean**:
   - The mean $\mu_{i+1}$ after observing $(x_{i+1}, y_{i+1})$ is:
     $$
     \mu_{i+1} = \Sigma_{i+1} \left( \phi(x_{i+1}) \sigma^2 y_{i+1} + \Sigma_i^{-1} \mu_i \right)
     $$
   - This equation combines:
     - The contribution from the previous mean $\mu_i$,
     - The new data point $(x_{i+1}, y_{i+1})$ scaled by the basis function $\phi(x_{i+1})$ and noise variance $\sigma^2$.

2. **Updating the Covariance**:
   - The updated covariance $\Sigma_{i+1}$ is:
     $$
     \Sigma_{i+1} = \left( \Sigma_i^{-1} + \sigma^2 \phi(x_{i+1}) \phi(x_{i+1})^T \right)^{-1}
     $$
   - This incorporates information from both the previous covariance $\Sigma_i$ and the new data point $x_{i+1}$.
### Summarizing the Sequential Update Process
1. **Initialization**: Set initial prior $p_0(w)$.
2. **Observe Data**: For each data point $(x_i, y_i)$.
3. **Formulate Likelihood**: Define the likelihood $p(y_i | x_i, w)$ for the data point as a Gaussian.
4. **Update Prior to Posterior**: Compute the posterior distribution for $w$ by multiplying the likelihood and prior.
5. **Set New Prior**: The posterior becomes the prior for the next iteration.
6. **Repeat**: Go to Step 2 for the next data point.
### Example with Numbers
#### Assumptions:
   - **Basis Function**: Identity function $\phi(x) = x$, meaning no transformation is applied to $x$.
   - **Prior Mean and Covariance**: $\mu_0 = 0$, $\Sigma_0 = 0.5$.
   - **Noise Variance**: $\sigma^2 = 0.2^2$.
   - **Ground Truth Model**: $f(x, a) = a_0 + a_1 x$, with $a_1 = 0.5$ and $a_0 = -0.3$.
#### Goal
   - Using sequentially incoming data points $(x_1, t_1), (x_2, t_2), \dots$, we aim to recover the parameters $a_0$ and $a_1$ of the ground truth model.
#### Update equations
The update formulas for each step $i$ are as follows:
1. **Posterior Covariance**: 
   $$
   \Sigma_{i+1} = \left( \frac{1}{\Sigma_i} + \frac{\phi(x_{i+1})^2}{\sigma^2} \right)^{-1}
   $$
2. **Posterior Mean**: 
   $$
   \mu_{i+1} = \Sigma_{i+1} \left( \frac{\mu_i}{\Sigma_i} + \frac{\phi(x_{i+1}) t_{i+1}}{\sigma^2} \right)
   $$

Where $\phi(x) = x$ because we are using the identity basis function.
#### Step 1: Observing $(x_1 = 1, t_1)$
1. **Calculate Posterior Covariance** $\Sigma_1$:
   $$
   \Sigma_1 = \left( \frac{1}{\Sigma_0} + \frac{x_1^2}{\sigma^2} \right)^{-1} = \left( \frac{1}{0.5} + \frac{1^2}{0.04} \right)^{-1}
   $$
   $$
   = \left( 2 + 25 \right)^{-1} = \frac{1}{27} \approx 0.0370
   $$

2. **Calculate Posterior Mean** $\mu_1$:
   $$
   \mu_1 = \Sigma_1 \left( \frac{\mu_0}{\Sigma_0} + \frac{x_1 \cdot t_1}{\sigma^2} \right) = 0.0370 \left( \frac{0}{0.5} + \frac{1 \cdot 0.512}{0.04} \right)
   $$
   $$
   = 0.0370 \cdot 12.8 \approx 0.5119
   $$
#### Step 2: Observing $(x_2 = 2, t_2)$
1. **Calculate Posterior Covariance** $\Sigma_2$:
   $$
   \Sigma_2 = \left( \frac{1}{\Sigma_1} + \frac{x_2^2}{\sigma^2} \right)^{-1} = \left( \frac{1}{0.0370} + \frac{2^2}{0.04} \right)^{-1}
   $$
   $$
   = \left( 27.03 + 100 \right)^{-1} = \frac{1}{127.03} \approx 0.0079
   $$

2. **Calculate Posterior Mean** $\mu_2$:
   $$
   \mu_2 = \Sigma_2 \left( \frac{\mu_1}{\Sigma_1} + \frac{x_2 \cdot t_2}{\sigma^2} \right) = 0.0079 \left( \frac{0.5119}{0.0370} + \frac{2 \cdot 0.4159}{0.04} \right)
   $$
   $$
   = 0.0079 \cdot (13.84 + 20.795) \approx 0.4159
   $$

#### Step 3: Observing $(x_3 = 3, t_3)$
1. **Calculate Posterior Covariance** $\Sigma_3$:
   $$
   \Sigma_3 = \left( \frac{1}{\Sigma_2} + \frac{x_3^2}{\sigma^2} \right)^{-1} = \left( \frac{1}{0.0079} + \frac{3^2}{0.04} \right)^{-1}
   $$
   $$
   = \left( 126.58 + 225 \right)^{-1} = \frac{1}{351.58} \approx 0.0028
   $$

2. **Calculate Posterior Mean** $\mu_3$:
   $$
   \mu_3 = \Sigma_3 \left( \frac{\mu_2}{\Sigma_2} + \frac{x_3 \cdot t_3}{\sigma^2} \right) = 0.0028 \left( \frac{0.4159}{0.0079} + \frac{3 \cdot 0.4475}{0.04} \right)
   $$
   $$
   = 0.0028 \cdot (52.65 + 33.56) \approx 0.4475
   $$

#### Step 4: Observing $(x_4 = 4, t_4)$
1. **Calculate Posterior Covariance** $\Sigma_4$:
   $$
   \Sigma_4 = \left( \frac{1}{\Sigma_3} + \frac{x_4^2}{\sigma^2} \right)^{-1} = \left( \frac{1}{0.0028} + \frac{4^2}{0.04} \right)^{-1}
   $$
   $$
   = \left( 351.58 + 400 \right)^{-1} = \frac{1}{751.58} \approx 0.0013
   $$

2. **Calculate Posterior Mean** $\mu_4$:
   $$
   \mu_4 = \Sigma_4 \left( \frac{\mu_3}{\Sigma_3} + \frac{x_4 \cdot t_4}{\sigma^2} \right) = 0.0013 \left( \frac{0.4475}{0.0028} + \frac{4 \cdot 0.4951}{0.04} \right)
   $$
   $$
   = 0.0013 \cdot (159.11 + 49.51) \approx 0.4951
   $$

#### Step 5: Observing $(x_5 = 5, t_5)$
1. **Calculate Posterior Covariance** $\Sigma_5$:
   $$
   \Sigma_5 = \left( \frac{1}{\Sigma_4} + \frac{x_5^2}{\sigma^2} \right)^{-1} = \left( \frac{1}{0.0013} + \frac{5^2}{0.04} \right)^{-1}
   $$
   $$
   = \left( 751.58 + 625 \right)^{-1} = \frac{1}{1376.58} \approx 0.0007
   $$

2. **Calculate Posterior Mean** $\mu_5$:
   $$
   \mu_5 = \Sigma_5 \left( \frac{\mu_4}{\Sigma_4} + \frac{x_5 \cdot t_5}{\sigma^2} \right) = 0.0007 \left( \frac{0.4951}{0.0013} + \frac{5 \cdot 0.5040}{0.04} \right)
   $$
   $$
   = 0.0007 \cdot (380.85 + 63) \approx 0.5040
   $$
#### Final Posterior Means and Covariances
After processing all data points sequentially, we have:
- **Posterior Means**: [0.5119, 0.4159, 0.4475, 0.4951, 0.5040]
- **Posterior Covariances**: [0.0370, 0.0079, 0.0028, 0.0013, 0.0007]
Each step shows how our estimate of the slope converges towards the true slope $a_1 = 0.5$ and how the uncertainty (posterior covariance) decreases as we observe more data points.

## Example: Bayesian Line Fitting
- Shows how our belief about the line's parameters (slope $w_1$​ and intercept $w_0$​) changes as more data points are observed.
- The "Hough Space" is the parameter space
![[Pasted image 20241026092502.png#invert|400]]
- **Prior in Hough Space**: The colored circle in the left plot represents our initial belief about $w_1$ (slope) and $w_0$ (intercept). We assume a Gaussian prior centered around zero for both parameters, showing no prior knowledge or preference for specific values of slope or intercept.
- **Data Space**: On the right, we see several random lines in red. These lines are samples from the prior distribution over $w_1$ and $w_0$. Since we haven’t seen any data yet, these lines represent a wide range of possibilities.
![[Pasted image 20241026092546.png#invert|400]]
- **Likelihood**: In Hough Space (parameter space), the likelihood function is introduced. Given the observed data point, the likelihood constrains the possible values of $w_1$ and $w_0$ that would make the line pass close to this point. The likelihood takes the shape of a line in this parameter space, meaning any combination of $w_1$ and $w_0$ that would yield a line near the observed point is more likely.
- **Updated Prior (Posterior)**: The prior is updated based on the likelihood, creating a more focused distribution that reflects the new knowledge from the data point. 
- **Data Space**: The set of red lines has been updated to reflect the posterior distribution. Since only one data point has been observed, there is still considerable variability in possible lines.

- The likelihood and updated prior get further refined with more observed datapoints

![[Pasted image 20241026092702.png#invert|400]]
- **Likelihood**: With twenty data points, the likelihood in Hough Space is extremely focused, allowing only a narrow range of $w_1$ and $w_0$ values that would fit all points well.
- **Updated Prior (Posterior)**: The prior distribution has become very narrow, as we are now confident in the estimates for $w_1$ and $w_0$.
- **Data Space**: The lines sampled from the prior are now closely aligned with the ground truth line (red line in the middle), fitting all data points with very little deviation.


Also called **Gradient-Free Optimization**, **Black-Box Optimization**, **Evolutionary Search**, or **Stochastic Search**
**"Black-Box" means** we make <mark style="background: #FFB86CA6;">no assumptions about the internal structure of the objective function</mark>.
- Unlike Q-Learning or Policy Gradient, we <mark style="background: #FFB86CA6;">don't track specific states, actions, or rewards</mark>.
- We only focus on <mark style="background: #FFB86CA6;">tuning the parameters of the agent directly</mark>.
- They optimize the agent’s parameters by observing outcomes of <mark style="background: #FFB86CA6;">entire episodes</mark> (not step-by-step updates like Q-learning).
## Why Use Black-Box Algorithms
### **Easy to Use**
- No need to model states, actions, or policies — just adjust parameters and see what happens.
- <mark style="background: #FFB86CA6;">Works with non-Markov environments</mark> (not every RL problem is an MDP).
### **Unbiased Optimization**
- This method <mark style="background: #FFB86CA6;">searches globally for good solutions rather than following gradients</mark>, which can get stuck in local minima.
### **Drawback: Sample Inefficiency**
- It needs **lots of episodes** to discover the best parameters.
- -> Need to run a full episode for every change in parameters

## Steps for Evolutionary Strategies (ES)
1. **Explore**:  
   - Sample parameters $\theta_i$ from the search distribution $p_k(\theta)$.
   - Exploration happens directly in **parameter space**, avoiding noise from stochastic actions.  
   - Returns are **less variable**, and exploration can be highly correlated over time, which can be beneficial.
2. **Evaluate**:  
   - Assess the quality of sampled parameters by executing a trajectory $\tau_i$ for each sample $\theta_i$.
   - Compute the performance $g(\theta_i)$ based on the return $R(\tau)$.
3. **Update**:  
   - Use the evaluated performance to update the search distribution $p_k$, improving the likelihood of sampling better-performing parameters in the next iteration.
## First and Second Order Methods
### **Gaussian Search Distributions**
- The complexity of the search distribution defines whether the method is **first-order** or **second-order**:
  1. **First Order**:
     - Uses a **diagonal Gaussian search distribution**:
       $$
       p_\omega(\theta) = \mathcal{N}(\mu, \text{diag}(\sigma)) \quad \text{or} \quad \mathcal{N}(\mu, \sigma^2 I)
       $$
       - Parameters $\omega = \{\mu, \sigma\}$.
     - **Advantages**:
       - Scales well to high-dimensional parameters (e.g., deep neural networks).
     - **Disadvantages**:
       - Convergence can be slow due to the simplistic nature of the search.
  2. **Second Order**:
     - Uses a **full covariance Gaussian distribution**:
       $$
       p_\omega(\theta) = \mathcal{N}(\mu, \Sigma)
       $$
       - Parameters $\omega = \{\mu, \Sigma\}$, where $\Sigma$ captures correlations between parameters.
     - **Advantages**:
       - More directed exploration in parameter space due to correlated updates.
       - Typically faster convergence.
     - **Disadvantages**:
       - Does not scale well for problems with more than 100-200 parameters due to the complexity of maintaining the full covariance matrix.
### **First-Order Stochastic Search**
#### **Algorithm Overview**
- Samples parameters $\theta$ from an **isotropic Gaussian** distribution:
  $$
  p_\omega(\theta) = \mathcal{N}(\mu, \sigma^2 I)
  $$
- **Likelihood Gradient**:
  - Computes the gradient of the objective $J(\omega)$ with respect to the distribution parameters:
    $$
    \nabla_\omega J(\omega) \approx \frac{1}{N} \sum_{i=1}^N g(\theta_i) \nabla_\omega \log p_\omega(\theta_i)
    $$
- Updates the mean $\mu$ (as an example):
  $$
  \nabla_\mu J(\omega) \approx \frac{1}{N} \sum_{i=1}^N g(\theta_i) (\theta_i - \mu)
  $$
- **Key Variation**:
  - Diagonal covariance adaptation allows for **Parameter-Based Policy Gradients** (PBPG).
### **Finite Differences as Baseline**
#### **Concept**:
Finite differences provide a baseline for understanding stochastic search. They approximate gradients by perturbing each parameter dimension.
1. **Deterministic Objective**:
   - Optimize:
     $$
     \theta^* = \arg\max g(\theta)
     $$
2. **Finite-Difference Gradient**:
   - Approximates the derivative:
     $$
     \frac{\partial g}{\partial \theta_i} \approx \frac{g(\theta_k + \epsilon e_i) - g(\theta_k - \epsilon e_i)}{2\epsilon}
     $$
     where $e_i$ is the unit vector for dimension $i$, and $\epsilon$ is a small perturbation.
3. **Drawbacks**:
   - Highly sample inefficient:
     - Requires $2 \times D$ evaluations per gradient step for $D$ dimensions.
### **Evolutionary Search (ES) vs. Finite Difference (FD)**
#### **Key Comparisons**:
1. **Finite Difference (FD)**:
   - Estimates the gradient of $g(\theta)$, not $J(\omega)$.
   - Sensitive to local variations, making it prone to getting stuck in suboptimal solutions.
2. **Evolutionary Search (ES)**:
   - Considers the **expected value** of the performance across the distribution.
   - With a high variance ($\sigma$), ES avoids getting caught by local variations.
   - Focuses on regions of the parameter space with **lower sensitivity**.
#### **Visual Explanation**:
- High variance in ES leads to smoother exploration across the parameter space, avoiding sharp local gradients that FD struggles with.
- ES can identify more robust solutions as it averages across a distribution, rather than relying on single-point perturbations.
Here’s a detailed explanation of the **Canonical Evolutionary Strategy (ES)**, broken down step by step:
## A Canonical Evolutionary Strategy
### **Key Idea: Addressing Gradient Sensitivity**
- **Problem with Standard Likelihood Gradient:**
  - The standard likelihood gradient can be highly sensitive to the **scaling of the reward**:
    - Different scales of rewards require different learning rates.
    - Large negative outliers (e.g., unsafe or poor parameter vectors) can distort the gradient.
- **Solution: Use Ranking Instead of Raw Performance:**
  - Instead of directly using the performance $g(\theta)$ of sampled parameters, we use a **ranking** of all samples to compute weights.
  - The weight $w_i$ for each sample is based on its rank $i$, ensuring the influence of outliers is minimized.
### **How Ranking Weights Are Computed**
- **Ranking Formula for Weights:**
  $$
  w_i = \frac{\log(M + 0.5) - \log(i)}{\sum_{j=1}^M \left( \log(M + 0.5) - \log(j) \right)}
  $$
  - $M$: Number of top-ranked samples (elites) to use.
  - $i$: Rank of the sample (best rank is $i=1$).
  - Higher ranks (better-performing samples) get higher weights.
  - All weights sum to 1, and no additional learning rate is needed.
### **Canonical Evolutionary Strategy Algorithm**
![[Pasted image 20241208092318.png#invert|400]]
#### **Steps in the Algorithm:**
1. **Initialization:**
   - Start with a search distribution $p(\theta) = \mathcal{N}(\mu, \sigma^2 I)$, where:
     - $\mu$: Mean of the parameter distribution.
     - $\sigma$: Fixed standard deviation.
2. **Sample Parameters:**
   - Draw $N$ parameter vectors $\theta_i$ from $\mathcal{N}(\mu, \sigma^2 I)$.
3. **Evaluate Performance:**
   - Compute the performance $g(\theta_i)$ for each sampled parameter vector $\theta_i$.
4. **Rank and Select:**
   - Sort the parameter vectors $\theta_i$ by their performance $g(\theta_i)$, with the best-performing samples ranked highest.
   - Select the top $M$ "elite" samples for the update.
5. **Update Mean:**
   - Compute the new mean $\mu_{k+1}$ of the distribution as a **weighted recombination** of the top $M$ samples:
     $$
     \mu_{k+1} = \mu_k + \sum_{i=1}^M w_i (\theta_i - \mu_k)
     $$
     or equivalently:
     $$
     \mu_{k+1} = \sum_{i=1}^M w_i \theta_i
     $$
     Here:
     - The weights $w_i$ are derived from the rank of each sample.
     - $\sigma$ is typically kept constant.
### **Advantages of Ranking-Based ES**
1. **Robustness to Scaling:**
   - Ranking removes sensitivity to reward scaling, as only the relative ordering matters.
   - Negative outliers (poor-performing samples) do not disproportionately affect the update.

2. **No Learning Rate Tuning:**
   - Since the weights sum to 1, no additional learning rate parameter is required, simplifying implementation.

3. **Weighted Recombination:**
   - The use of weighted recombination based on ranking ensures that better-performing samples have a larger influence on the updated mean.
### **Challenges of Ranking-Based ES**
1. **Loss of Direct Optimization:**
   - Using ranking instead of $g(\theta)$ sacrifices direct optimization of the objective $g(\theta)$.
   - However, ranking has been shown to work better in practice for many problems.
2. **Sample Inefficiency:**
   - As with all evolutionary strategies, this approach can require a large number of samples to converge.
## Second Order Stochastic Search Algorithms
### **Cross-Entropy Method (CEM)**
![[Pasted image 20241208092646.png#invert|400]]
#### **Key Idea:**
The **Cross-Entropy Method (CEM)** is a popular **evolutionary strategy (ES)** that refines the search distribution iteratively by focusing on the best-performing samples.
#### **Steps of CEM**:
1. **Sample Parameter Vectors**:
   - Draw $N$ samples $\theta_i$ from the Gaussian distribution $\mathcal{N}(\mu, \Sigma)$.
   - $\mu$: Mean of the distribution.  
   - $\Sigma$: Covariance matrix.
2. **Evaluate Performance**:
   - For each $\theta_i$, compute its performance $g(\theta_i)$.
3. **Rank Samples**:
   - Sort the parameter vectors $\theta_i$ by their performance $g(\theta_i)$ (best-performing samples ranked highest).
4. **Update Mean**:
   - Compute the new mean $\mu_{k+1}$ using only the top $M$ "elite" samples:
     $$
     \mu_{k+1} = (1 - \alpha)\mu_k + \alpha \mu_{\text{elites}}
     $$
     Here, $\alpha$ is the learning rate, and $\mu_{\text{elites}}$ is the mean of the top $M$ samples.
5. **Update Covariance**:
   - Update the covariance matrix $\Sigma$ using the elite samples:
     $$
     \Sigma_{k+1} = (1 - \alpha)\Sigma_k + \alpha \Sigma_{\text{elites}}
     $$
#### **Advantages of CEM**:
- Efficient for both first-order and second-order optimization.
- Updates both the mean and the full covariance matrix, enabling directed exploration.
- Simple to implement and very flexible.
## **Trust-Region Methods for Stochastic Search**
### **How It Works**:
- Adjust the search distribution $p(\theta)$ while keeping it "close" to the current distribution $p_{\text{old}}(\theta)$.
- "Closeness" is measured using the **Kullback-Leibler (KL) Divergence**:
  $$
  \text{KL}(p_{\text{old}} \| p) = \int p_{\text{old}}(\theta) \log \frac{p_{\text{old}}(\theta)}{p(\theta)} d\theta
  $$
  - Ensures that the updated distribution does not deviate too far from the previous one.
### **Steps**:
1. Maximize the expected fitness:
   $$
   \arg\max_\omega \int p_\omega(\theta) g(\theta) d\theta
   $$
2. Constrain the KL divergence:
   $$
   \text{KL}(p_{\text{old}} \| p_\omega) \leq \epsilon
   $$
   Here, $\epsilon$ is a trust-region threshold controlling the allowed change in the distribution.
### **3. Gaussian Distributions for Trust-Regions**
#### **Compatible Surrogate**:
- Assume the reward function $g(\theta)$ can be approximated by a quadratic surrogate model:
  $$
  g(\theta) \approx \theta^T A \theta + a^T \theta + a_0
  $$
  - $A$: Quadratic term (Hessian-like).  
  - $a$: Linear term.  
  - $a_0$: Constant term.
- Use this surrogate model to update the parameters of the Gaussian search distribution $\mathcal{N}(\mu, \Sigma)$.
#### **Key Updates**:
1. Update the precision matrix ($\Sigma^{-1}$) using:
   $$
   \Sigma_{k+1}^{-1} = \Sigma_k^{-1} + \frac{A}{\eta}
   $$
   where $\eta$ controls the weight of the quadratic reward model.

2. Update the mean:
   $$
   \mu_{k+1} = \Sigma_{k+1} \left( \Sigma_k^{-1} \mu_k + \frac{a}{\eta} \right)
   $$

## **Solution for the Search Distribution**
### **Objective:**
Find the optimal search distribution $p(\theta)$ while balancing two objectives:
1. Minimize KL divergence:
   $$
   \text{KL}(p_{\text{old}} \| p) \leq \epsilon
   $$
2. Maximize entropy:
   $$
   H(p) - H(p_{\text{old}}) \leq \gamma
   $$
   Here, $H(p)$ is the entropy of the distribution, ensuring sufficient exploration.
### **Key Updates**:
1. **Precision Matrix**:
   $$
   \Sigma_{k+1}^{-1} = \frac{\eta}{\eta + \kappa} \Sigma_k^{-1} + \frac{1}{\eta + \kappa} A
   $$
2. **Mean**:
   $$
   \mu_{k+1} = \Sigma_{k+1} \left( \frac{\eta}{\eta + \kappa} \Sigma_k^{-1} \mu_k + \frac{1}{\eta + \kappa} a \right)
   $$
   - $\eta$ controls trust-region size.
   - $\kappa$ controls the balance between exploration and exploitation.

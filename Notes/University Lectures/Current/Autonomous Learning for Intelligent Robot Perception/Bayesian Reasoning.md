- Suppose a robot stops in front of a door. It has a sensor (e.g. a camera) to measure the state of the door (open or closed). 
- **Problem**: the sensor may fail
![[Pasted image 20241025123002.png#invert|400]]
- **Question**: How can we obtain knowledge about the environment from sensors that may return incorrect results?
- **Answer**: using probabilities!
## Mathematical Formulation of the Example
- We define 2 random variables $z$ and $open$ where $z$ is "light on" or "light off"
- **Question:** What is $p(open|z)$
### Diagnostic Reasoning
   - **Diagnostic reasoning** refers to finding $p(\text{open} | z)$, meaning we want to determine the probability that the door is open, given some evidence $z$ (like a sensor reading).
   - Diagnostic reasoning moves from **effects** to **causes**. We observe the effect (sensor reading) and infer the cause (whether the door is open).
### Causal Reasoning
   - **Causal reasoning** refers to finding $p(z | \text{open})$, meaning we want to find the probability of observing the evidence $z$ (sensor reading), given that we know the door is open.
   - Causal reasoning moves from **causes** to **effects**. We assume the door is open and ask what the likelihood is of observing a specific sensor reading.
### Bayes Rule Application
   - Bayes' Rule helps connect diagnostic and causal reasoning. Even if we have **causal knowledge** (i.e., $p(z | \text{open})$), Bayes' Rule allows us to infer the diagnostic probability $p(\text{open} | z)$.
   $$
   p(\text{open} | z) = \frac{p(z | \text{open}) p(\text{open})}{p(z)}
   $$
   Where:
   - $p(z | \text{open})$: The likelihood of observing $z$ given the door is open.
   - $p(\text{open})$: The prior probability that the door is open.
   - $p(z)$: The total probability of observing $z$, calculated using the **law of total probability**:
     $$
     p(z) = p(z | \text{open}) p(\text{open}) + p(z | \neg \text{open}) p(\neg \text{open})
     $$
### Example with Numbers**
1. **Given Sensor Model**:
   - The sensor model provides us with probabilities for detecting whether the door is open or not:
     - $p(z | \text{open}) = 0.6$: The probability of the sensor reading $z$ given that the door is open.
     - $p(z | \neg \text{open}) = 0.3$: The probability of the sensor reading $z$ given that the door is **not** open.
2. **Prior Probability**:
   - We are given the **prior probability**:
     - $p(\text{open}) = 0.5$: The probability that the door is open (before observing the sensor reading).
     - $p(\neg \text{open}) = 0.5$: The probability that the door is not open.
3. **Applying Bayes’ Rule**:
   Using the provided data, we apply Bayes' Rule to compute $p(\text{open} | z)$, the probability that the door is open, given the sensor reading $z$.
   We first calculate $p(z)$ using the **law of total probability**:
   $$
   p(z) = p(z | \text{open}) p(\text{open}) + p(z | \neg \text{open}) p(\neg \text{open})
   $$
   Substituting the given values:
   $$
   p(z) = 0.6 \cdot 0.5 + 0.3 \cdot 0.5 = 0.3 + 0.15 = 0.45
   $$

   Now, apply Bayes' Rule:
   $$
   p(\text{open} | z) = \frac{p(z | \text{open}) p(\text{open})}{p(z)} = \frac{0.6 \cdot 0.5}{0.45} = \frac{0.3}{0.45} = \frac{2}{3} \approx 0.67
   $$
4. **Conclusion**:
   - After applying Bayes’ Rule, we find that the probability that the door is open, given the sensor reading $z$, is $0.67$ or $67\%$.
   - The evidence $z$ **raises the probability** that the door is open from the prior $0.5$ (50%) to $0.67$ (67%).

## Combining Evidence
1. **New Observation**:
   - Suppose our robot has obtained another observation, $z_2$, at a later time, in addition to the first observation $z_1$.
   - The question is how to integrate this new information to update our belief about whether the door is open or not.
2. [[Bayes Rule#Bayes Rule with Background Knowledge|Bayes' Rule with Background Knowledge]]:
   - To estimate $p(\text{open} | z_1, z_2)$, we apply Bayes' Rule:
     $$
     p(\text{open} | z_1, z_2) = \frac{p(z_2 | \text{open}, z_1) p(\text{open} | z_1)}{p(z_2 | z_1)}
     $$
   - Here, the tricky term is $p(z_2 | \text{open}, z_1)$, which requires knowing how the new observation $z_2$ depends on both the state of the door being open and the first observation $z_1$.
### Markov Assumption
   - The **Markov Assumption** simplifies the problem. It states that **if we know the state of the door (whether it's open or not), the new observation $z_2$ is conditionally independent of the previous observation $z_1$**.
   - This means that $z_2$ depends only on the state of the door and not on the previous observation $z_1$.
   - Formally, this is written as:
     $$
     p(z_2 | \text{open}, z_1) = p(z_2 | \text{open})
     $$
   - This assumption allows us to focus on the current observation without needing to account for all past observations when updating our belief about the state.
### Example with Numbers
1. **Second Sensor**:
   - Let’s assume we have a second sensor, and we are given the following probabilities:
     - $p(z_2 | \text{open}) = 0.5$: The probability of observing $z_2$ if the door is open.
     - $p(z_2 | \neg \text{open}) = 0.6$: The probability of observing $z_2$ if the door is not open.
     - $p(\text{open} | z_1) = \frac{2}{3}$: From previous calculations, this is our belief about the door being open given the first observation.
2. **Updating the Belief**:
   - Using Bayes' Rule, we update our belief about the door being open given both observations $z_1$ and $z_2$:
     $$
     p(\text{open} | z_1, z_2) = \frac{p(z_2 | \text{open}) p(\text{open} | z_1)}{p(z_2 | \text{open}) p(\text{open} | z_1) + p(z_2 | \neg \text{open}) p(\neg \text{open} | z_1)}
     $$
   - Substituting the values:
     $$
     p(\text{open} | z_1, z_2) = \frac{0.5 \cdot \frac{2}{3}}{0.5 \cdot \frac{2}{3} + 0.6 \cdot \frac{1}{3}} = \frac{\frac{1}{3}}{\frac{1}{3} + \frac{1}{5}} = \frac{5}{8} = 0.625
     $$
   - Thus, after the second observation, the probability that the door is open has decreased to 0.625 (62.5%).
3. **Interpretation**:
   - In this case, the second observation $z_2$ **lowers** the probability that the door is open, compared to our belief after only the first observation. This shows how new evidence can change our prior belief.
### **Slide 4: General Form**
1. **Recursive Bayesian Updating**:
   - When there are multiple observations $z_1, z_2, \dots, z_n$, we can generalize this process using recursion.
   - Based on the **Markov Assumption**, each new observation $z_n$ only depends on the current state $x$ (whether the door is open or not) and not on past observations:
     $$
     p(x | z_1, \dots, z_n) = \eta_n p(z_n | x) p(x | z_1, \dots, z_{n-1})
     $$
   - This recursive relationship allows us to iteratively update our belief as new observations come in.
2. **Final General Form**:
   - The general recursive form for updating probabilities based on a sequence of observations is:
     $$
     p(x | z_1, \dots, z_n) = \prod_{i=1}^{n} \eta_i p(z_i | x) p(x)
     $$
   - This shows that we multiply the likelihoods $p(z_i | x)$ of each observation given the current state and scale by a normalizing factor $\eta_i$.
## State Transitions
![[Pasted image 20241025141951.png#invert|400]]
- The outcome of an action is modeled as a random variable $U$, where $U=u$ represents the **state after performing an action**.
- -> If the door is open, the action “close door” succeeds in 90% of all cases
### Discrete and Continuous State Spaces
   - To compute the probability of a certain state $x$ after performing an action $u$, we need to account for all possible **previous states** $x'$.
   - If the state space is discrete (like "open" and "closed"), the probability is computed by summing over all previous states $x'$:
     $$
     p(x | u) = \sum_{x'} p(x | u, x') p(x')
     $$
   - This accounts for the probability of transitioning to $x$ given the action $u$, weighted by the prior probability of each possible previous state $x'$.
   - If the state space is continuous (like a range of possible positions), we integrate over the previous states $x'$:
     $$
     p(x | u) = \int p(x | u, x') p(x') dx'
     $$
   - This is the continuous equivalent, where the integration accounts for all possible prior states.
### Example with Door State
   - Let’s calculate the probability that the door is still open after performing the "close door" action, using the previous formula:
     $$
     p(\text{open} | u) = \sum_{x'} p(\text{open} | u, x') p(x')
     $$
   - We consider the two possible prior states: the door could have been either open or closed before performing the action:
     $$
     p(\text{open} | u) = p(\text{open} | u, \text{open'}) p(\text{open'}) + p(\text{open} | u, \neg \text{open'}) p(\neg \text{open'})
     $$
   - Substituting the given probabilities:
     - $p(\text{open} | u, \text{open'}) = 0.1$: If the door was open, it stays open 10% of the time.
     - $p(\text{open'}) = \frac{5}{8}$: The prior probability that the door was open.
     - $p(\text{open} | u, \neg \text{open'}) = 0$: If the door was already closed, it can’t reopen after performing the action.
     - $p(\neg \text{open'}) = \frac{3}{8}$: The prior probability that the door was closed.

   - The total probability that the door is open after the action:
     $$
     p(\text{open} | u) = 0.1 \cdot \frac{5}{8} + 0 \cdot \frac{3}{8} = \frac{1}{16} = 0.0625
     $$

2. **Complementary Probability**:
   - The probability that the door is closed after the action is:
     $$
     p(\neg \text{open} | u) = 1 - p(\text{open} | u) = \frac{15}{16} = 0.9375
     $$

## Sensor Update and Action Update
1. **Combining Sensor and Action Updates**:
   - So far, we’ve learned two different ways to update the system’s state:
     - **Sensor Update**: $p(x | z)$, which updates the belief about the state based on sensor measurements.
     - **Action Update**: $p(x | u)$, which updates the belief about the state based on the outcome of an action.
2. **Belief Update**:
   - Now, we combine both updates to define the **belief** about the current state after a sequence of actions and sensor measurements.
   - The **belief** at time $t$ is defined as:
     $$
     \text{Bel}(x_t) = p(x_t | u_1, z_1, \dots, u_t, z_t)
     $$
   - This is the probability of being in state $x_t$ at time $t$, given all the actions $u_1, \dots, u_t$ and sensor measurements $z_1, \dots, z_t$ up to that point.
## Graphical Representation
- The overall process of **state estimation** and **sensor updates** over time can be represented using a **Dynamic Bayesian Network** (DBN).
![[Dynamic Bayesian Network]]
## Applications of Bayesian Reasoning
### Localization
   - The goal is to determine the probability of the current position $x_t$ of a robot given a sequence of previous actions $u_1, u_2, \dots, u_t$, observations $z_1, z_2, \dots, z_t$, and a map $m$.
   - The equation is:
     $$
     p(x_t \mid u_1, z_1, \dots, u_t, z_t, m) \propto p(z_t \mid x_t, m) \int p(x_t \mid u_t, x_{t-1}, m) p(x_{t-1} \mid u_1, z_1, \dots, u_{t-1}, z_{t-1}, m) \, dx_{t-1}
     $$
   - **Explanation**: 
     - $p(x_t \mid u_1, z_1, \dots, u_t, z_t, m)$ is the probability of the robot's current position given its actions, observations, and the map.
     - $p(z_t \mid x_t, m)$ is the likelihood of observing $z_t$ if the robot is at position $x_t$.
     - The integral term represents summing over all possible previous states $x_{t-1}$, where:
       - $p(x_t \mid u_t, x_{t-1}, m)$ is the probability of transitioning to $x_t$ given the previous state $x_{t-1}$ and the action $u_t$.
       - $p(x_{t-1} \mid u_1, z_1, \dots, u_{t-1}, z_{t-1}, m)$ is the prior probability of being at $x_{t-1}$ given all previous actions and observations.
### Predictive Distribution for Regression (e.g., Mapping)
   - This equation is about predicting an output $y^*$ for a new input $x^*$ based on observed data $X$ and a parameter $w$.
   - The equation is:
     $$
     p(y^* \mid X, y, x^*) = \int p(y^* \mid x^*, w) p(w \mid X, y) \, dw
     $$
   - **Explanation**:
     - $p(y^* \mid X, y, x^*)$ is the predictive distribution of $y^*$, given the training data $X, y$ and the new input $x^*$.
     - $p(y^* \mid x^*, w)$ is the likelihood of $y^*$ given $x^*$ and model parameters $w$.
     - $p(w \mid X, y)$ is the posterior distribution of $w$ given the training data. This accounts for all possible values of $w$ in the prediction, integrating over their probabilities.
### Object Classification
   - This formula deals with classifying objects, for example, predicting whether $y_* = +1$ (belongs to a class) given observations.
   - The equation is:
     $$
     p(y_* = +1 \mid X, y, x_*) = \int p(y_* \mid f_*) p(f_* \mid X, y, x_*) \, df_*
     $$
   - **Explanation**:
     - $p(y_* = +1 \mid X, y, x_*)$ is the probability that the new data point $x_*$ belongs to the class $+1$.
     - $p(y_* \mid f_*)$ is the likelihood that $y_* = +1$ given the function output $f_*$.
     - $p(f_* \mid X, y, x_*)$ is the distribution of $f_*$ given the observed data $X, y$ and the new input $x_*$.
     - This integral sums over all possible values of $f_*$, taking into account the uncertainty in the model’s output.
### What is Common in These Problems?
#### Representation of Distributions and Expected Values
   - In probabilistic reasoning, we often need to compute expectations of certain functions, represented as $E_p[u]$, where $u$ is a random variable.
   - To compute the expected value of $u$ under distribution $p$, we use:
     $$
     E_p[u] = \int u(w) p(w) \, dw
     $$
   - Here, $u(w)$ might be a function (such as likelihood) evaluated at $w$, and $p(w)$ is the probability distribution over $w$.
- Often, these expectations can only be computed approximately -> to do this, we can use [[Probability Sampling Techniques]] like [[Monte-Carlo Estimation]]
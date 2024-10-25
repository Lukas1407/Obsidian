The theorem states that for two random variables $X$ and $Y$, the **conditional probability** $p(x | y)$ can be written as:
$$
p(x | y) = \frac{p(y | x) p(x)}{p(y)}
$$
This equation allows us to **update** our beliefs about $X$ given information about $Y$. It is especially useful in many areas like machine learning, statistics, and decision making.
### **Understanding the Terms**:
- **$p(x | y)$**: This is the **posterior probability**. It tells us the probability of $X = x$ after observing $Y = y$. In other words, it answers: "What is the probability of $X$ given that we know $Y$?"
- **$p(y | x)$**: This is the **likelihood**. It tells us how likely it is to observe $Y = y$, given that $X = x$. This helps update the probability of $X$.
- **$p(x)$**: This is the **prior probability** of $X$. It represents the probability of $X$ before observing any information about $Y$.
- **$p(y)$**: This is the **marginal probability** of $Y$ (the normalizer), which ensures that the total probability across all possible values of $Y$ sums to 1. It's calculated by summing (or integrating) over all possible values of $X$:
  $$
  p(y) = \sum_x p(y | x) p(x)
  $$
### **Proof of Bayes' Rule**:
The proof is based on the **definition of conditional probability** and consists of three main steps:
1. **Step I**: 
   $$
   p(x | y) = \frac{p(x, y)}{p(y)}
   $$
   - This is the **definition** of conditional probability. It states that the probability of $X$ given $Y$ is the joint probability of $X$ and $Y$ divided by the marginal probability of $Y$.
2. **Step II**: 
   $$
   p(y | x) = \frac{p(x, y)}{p(x)}
   $$
   - This is also the **definition** of conditional probability, but applied the other way around: the probability of $Y$ given $X$ is the joint probability of $X$ and $Y$ divided by the marginal probability of $X$.
3. **Step III**: 
   $$
   p(x, y) = p(y | x) p(x)
   $$
   - From Step II, we can rearrange the equation to express the joint probability $p(x, y)$ as the product of $p(y | x)$ and $p(x)$. This shows that the joint probability is the product of the likelihood and the prior.
4. **Final Step (Combining the Results)**:
   - Substitute the result from Step III into the equation from Step I:
     $$
     p(x | y) = \frac{p(x, y)}{p(y)} = \frac{p(y | x) p(x)}{p(y)}
     $$
   - This completes the proof of **Bayes' Rule**. It shows that the posterior probability $p(x | y)$ can be calculated by multiplying the likelihood $p(y | x)$ with the prior $p(x)$, and then normalizing by $p(y)$.
### **Practical Interpretation**:
Bayes' Rule is essential for updating probabilities as new information becomes available. Here's an example of how it works in practice:
- **Prior** ($p(x)$): Initially, you have a belief about $X$. For instance, if you think it will rain tomorrow, your prior might be based on the weather forecast.
- **Likelihood** ($p(y | x)$): You observe new information, such as dark clouds in the sky. The likelihood is the probability of seeing dark clouds given that it will rain.
- **Posterior** ($p(x | y)$): After seeing the clouds, you update your belief about whether it will rain. The posterior gives you a more informed probability after accounting for the new evidence.
## Bayes Rule with Background Knowledge
- The slide introduces how to apply **Bayes' Rule** when there is **additional background information** available, represented by $z$.
   - The rule now becomes:
     $$
     p(x | y, z) = \frac{p(y | x, z) p(x | z)}{p(y | z)}
     $$
   - Here:
     - $z$ represents background knowledge (additional known variables or conditions).
     - $p(y | z)$ is the **normalizing factor** that makes sure the total probability sums to 1, given the background knowledge $z$.
2. **Normalizer**:
   - The expression $p(y | z)$ is called the **normalizer**, and it ensures that the posterior probabilities $p(x | y, z)$ are properly scaled.
   - In shorthand, $p(y | z)^{-1}$ is denoted as $\eta$, which represents this **normalizer** or scaling factor.
   - Thus, the updated equation is:
     $$
     p(x | y, z) = \eta p(y | x, z) p(x | z)
     $$
   - The **normalizer** ensures that probabilities are normalized, meaning the total probability is 1.
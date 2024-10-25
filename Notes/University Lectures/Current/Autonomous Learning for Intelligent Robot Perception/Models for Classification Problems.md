## Generative Models
   - **Class-conditional probabilities**: The generative model first calculates the probability of the data $x$ given a specific class label $k$, i.e., $p(x | y = k)$. This means the model tries to learn how the data is generated based on a class.
   - **Prior probability**: It also requires the prior probability $p(y = k)$, which represents how likely each class is to occur (i.e., without looking at the input data).
   - **Posterior probability**: Using **Bayes' Rule**, the generative model then combines these two pieces of information to calculate the posterior probability $p(y = k | x)$, which is the probability of the class given the data. 
     - **Bayes' Rule**: 
       $$
       p(y = k | x) = \frac{p(x | y = k) \cdot p(y = k)}{p(x)}
       $$
     - Generative models explicitly model the joint distribution $p(x, y)$, which can be useful for tasks like generating new data.
## **Discriminative Model**
   - Unlike generative models, **discriminative models** don’t concern themselves with how the data is generated. Instead, they focus directly on the **posterior probability** $p(y = k | x)$, i.e., the probability of the class $k$ given the input data $x$. 
   - These models aim to distinguish between different classes without modeling the underlying data distribution. Examples include logistic regression and support vector machines (SVM).
   - Discriminative models usually perform better than generative models in tasks focused purely on classification.
## Discriminative Function
   - This approach involves learning a direct function $f(x)$ that maps input features $x$ to output labels $y$.
   - Instead of dealing with probabilities (as in the previous two models), discriminative functions try to find a direct relationship between input and output. In this case, the goal is to classify the data points by learning a function $f(x)$ that best separates or maps inputs to the correct class labels. Neural networks are an example of discriminative function-based models.
   
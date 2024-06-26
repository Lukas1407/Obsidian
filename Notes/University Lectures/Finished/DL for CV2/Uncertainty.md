Deals with understanding how certain a model is in its predictions. This is especially important for safety critical applications like medical imaging or autonomous driving.

## What causes uncertainty?
- Incomplete coverage of the domain: Encountering classes that were not present in the training data
- Noise in the observations: Objects may look different to the model because of such noise
- Imperfect model
- Biased dataset

## Types of uncertainty
### Aleatoric Uncertainty
- The data uncertainty due to the inherent randomness of the data
- Can in theory be explained away by unlimited sensing
- Cannot be reduced, but learned by the model
1. Homosedastic: if the noise is the same for each sample
2. Heterosedastic: If the noise changes for each sample
### Epistemic Uncertainty
- The uncertainty in the model: measures what the model doesn't know
- Mostly due to lack of training data
- In theory can be explained away by unlimited training data

## Measuring the quality of the uncertainty
### Expected Calibration Error (ECE)
- The predicted probability should ideally reflect the actual likelihood of the answer being correct
- A perfect model is thereby given as:$$\mathop{\mathbb{P}(a_{pred}=a_{true}|conf(a_{pred})=p)=p}$$
- The ECE is calculated as:$$ECE=\sum_{i=1}^{K} \frac{N_{bin_{i}}}{N_{total}}|acc(bin_{i})-conf(bin_{i}),$$ with $K$ being the number of bins the samples are divided into.
- Reliability Diagram:
- ![[Pasted image 20240223155748.png|400]]

## Measuring uncertainty
### Why not use the softmax probability?
Softmax produces pseudo-probabilities, meaning they can be high but the answer might still be incorrect. This is because the NNs are overconfident:
- They overfit to 0 or 1 predictions because of the use of losses such as CE
- -> Datapoints that are far away from the training data or of different classes than the training data can get classified close to 0 or 1
### Calibration-based methods
- Recalibrate the probabilities with some held-out data
- For example: Temperature Scaling:$$conf(a_{pred}) = \max_{\{a \in A\}} \frac{exp(\frac{y_{a}}{\tau})}{\sum_{\hat a \in A}exp(\frac{y_{\hat a}}{\tau})}$$
- Held-out data must roughly follow the distribution of the training data
### Bayesian Neural Networks (BNNs)
- Have a distribution over weights instead of scalar weights
- Output a distribution instead of pseudo-probable point estimates
- Problem: Inference is hard due to intractable integral
- Solution: Monte Carlo Estimates:
	- Average N probabilistic forward passes where dropout is active
	- Mean is the output, Variance measures the confidence
### Model Ensembles
- Train multiple (different) models and calculate the variance in their predictions to estimate the confidence
- Problem: Computationally expensive
- Solution: Batch Ensembles:
	- All models share the same weight matrix $W_{i}$ and have 2 learnable vectors $r_{i}$ and $s_{i}$ with which they scale the shared weights via an outer product: $$\bar{W_{i}} = W_{i}*F_{i},$$ where $F_{i}=s_{i}*r_{i}^T$ 
	- -> reduces computational and memory cost while only marginally reducing ECE performance 
 
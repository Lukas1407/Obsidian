Transfer the the knowledge learned from one task to a different but related task. Usually we have a lot of training data for the base task, but only little for the target task.

A task T consists of a label space y and an objective predictive function f.
A domain D consists of a feature space χ and a marginal probability distribution P(X).

## Domain Adaption
- Deals with the problem of adapting a model (e.g. a [[Convolutional Neural Network|CNN]]) to a previously unseen domain with a different marginal distribution 
- Main idea: Labeled training data is only available for a source domain $D_{s}$ while the target domain $D_t$ contains little or no labeled data 
- Objective: Still classify/segment/detect instances correctly in the target domain
- Main challenge: domain shift or domain gap between the different distributions and adapt the model in a way that performs reasonable on the target domain

- Deep CNNs are very sensitive to changes in data distribution. Unaddressed domain shift leads to significant decline in performance!

### Classification
- Can be categorized based on the data we have for the target domain $D_{t}$:![[Pasted image 20240226094903.png]]
	- Semi-Supervised DA: A few labeled samples from $D_{t}$
	- Unsupervised DA: Only unlabeled samples from $D_{t}$ 
	- Domain Generalization: No training data from $D_{t}$

- Can be categorized based on the classes from the source and target domain $C_{s}, C_{t}$:![[Pasted image 20240226095154.png]]

### Methods
1. [[Domain Adversarial Neural Network (DANN)]]
2. [[Domain Specific Batch Normalization (DSBN)]]
3. [[Cycle-Consistent Adversarial Domain Adaptation (CyCADA)]]
4. [[Domain Alignment]]
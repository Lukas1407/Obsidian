- Assumptions: 
	- Each data point must be represented by 2 conditionally independent views 
	- Conditionally independent, meaning that they are complementary rather than repetitive
	- Each view must be sufficient to train a good model

## Approach
1. Train a different model on each view with the labeled data
2. Pass the unlabeled data trough the models
3. If the predicted probability of a unlabeled sample is above some threshold, add it to the training data of the other model together with the predicted label
4. Retrain the models with the possibly new training data
5. Repeat until no more samples are added

## Problems
- The models then to overfit in the initial phase
	- Solution: Add pre-training 
- The models are prone to predict a biased distribution over the categories in the co-training phase
	- Solution: Add diversity preserving co-training, which balances the unlabeled samples based on the learned features, such that the feature space is uniformly represented in the added pseudo-labeled images
Models that are able to continually acquire knowledge from a sequence of tasks, even if the training data for previous tasks is not or not fully available.

Main Problem: [[Catastrophic Forgetting]]

## 3 Major Directions
1. Regularization:
	- Aim to only change the weights that have no or minimal contribution to the previous tasks
	- Advantage: Lightweight
	- Disadvantage: Difficult to avoid forgetting
	- Examples: [[Elastic weights consolidation (EWC)]]
2. Architecture Growing ^fdf1db
	- Add new layers/components if trained on a new task
	- Advantage: Easy to avoid forgetting
	- Disadvantage: Increase in memory consumption
3. Memory Replay
	- Store some/all training data from the previous tasks
	- If trained for a new task, also use some/all the stored training data
	- Advantage: Easy to avoid forgetting
	- Disadvantage: Increase in memory consumption  
	- How to populate the memory?:
		- Reservoir sampling: put all encountered data in the memory. When full, replace uniformly
		- Class-Balancing Reservoir Sampling (CBRS): class balanced sample replacement 
		- Weighted replay Gradient Episodic Memory (GEM): greedily maximize the variance of gradients directions of the samples in the memory. Require knowledge of task labels 
		- Gradient based Sample Selection (GSS) Incremental method to improve GEM without requiring to have task labels- thus suitable for online CL
	- Which samples to use from the memory?:
		- All
		- Randomly
		- Samples that suffer from an increase in loss given the estimated parameters update of the model

## Evaluating Continual Learning
1. Task-incremental learning (Task-IL)
	- Given the test sample and the task id, solve the task
2. Domain-incremental learning (Domain-IL)
	- Given the test sample and not the task id, solve the task
3. Class-incremental learning (Class-IL)
	- Given the test sample and not the task id, predict the task id and solve it
- This is usually done for all previously trained tasks and averaged
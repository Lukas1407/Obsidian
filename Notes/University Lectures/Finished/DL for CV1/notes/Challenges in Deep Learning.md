
## Underfitting
- The model was not able to learn generalizable features that capture the underlying task
![[Pasted image 20240305081628.png#invert|300]]
- Why does this happen?:
	- Too little training data
	- The model was too simple in relation to the available training data -> bad parameter vs training data ratio
	- Unclean training data (too much noise)
	- Excessive regularizations to prevent [[Challenges in Deep Learning#Overfitting|overfitting]]
- Techniques to reduce underfitting:
### Increase Training data
- Use more (clean) training data
- Remove the noise from the training data
- Use [[Data Augmentation]]
### Increase Model Complexity
- Allow the model to capture more information from the provided training data
- May be done by [[Continual Learning#^fdf1db|Architecture Growing]]
### Longer Training time
- Increase the training epochs
### Model Ensembles
- Train multiple models and combine their outputs to get better generalization
- Can be different models (heterogeneous)
- Can be the same model (homogeneous)
- Disadvantages:
	- Increase cost
## Overfitting
- The model recognized the training data and is not able to generalize the knowledge to similar data to the training data
![[Pasted image 20240305083207.png#invert|300]]
- Why does this happen?:
	- Trained for too long
	- The model was too complex in relation to the available training data -> bad parameter vs training data ratio
	- Too little training data
	- Unclean training data (too much noise)
- Techniques to reduce underfitting:

### Weight regularization
- This is a general term for adding penalty terms to the loss function based on some measure of the model complexity
- [[Challenges in Deep Learning#Weight Decay|Weight decay]] is a special case of weight regularization, where the penalty term is based on the L2 norm of the weights
- Other types of weight regularization include L1 regularization, where the penalty term is based on the sum of the absolute values of the weights, and elastic net regularization, where the penalty term is a combination of L1 and L2 regularization
- Weight regularization reduces overfitting by imposing a constraint on the model weights, which prevents them from growing too large and overfitting the data. The penalty terms are controlled by hyperparameters that determine the strength of the regularization
### Weight Decay
- This is a regularization method that adds a penalty term to the loss function based on the size of the model weights
- The penalty term is proportional to the sum of the squared weights, which is also called the L2 norm
-  Weight decay reduces overfitting by shrinking the weights towards zero, which makes the model simpler and less prone to memorize the noise in the data
-  The penalty term is controlled by a hyperparameter called the weight decay coefficient, which determines how much the weights are penalized. A common formula for the loss function with weight decay is:
$$L(w)=L_{data}​(w)+2\lambda||w||^2,$$

where $L(w)$ is the total loss, $L_{data}​(w)$ is the data loss, $λ$ is the weight decay coefficient, and w is the vector of model weights
### Cross-Validation
- This is a technique for estimating the generalization performance of a model by splitting the data into multiple subsets and evaluating the model on each subset
- The most common type of cross-validation is k-fold cross-validation, where the data is divided into k equal-sized folds, and the model is trained on k-1 folds and tested on the remaining fold
- This process is repeated k times, and the average test performance is reported as the cross-validation score
- Cross-validation reduces overfitting by providing a more reliable estimate of the model’s performance on new data, which can help in selecting the best model and tuning the hyperparameters
### Increase Training data
- Use more (clean) training data
- Remove the noise from the training data
- Use [[Data Augmentation]]
### Dropout
- This is a regularization technique for neural networks that randomly drops out some of the units in the network during training
- Dropout prevents co-adaptation of features, which means that the network cannot rely on the presence of specific units to make predictions
- Dropout reduces overfitting by creating a more sparse and diverse network, which can generalize better to new data
- Dropout is applied with a probability p, which determines how many units are dropped out. A common value for $p$ is 0.5, which means that half of the units are dropped out
- Dropout is only active during training
- At inference, we need to scale down the weights to be of similar magnitude as during training to apply the learned knowledge
	- This is done by weighing the weights by the dropout rate $p$
### [[Batch Normalization]]
- Was not primarily developed to counter overfitting, but is very effective against it
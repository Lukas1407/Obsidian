Problems of fine-tuning large models with billions of parameters:
- High memory consumption, as we may update all the weights
- High computational cost
- Infeasible on consumer hardware
- Prone to [[Catastrophic Forgetting|catastrophic forgetting]]
Solution: Only fine tune a small amount of the weights, called partial fine-tuning

## Partial Fine-Tuning
### Adapters
- Add small NNs with 2 linear layers in between the model at different locations
- Only train the weights of these
- This allows fine-tuning while retaining generalization
- Advantages:
	- Also allows adaption to different domains and even input dimensions
- Disadvantages:
	- Adds latency at inference
### Low Rank Adaption (LoRA)
- Idea: the pre-trained model has already learned good and useful features -> the gradient updates $\Delta W$ are sparse, they have inherently low rank $r$
- Approximate update matrix: $$\Delta W_{q} \approx W_{q-up} * W_{q-down}$$![[Pasted image 20240226094131.png]]
- Advantages: 
	- No additional parameters, meaning no latency at inference
- Disadvantages:
	- Model architecture stays the same → Cannot be applied on domains from other dimension
Interactive segmentation describes an iterative feedback loop, where user-provided corrections to the model’s output inform subsequent predictions, leading to updated predictions. User guidance is provided in the form of, e.g., clicks, scribbles, or other interactions.![[Pasted image 20240226124746.png]]

## Types of Interactions
1. Clicks:
	- A point $c_{i} = (x_{i} , y_{i})$
	- Advantages:
		- Quick,
		- Precise: Can be placed in tight spots to correct small errors 
		- Easy to simulate during training: Center of largest object / error, Extreme points, Random click 
	- Disadvantages:
		- Ambiguous: Not always clear what the user intends 
		- May require many clicks for complex object
2. Scribbles:
	- Set of n points $S = \{c_{1} , c_{2} ,…,c_{n} \}$ 
	- Advantages:
		-  Flexible and precise: Can approximate any shape
		- Low ambiguity due to its expressiveness 
	- Disadvantages:
		- Simulations are possible but introduce a “user shift”
			- User shift: Discrepancy between simulated interactions during training and real interactions during evaluation. Occurs due to the larger flexibility and “infinite” ways to simulate it 
		- Takes slightly more time to draw
3. Bounding Boxes:
	- Points representing a rectangular region
	- Advantages:
		- Quick: Requires 2 – 4 clicks 
		- Localizes the context Model: can ignore everything outside the box 
		- Easy to simulate
		-  Can be represented in many ways
	- Disadvantages:
		- Low precision: Bounding box contains background information as well
4. Boundary Polygons:
	- Sequence of $m$ vertices $P = \{v_1 , v_2 ,…, v_m\}$ lying on the boundary of the object 
	- Advantages: 
		- Gives control to the user to exactly fit the boundary and correct the initial prediction 
	- Disadvantages:
		- Takes more time to drag all vertices to the correct position 
		- Difficult to simulate: Many possible ways to correct a vertex
5. Text prompts: 
	- Intuitive
	- Can eliminate ambiguity of clicks when combined
6. Eye gaze:
	- Intuitive 
	- Very Quick 
	- Removes the need of touch E.g. touching a screen in a surgery room or splitting attention while driving

## Guidance Signal
A guidance signal is a representation of the user interactions in a form in which the model can process it. 
- This can be an explicit representation that involves transforming the user interaction into an additional structured input for the model to process and learn from
- Or implicit, where user interaction information is subtly integrated into the model’s learning process without the provision of explicit structured input.

## Robot User
A simulated model that mimics the behaviour of a real human annotator. The robot user leverages ground-truth labels to simulate user interactions at plausible locations.
### Workflow:
1. Robot user provides initial interaction
2. Model segments based on that interaction
3. Robot user provides new interaction in miss-segmented regions
4. Model updates segmentation
5. Repeat and after some steps calculated the loss

## Active Learning
Two types of active learning:
1. Focused on data annotation
	- Annotators label a few samples (annotation budget) with interactive segmentation 
		- Model is trained on annotation budget 
		- Predicts on the rest of unlabeled data 
		- “Most informative samples” are selected for further annotation and added to annotation budget 
			- Informative := Adding annotations to it would benefit the model training
			- Most often associated with prediction uncertainty
		- All steps are repeated until the model reaches a certain performance on an independent test dataset 
	- In the end: 
		- Most important samples are labelled and can be used for model training 
		- A model is already trained well and can be deployed
2. Focused on model refinement
	- Goal: Improve model, not annotate more data 
	- Same sampling selection strategies as for Data Annotation 
		- Most informative samples or hard-sample mining (worst model performance) 
	- Final goal is to deploy a robust model

## Example
![[Pasted image 20240226130103.png]]

## Evaluation Metrics
### Segmentation Performance 
- NoC@90: Number of Clicks (NoC) at 90% performance (typically IoU or Dice) 
- IoU@10: IoU at 10 clicks
- Dice @10: Dice at 10 clicks 
- NoC / IoU curves 
- NoC / Dice curves 
- Consistent Improvement (CI): % of iterations where adding an interaction improves the segmentation
### Usability 
- User Time: Time it takes to annotate an image in seconds
-  Machine: Time Inference time for an image 
- Scribble Length: Mean number of pixels in scribbles
-  NASA-TLX: Score Perceived workload in terms of mental demand, frustration etc. 
- System Usability Scale (SUS): Likert-scale questionnaire to quantify usability

## Models
1. [[Segment Anything Model (SAM)]]
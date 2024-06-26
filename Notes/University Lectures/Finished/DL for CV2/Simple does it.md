- Weakly supervised model for semantic segmentation
- Uses bounding boxes as supervision signal
- 3 Assumptions about the bounding boxes:
	1. Everything outside the bounding box is background
	2. The bounding box gives hints about the size of the object
	3. The object inside the bounding box is visually distinct from the background
## Naive approach
1. Start with the bounding box as target
2. Predict the mask and feed the predictions as target into the next stage![[Pasted image 20240226082958.png]]
-> Better result than the bounding box
But: degenerates quickly

## Improvements 
### Add rules based on the Assumptions
1. Reset pixels outside of the bounding box
2. If the bounding box of the generated mask is to small with respect to the ground truth bounding box, reset it
3. Filter segmentations to better follow the object boundaries
### Start with smaller bounding boxes
- Smaller bounding boxes with the same center coordinates![[Pasted image 20240226084655.png|400]]
- This leads to lower recall but higher precision, which maybe beneficial for early training
### Start with coarse masks
- Use [[GrabCut]] and [[MCG]] to approximate the mask form the bounding box

## Results![[Pasted image 20240226085515.png]]
Train the model to predict some held-back part of the data (pretext task).
-> Form of unsupervised learning
This pretext task is usually chosen such that the model learns what we real care about.
After that we usually fine-tune for some downstream task where labels are available.

## Examples of pretext tasks
### Relative patch position for context prediction
- Split image into multiple patches
- Train to predict the position of a patch relative to some reference patch ![[Pasted image 20240224144335.png|400]]
### Predict patch position in a jigsaw puzzle
- Split image into 9 patches
- Predict the position of all 9 patches
### Inpainting
- Remove a random region from the image
- Train to predict the missing region
- -> Model needs to learn the entire context and content of the image![[Pasted image 20240224144740.png|400]]
### Frame ordering
- Randomly shuffle the frames of a video with a moving object
- Train to predict the correct order of the frames
### Colorization
- Gray scale the image or video frames
- Train to predict the color of the (moving) objects 
### Clustering
- Add pseudo labels to the data based on similarity
- Train the model to predict those pseudo labels
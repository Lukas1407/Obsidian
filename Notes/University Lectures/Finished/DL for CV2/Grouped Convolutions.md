Idea: divide input volume into groups. Filters only "work" on their group.
- Each filter only has 1/g amount of work and parameters 
- But each filter also only sees 1/g channels and cannot work on all information
Solution: [[Depthwise Separable Convolution]], [[ShuffleNet]]
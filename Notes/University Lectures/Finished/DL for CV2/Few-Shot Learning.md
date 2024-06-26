Compared to [[Zero-Shot Learning]], we no have some labeled training data for the target classes. The zero-shot assumption still holds.

N-way K-shot few shot learning task: N new classes with K labeled samples.

## Approaches
1. Metric Learning: Learn to project the image to an embedding space using a distance loss function that aims to establish similarity or dissimilarity between images
2. Meta Learning: Learn a learning strategy to adjust well to a new few-shot learning task
3. Augmentation-based: Synthesize more data from the novel classes to facilitate the regular learning
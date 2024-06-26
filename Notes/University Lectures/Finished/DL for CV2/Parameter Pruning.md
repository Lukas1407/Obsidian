- Removing redundancy/low value information from the network 
- Pruning starts with a "bigger/heavier" network and tries to reduce the size 
- Objective: Eliminate neurons or whole filters (in a [[Convolutional Neural Network|CNN]]) while maintaining the metric (e.g. accuracy) 
- Can help to remove e.g. multiple filters that learned (almost) the same feature like edge detection or color features 
- Redundancy is actually quite common in NNs: Think about training with dropout, where often 50% of the values are randomly zeroed

## Common steps
1. Find unimportant filters according to some metric 
2. Remove filters and adjust the filters of the subsequent layer 
3. Fine-tune to "repair" the damage 
4. Repeat until the target pruning percentage is achieved

## Ways to determine redundant filters
1. Sum of absolute weight values in a filter: Small weights tend to produce weak activations and do not contribute much.
2. Average Percentage of Zeros in a filter: Considers the sparsity of a filter, many zeros = information loss 
3. Phrasing it as an optimization problem: Try to find a filter that affects the output of the following layer the least, removes it and fine-tunes the network. 
4. Iterative pruning approach, temporarily removing filters while monitoring the sensitivity metric of a detection task. Filters leading to the smallest drop are removed. No fine-tuning needed after every step.
### Differences in pruning setups
1. Iterative vs. one-shot methods: Iterative setups only remove a small amount of filters per step. 
2. Fine-tuning: Iterative methods often retrain after every pruning step, others only at the end. 
3. Structured vs. Non-structured pruning: Structured pruning removes whole filters, non-structured removes single weights to induce sparsity. This often requires special hard- or software to handle. 
4. Global vs. Local pruning: Global pruning considers all filters, local e.g. only a single layer
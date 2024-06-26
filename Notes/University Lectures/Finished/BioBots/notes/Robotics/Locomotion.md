## Different Types of Locomotion
- Swimming
- Walking
- Hopping
- Flying
- Creeping

## Types of 4 legged Locomotion
1. Insect-like
2. Reptile-like
3. Mamma-like
![[Pasted image 20240301081157.png#invert|]]
### Mammal-like Locomotion
1. Toe-walking: Dog, cat
2. Sole-walking: Bear
3. Tips-walking: Horse, Elephant
![[Pasted image 20240301081457.png#invert|]]

## Legged Locomotion Patterns
Can be classified based on how many legs are touching the ground at each time:
1. Tripod: at least 3 legs
2. Tetrapod: at least 4 legs
3. Pentapod: at least 5 legs
This can be visualized via a gait diagram:
![[Pasted image 20240301123318.png#invert| ]]
More legs lead to higher stability, as we only need 3 legs touching the ground to be in a stable state. More legs mean we have more redundancy.
### Phase Shift
- Defines the locomotion pattern based on the start time of the step phase for each leg:$$\phi_{i}=\frac{start\ of\ stem\ phase\ of\ leg\ i}{ T}$$
### Duty Factor
- Defines the locomotion pattern based on the duration of the stem phase of each leg:$$\beta_{i}=\frac{stem\ time\ of\ leg\ i}{T}$$
![[Pasted image 20240301130054.png#invert|400]]


## Stability
Stability while in locomotion is primarily influenced by the locomotion pattern.
### Static Stability
- The ability that the locomotion can stop at any time and the system stays stable
- The legs touching the ground create a polygon:![[Pasted image 20240301124316.png#invert]]
- If the center of mass is inside that polygon, it is statically stable
- -> At least 3 legs need to be on the ground
- -> The longer that legs, that larger the polygon
#### Static Stability Margin
The minimal distance from the center of mass to the vertices of the polygon
### Dynamic Stability
Refers to the ability to maintain balance and control while in motion. It takes dynamic effects onto consideration like velocity, inertia, making it more complex to calculate that static stability.
#### Force Angle Stability Margin (FASM)
#### Zero Moment Point (ZMP)
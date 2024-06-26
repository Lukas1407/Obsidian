
## Requirements to the Architecture
- Supports parallelism: Behavior is calculated on different processes
- Hardware dependent: Behavior is dependent on the specific hardware
- Modularity: Simple modules that allow extensibility
- Robustness: The system should be robust
- Flexible at run time: The system should be adaptable
- Effectiveness to the goal

## Subsumption
- Decomposing the complete behavior into sub-behaviors 
- These sub-behaviors are organized into a hierarchy of layers
- Higher levels are able to subsume lower levels (= integrate/combine lower levels to a more comprehensive whole)![[University Lectures/Finished/BioBots/notes/images/Diagram.svg]]
- The outputs of layers can be commands to actuators, or signals that suppress, inhibit, or replace the outputs of other layers

## Behavior Networks
- Weak hierarchy: The network is organized into layers of behaviors, but some behaviors can bypass the hierarchy and directly influence other behaviors in different layers
- Competitive antagonism: The network uses a mechanism of selecting the best behavior among several competing ones, based on their inputs, motivations, and outputs
- Dynamic adaptation: The network can change its behavior according to the changing input parameters, by adjusting the motivation levels of each behavior node
- Virtual sensors: Each behavior node has two virtual sensors, reflection and activity, that measure the internal state of the node, such as its effort and satisfaction
- Fusion nodes: A special type of behavior node that combines the outputs of multiple behavior nodes into one output, to enable cooperation among behaviors![[University Lectures/Finished/BioBots/notes/images/Diagram 1.svg]]
- State $r$: This is the reflection sensor, which shows the satisfaction of the node with the current situation. It can be used to measure the progress or the success of the node’s task. A high value of $r$ indicates a high satisfaction, while a low value indicates a low satisfaction.
- Activity $a$: This is the activity sensor, which shows how much effort the node has to bring to accomplish its task. It can be used to measure the difficulty or the complexity of the node’s task. A high value of $a$ indicates a high effort, while a low value indicates a low effort.
- Motivation $\iota$: This is the motivation sensor, which shows how much the node wants to perform its task. It can be used to measure the importance or the urgency of the node’s task. A high value of $\iota$ indicates a high motivation, while a low value indicates a low motivation.
> [!summary] 
> A genetic algorithm (GA) is a process of finding a [[Heuristic]], inspired by the process of [[Natural Selection]]. GA belong to the larger class of [[Evolutionary Algorithms (EA)]].

In a genetic algorithm, a population of candidate solutions (called individuals or phenotypes) to an optimization problem is evolved toward better solutions. Each candidate solution has a set of properties (its chromosomes or genotype) which can be mutated and altered.

The evolution usually starts from a population of randomly generated individuals, and is an iterative process. The population in each iteration is called a generation. In each generation, the [[Fitness]] of every individual in the population is evaluated.
The more fit individuals are, the higher their probability of being selected for the next generation is. 
Each individual's genome is modified (recombined and possibly randomly mutated) to form a new generation. The new generation of candidate solutions is then used in the next iteration of the algorithm. 
Commonly, the algorithm terminates when either a maximum number of generations has been produced, or a satisfactory fitness level has been reached for the population.

A typical genetic algorithm requires:
1. A genetic representation of the solution domain
2. A [[Fitness]] function to evaluate the solution domain

## Representation of Individuals
- Usual done using a binary array which represents the phenotype
- This makes it easy to mutate and facilitates [[Crossing Over]]
## Initialization
- The population size depends on the nature of the problem, but typically contains several hundreds or thousands of possible solutions. 
- Often, the initial population is generated randomly, allowing the entire range of possible solutions
## Selection
- Individual solutions are selected through a fitness-based process
- Fitter solutions (as measured by a fitness function) are typically more likely to be selected
- Also maybe select random samples to ensure genetic diversity
## Genetic Operators
- The next step is to generate a second generation population of solutions from those selected
- This is done through a combination of genetic operators: crossover (also called recombination), and mutation
- For each new solution to be produced, a pair of "parent" solutions is selected for breeding from the pool selected previously. By producing a "child" solution using the above methods of crossover and mutation, a new solution is created which typically shares many of the characteristics of its "parents"
- These processes ultimately result in the next generation population of chromosomes that is different from the initial generation
- Generally, the average fitness will have increased by this procedure for the population, since only the best organisms from the first generation are selected for breeding, along with a small proportion of less fit solutions
- These less fit solutions ensure genetic diversity within the genetic pool of the parents and therefore ensure the genetic diversity of the subsequent generation of children
> [!abstract] Definition
> The DRY principle emphasizes the importance of avoiding duplication in your code. 

- Copy & paste decreases:
	- Understandability 
		- Code is less compact 
		- An identical concept needs to be understood multiple times 
	- Maintainability / Evolvability
		- Losing track of copies 
		- Horror: Undocumented dependencies from one copy to the other(s) 
		- Need to find and modify all copies when removing bugs or changing behaviour
## Details
- **Reduces Redundancy**: Each piece of knowledge or logic should have a single, unambiguous representation within a system. Duplicating code leads to multiple places to update when changes are needed, which is inefficient and error-prone.
- **Improves Maintainability**: When logic is consolidated, maintaining and updating code becomes simpler. Any modification needs to be done in only one place, reducing the risk of missing some instances during bug fixes or updates.
- **Enhances Understandability**: Having a single source of truth for each behavior or piece of logic means that developers need to understand the concept once, rather than reinterpreting the same logic scattered across the codebase.
- **Practical Implementation**:
    - **Extract Methods**: Commonly used code blocks should be extracted into their own methods.
    - **Parameterization**: Use parameters to generalize methods, allowing them to handle various data or behaviors that are similar but not identical.
    - **Use Abstraction**: Abstract classes or interfaces can be used to generalize and encapsulate common behavior, which can then be shared across multiple implementations.
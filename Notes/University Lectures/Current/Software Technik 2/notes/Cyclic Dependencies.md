> [!abstract] Definition
> A **cyclic dependency** occurs when two or more components depend on each other either directly or indirectly, creating a cycle. This means that component A depends on component B, which in turn depends on component A (directly or through other components), forming a loop. 

### Strategies to Manage Cyclic Dependencies
1. **Refactoring**:
    - **Decompose Monolithic Structures**: Break down large components with intertwined dependencies into smaller, more manageable pieces that have clear, single-direction dependencies.
    - **Use Design Patterns**: Employ design patterns like Facade, Adapter, or Mediator to manage dependencies between modules effectively.
2. **Dependency Inversion Principle**:
    - One of the SOLID principles, it suggests that high-level modules should not depend on low-level modules, but both should depend on abstractions. Abstractions should not depend on details, but details should depend on abstractions. This principle can be used to invert the direction of dependencies so that they flow toward stability rather than away from it.
3. **Layered Architecture**:
    - Structure the system in layers where each layer only interacts with the layer directly below it. This architectural style can help prevent the formation of dependencies between higher-level components.
4. **Tools and Analysis**:
    - Use static code analysis tools to detect and visualize cyclic dependencies. Regular checks with these tools can help catch and resolve cycles early in the development process.

## Example: Find all the Dependencies
![[Pasted image 20240701114642.png#invert|800]]
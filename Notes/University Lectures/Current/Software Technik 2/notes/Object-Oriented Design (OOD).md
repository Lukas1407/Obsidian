> [!abstract] Definition
> Object-Oriented Design (OOD) is a fundamental approach in software engineering focused on designing software systems using objects. Objects are instances of classes that encapsulate data and behaviors, interacting with one another to form a complete system. Each object maintains its own state and has operations to modify that state, contributing to the overall functionality of the software. 

Ian Sommerville, a prominent author in software engineering, describes an object-oriented system as one "made up of interacting objects that maintain their own local state and provide operations on that state information." This encapsulation of state and behavior within objects allows for modular, maintainable, and reusable code.
## No Clear Separation Between Design and Coding Principles
In object-oriented development, the principles of design and coding often intertwine. Good design principles directly influence how code is written, structured, and maintained. The seamless integration of design and coding principles ensures that the system is built with maintainability, scalability, and robustness in mind.
## Principles of Good OO Design
### [[SOLID Principle]]
- These 5 principles, often referred to as the SOLID principles, were popularized by Robert C. Martin and are intended to promote better software design and maintainability
### [[Law of Demeter]] 
### [[Boy Scout Rule]]
### [[Principle of Least Surprise]]
### [[Coding Conventions]] 
### [[Don’t repeat yourself (DRY)]] 
### [[Keep it simple, stupid (KISS)]] 
### [[You ain’t gonna need it (YAGNI)]] 
### [[Single Level of Abstraction]]
### [[Refactoring]]

## Comparison to [[Object-Oriented Analysis (OOA)]]
- **Purpose**: OOD takes the concepts identified during OOA and refines them into a blueprint for building a software system that meets the requirements and constraints identified in the analysis phase.
- **Focus**: This phase is about defining how the system will be implemented, which often includes making compromises and optimizations not reflected in the OOA.
- **Key Activities**:
    - **Prescribing Implementation Details**: Transforming the domain model into a detailed design that can be directly implemented, including decisions on technology, platforms, and system architecture.
    - **Enhancing Software Structure**: Incorporating additional technical classes that do not directly model real-world concepts but are necessary for the functioning of the system (e.g., classes for data access, user interface management, session handling).
### Key Differences and Considerations
- **Classes in OOA and OOD**: While there is a strong correlation between the classes in OOA and OOD (often they share names and basic attributes), OOD classes include implementation-specific methods and properties. OOD classes also integrate system considerations such as performance, security, and concurrency which are typically absent in OOA.
- **Additional Classes in OOD**: OOD often introduces new classes that do not have direct counterparts in the OOA. These classes handle technical aspects of the system such as scheduling, access control, logging, and data conversion.
- **Avoiding Cyclic Dependencies**: A common issue in OOD is the need to manage dependencies carefully to avoid cycles. Cyclic dependencies can lead to complex, tightly coupled systems that are hard to maintain. OOD strives to organize dependencies in a tree-like structure where navigation through the system is straightforward and logical.
- **Conversion from OOA to OOD**:
    - **Identify Root Classes**: Determining a root class from which all other classes can be accessed helps in avoiding cyclic dependencies and simplifies the system architecture.
    - **Pitfall of Cyclic Dependencies**: It's common to inadvertently retain some cyclic dependencies from OOA into OOD. Careful design and refactoring are necessary to eliminate these cycles and ensure a clean, maintainable architecture.
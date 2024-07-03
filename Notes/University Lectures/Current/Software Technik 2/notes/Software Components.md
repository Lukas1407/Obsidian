> [!abstract] Definition
> A **software component** is a unit of composition with contractually specified interfaces and explicit context dependencies only. A software component can be independently deployed and is subject to composition by third parties. This definition emphasizes that components are self-contained, interact through well-defined interfaces, and are reusable across different systems and contexts. 
### Clarifying Common Questions
1. **Does a component have state?**
   - **Answer**: Yes, a component can have state. Unlike simple functions or stateless services, components often encapsulate both behavior and state. This state is managed internally and exposed through its interface in a controlled manner.
2. **Is a component (more or less than) an object?**
   - **Answer**: A component can be thought of as a higher-level construct than an object. While objects in object-oriented programming are defined primarily by their class and exhibit behavior through methods, components are larger in scope, often encapsulating several objects and focusing on fulfilling a specific business or technical functionality via their interfaces. Components also manage their lifecycle and dependencies more explicitly than individual objects.
3. **Is a component a module?**
   - **Answer**: The terms "component" and "module" are sometimes used interchangeably, but there are distinctions. Modules typically refer to parts of a system that are grouped based on functionality and can include one or more components. A module is more about the organization of code, while a component is about reusability and encapsulation. A component is designed to be plugged into different systems and is self-contained, often distributed as a package, whereas a module is a structural part of a single system.
## Definition: Component
> [!abstract] Definition
> A component is a contractually specified building block for software which can be composed, deployed, and adapted without understanding its internals. 
- **Contractually Specified**: A software component must adhere to a defined specification or contract. This contract typically includes the component's interfaces, expected behaviors, and interactions with other components. It ensures that the component can be used without knowledge of its internal implementation details.
- **Composability**: Components are designed to be easily integrated with other components. The ability to combine components in various configurations allows for building complex systems from simpler, reusable pieces.
- **Deployment and Adaptation**: Components are structured so that they can be deployed independently and adapted to different environments or systems without modifying their internals. This independence from the deployment context underscores their versatility.
- **Transparency for Tools, Not Users**: While components are generally treated as black boxes, some internal information might be exposed for the purposes of debugging, optimization, or interfacing with development tools. This does not contradict the component being a black box from the perspective of its use in application logic.
## Why Objects Are Not Components
- **Inheritance vs. Composition**: In object-oriented programming, objects often interact and relate through inheritance. However, inheritance exposes the internal structure and behavior of objects, potentially leading to tight coupling between base classes and subclasses. This coupling can conflict with the black-box principle, where the internals of a component should not influence its use or interaction with other components.
```java
class A { 
	public T m { 
		... 
		x(); 
		... 
	} 
	public T x(){
		...
	} 
} 
class B extends A { 
	public T x(){
		 ... // redefinition 
	}
}
... 
B b = new B(); 
b.m(); // -> result unclear for developer of A and B!
```
- In your example, class `A` has a method `m` which uses another method `x` within the same class. If another class `B` inherits from `A` and overrides method `x`, the behavior of `m` when called on an instance of `B` might change in unexpected ways. This shows that the behavior of `B` is not entirely independent of `A`, and hence `B` (as an object) cannot be considered a component under the definition that emphasizes black-box operation.
- **Issue Highlighted**: When `b.m()` is called, and `B` has overridden `x`, the result of `b.m()` depends on the implementation of `x` in `B`. This dependency on the internal structure and behavior of `A` makes `B` not a black-box component, as its behavior cannot be fully understood without knowing about `A`'s implementation of `m` and `x`.
## Components in Java: Mimicking Behavior
- There is no such thing as a component in Java
- In Java, components can be mimicked by using classes or groups of classes that interact with each other through well-defined interfaces. These interfaces serve as contracts, dictating how components can communicate without needing to know the underlying implementation details. Here's how you can achieve this:
	1. **Using Interfaces and Classes**: Define interfaces that abstract the functionality of the component. Implement these interfaces in one or more classes that contain the business logic, ensuring these implementations are swappable without affecting users of the interface.
	2. **Package Structure**: Organize related classes into packages that serve as components. Use package-private visibility to hide implementation details from users of the component, exposing only what is necessary through public interfaces.
	3. **Dependency Injection**: Utilize frameworks like Spring or Google Guice to manage components. These frameworks support dependency injection, which helps in managing component dependencies through interfaces, making the system more modular and easier to test.
## Components in an Enterprise Context
In enterprise applications, components are often conceptualized as **business components**. These components typically:
- **Feature-Oriented**: Focus on specific business capabilities or domain functionalities, such as order management, customer relations, or inventory control.
- **Form Application and Domain Layers**: Represent the core of the business logic, dealing with data management and business rules.
- **Infrastructure Components**: Components aren't limited to business logic; they can also be infrastructure-related, like those managing database connections (e.g., using JDBC).
### Corresponding Pattern: Service Layer
The **Service Layer** pattern, as described by Martin Fowler and akin to Larman’s Application Layer, is particularly relevant here:
- **Definition**: This layer defines the application's boundaries with a suite of services that not only make operations available to external consumers but also coordinate responses and control application logic flows.
- **Responsibilities**:
    - **Application's Boundary**: Acts as the communication front for external calls to the application, delineating what operations are permissible.
    - **Coordinates Operations**: Manages how the application responds to each operation, often involving orchestrating calls to multiple components within the application.
    - **Abstracts Complexities**: By encapsulating the business logic and interactions within the service layer, it abstracts the complexities from the user interfaces and external clients.
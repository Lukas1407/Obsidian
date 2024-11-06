## Principles
### Single Responsibility Principle (SRP)
- An object or class should have only one reason to change, meaning it should have only a single job or responsibility. 
- This principle reduces complexity by limiting the impact of changes and making the system easier to understand
#### Example
```java
public void putInFood(Food f) {
	f.fridge=this;
	foods.add(f);
}
```
- Assigning the fridge filed should be done by a separate method
#### Core and Cross-Cutting Concerns
1. **Core Concerns**:
   - Each responsibility or duty of a class should pertain to only one aspect of the software's functionality. For instance, a class handling user data should only manage aspects directly related to user data, such as storing and retrieving user details, and not take on additional responsibilities like managing user permissions or logging data access, which might be handled by other classes.
2. **Cross-Cutting Concerns**:
   - Despite focusing on a single responsibility, a class may still interact with other parts of the system through "cross-cutting concerns" which are aspects of a program that affect other parts of the program and need to be handled consistently across different parts of an application (like logging, security, data handling etc.). SRP does not prevent a class from interacting with various parts of the system, but these interactions should be managed through delegation to other classes designed to handle those concerns, rather than the class itself handling multiple responsibilities.
#### Indications of SRP Violations ("Bad Smells")
- **Big Class**: A class that has grown too large is often a sign that it's taking on too much responsibility. Common metrics might include classes that have more than 200 lines of code or more than 15 methods or fields. These "big classes" are typically harder to maintain, understand, and debug.
#### Refactoring Strategy
- **Extract Class**: When you identify that a class is handling more than one responsibility, a useful refactoring strategy is to extract part of its functionality into a new class. This involves moving related fields and methods out of the original class into a new one that specifically handles that responsibility.
#### Benefits of SRP
1. **Code is Easier to Understand**:
   - Smaller, well-defined classes are easier to understand because their functionality is limited and focused. Developers can quickly grasp what the class is responsible for without needing to understand a broader context.
2. **Adding/Modifying Functionality Should Affect Few Classes**:
   - With SRP, changes tend to be localized. Adding a new feature or modifying an existing one should ideally affect only a limited number of classes. This reduces the chances of inadvertently impacting unrelated parts of the system.
3. **Minimized Risk of Breaking Code**:
   - Since each class has a single responsibility, the chances of breaking something in the class when making modifications are reduced. Changes in one part of the system are less likely to impact other parts, which leads to safer, more robust code development.
### Open/Closed Principle (OCP)
   - Software entities (classes, modules, functions, etc.) should be open for extension but closed for modification. 
   - This means you should be able to add new functionality without changing the existing code, which helps in maintaining old code safely while integrating new features.
- -> Idea: Modify behaviour by adding new code, not by changing old code
- The Open/Closed Principle is strongly related to the [[Information Hiding Principle]] 
#### Example
```java
for (Shape shape : ShapeList) {
    switch (shape.getType()) {
        case SQUARE:
            square.draw();
            break;
        case CIRCLE:
            circle.draw();
            break;
    }
}
```
- In this snippet, each time a new `Shape` type is added (like a triangle), the switch statement must be modified. This violates the OCP because it requires changes to existing code when adding new functionality.
##### Improved Code (Adheres to OCP):
``` java
for (Shape shape : ShapeList) {
    shape.draw();
}
```
- In this improved version, each `Shape` object knows how to draw itself. Here's how this design adheres to OCP:
	- **Open for Extension**: You can introduce new shapes into the program simply by creating new classes that implement the `Shape` interface or extend a `Shape` abstract class with a `draw` method.
	- **Closed for Modification**: The loop that draws the shapes does not change. No matter how many shapes you add, this code remains unchanged.
#### Benefits of Following OCP
1. **Reduced Testing**: Changes to existing code can lead to new bugs in previously tested and validated functionality. By extending without modifying existing code, the risk of affecting existing functionality is minimized.
2. **Improved Maintainability**: The code is easier to understand and maintain because modifications are made by adding new code, not by changing existing code that already works.
3. **Enhanced Scalability**: The application can grow by adding new functionality with minimal impact on the existing codebase.
### Liskov Substitution Principle (LSP)
   - Objects of a superclass should be replaceable with objects of its subclasses without affecting the correctness of the program. 
   - This principle ensures that a subclass can stand in for a superclass without errors, maintaining the integrity of the design.
#### Example
- This classic example illustrates a common violation of LSP. Mathematically, a square is a special type of rectangle, but if you model Squares as derived from Rectangles in an object-oriented program, you can run into problems.
- If you have a function designed to work with rectangles, it might set the width and height independently:
```java
void f(Rectangle r) {
    r.setWidth(5);
    r.setHeight(4);
    assert(r.getWidth() * r.getHeight() == 20);  // Expects area to be 20
}

```
- If `Square` extends `Rectangle`, and you pass a `Square` object to this function:
```java
f(new Square());

```
- The function `f` would fail because setting the width or the height of a square necessarily changes both dimensions, violating the function's expectation that width and height can be set independently.
#### Relation to Design by Contract (DbC)
LSP is closely related to Bertrand Meyer's Design by Contract (DbC), which stipulates:
- When redefining a routine in a derivative, you may only replace its precondition with a weaker one, and its postcondition with a stronger one.
##### Application in LSP:
- **Rectangle's setWidth Postcondition**: After `setWidth(w)`, the rectangle's width should be `w`, and its height should remain unchanged.
- **Square's setWidth Postcondition**: After `setWidth(w)`, both the square's width and height should be `w`, which actually changes the postcondition by also affecting the height—a clear violation under LSP if `Square` is a subclass of `Rectangle`.
#### Solutions for LSP Compliance
1. **Common Base Class**:
    - Rather than having `Square` inherit from `Rectangle`, both could inherit from a less specific base class like `Shape` that provides behavior applicable to all shapes without specific dimension behavior.
2. **Interface Segregation**:
    - Use interfaces to segregate behaviors that are not common among all subclasses, thus avoiding forcing a subclass to implement behaviors that don’t logically apply.
3. **Adjust Postconditions**:
    - Change the design so that the postconditions do not assume properties about dimensions that are not universally applicable (like dropping “height = height” from Rectangle’s postcondition).
### Interface Segregation Principle (ISP)
   - Clients should not be forced to depend on interfaces they do not use. 
   - This principle advocates for creating more, smaller, role-specific interfaces rather than a large, all-encompassing interface. 
   - It reduces the side effects of changes and improves the modularity of the code.
#### ISP in Detail
- **Lean Interfaces**: Each interface should remain small and focused on a single responsibility, similar to the Single Responsibility Principle but applied at the interface level.
- **High Cohesion**: Interfaces should be tightly focused around a set concept or functionality.
- **Avoid Interface Pollution**: Interfaces should not inherit or include methods that don't logically belong to them just to satisfy a particular use case or a specific subclass.
#### Refactorings
- **Extract Interface**: If an existing class does too many things, an interface for each functionality can be extracted. This ensures that clients that need only part of the functionality aren't forced to implement or depend on methods they don't use.
- **Extract Superclass**: Similar to extracting an interface but involves creating a more general class that contains only the functionality common to several subclasses.
#### Example
![[Pasted image 20240628122202.png#invert|400]]

#### Benefits of Following ISP
- **Decoupling**: Clients are not forced to implement interfaces they do not use. This reduces the dependencies within the application, leading to less coupling.
- **Ease of Maintenance**: Changes in one part of the system are less likely to affect other parts because the interfaces are more focused and segregated.
- **More Reusable Code**: Smaller, well-defined interfaces are more likely to be reusable across different parts of a system or even in different systems.
### Dependency Inversion Principle (DIP)
DIP consists of two main components:
#### A. High-Level Modules Should Not Depend on Low-Level Modules; Both Should Depend on Abstractions.
- **High-Level Modules**: These are classes or components that implement complex logic or higher-level operations. They orchestrate the operation of multiple lower-level modules to perform their tasks.
- **Low-Level Modules**: These modules handle more detailed, foundational operations such as data access or specific device controls.
- **Abstractions**: Typically represented by interfaces or abstract classes, abstractions provide a layer that high-level and low-level modules can both depend on, without having to depend directly on each other.
#### B. Abstractions Should Not Depend on Details; Details Should Depend on Abstractions.
- **Details (Concretions)**: These are concrete implementations of high-level modules and low-level modules.
- **Abstractions to Details**: The principle dictates that details should be designed based on abstractions rather than abstractions being tailored to fit the details. This part of the principle prevents the design from being too tightly coupled to specific implementations.
#### Example
- Violation of DIP:
![[Pasted image 20240628122448.png#invert|200]]
- In the first scenario with the `Copy` function, the design violates DIP by having the high-level `Copy` module directly depend on low-level modules (`ReadKeyboard` and `WritePrinter`). Changes in these low-level modules would necessitate changes in the `Copy` module.
- Improved Design (Adheres to DIP):
![[Pasted image 20240628122519.png#invert|200]]
- In the better design, `Copy` depends only on abstractions (`Reader` and `Writer` interfaces). This setup allows the substitution of different implementations (like reading from a file instead of a keyboard, or writing to a network stream instead of a printer) without altering the `Copy` module. This design adheres to both parts of DIP by depending on abstractions and ensuring those abstractions do not depend on specific details.
#### Benefits of DIP
1. **Flexibility**: Systems designed with DIP are more flexible. They can easily adapt to new requirements or changes in the environment by introducing new implementations of abstractions without altering the high-level modules.
2. **Reusability**: Modules that depend on abstractions are more reusable. They can work with any implementation of the interface they depend on, not just the one that was originally designed.
3. **Testability**: It's easier to test modules that follow DIP. High-level modules can be tested independently from their low-level dependencies by substituting those dependencies with mocks or stubs that also implement the required abstractions.
4. **Reduced Coupling**: DIP significantly reduces the coupling between different parts of a system. This reduction in dependency complexity leads to systems that are easier to understand, maintain, and scale.

## Example: Find the violations of SOLID principles:
![[Pasted image 20240628122600.png#invert|800]]
### Solution
![[Pasted image 20240628122618.png#invert|800]]
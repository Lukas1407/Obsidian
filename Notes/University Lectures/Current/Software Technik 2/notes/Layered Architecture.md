> [!abstract] Definition
> **Layered architecture** is a popular design paradigm used to structure applications into groups of subtasks at different levels of abstraction, thereby organizing the system into layers where each layer performs a specific role. 

![[Pasted image 20240701094354.png#invert|400]]


## Benefits of Layered Architecture
1. **Reduces "Accidental" Complexity**
   - **Simplification**: By dividing the system into manageable layers, each handling a specific aspect of the application (e.g., presentation, business logic, data access), the overall complexity of the system is managed more effectively. This separation allows developers to focus on one area without worrying about the details of other areas.
2. **Improves Modifiability**
   - **Flexibility**: Changes in one layer usually don't affect other layers. For instance, changing the data access logic won't affect the presentation layer, provided the interface between them remains consistent.
3. **Clear Separation of Concerns**
   - **Focus**: Each layer focuses on a distinct set of responsibilities. This clarity enhances maintainability and scalability as each layer can evolve independently based on its specific concerns.
4. **Independent Exchangeability**
   - **Replaceability**: Layers can be independently replaced or modified as long as they adhere to the same external interface. This is particularly useful when updating or enhancing specific parts of the system, such as swapping out a database or changing the business logic algorithms.
5. **Simplified Testing**
   - **Isolation**: Each layer can be tested independently, which simplifies unit testing and debugging. For example, the data access layer can be tested separately from the business logic layer.
## Drawbacks of Layered Architecture
1. **Increases the Number of Classes**
   - **Additional Complexity**: The need to interface between layers often results in a greater number of classes and interfaces, such as facades or data transfer objects (DTOs), which can add to the complexity.
2. **Overhead Through Facades or Data Transfer Objects**
   - **Facades**: Facade patterns are often used to provide a simplified interface to a complex subsystem of classes, which can add a layer of abstraction that might complicate the system if not well managed.
   - **Data Transfer Objects**: DTOs are used to transfer data between layers, particularly useful when data needs to be passed between the presentation layer and the business logic layer without exposing complex business logic or database code to the client side.
3. **Performance Concerns**
   - **Latency**: Each call from one layer to another can introduce latency, especially if not well-optimized. For complex operations that involve multiple layers, this can potentially lead to performance bottlenecks.
## Classifiction
**Layered Architecture** can be classified as both an [[Architectural Styles|architectural style]] and an [[Architectural Patterns|architectural pattern]]:
1. **As an Architectural Style**: Layered architecture is considered a style because it describes a fundamental structural organization with layers that are abstracted to handle different aspects of the application. It provides a high-level guideline on how applications can be decomposed into grouped functionalities such as presentation, business logic, and data access layers.
2. **As an Architectural Pattern**: It also qualifies as an architectural pattern because it offers a specific and repeatable solution to the problem of organizing an application. It prescribes how to structure the application so that responsibilities are well-defined and separated among the layers, promoting organized development and maintenance efforts.

## Layers
- **[[User Interface Layer]]**:
    - Also known as the Presentation or View layer.
    - Responsible for displaying information to the user and interpreting user commands.
- **[[Application Layer]]**:
    - Also referred to as Workflow, Process, Mediation, or App Controller.
    - Acts as a mediator between the UI layer and the domain layer, handling tasks such as application logic and user process orchestration.
- **[[Domain Layer]]**:
    - Also known as Business, Application Logic, or Model.
    - Central layer where most of the business logic and application rules are implemented.
    - This is where object design primarily happens, influenced by the domain model.
- **Business Infrastructure**:
    - Known as Low-level Business Services.
    - Provides utility services and components that support the business logic but are not business logic themselves.
- **Technical Services Layer**:
    - Also referred to as Technical Infrastructure or High-level Technical Services.
    - Encompasses infrastructure and services that support technical capabilities such as data access, messaging, and other cross-cutting concerns.
- **Foundation Layer**:
    - Known as Core Technical Services or Base Infrastructure.
    - Includes fundamental utilities and services required by other higher-level layers, such as logging, configuration management, and foundational libraries.
### Example
![[Pasted image 20240701095251.png#invert|800]]
#### Solution
![[Pasted image 20240701095503.png#invert|800]]
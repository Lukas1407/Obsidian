### Layers of Clean Architecture
**Clean Architecture** proposes that software should be organized into layers with each layer having distinct responsibilities and being separated from others by defined boundaries. This separation is crucial for several reasons:
1. **Separation of Concerns**: Each layer addresses a specific aspect of the application. For example, the presentation layer handles all UI logic, whereas the business logic layer deals only with the core functionalities of the application. This clear separation helps developers understand where code should be placed and how it should interact with other parts of the system.
2. **Independence from Frameworks and Technology**: By structuring the software so that business logic and other core functionalities do not depend on the UI or database, the system becomes less bound to specific technologies. This independence allows for easier updates and changes to technologies and frameworks without impacting the core business logic.
3. **Improved Testability**: When layers are well-separated, they can be tested independently. For instance, business rules can be tested without the UI, and database interactions can be tested separately from business logic. This not only simplifies testing but also ensures that tests are more reliable and less complex.

## The “Clean Architecture” Style
![[Pasted image 20240701114744.png#invert|400]]
The diagram depicts several concentric circles, each representing different layers or areas of the software, from the most fundamental at the center to infrastructure-related aspects at the outer layers:
1. **Entities (Yellow Circle)**:
    - **Description**: Entities encapsulate enterprise-wide business rules. They can be objects with methods or data structures with functions applied to them. Entities are the most internal part of the architecture and should not be dependent on anything external.
    - **Role**: They represent the application's business objects and rules that are the least likely to change when something external changes.
2. **Use Cases (Red Circle)**:
    - **Description**: Use cases contain application-specific business rules. They encapsulate and implement all the use cases of the system.
    - **Role**: They orchestrate the flow of data to and from the entities and direct those entities to use their enterprise business rules to achieve the goals of the use case.
    - **Implements System Use Cases**: It directly realizes the use cases of the system, detailing how user requests are processed and what outcomes are produced based on various business rules
    - Any modification in the way use cases operate would directly lead to changes in this layer’s code. This layer adapts to changes in business rules or processes while shielding the rest of the system from these changes, maintaining the stability of external interfaces and entities.
3. **Interface Adapters (Green Circle)**:
    - **Description**: This layer consists of adapters that convert data from the format most convenient for the use cases and entities, to the format most convenient for some external agency such as the database or the web.
    - **Components**: It typically includes Presenters, Views, and Controllers.
    - **Role**: Controllers accept input and convert it to commands for the model or view. Presenters take data from the model and format it for display in the view.
    - This layer effectively separates the use case layer from external concerns by mediating the data transfer, allowing the business logic to remain isolated from the format and the transmission requirements of external interfaces.
4. **Frameworks and Drivers (Blue Circle)**:
    - **Description**: This outermost layer is generally composed of frameworks and tools such as the Database, the Web Framework, etc.
    - **Role**: Its purpose is to provide concrete implementations for the interface adapters. It could include tools, external libraries, or frameworks used to keep the application running.
## Core Principles of Clean Architecture
1. **The Dependency Rule**:
    - This is a critical rule in Clean Architecture which states that source code dependencies can only point inwards. No code in an inner circle can know anything at all about the code in an outer circle. Specifically, the name of anything declared in an outer circle must not be mentioned by the code in the inner circle.
    - **Implication**: By adhering to this rule, the system becomes decoupled from externalities like UI frameworks or databases, making the system’s core easy to test and maintain.
### Example
![[Pasted image 20240701115707.png#invert|600]]
#### Solution
![[Pasted image 20240701115723.png#invert|600]]


## Benefit of Clean Architecture
### 1. Independence of Frameworks
- **Benefit**: Clean Architecture promotes the design of software systems that are not tightly coupled to specific frameworks or libraries. This independence allows developers to use the features of frameworks without being locked into their limitations.
- **Impact**: You can switch out frameworks relatively easily without significant changes to the core business logic. This flexibility reduces the risk of being stuck with outdated or unsupported technologies and makes it easier to adopt new advancements.
### 2. Testable Systems
- **Benefit**: One of the primary advantages of Clean Architecture is the ability to test business rules without any dependency on external elements like databases, web servers, or user interfaces.
- **Impact**: This enhances the testability of the system, allowing for more robust, reliable tests that run quickly and can be automated easily. Since the business logic can be tested in isolation, it ensures that the core functionality of the application is sound and works as expected under various conditions.
### 3. Independence of UI
- **Benefit**: The user interface (UI) can be changed or replaced without affecting the underlying business logic. This is possible because the business logic is decoupled from the UI layer.
- **Impact**: This flexibility allows the UI to evolve according to user feedback, accessibility standards, or new design trends without requiring changes to the core application logic. It also facilitates the development of different interfaces (web, mobile, desktop) for the same application without duplicating business logic.
### 4. Independence of Database
- **Benefit**: Business rules are not tied to any specific database vendor or technology. This is achieved by abstracting data access behind interfaces that the business logic uses.
- **Impact**: You can change database technologies as needed—for instance, from a relational database to a NoSQL database—without modifying the business logic. This capability is crucial for scaling, performance optimization, and cost management, as different stages of application growth may require different data storage solutions.
### 5. Independence of External Agency
- **Benefit**: Business rules do not know about or depend on external systems, whether they are third-party services, APIs, or other system components.
- **Impact**: This ensures that the core application is not affected by changes or failures in external systems. It also simplifies security implementations since the business logic is isolated from external threats. Moreover, it enables the business logic to be reused in different contexts or integrated with different applications, increasing the reusability and flexibility of the code.
## Path to Clean Architectures
![[Pasted image 20240701115932.png#invert|250]]
- Problem: dependency in opposite direction of Dependency Rule
### Solution
- **Dependency Inversion**: To address this, the [[SOLID Principle#Dependency Inversion Principle (DIP)|Dependency Inversion Principle]] dictates that both high-level and low-level modules should depend on abstractions (e.g., interfaces or abstract classes).
- **Implementation**: This is typically implemented by defining interfaces in a layer that both the higher layer and lower layer can depend on. Then, the lower layer (more concrete implementations) will implement these interfaces, and the higher layer will interact with these implementations through the interface abstraction.
**How it works**:
- High-level modules define interfaces that represent the operations they need from the low-level modules.
- Low-level modules implement these interfaces.
- The high-level modules then interact with these lower modules purely through the interface, not knowing about the concrete implementation.
![[Pasted image 20240701120055.png#invert|600]]
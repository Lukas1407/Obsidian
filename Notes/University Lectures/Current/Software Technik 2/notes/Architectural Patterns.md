> [!abstract] Definition
> Architectural patterns serve as templates for structuring software architectures, addressing common problems with established, generalized solutions. These patterns encapsulate best practices derived from the collective experience of software development professionals. Understanding these patterns and when to apply them can significantly aid in designing robust, scalable, and maintainable systems. 
## Architectural vs. Design Patterns
**1. Scope and Scale**
- **Architectural Patterns**: Concern the high-level structure of the system as a whole. They dictate the macro-organization of the software, including the way in which components are divided, how they interact, and how control flows between them. An architectural pattern impacts the system’s global structure and addresses concerns such as scalability, reliability, and availability.
- **Design Patterns**: Typically focus on solving specific problems in a component or the interaction between components. They are more localized and often address smaller-scale issues within a system, such as object creation, behavior delegation, or the structuring of classes.
**2. Rule of Thumb**
- A pattern is considered an architectural pattern if it influences or crosses the boundaries of architectural elements, affecting how major components of a system are organized and interact, like the Model-View-Controller (MVC) pattern that separates data, user interface, and control logic.
## Groups of Architectural Patterns
Architectural patterns can generally be categorized based on the system aspects they predominantly affect:
**1. Domain/Business Logic**
- These patterns deal with the structuring of the domain logic of the application. They help in organizing business logic in a maintainable and scalable way.
- **Example**: Domain-Driven Design (DDD) helps manage complexity by connecting the implementation to an evolving model of the core business concepts.
**2. Data Sources and Object-Relational Mapping**
- Patterns that facilitate the integration of databases with object-oriented programming languages.
- **Example**: Repository pattern abstracts the data layer, providing a cleaner separation and easier data access and management.
**3. Web Presentation and Session Handling**
- Concerned with the structure and management of user interfaces, especially in web applications.
- **Example**: Model-View-Presenter (MVP) pattern, where the presenter handles user input logic and acts as a middleman between the model and the view.
**4. Distribution and Concurrency**
- These patterns address the performance, scalability, and reliability of applications across distributed systems.
- **Example**: Client-Server pattern where client machines interact with a central server that handles business logic and state management.
**5. Basic Issues**
- Address fundamental software design issues applicable across various types of applications.
- **Example**: Layered architecture pattern organizes the software into layers with specific roles and restrictions, promoting an organized development environment.
### Examples of Architectural Patterns
- **[[Model-View-Controller (MVC)]]**: Separates an application into three interconnected components, allowing for efficient code reuse and parallel development.
- **[[Client-Server]]**: Manages the workloads between service requesters (clients) and service providers (servers), useful in networked applications.
- **[[[Blackboard]]]**: Used in problem-solving where no deterministic solution strategies are known by organizing a system to use an incremental building-up of a solution, with an architectural set-up that allows several specialized subsystems to contribute independently.

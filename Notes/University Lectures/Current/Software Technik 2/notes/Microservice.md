> [!abstract] Definition
> The concept of microservices is a distinctive and influential architectural style in modern software development. It emphasizes creating a single application as a suite of small, independently deployable services, each running in its own process. 

### Microservices Architecture
1. **Small, Focused Services**:
    - Each service in a microservice architecture is small, focused on doing one thing well, and represents a specific business capability.
2. **Independent Deployment**:
    
    - Microservices are designed to be independently deployable. This means that each service can be updated, deployed, and scaled without affecting other services in the application. This independence greatly enhances the agility of the development process, allowing for quicker iterations and deployments.
3. **Independent Scalability**:
    
    - Each microservice can be scaled independently based on its specific load or demand. For example, a user authentication service may require more resources during peak times compared to a service that processes background tasks.
4. **Diverse Technology Stack**:
    
    - Microservices can be written in different programming languages and use different data storage technologies. This allows teams to choose the best tools and technologies suited for their specific service’s requirements.
5. **Decentralized Data Management**:
    
    - Each microservice manages its own database, either different instances or entirely different types of databases. This decentralized approach to data management helps avoid the database integration bottlenecks typical in monolithic architectures.
6. **Lightweight Communication**:
    
    - Microservices typically communicate with each other using lightweight, stateless communication mechanisms such as HTTP/REST APIs. This simplifies the interaction between services but also requires careful design to handle partial failures and ensure data consistency.
7. **Team Autonomy**:
    
    - Microservices allow different teams to manage different services. Each team is responsible for the full lifecycle of their service—from development and testing to deployment and operations—encouraging a more decentralized approach to app development and management.

### Microservices vs. Monolith

- **Monolithic Architecture**: A traditional monolithic application puts all its functionality into a single process and scales by replicating the monolith on multiple servers. This can be simpler to develop and deploy initially but becomes increasingly complex to scale and maintain as the application grows and updates become more frequent.
    
- **Microservices Architecture**: In contrast, microservices distribute functionality across several smaller, independent services that communicate over a network. This approach offers greater flexibility and resilience but requires more sophisticated coordination and infrastructure management.
    

### Goal and Considerations

- **Goal**: The primary goal of adopting microservices is to decouple services to achieve independent development and deployment, which enhances agility and scalability.
    
- **Consideration**: While microservices offer significant advantages in flexibility and scalability, they are not always the best approach for every situation. The complexity of managing many independent services and the overhead of handling interservice communication can be challenging.

### Characteristics of a Microservice Architecture

1. **Componentization via Services**:
    
    - Microservices are organized as independently deployable services, each representing a specific piece of functionality. Unlike traditional monolithic components, which often include a mixture of user interface, business logic, and data access code, microservices focus solely on specific business capabilities.
2. **Organized Around Business Capabilities**:
    
    - Each microservice is centered around a single business capability, aligning services closely with specific business functions. This organization allows teams to understand better and respond to business needs quickly.
3. **Products Not Projects**:
    
    - Microservice architectures promote the view of software as a product rather than a project. This shift encourages ongoing development and support, focusing on continuous improvement rather than viewing software development as having a definitive start and end.
4. **Smart Endpoints and Dumb Pipes**:
    
    - Communication between services in a microservice architecture typically follows the principle of "smart endpoints and dumb pipes." This means the services handle the business logic and data transformations, while the communication mechanisms (pipes) remain as simple and as dumb as possible (e.g., HTTP, AMQP).
5. **Decentralized Governance**:
    
    - Rather than a strict, centralized governance model, microservice architectures enable individual teams to make decisions regarding the technology and design choices for their services. This flexibility enhances agility but requires strong coordination and communication standards.
6. **Decentralized Data Management**:
    
    - Each microservice manages its own database, allowing it to control its data schema and management independently of other services. This approach helps avoid data integration and consistency issues common in monolithic architectures but can introduce complexity in maintaining data integrity across services.
7. **Infrastructure Automation**:
    
    - Microservice architectures rely heavily on automated environments for testing, deployment, and scaling. Tools like Docker, Kubernetes, and continuous integration/continuous deployment (CI/CD) pipelines are integral to managing the lifecycle of microservices efficiently.
8. **Design for Failure**:
    
    - Given that microservices operate as a distributed system, designing for failure is crucial. This includes implementing strategies such as circuit breakers, fallbacks, and retries to handle failures gracefully and maintain service availability.
9. **Evolutionary Design**:
    
    - Microservice architectures are built with change in mind, allowing for continuous evolution of services as business needs change. This approach supports iterative updates and improvements without the need for large-scale redesigns.

### Componentization via Services

- **Services as Deployable Components**:
    
    - In microservice architectures, services are treated as deployable components that can be developed, deployed, and scaled independently. This independence is crucial for enabling the rapid iteration and robustness of the architecture.
- **Explicit Component Interfaces**:
    
    - Services define their interactions through explicit interfaces using mechanisms like RESTful APIs or messaging protocols. This explicitness ensures that services can interact seamlessly despite being developed by different teams using potentially different technologies.
- **Consequences of Using Components**:
    
    - The modular nature of microservices allows for high flexibility in deployment and scalability. Changes to one service generally do not impact others, reducing the risk associated with deployments and enabling faster innovation cycles.

### Organized Around Business Capabilities

The concept of organizing services around business capabilities in a microservice architecture directly aligns with **Conway's Law**, which posits that organizations design systems that mirror their own communication structures. In the context of microservices, this principle supports the idea that each microservice should represent a discrete business capability that reflects how the organization operates and communicates internally.

#### Key Points:

1. **Service Division**: Microservices are divided according to business functionality, with each service handling a distinct segment of the business. For example, an e-commerce application might have separate microservices for user management, product management, order processing, and payment processing.
    
2. **Autonomy and Isolation**: By focusing on specific business capabilities, each microservice can be developed, deployed, and scaled independently from others. This isolation minimizes dependencies and simplifies understanding each service's role within the larger system.
    
3. **Reflecting Organizational Structure**: According to Conway's Law, the architecture of the system (in this case, microservices) will reflect the organizational structure. This means that if a company is organized into departments that handle different aspects of the business, the microservices architecture will likely mimic this structure with services dedicated to each department's functions.
    

### Products not Projects

The shift from viewing software as a series of projects to treating it as a product is fundamental in microservice architecture. This approach changes how software is conceived, developed, and maintained over its lifecycle.

#### Key Points:

1. **Continuous Development**: Unlike projects that have a definite beginning and end, products are continually improved based on user feedback and business needs. This continuous development cycle is well-suited to microservices, where each service can evolve independently to better serve its business function.
    
2. **Personal Relationships**: The granular nature of microservices facilitates closer relationships between service developers and their end-users. This closeness allows for more immediate feedback and a deeper understanding of the user’s needs, which in turn can lead to more effective and user-centric service enhancements.
    
3. **Responsibility of Teams**: In many microservice environments, the teams responsible for developing services are also responsible for deploying, managing, and troubleshooting them — a practice known as DevOps. This integration of development and operations encourages greater ownership and accountability, leading to higher quality services and more stable deployments.

### Smart Endpoints and Dumb Pipes

#### Concept

This design principle suggests that in a microservices architecture, the complexity should be placed into the services themselves (the "endpoints"), while the communication between services (the "pipes") should be as simple and dumb as possible.

#### Characteristics

1. **Decoupling and Cohesion**: Each microservice is highly decoupled from others, focusing solely on fulfilling its specific business capability. This decoupling allows services to evolve independently of one another.
2. **Domain Logic Ownership**: Microservices own their domain logic and process data independently. They act like small, self-contained applications that handle everything from data receiving to processing and responding.
3. **Unix-like Filtering**: Microservices work similarly to Unix filters by receiving requests, processing them through internal logic, and producing responses. This streamlined process enhances clarity and efficiency.
4. **Simple Communication**: Communication between services is facilitated through simple, stateless mechanisms such as HTTP and REST. This simplicity ensures that the services remain loosely coupled and the architecture scalable.

### Decentralized Governance
Decentralized governance in microservices architecture allows individual teams to make decisions regarding the technology and practices best suited to their specific services. This independence encourages innovation and agility within teams.
#### Characteristics
1. **Tool Production**: Microservices teams often develop tools that can be reused by others within the organization. This approach leverages practical solutions that have proven effective in real-world applications.
2. **Shared Libraries and Tools**: Tools and libraries developed by one team are shared across the organization, promoting a culture of collaboration and reuse. This practice helps in maintaining consistency while allowing flexibility where needed.
3. **Evolving Service Contracts**: Microservices adopt patterns like the Tolerant Reader and Consumer-Driven Contracts to manage changes in service interfaces without breaking functionality. These patterns allow services to evolve their APIs independently while maintaining backward compatibility.
#### Applied Patterns
##### Tolerant Reader
- The Tolerant Reader pattern is about building services that are robust to changes in the data they consume. This pattern encourages services to ignore unknown fields and make minimal assumptions about the data structure. This approach is particularly useful in a microservices environment where different teams may evolve their service APIs independently.
**Benefits**:
- Increases the robustness of services against changes in the data format or structure sent by other services.
- Helps maintain compatibility even as services evolve, reducing the risk of disruptions due to tightly coupled dependencies.
##### Consumer-Driven Contracts
**Concept**:
- In Consumer-Driven Contracts (CDC), the consumers of a service participate in defining the expectations of the service’s output. This approach ensures that a service meets the actual needs of its consumers, rather than theoretical or outdated requirements.
**Operation**:
- Before a service is called, the consumer specifies what it needs from the service, which can include aspects of the data schema, service interfaces, and policies.
- The service then implements a contract that encompasses the union of all its consumers' needs, which it must satisfy.
**Benefits**:
- Ensures that services are developed with a clear understanding of consumer requirements, which enhances service usability and relevance.
- Facilitates independent service evolution while maintaining compatibility, as changes to the service are verified against the contract specified by the consumer.
### Decentralized Data Management
- Microservices architectures often decentralize data management, allowing each service to manage its own database. This could involve different instances of the same database technology or entirely different database systems tailored to the service’s specific needs.
**Challenges**:
- Managing consistency across these decentralized databases can be complex, especially in transactions that span multiple services.
- Ensures data isolation and autonomy but requires sophisticated strategies to maintain data integrity and consistency.
### Infrastructure Automation
- Infrastructure automation is integral to managing microservices, especially for teams practicing Continuous Integration (CI) and Continuous Delivery (CD).
**Practices**:
- Automation covers the provisioning, scaling, and management of the infrastructure. It includes automated setups for testing, deployment, and monitoring.
- Common tools include Docker for containerization, Kubernetes for orchestration, and Jenkins for continuous integration.
**Benefits**:
- Reduces the potential for human error in deployment processes.
- Supports frequent and reliable service updates and rollbacks, enabling rapid iteration and responsiveness to changes.
## Internals of Microservices
![[Pasted image 20240704080113.png#invert|400]]
### Components of the Microservice Architecture
1. **Resources**:
   - Act as the entry points for incoming requests.
   - They map application-level protocols to domain objects, performing initial request validation and generating appropriate responses based on the outcomes of business transactions.
2. **Service Layer**:
   - Coordinates high-level business logic and service operations.
   - Orchestrates the flow of data between the domain layer and repositories, managing the core functionalities of the application.
3. **Domain**:
   - Represents the business domain with a model comprising all the necessary business logic.
   - Domain objects handle specific business rules and interact closely with repositories to perform data operations.
4. **Repositories**:
   - Manage collections of domain entities and handle persistence operations.
   - Typically backed by a database or other persistent storage mechanisms, they abstract the complexity of data handling from the domain logic.
5. **Data Mappers / ORM (Object-Relational Mapping)**:
   - Facilitate interaction between the domain objects and the database.
   - Translate between the database's relational structure and the application's object model, simplifying data manipulation and queries.
6. **External Datastore**:
   - Stores data outside the microservice, usually in a separate database.
   - Accessed by the service through data mappers or ORM tools, managing state persistence beyond the lifetime of individual request transactions.
7. **Gateways**:
   - Act as brokers between the microservice and external services.
   - Encapsulate the logic for sending requests to and receiving responses from external services, handling data marshalling and network communication.
8. **HTTP Client**:
   - Utilized by gateways to manage the technical aspects of API communication, such as making HTTP requests and handling responses.
   - Ensures reliable interaction with external services, handling errors and retries as necessary.
### Interactions and Communications
- **Internal Communications**: Within the microservice, communications are typically fine-grained, involving frequent and detailed interactions between components like the domain, service layer, and repositories.
- **External Communications**: Interactions with external services and datastores are more coarse-grained to minimize network chattiness and latency. These communications must be designed carefully to ensure performance and resilience, given the risk of network issues and service outages.
### Design Considerations
- **Network Resilience**: Since external communications cross network boundaries, the architecture must be resilient to network failures. This includes implementing strategies for error handling, fallbacks, and retries within the gateways.
- **Testing**: The distributed nature of microservices and the presence of external communications require sophisticated testing strategies. Testing must account for variable network conditions, external service availability, and the potential for longer execution times.

## Clean Architecture and Microservices
- **Component as Microservice**: In the context of Clean Architecture, a "component" can be thought of as a microservice. Each microservice is a self-contained component that includes specific business capabilities, adhering to the principles of Clean Architecture by ensuring that dependencies are controlled and interactions are managed through well-defined interfaces.
- **Decoupled System Design**: Clean Architecture promotes the decoupling of software components, making it an ideal architectural style for microservices. This decoupling facilitates the microservices' ability to function, evolve, and scale independently without affecting the rest of the system.
## Design for Failure
- **Resilience by Design**: Microservices must be designed to handle failures gracefully. This involves strategies to detect, isolate, and recover from failures without user intervention.
- **Online Monitoring**: Effective monitoring is key in microservices architectures. It involves continuously checking both technical metrics (like database queries per second) and business metrics (like transaction volumes). This dual focus helps quickly identify and address issues before they affect the system's overall performance.
- **Resilience Testing and Tools**: Tools like Netflix's Simian Army are used to test the resilience of microservices by intentionally introducing failures (such as shutting down servers or severing network connections) to ensure that the system can handle and recover from real-life issues. These tools help validate the effectiveness of resilience strategies under controlled conditions.
- **Circuit Breaker Pattern**: This pattern is crucial in a microservices environment. It prevents a network or service failure from cascading to other parts of the system. Here’s how it works:
    - **Detection**: The circuit breaker detects failures and monitors for a threshold to be exceeded.
    - **Open State**: Once the failures reach a certain threshold, the circuit breaker trips, and the circuit opens, stopping the flow of calls to the failing service, preventing further strain.
    - **Fallback**: During this open state, fallback mechanisms can be triggered to maintain service availability, such as returning a default response or invoking a backup service.
    - **Recovery**: The circuit remains open for a predetermined period, after which it enters a half-open state to test if the underlying problem has been fixed. If the service is stable, the circuit closes; otherwise, it reopens until stability is restored.
## Evolutionary Design in Microservices
1. **Service Decomposition**:
   - **Purpose**: Allows developers to manage and adapt changes in applications more effectively by breaking the system into smaller, manageable pieces.
   - **Criteria for Decomposition**: Components are chosen based on their ability to be replaced or upgraded independently without impacting other parts of the system.
2. **Design Principles**:
   - **User Functionality Over Technology**: When slicing up an application into microservices, the focus is on user functionality rather than technological layers (as typically seen in n-tier architectures). This means services are defined around business capabilities.
   - **Independent Components**: The ideal component should be such that rewriting it does not affect its collaborators. This independence facilitates easier updates and maintenance.
3. **Versioning**:
   - **Avoid Manual Versioning**: Efforts are made to design systems that minimize the need for manual control over versioning by using techniques that allow services to be backward compatible or side-by-side operable.
## When to Use Microservices
#### Benefits
- **Strong Module Boundaries**: Enhances the modular structure, crucial for managing large teams by ensuring clear separation of responsibilities.
- **Independent Deployment**: Each microservice can be deployed independently, reducing the risk of system-wide failures due to a single service’s issues.
- **Technology Diversity**: Allows the use of different programming languages, frameworks, and databases within the same application, optimizing each service’s stack for its specific needs.
#### Costs
- **Distribution Challenges**: Distributed systems are inherently complex to program due to issues like latency and the unreliability of network calls.
- **Eventual Consistency**: Strong consistency is hard to achieve in distributed systems, requiring developers to handle eventual consistency, where data across the system may not be immediately consistent but achieves consistency over time.
- **Operational Complexity**: Requires a skilled operations team to manage numerous services, particularly with frequent deployments and updates.
### Graph Analysis: Microservices vs. Monolith Productivity
![[Pasted image 20240704080543.png#invert|600]]
- **Low Complexity**: Monoliths may be more productive due to their simplicity and less overhead in managing distributed components.
- **High Complexity**: Microservices tend to be more productive as they reduce coupling between components, making the system easier to manage despite the inherent complexities of each service.
### Conclusion
Choosing between microservices and a monolithic approach depends on various factors, including team size, system complexity, and operational capabilities. Microservices offer significant advantages in flexibility, scalability, and resilience but require careful management and sophisticated operational practices to handle their complexity and distributed nature. This decision should align with the long-term strategic goals of the organization and the specific requirements of the application being developed.

## How to begin a new Microservice Project
### Option 1: Start with a Clean Monolith, Separate Later
- **Simplicity and YAGNI (You Ain't Gonna Need It)**: This approach adheres to the principle of not building features or using technologies until they are actually needed. Starting with a monolith means focusing on building and validating business functionalities without the overhead of managing multiple services.
- **Unclear Service Boundaries**: Early in a project, it's often not clear what the boundaries between different parts of the application should be. Starting with a monolith allows the team to gain better insight into the domain and business needs, which can inform a more effective and informed separation into microservices later on.
- **Prepare with Clean Architecture**: Even within a monolith, applying principles of clean architecture (like decoupling components and maintaining clear interfaces) can prepare the system for a smoother transition to microservices in the future.
### Option 2: Start with Microservices
- **Enforces Modularity**: By starting with microservices, the project inherently adopts a modular structure from the beginning. Each service is built around a specific business capability, promoting better separation of concerns.
- **Avoids Future Messiness**: There's a common belief that monolithic applications tend to grow messy and entangled over time, making them hard to scale or adapt. Starting with microservices can preempt these issues by keeping boundaries enforced and dependencies minimized.
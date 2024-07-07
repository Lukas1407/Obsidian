
## Motivation
1. **Diverse Stakeholders**: In an e-commerce setting, stakeholders such as consumers, merchants, order managers, finance departments, card issuers, and payment processors each have distinct perspectives and terminologies. This diversity makes it challenging to establish a common language or domain model that accurately represents all needs.
2. **Lost in Translation**: The difference in language and perspective among stakeholders often leads to business rules and logic getting lost or misunderstood when translating real-world processes into software solutions.
### First Design Approach: Smart UI (Anti-pattern)
- **Smart UI**: This approach concentrates all functionality, including business logic, in the webshop’s interface. It typically involves delivering HTML directly from the frontend, with the database and backend logic closely integrated into the frontend layer.
- **Issues with Smart UI**:
    - Scalability and maintenance problems due to the tight coupling of user interface and business logic.
    - Difficulties in managing changes and updates, as business logic is scattered throughout UI components.
### Second Design Approach: Enterprise Architecture with Layered Design
- **Layered Architecture**:
    - **Presentation Layer**: Manages the graphical user interface and views for different stakeholders, acting as the face of the application.
    - **Domain Layer**: Central to DDD, this layer contains the domain model that encapsulates the business logic and rules. It represents the shared language and understanding of the business process across different parts of the application.
    - **Data Source Layer**: Handles data persistence and interactions with external services like payment providers.
- **Challenges with Layered Design**:
    - The domain model can become overly complex and cumbersome due to the broad range of concepts and use cases it needs to support, making it difficult to maintain and evolve.
    - Different team preferences and skill sets can lead to neglect of the domain model. For example, backend developers may focus on data source layers, while UI designers concentrate on the presentation, often leaving the critical domain model and business logic to less experienced team members like interns.
### Conclusion and DDD Benefits
Domain-Driven Design aims to address these issues by:
- **Focusing on the Core Domain**: Centralizing the business logic within a well-defined domain model that acts as a structured and organized representation of the business entities and their interactions.
- **Ubiquitous Language**: Establishing a common language derived from the domain model that is shared among all team members and stakeholders. This helps ensure that business rules are clearly understood and consistently applied across all layers of the application.
- **Bounded Contexts**: Decomposing the system into smaller, manageable parts (contexts), each with its own domain model and ubiquitous language. This helps manage complexity by allowing different parts of the system to evolve independently while adhering to their specific models and rules.

## Domain-Driven Design Overview
1. **Application Domain**: This is the specific area of interest or activity that the software addresses. In the case of a webshop, application domains can include user registration, order processing, accounting, and shipping.
2. **Domain-Driven Design (DDD)**: DDD is a methodology for developing software that deeply integrates the domain's complexity into the software's design, ensuring the language used in the code reflects the language used by domain experts.
### Key Steps in Domain-Driven Design
1. **Identify Bounded Contexts**: These are explicit boundaries within which a particular domain model applies. Each bounded context correlates to different parts of the organization or different application domains.
2. **Ubiquitous Language**: This is a common language developed for each bounded context, bridging communication between domain experts and developers. It helps avoid miscommunication and ensures the software accurately reflects domain concepts.
3. **Domain Model**: A conceptual model that solves specific domain-related problems. It's derived from the ubiquitous language and should be directly reflected in the software's code.
4. **Architecture**: Historically, a layered architecture was recommended for DDD to separate concerns effectively. Nowadays, many advocate for a clean architecture to better accommodate changing technologies and maintain the integrity of the domain model.
5. **Context Map**: This tool maps the relationships and interactions between different bounded contexts within the system. It helps manage complexity by clearly defining how different parts of the system interact and integrate with one another.
### Implementation in a Webshop Example
![[Pasted image 20240704113817.png#invert|400]]
In the diagram, various concepts like Customer, Order, Invoice, Payment, etc., are sketched as part of the webshop domain. These elements would be part of the ubiquitous language for the webshop's bounded context.
#### What could be Bounded Contexts?
- **Customer Management**: Handling customer information, registration, and authentication.
- **Order Processing**: Managing orders, from creation through processing to completion.
- **Payment Processing**: Handling transactions, payment validations, and billing.
- **Product Management**: Managing product listings, stock levels, and descriptions.
### Challenges and Benefits
- **Complexity Management**: By dividing the system into bounded contexts, DDD helps manage complexity by isolating domain-specific logic and behaviors. This isolation reduces dependencies and makes the system more maintainable.
- **Ubiquitous Language**: Ensures all team members—both technical and non-technical—use the same language, reducing the risk of miscommunication and ensuring that the software aligns closely with business needs.
### Bounded Context Explained
- **Definition**: A bounded context sets the boundaries within which a specific ubiquitous language is valid and where a particular domain model applies. It acts as a linguistic and conceptual boundary that encompasses all the terms, team agreements, and functionalities pertinent to a specific aspect of the domain.
- **Scope Determination**: The scope of a bounded context is determined by factors such as the application domain's specific needs (what part of the domain it covers), the organization of teams (how teams are structured around the project), and the physical manifestation of the system (how the codebase and database schemas are organized).
### Importance of Bounded Contexts
- **Consistency Enforcement**: Within a bounded context, the ubiquitous language and the domain model must be consistently applied. This means all terms, models, interactions, and integrations within the context must align with the agreed-upon models and language, without interference from external models or languages.
- **Focused Scope**: Teams should concentrate efforts and problem-solving within their bounded context, avoiding the dilution of focus by issues or requirements from outside the context.
### Relations and Context Maps
- **Context Maps**: These are diagrams or documents that detail how different bounded contexts interact, showing the relationships and dependencies between them. Context maps help teams understand where integration points exist and how data flows between contexts.
- **Shared Entities and Events**: The map will often highlight which entities (like 'Customer' or 'Product') are shared across contexts and how they might serve different roles or carry different data in each context.
### Practical Example
Consider a webshop with several bounded contexts:
- **Customer Management Context**: Manages customer data, authentication, and profile management. Here, the customer might be understood in terms of login credentials, profile settings, and personal preferences.
- **Order Processing Context**: Handles everything from the placement of orders to their fulfillment. In this context, the customer is viewed primarily through the lens of their transactions and interactions with the order system.
- **Product Management Context**: Deals with inventory, descriptions, and pricing. The products here relate closely to how they are presented in the catalog and how they are referenced in orders and line items.
#### Context Map Details:
- **Customer**: Shared across contexts like Customer Service, Order Processing, and Marketing. Each context uses different attributes of the customer—Customer Service might need access to service history, while Marketing would use demographic and preference data.
- **Products**: Exist in both the Catalog (for browsing and selection) and in Orders (as part of LineItems to be processed).
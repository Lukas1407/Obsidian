![[Pasted image 20240707085433.png#invert|500]]
The diagram and description you provided refer to **Strategic Design** within the Domain-Driven Design (DDD) framework. This approach focuses on the broader organizational and systemic aspects of software architecture. ### Domains and Bounded Contexts
- **Domains** represent different areas of business logic and functionality within a software system. Each domain focuses on a particular aspect of the business, such as user registration, product catalog, order management, etc.
- **Bounded Contexts** are specific boundaries within which a particular domain model applies. They encapsulate and define the limits of applicability for a particular model, ensuring that models within different contexts do not conflict with each other.
## Strategic Design Elements in the Diagram
- **Core Domain**: This is the primary focus of the business and system. In the provided diagram, the "Orders" context is part of the core domain, central to the business operations (managing orders).
- **Supporting Domain**: These are necessary for the business but do not constitute its primary focus. The "Catalog" context, managing product listings, supports the core domain but isn't the primary business driver.
- **Generic Domain**: These include functionalities common to many systems and do not give competitive advantage by themselves, such as "Accounting" and "User Registration."
## Context Map
- The **Context Map** visually depicts how different bounded contexts interact and relate to one another. It shows:
  - Dependencies between contexts.
  - Shared models or data.
  - Flow of information and control among various parts of the system.
- It also might define **Integration Patterns** such as:
  - **Published Interface**: Services from one context available to others.
  - **Translation Layer or Anti-Corruption Layer**: Mechanisms to prevent different domain models from corrupting one another, ensuring that interactions between contexts do not lead to adverse impacts on the integrity of a context's domain model.

## Strategic Design Elements
The concepts you've mentioned are central to **Domain-Driven Design (DDD)**, a software development approach focused on complexity management through the modeling of the business domain. Here’s an explanation of each:
![[swt5.png#invert|400]]
### Core Domain
- **Core Domain**: This is the most critical part of the application, central to the business strategy and its operations. It's where you should focus most of your efforts, as it encapsulates the primary business processes and rules that give the company its competitive advantage.
### Shared Kernel
- **Shared Kernel**: This refers to a common subset of the domain model that multiple teams or bounded contexts agree to share. It typically includes common models, systems, or functions that are integral across various parts of the application. Coordination and integration effort is necessary to maintain consistency in the shared kernel.
### Supporting Domain
- **Supporting Domain**: These are parts of the domain that are important but not core to the business's purpose. They support the core domain by handling necessary but non-central tasks, thus allowing the core domain to focus on business-critical operations.
### Generic Subdomain
- **Generic Subdomain**: This includes common functionalities that are not specific to the business and can often be handled by off-the-shelf solutions or outsourced services. Examples might include authentication, payment processing, or logging services.
## Interactions Between Domains
- **Consumer/Supplier Development Teams**: This pattern structures the relationship between teams where one team (supplier) provides a service or functionality used by another team (consumer). The consumer team depends on the supplier for certain aspects of their functionality.
![[Pasted image 20240707085742.png#invert|300]]
- **Conformist**: This pattern is adopted when a downstream team decides to conform to the model of an upstream team without alteration, reducing the complexity of integration but at the cost of some development freedom.
![[Pasted image 20240707085752.png#invert|300]]
- **Anticorruption Layer (ACL)**: This layer acts as a barrier and translator between different subsystems, protecting one domain from the noise and undesirable complexities of another. It prevents the upstream model from corrupting the downstream model.
![[Pasted image 20240707085807.png#invert|300]]
- **Open Host Service**: This approach involves creating a service with a defined protocol that allows external services (third parties) to interact with the application reliably and securely.
![[Pasted image 20240707085817.png#invert|300]]
- **Published Language**: This is a common language or protocol defined for communication between different bounded contexts or systems, ensuring that data exchanged is understood by all parties involved.
![[Pasted image 20240707085825.png#invert|300]]

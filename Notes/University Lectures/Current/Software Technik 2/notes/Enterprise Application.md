> [!abstract] Definition
> Enterprise Application Software are large-scale software solutions designed to support business operations and processes across organizations. They are integral to managing vast amounts of data, supporting complex business transactions, and facilitating user interactions across different departments and external entities. 

## Properties
- **Persistence of Data**: Data in enterprise applications outlives the applications and hardware themselves, necessitating robust systems for data integration and migration.
- **Volume of Data**: Enterprise applications often handle millions of records, requiring systems to provide efficient access and processing capabilities to manage large datasets effectively.
- **Concurrent Data Access**: Such applications need to support simultaneous access by potentially hundreds of users, which introduces complexities in ensuring data consistency and managing concurrent operations without performance degradation.
- **User Interface Diversity**: They must cater to a wide range of users, from novices to experts, necessitating diverse and intuitive user interfaces.
- **System Integration**: Enterprise applications frequently need to integrate with other internal and external systems via various mechanisms like REST APIs, SOAs (Service-Oriented Architectures), CORBA (Common Object Request Broker Architecture), or even through simpler batch files.
- **Varying Interpretations**: In large organizations, the same terms can have different meanings across departments, leading to challenges in ensuring uniform understanding and processing within the system.
- **Complex Business Logic**: The business rules and processes that these applications support are often complex and idiosyncratic. They might not always be logical from a software development perspective, and often these rules cannot be altered to suit the software; instead, the software must be flexible enough to adapt to these business needs.
## Examples of Enterprise Applications
- **Payroll Systems**: Manage employee compensation, benefits, and tax withholding.
- **Online Shops**: Handle product listings, customer orders, payments, and logistics.
- **Patient Record Management**: Manage patient data, treatment records, and billing information in healthcare settings.
- **Shipping Tracking**: Track logistics and delivery status of goods across various transportation modes.
- **Leasing Systems**: Manage agreements, payments, and records for property or equipment leases.
### Counter Examples
- **Telecommunication Systems**: More focused on network management and communication rather than typical business transaction management.
- **Word Processors**: Primarily for document creation and editing, lacking complex business process integrations.
- **Operating Systems**: Manage computer hardware and software resources, not directly involved in business process support.
- **Plant Controllers**: Involved in managing and controlling industrial operations, distinct from typical business data processing.
### Large Variety of Systems
#### 1. **Web Shop**
- **Users**: Potentially large number of concurrent users browsing and purchasing products.
- **Business Logic**: Relatively straightforward and well-established, involving shopping cart management, order processing, product availability, shipping, and payment systems.
- **User Interface Needs**: Must be adaptable to various user devices to ensure a broad reach and user accessibility.
#### 2. **Leasing Management System**
- **Business Logic**: Complex, dealing with calculations for monthly billing, managing scenarios like early returns or late payments, and validating leasing agreements.
- **Users**: Typically fewer concurrent users, but requires high reliability and accuracy given the financial and contractual obligations involved.
#### 3. **Expense Tracking for a Small Company**
- **Business Logic**: Simpler than the leasing system, focused on tracking company expenses.
- **Flexibility and Extensibility**: Important to accommodate potential future features like automatic reimbursements, updates to tax regulations, integration with reporting tools or external systems like airline reservations.
- **Constraints**: Tight time-to-market and budget, necessitating a solution that’s both cost-effective and quick to deploy.
## Layers of EA
The architectural layers described are common across many types of enterprise applications, providing a structured approach to development and maintenance:
### Presentation Layer (Front End)
- **Function**: Manages interactions between the user and the software.
- **Responsibilities**: Displays information to the user and interprets user commands. This layer ensures that the application's interface is user-friendly and accessible across different devices and platforms.
### Domain Layer (Middle, Business)
- **Function**: Handles the core work the application is designed to perform within the domain.
- **Responsibilities**: Includes performing calculations on data, deciding what data to load and store, and executing the business logic that drives the application processes. This layer is crucial for implementing the business rules and processes specific to the enterprise.
### Data Source Layer
- **Function**: Manages communication with other systems and services that the application needs to interact with.
- **Example**: Interactions with databases where the application’s data is stored and retrieved.

## Patterns in Enterprise Architecture
1. **Purpose of EA Patterns**:
    - EA patterns offer standardized solutions to common problems encountered in the layers of enterprise applications. These patterns streamline development by providing proven techniques that simplify complex decisions in software design.
2. **Scope and Specificity**:
    - **Layer-Specific Solutions**: Each pattern is typically specific to challenges found within a particular layer of an application, such as the presentation, business logic, or data access layers.
    - **Interactions Between Patterns**: Patterns often interact with one another across different layers, necessitating an understanding of how changes in one pattern may affect another.
3. **Pattern Families**:
    - **Family Approach**: A group of patterns that address a common problem but offer different approaches depending on specific needs or contexts.
    - **System Requirements Consideration**: The choice of a pattern depends on the specific requirements of the system being developed. Each pattern has its strengths and weaknesses, and selecting the right one involves weighing these factors against the system’s needs.
    - **Relational Decision Making**: Choosing a pattern is not an isolated decision; it needs to be made in context with other patterns used across the system to ensure compatibility and cohesion.
### Philosophical View on Patterns
1. **Starting Point, Not Final Solutions**:
    - **Practice-Based**: Patterns are derived from practical experiences and what has been found to work well in real-world scenarios.
    - **Abstract Solutions**: While they provide a framework, patterns abstract away from all the potential problems they solve, offering a "half-baked solution." This means they need to be adapted and fleshed out to fit specific situations.
    - **Multiple Options**: Often, several patterns might solve a problem, and the choice between them requires careful consideration.
2. **Patterns as Styles**:
    - **Stylistic Guidelines**: In enterprise architecture, patterns can be seen as stylistic approaches to solving design problems. They propose structured methods but should not rigidly dictate every aspect of the system’s design.
    - **Non-Exclusivity**: While a particular pattern should not be mixed within the context of solving a single problem (to maintain clarity and avoid conflicts), it is entirely appropriate to use different patterns for different problems within the same system.
### Domain Logic Patterns
![[Domain Logic Patterns]]
### Data Source Architectural Patterns
![[Data Source Architectural Patterns]]
### Object-Relational Structural Patterns
![[Object-Relational Structural Patterns]]
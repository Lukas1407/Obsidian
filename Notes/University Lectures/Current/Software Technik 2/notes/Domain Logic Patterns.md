> [!abstract] Definition
> The concept of **Domain Logic Patterns** specifically addresses how business logic is structured and implemented within an enterprise application. This subset of patterns is crucial because it directly handles the operations that define the core functionalities and business rules of the application. 
### Challenge Addressed
- **Complexity of Domain Logic**: Business logic often involves complex validations, calculations, and adherence to business-specific concepts. This complexity arises from the need to enforce business rules consistently and accurately across an application.
### Needs and Considerations
- **Changeability**: Business rules are not static; they evolve over time due to changes in business strategies, regulations, or market conditions. The domain logic must be adaptable to these changes without requiring a complete overhaul of the system.
- **Integration**: Domain logic needs to interact seamlessly with other parts of the system—namely, the presentation layer that handles user interaction and the data source layer that manages data persistence and retrieval.
- **Context-Specific**: The choice of how to implement domain logic depends greatly on the specific requirements and constraints of the system at hand. For example, a system that handles frequent changes in business rules may benefit from a more modular and easily configurable approach.
### Example Explained
#### **Recognize Revenues for a Contract**
- **Process Description**: This example involves recognizing revenues from contracts, which is a common requirement in financial and business systems. Here's how domain logic can be applied:
  1. **Identify the Product**: First, the system needs to determine which product or service is associated with a given contract.
  2. **Apply Calculation Algorithm**: Each product might have different rules for how revenue should be recognized. The domain logic must select and apply the correct revenue recognition algorithm based on the product type.
  3. **Create Result Object**: After the calculations are done, the system generates a result object that encapsulates the calculated revenue, which can then be used for reporting, analytics, or further processing.
### Key Domain Logic Patterns
To manage the complexities and requirements mentioned, several domain logic patterns can be utilized, each suited to different scenarios:
- **[[Domain Logic Patterns#Transaction Script|Transaction Script]]**: Organizes business logic by procedures where each procedure handles a single request from the presentation.
- **[[Domain Logic Patterns#Domain Model|Domain Model]]**: Uses an object-oriented model to represent complex and interconnected business data as well as the behavior associated with this data.
- **[[Domain Logic Patterns#Table Module|Table Module]]**: A single instance that handles business logic for all rows in a database table or view.
- **Service Layer**: Acts as a conduit between the presentation layer and the domain model, routing commands to the model to carry out business tasks.
Each pattern has its strengths and is chosen based on the specific needs of the application, such as the complexity of the domain logic, the frequency of changes to the business rules, and the integration needs with other parts of the system. 

### Transaction Script
The **Transaction Script** pattern is a structural design pattern often used in enterprise applications where business logic needs handling in simple and straightforward procedures. Each transaction script contains all the logic required for a specific business transaction encoded in a single procedure, from data retrieval and manipulation to the final action like data persistence.

The Transaction Script organizes business logic by creating scripts that process each transaction from start to finish. Here's how it typically works, using the context of a Recognition Service for revenue calculation:
1. **Single Procedure**: All the logic to handle a specific transaction (e.g., calculating revenue recognitions for a contract) is placed in a single method or function. This script performs all necessary steps sequentially: data fetching, processing, and update/write-back operations.
2. **Data Interaction**: The script interacts directly with the database or data source, typically through a gateway or DAO (Data Access Object). For example, it might start by retrieving contract details from the database.
3. **Logic Execution**: After fetching the necessary data, the script executes domain-specific logic to calculate what is needed, in this case, revenue recognitions based on the contract details.
4. **Data Update**: Finally, the script updates the database with the results of its calculations, such as inserting revenue recognition entries.
#### Diagram Explanation
![[Pasted image 20240704082101.png#invert|400]]
The diagram you provided illustrates a typical implementation of a Transaction Script:
- **Recognition Service**: This is the component where the transaction script resides. It includes a method `calculateRecognitions(contractID)` which embodies the transaction script.
- **Data Gateway**: Acts as the mediator between the database and the script, providing a simplified and abstracted interface to the database. Functions like `findContract(contractID)` fetch data, which the script uses to perform its calculations.
- **Contract Result Set**: The data needed for the transaction, fetched from the database, which is then used by the script to calculate and subsequently update the database through another gateway method like `insert revenue recognition`.
#### Advantages
- **Simplicity**: Transaction Scripts are straightforward to write and understand, making them excellent for simple business rules and when quick development is necessary.
- **Clear Boundaries**: These scripts clearly define transaction boundaries, making transaction management (commit or rollback) relatively straightforward.
#### Problems
- **Scalability**: As business logic becomes more complex, Transaction Scripts can grow large and unwieldy, making them harder to manage and understand.
- **Code Duplication**: There’s a risk of code duplication as similar procedures are required across different scripts, which can lead to maintenance challenges and inconsistencies.
- **Hard to Maintain**: With increased complexity, maintaining and extending transaction scripts can become difficult, particularly if the business logic changes frequently.
#### Use Cases
Transaction Scripts are best suited for applications with relatively simple business logic, where the overhead of a more complex Domain Model isn't justified. They are also useful in scenarios where an application starts small but might need to scale up gradually, allowing for refactoring to more complex patterns like Domain Model when and if necessary.
### Domain Model
The **Domain Model** is a foundational concept in software engineering, particularly within the context of object-oriented programming, where it serves as a structural representation of the business logic or the real-world business entities and the relationships between them. Here's a detailed explanation based on the description and the diagram you provided:

- **Artefact in Unified Process**: In software development, especially in methodologies like Unified Modeling Language (UML) or Unified Process, a domain model is a visual representation of conceptual classes or real-world entities within the domain of interest.
- **Domain Layer**: This is the part of an application where business logic is centralized. It represents the core functionality of the application, handling data and behavior that are critical to the underlying business.
- **Domain Model Pattern**: This design pattern involves building an object model of the domain that incorporates both behavior and data.
#### Diagram Explanation
![[Pasted image 20240704082325.png#invert|500]]
- **a Contract, a Product**: These represent entities in your domain model. In this context, a contract could be a sales agreement, and a product could be any item or service offered.
- **Recognition Strategy**: This is a strategy object that defines the algorithm for revenue recognition, which can vary depending on the type of product or the terms of the contract.
- **Revenue Recognition**: This is likely an object that represents the financial recognition of revenue as per accounting standards, which might be calculated and recorded over time as per the recognition strategy.
#### How It Works (Based on the Diagram)
1. **Contract**: The process begins with a contract entity which has a method `calculateRecognitions` that starts the revenue recognition process.
2. **Product**: Associated with the contract, it determines how recognitions are calculated, possibly based on the type of product.
3. **Recognition Strategy**: This object encapsulates the logic for how revenue recognitions are calculated based on the contract details.
4. **Revenue Recognition**: New instances of this object are created as a result of the calculations done by the recognition strategy, representing the actual entries of recognized revenue.
#### Advantages
- **Complex Logic Organization**: By embedding business logic within domain entities, the model closely aligns with real-world business operations and rules, making it easier to manage complex logic.
- **Intuitive Mapping**: The model provides an intuitive, one-to-one mapping between the business domain and the software implementation, reducing the representational gap.
#### Problems
- **Learning Curve**: For those unfamiliar with object-oriented design, the domain model pattern can be challenging to grasp and implement correctly.
- **Data Source Mapping**: Integrating this model with traditional relational databases can be complex, often requiring additional mapping layers like Object-Relational Mapping (ORM) frameworks.
#### Use in Software Development
The domain model is particularly useful in systems where business rules are complex and subject to change. It promotes flexibility and scalability in the development process by allowing developers to add, modify, or extend business entities and their interactions independently of other system components.
### Table Module
The **Table Module** pattern you're asking about is a design approach commonly used in applications where the business logic closely interacts with the database structure. Here’s an expanded explanation based on the pattern and the illustration you've provided:

- **One Class Per Table/View/Query**: In this pattern, a single class encapsulates the business logic for a specific database table or a collection of related data (like a view or a specific query).
- **Data Handling**: This class operates directly on data sets (e.g., record sets retrieved from the database), applying business logic to each row or collection of rows.
#### Functionality and Flavors
- **Data Set Instance**: Typically, each instance of a table module is linked to a dataset, which could represent a result set from a database query. This is common in environments like COM or .NET where data-binding and dataset manipulation are integral features.
- **Static Methods Version**: Alternatively, the table module pattern might employ static methods that act on provided datasets without maintaining state within object instances. This approach treats the data more like a structural table, without embedding object-oriented features like instance identity.
#### Diagram Explanation
![[swt1.png#invert|500]]
- **a Contract, a Product, a Revenue Recognition**: These are conceptual entities represented in the diagram. The diagram suggests a process where:
    1. **Contract and Product Data**: New instances of data sets are created, possibly representing rows or sets of rows from database tables corresponding to contracts and products.
    2. **Business Logic Execution**: Methods like `getProductType` and `calculateRecognitions` apply specific business rules. For instance, determining the product type and calculating revenue recognitions based on the type of product.
    3. **Data Insertion**: After calculations, the results (revenue recognitions) are inserted back into the database, updating or creating new entries.
#### Advantages
- **Simplicity**: The mapping to database structures is straightforward, making it easier to visualize and manage.
- **Separation of Concerns**: Each table module handles the logic for its respective data scope, segregating responsibilities clearly within the application architecture.
#### Problems
- **Limited Complexity Handling**: Since objects don’t maintain identities and complex state beyond the scope of individual function calls, it can be challenging to implement more sophisticated business logic.
- **Object Identity Lack**: The lack of object instances means that it is not suitable for scenarios where data needs to be treated as objects with behaviors and state across different operations.
#### Usage Context
The Table Module pattern is particularly useful in systems where database interactions dominate the application logic and where the application environment supports robust data set management (like in .NET with its DataSet class). It's less suitable for applications requiring rich domain models with complex behaviors and interactions between objects.

## When to use What?
![[Pasted image 20240704082907.jpg#invert|400]]
The graph you're referring to illustrates the relationship between the complexity of domain logic and the effort required to enhance the system using different architectural patterns:
- **Transaction Script**: Best for applications with simple domain logic. As complexity increases, the effort to enhance the system also rises steeply. This pattern centralizes all logic in a single procedure, which makes it simple initially but hard to maintain as complexities grow.
- **Table Module**: Suitable for applications with moderate domain logic complexity. This pattern organizes logic around tables in the database, making it easier to manage than transaction scripts when dealing with a moderate amount of business logic.
- **Domain Model**: Ideal for complex domain logic. This pattern initially requires more effort to set up but scales better as complexity increases. It's the most flexible in terms of handling complex business rules and behaviors due to its rich, object-oriented structure.
### Choosing a Pattern Based on Other Factors:
- **Complexity of Mapping to Data Sources**: If the data source integration is complex, a domain model may offer better abstraction and encapsulation, handling complex interactions more effectively than the other two.
- **Developer Familiarity with Domain Models**: If developers are familiar with object-oriented programming and domain modeling, leveraging a domain model can lead to better organized and more maintainable code.
- **Development Tools**: Some tools and frameworks are optimized for specific patterns. For instance, ORMs (Object-Relational Mappers) work well with domain models, while script-oriented environments might favor transaction scripts.
### Example System Recommendations:
- **Web Shop**: Given its likely simpler domain logic and high user traffic, a Table Module or Transaction Script might be more appropriate due to their straightforward nature and easier scalability across multiple database operations.
- **Leasing Management System**: Due to its complex business rules (like handling early returns, late payments, etc.), a Domain Model is recommended as it better supports complex logic and interactions within the system.
- **Expense Tracking System**: Depending on the scale and future requirements, either a Transaction Script for simplicity or a Domain Model for better handling of possible complex features could be suitable.
### Integration of Patterns:
In practice, different patterns can be combined within the same system to leverage their respective strengths according to the specific needs of different parts of the application. This approach allows for flexibility in development and maintenance, adapting to the changing needs of the business and the evolution of the software itself.
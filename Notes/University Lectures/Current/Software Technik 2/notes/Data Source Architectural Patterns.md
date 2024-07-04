These patterns provide structured approaches to decouple the domain logic from the data access logic, facilitating cleaner and more maintainable code.
- **Record Set**: Mimics a table of records and is typically used with data that fits a tabular format, directly reflecting results from SQL queries (e.g., JDBC ResultSet).
- **Table Data Gateway**: Acts as an object that abstracts the access to a single table or view. Methods on this gateway correspond to SQL queries or updates to the table.
- **Row Data Gateway**: Each object corresponds to a row in the database. Unlike Active Record, it doesn't contain domain logic, focusing only on the data transfer aspects.
- **Active Record**: Combines data access logic with domain logic. Each instance represents a row in the database, with properties mapped to the columns, and also encapsulates database access methods.
- **Data Mapper**: A layer of software that separates domain objects from database operations, enhancing SRP (Single Responsibility Principle) by allowing the domain model to focus on business logic without being burdened by data access code.
- **Identity Map**: Ensures that each object gets loaded only once by keeping a map of objects based on their database IDs. This helps maintain consistency across transactions and reduces database reads.
## Record Set
- A basic pattern, not a architectural pattern
A Record Set acts as an in-memory database table, containing rows and columns directly comparable to what you would expect from an SQL query result. It serves as a versatile data structure for managing tabular data within an application, allowing for operations such as selection, iteration, and even modification before any changes are sent back to the database.
### Key Characteristics and Usage:
- **In-memory Representation**: This means the data is loaded into the system's memory, resembling the structure of a database result set but is far more flexible for in-application processing.
- **Generated and Manipulated by All System Parts**: The Record Set can be accessed and manipulated by different components of an application, making it a central structure for data handling.
- **Framework Support**: Typically, Record Sets are not manually created from scratch; they are provided by data access frameworks like ADO.NET in .NET environments or JDBC in Java environments. These frameworks offer methods to execute queries and return data in the form of Record Sets.
- **Serialization Capabilities**: When Record Sets are equipped with serialization capabilities, they can be used for remote data access, acting like Data Transfer Objects (DTOs). This is particularly useful in distributed applications where data needs to be transmitted between different tiers or services.
### Example Code Snippet Explanation:
The provided code snippet illustrates how a `ResultSet` object in JDBC is used to query a database and iterate through the results:
```java
ResultSet rs = stmt.executeQuery(sql);
while (rs.next()) {
    int id = rs.getInt("id");
    String firstname = rs.getString("Fname");
    String lastname = rs.getString("Lname");
    int age = rs.getInt("age");
}

```
- **Execute Query**: `stmt.executeQuery(sql)` executes the SQL query and returns the result in a `ResultSet`.
- **Iterating Over Results**: The `while (rs.next())` loop iterates over each row of the result set.
- **Data Extraction**: Inside the loop, data from columns is retrieved using methods like `getInt` and `getString`, which take the column name as an argument.
### Considerations for Placing DB Access Code:
- **Separation of Concerns**: Database access code should ideally be placed in a repository or a data access layer. This separation helps in isolating the business logic from data access logic, making the system easier to maintain and scale.
- **Strongly Typed Record Sets**: Technologies like ADO.NET allow for the creation of strongly typed Record Sets, which can reduce runtime errors and enhance code clarity by ensuring type safety.
## Table Data Gateway
A Table Data Gateway acts as an intermediary between the application code and the database. It provides a simple, one-to-one mapping to a database table: each instance of a gateway handles all operations for a specific table in the database.
### Key Responsibilities
- **CRUD Operations**: The gateway typically includes methods for all basic CRUD (Create, Read, Update, Delete) operations. Each method corresponds to a specific SQL statement needed to perform these operations on the database table it represents.
- **SQL Encapsulation**: All SQL queries are encapsulated within the gateway. This centralizes the data access logic, making it easier to maintain and modify.
- **Parameter Mapping**: Inputs provided by the application (such as parameters for queries or updates) are mapped directly to SQL statements within the gateway methods.
### Gateway vs. Façade
- **Gateway**: As you mentioned, a gateway encapsulates access to an external system or resource. In the context of a Table Data Gateway, it provides access to a database table. The gateway is typically written by the client (developer using the gateway) to fit specific use cases within the application, tailoring the interface to the application’s needs.
- **Façade**: While similar to a gateway in that it simplifies interaction with a complex system, a façade usually abstracts a larger part of the system or a more complex API. It’s generally provided by the library or service developers to make using the service easier.
## Active Record
The **Active Record** pattern is a design pattern often used in object-relational mapping where each object represents a single row within a database table, encapsulating both the data access and the domain logic related to that data. Here’s a deeper look at the Active Record pattern:
### Concept
- **Encapsulation of Data Access**: Active Record objects directly handle their database interactions. This includes methods for CRUD operations (Create, Read, Update, Delete) directly embedded in the objects.
- **Domain Logic Integration**: These objects also encapsulate domain logic that pertains to the data they represent, allowing you to treat database rows as objects with behavior, not just data.
![[Pasted image 20240704084106.png#invert|200]]
### Characteristics
- **Isomorphism with Database Schema**: Each Active Record object mirrors a row in a database table, with object fields corresponding directly to table columns. This close alignment makes it straightforward to understand and map.
- **Static Finder Methods**: Active Record classes typically provide static methods that perform SQL queries and return instances of the class. These methods act as a bridge between the database and the object-oriented usage in the application.
### When to Use Active Record
- **Simple Domains**: Active Record is most suitable for applications with simple domain logic, where the domain logic can be neatly contained within the Active Record classes without leading to excessive complexity.
- **Direct Database Representations**: It works best when the database schema and the domain model can be directly mapped, making it easy to translate rows into objects and vice versa.
### Limitations
- **Complex Relationships and Inheritance**: Active Record is less suitable for domains with complex relationships or extensive use of inheritance. In such cases, the pattern can lead to large classes with mixed responsibilities (database access and complex business logic), which can become difficult to manage and maintain.
- **Scalability in Complexity**: As business logic becomes more complex, the Active Record pattern may lead to models that are hard to maintain and extend, particularly if they are tightly coupled to the database schema.
## Row Data Gateway
The **Row Data Gateway** pattern is a design pattern used in applications where database access and business logic need to be decoupled to manage complexity effectively. Here's an explanation of this pattern:
### Concept
- **Separation of Concerns**: Unlike the Active Record pattern where an object contains both data access logic and business logic, Row Data Gateway separates these concerns. Each instance of a Row Data Gateway corresponds to a row in the database table, but it does not contain business logic.
![[swt3.png#invert|200]]
### Motivation
- **Complexity Management**: Mixing database access with business logic in the same object can increase complexity, especially when different SQL variations are needed or when no abstraction layer like JDBC is used.
- **Testing and Refactoring Challenges**: Embedding database operations within business logic objects makes testing and refactoring more cumbersome and slower due to direct database access.
### Solution
- **Gateway Object**: Acts like a facade over a single database row. The object provides a simple interface to interact with data in a type-safe manner, mimicking the structure of the database record it represents.
- **Finder Classes**: For each table, there typically exists a corresponding "finder" class responsible for retrieving instances of Row Data Gateway objects. This class handles the creation of these objects and abstracts any SQL queries needed to fetch the data.
### Advantages
- **Code Generation**: Because Row Data Gateways align closely with the structure of the database, they are excellent candidates for code generation. Tools can automatically generate these objects based on database schema metadata.
- **Database Decoupling**: Since the business logic is not contained within the Row Data Gateway, changing the database schema does not necessarily require changes in the business logic layer. This separation allows for easier maintenance and updates.
### In Practice
- **Identity Map**: Often used alongside Row Data Gateways to improve performance by reducing database lookups. This pattern ensures that each row is loaded only once by keeping a map of retrieved objects.
- **Data Access Layer**: Acts as a clear demarcation line between the business logic and data access code. This setup shields the domain model from any changes in the database structure, thereby facilitating better maintenance and scalability.
## Identity Map
- Ensure that each object gets loaded only once by keeping every loaded object in a map. 
- Look up objects using the map when referring to them.
- Objects are retrieved from DB only once
![[Pasted image 20240704084359.png#invert|400]]

## Data Mapper
The "Data Mapper" pattern, as detailed in the diagram, is a software architectural pattern that acts as a mediator layer between the database and the in-memory objects used within an application. This pattern is essential in applications where the database structure and the domain model are independent of one another, helping to manage data translations between the two without the domain objects knowing about the presence of the database.
### Key Concepts and Workflow:
1. **Separation of Concerns**: Data Mappers ensure that the in-memory objects (e.g., domain objects like `Person`) remain independent of the database logic. They handle all interactions with the database, performing CRUD operations without exposing database details to the domain logic.
2. **Mapper Operations**:
    - **Finding Objects**: For instance, to retrieve a person from the database, a client would use the `find` method provided by the mapper instead of directly querying the database. This abstracts away the SQL queries from the business logic.
    - **Inserting, Updating, Deleting**: These actions are also managed through the mapper, maintaining a clean separation between the database operations and the domain logic.
3. **Mapping Logic**: The mapping between database tables and in-memory objects can be explicitly defined in the mapper, translating database rows into objects and vice versa. This includes handling different field types, managing relationships, and more.
4. **Benefits**:
    - **Flexibility in Object and Database Design**: Changes to the database schema do not require changes in the domain objects and vice versa, reducing the coupling between database and business logic.
    - **Reusability and Maintainability**: Since the database operations are centralized within the data mappers, they can be reused and maintained more effectively.
5. **Complexity Management**: The pattern is particularly useful in complex domains where the business logic necessitates a rich object model, which doesn't directly map to the database structure.
6. **Tool Support**: Modern implementations of the Data Mapper pattern often utilize frameworks that automatically handle the mappings based on annotations or configurations, reducing the need for manual mapper creation.
## When to use What?
The choice of data source architectural pattern largely depends on the complexity of the application's domain logic, the degree of coupling between the data layer and the business logic, and the architectural style employed. Here’s a breakdown of which data source architectural pattern could be suitable for each scenario described:
### 1. A Web Shop Employing Transaction Script
- **Suitable Pattern**: **Table Data Gateway**
   - **Rationale**: Transaction Script consolidates business logic into single procedures, where each procedure handles the entire process for a transaction. A Table Data Gateway simplifies access to data by providing a gateway to a database table and encapsulating all the SQL queries. This matches well with the straightforward nature of Transaction Script, where scripts directly interact with the database through simple calls.
### 2. A Leasing System Employing Domain Model
- **Suitable Pattern**: **Data Mapper**
   - **Rationale**: The Domain Model pattern involves complex business logic with a rich domain object model, which does not directly map to the database structure. The Data Mapper pattern is ideal here as it allows for the complex mapping between the domain objects and the database while keeping the domain model independent from the data source layer. It’s particularly effective for scenarios where domain logic is highly intricate and database schemas are subject to change.
### 3. An Expense Tracking System Employing Table Module
- **Suitable Pattern**: **Row Data Gateway**
   - **Rationale**: In systems employing the Table Module pattern, business logic is encapsulated within classes that represent a table or a set of related tables. The Row Data Gateway can serve well here by providing a fine-grained control over rows in the database tables, with one instance per row. It allows for a cleaner separation of concerns where each table row is managed individually, mirroring the structured approach of Table Module.
### Summary: When to Use What?
- **Transaction Script**: Best paired with **Row Data Gateway** for more explicit interface management or **Table Data Gateway** when working with frameworks that manage record sets.
- **Domain Model**: 
  - **Simple logic**: **Active Record** is sufficient.
  - **Complex logic with complex mappings**: **Data Mapper** is necessary.
- **Table Module**: Typically goes well with **Table Data Gateway**, especially if the environment heavily uses Record Set frameworks like .NET or JDBC.
- **Combination**: It's possible to combine patterns such as **Data Mapper** with **Gateways** to encapsulate access to tables treated as external services, which can enhance modularity and maintainability.

![[Pasted image 20240704091014.png#invert|400]]
The diagram you've provided showcases a typical object-oriented (OO) domain model featuring inheritance, which is a common scenario in domain-driven designs. This model presents a challenge when trying to persist such structures into a relational database (DB) due to the inherent differences between object-oriented and relational paradigms.

- The complexity of mapping an object-oriented structure to a relational database often involves several strategies, each handling the translation of OO features like inheritance and relationships to relational table structures.

## Single Table Inheritance
![[Pasted image 20240704091227.png#invert|600]]
The diagram you provided illustrates the **Single Table Inheritance** pattern applied to an object-oriented class hierarchy involving `Person`, `Contact`, `Researcher`, and `Industrial Representative`. This pattern maps these different classes, despite their varying attributes, into a single relational database table.
### Explanation of Single Table Inheritance:
- **Single Table Inheritance** stores all records for an inheritance hierarchy in one table. This table includes columns for all attributes of all classes in the hierarchy, whether or not a particular row (representing an instance) uses them.
#### In your diagram:
- The `Person` table includes columns for every attribute in the entire hierarchy:
  - **uid**: A unique identifier for each record, serving as the primary key.
  - **type**: A discriminator column that indicates the specific subclass to which each record belongs. This helps in determining how to instantiate objects and which fields are relevant.
  - **firstName**, **lastName**: Attributes inherited from the abstract `Person` class.
  - **emailAddress**: Introduced in the `Contact` subclass.
  - **ORCID**, **researchInstitute**: Specific to the `Researcher` subclass.
  - **company**: Specific to the `Industrial Representative` subclass.
### Advantages of Single Table Inheritance:
- **Simplicity**: The database schema remains simple with only one table to manage for the entire hierarchy.
- **No Joins Needed**: Retrieving an object doesn't require joins between different tables, which can improve performance when querying.
- **Flexibility in Schema Evolution**: Refactoring the domain model (like moving attributes between classes) does not necessitate changes in the database schema.
### Disadvantages of Single Table Inheritance:
- **Sparse Table**: Since the table contains columns for all class attributes, many columns will be empty (null) for rows where the class does not have those attributes, potentially leading to inefficient data storage.
- **Confusion and Maintenance Issues**: It can be confusing to have a table with many unused fields. Maintenance might become cumbersome, especially if the class hierarchy changes frequently.
- **Performance Issues**: As the table grows, the performance might degrade due to the size of the table and the overhead of managing indexes on many possibly sparse columns.
- **Loss of Performance**: Particularly in systems with heavy read/write operations, the large size of the table and the numerous indexes necessary to maintain performance can lead to frequent locking and potential bottlenecks.
### When to Use:
- Single Table Inheritance is best used when:
  - The subclasses do not significantly differ in terms of the number of unique fields.
  - The application does not suffer from having null values in the database.
  - The performance implications of having a large, sparsely populated table are not a significant concern.
## Class Table Inheritance
![[Pasted image 20240704091417.png#invert|600]]
The diagram you provided illustrates the **Class Table Inheritance** pattern applied to an object-oriented class hierarchy. This pattern is used to represent an inheritance hierarchy of classes where each class in the hierarchy corresponds to a separate table in the database.
### Explanation of Class Table Inheritance:
- **Class Table Inheritance** involves creating a separate database table for each class in the hierarchy. Each table contains columns for the attributes of the class it represents, along with a foreign key that links to the table of the parent class, if applicable.
#### In your diagram:
- **Person Table**: Contains common attributes for all persons, such as `uid`, `firstName`, and `lastName`. The `uid` serves as an automatic primary key.
- **Contact Table**: Includes the `emailAddress` along with a `personId` that serves as a foreign key linking back to the `Person` table.
- **Researcher Table**: Stores attributes specific to researchers, such as `ORCID` and `researchInstitute`, and links to the `Person` table via the `personId` foreign key.
- **Industrial Representative Table**: Holds details specific to industrial representatives like `company`, with `personId` linking back to the `Person` table.
### Advantages of Class Table Inheritance:
- **Relevance of Columns**: Each table contains only the columns relevant to its class, making the tables more understandable and preventing the wastage of space.
- **Mapping to Legacy Schemas**: It's easier to map a legacy database schema using this approach, as the database design closely mirrors the domain model.
- **Direct Schema Representation**: The relationship between the domain model and the database schema is straightforward and intuitive.
### Disadvantages of Class Table Inheritance:
- **Multiple Table Access**: Loading a complete object usually requires joining multiple tables, which can reduce performance due to the need for multiple queries or complex joins.
- **Schema Refactoring**: Modifications in the class hierarchy, such as moving fields between classes, require changes to the database schema, potentially making refactoring more cumbersome.
- **Normalization and Complexity**: While high normalization reduces redundancy and saves space, it can make the schema harder to understand and manage, especially with complex hierarchies.
- **Performance Concerns**: Frequent accesses to the supertype table (in this case, the `Person` table) can become a bottleneck, especially in large and complex systems.
### When to Use:
- **Class Table Inheritance** is suitable when:
  - The database schema needs to clearly reflect the domain model.
  - Each class has distinct attributes that do not overlap significantly with those of other classes in the hierarchy.
  - There is a need to minimize null values and ensure that table space is efficiently used.

## Concrete Table Inheritance
![[Pasted image 20240704091442.png#invert|600]]
The diagram you provided demonstrates the **Concrete Table Inheritance** pattern, which represents an inheritance hierarchy of classes with one table per concrete class in the database. This pattern is a variant of mapping object-oriented class hierarchies to relational databases, focusing on concrete classes rather than the entire class hierarchy.
### Explanation of Concrete Table Inheritance:
- **Concrete Table Inheritance** involves creating a separate table for each concrete class in the hierarchy. Each table includes columns for the attributes of the class it represents, including inherited attributes.
#### In your diagram:
- **Contact Table**: Serves as the primary table and includes common attributes such as `firstName`, `lastName`, and `emailAddress`.
- **Researcher Table**: Includes attributes from the Contact class (`firstName`, `lastName`, `emailAddress`) plus specific attributes (`ORCID`, `researchInstitute`).
- **Industrial Representative Table**: Similar to the Researcher table, it includes attributes from the Contact class plus the specific attribute (`company`).
### Advantages of Concrete Table Inheritance:
- **Self-contained Tables**: Each table contains all the data necessary for the concrete class it represents, which eliminates the need for joins when accessing data specific to a concrete class.
- **Simpler Queries for Concrete Classes**: Accessing data involves straightforward queries without joins, improving performance and reducing complexity for operations that target specific types of objects.
- **Direct Table Access**: Useful in applications where the database is accessed directly, as each table's structure is clear and tailored to specific needs.
### Disadvantages of Concrete Table Inheritance:
- **Refactoring Complexity**: Changes in the class hierarchy, such as moving attributes up or down the hierarchy, require modifications to all relevant tables, which can be cumbersome and error-prone.
- **Redundancy**: Each table duplicates the inherited attributes, which can lead to data redundancy and an increase in storage requirements.
- **Superclass Queries**: Queries involving superclass entities need to check multiple tables (one for each subclass), which can complicate queries and reduce performance, especially if there are many subclasses.
### When to Use:
- **Concrete Table Inheritance** is suitable when:
  - Each concrete class operates independently, and there's little need to treat objects polymorphically.
  - The application benefits from direct and straightforward database table access without the overhead of joining tables.
  - The database schema and the application are stable with little expectation of frequent refactoring involving the class hierarchy.

## When to use What?
The advice on "When to use what?" regarding inheritance patterns in databases—especially when employing a Domain Model with a Data Mapper—is rooted in balancing trade-offs between data duplication, database structure integrity, and query performance. Here’s a breakdown of how to choose among Single Table Inheritance, Class Table Inheritance, and Concrete Table Inheritance based on these factors:
### Single Table Inheritance
- **Use when**: You have a simple hierarchy with few nullable fields and performance is a priority over data normalization. This approach minimizes the number of joins and simplifies queries.
- **Considerations**: This pattern can lead to sparse tables with many null fields if your subclasses have many unique fields, leading to potential waste of space and confusing schema.
### Class Table Inheritance
- **Use when**: Each class in the hierarchy has distinct attributes that justify separate tables, but they also share common attributes sufficiently to warrant a clear hierarchical mapping. This pattern is ideal when database schema clarity and data integrity are more critical than query speed.
- **Considerations**: This pattern requires joins to reconstruct objects from the database, which can impact performance. It's suitable for applications where the clarity of the database schema and alignment with the domain model are more important than optimal query performance.
### Concrete Table Inheritance
- **Use when**: You need the highest performance for accessing concrete classes and can manage the redundancy of having separate tables for each class, each duplicating the full set of inherited attributes.
- **Considerations**: It's highly efficient for direct data operations on concrete classes but can become cumbersome if the hierarchy changes or if operations across the hierarchy are common, as it may require complex joins or unions.
### When to Mix Patterns
- **Strategic Mixing**: Start with Single Table Inheritance for simplicity and speed, especially in early development stages or when the class hierarchy is not deep. As the application grows or if certain branches of the inheritance tree become complex, consider implementing Class or Concrete Table Inheritance where necessary to better manage data integrity and reduce table sparsity.
- **Incremental Refactoring**: As applications evolve, you may find that certain parts of your database schema become inefficient. Gradually introduce more appropriate inheritance patterns in these areas to optimize data access patterns and application performance.
### Using Commercial Tools
- **Leverage Existing Solutions**: Rather than building your custom solutions for database handling, consider using commercial or open-source ORM (Object-Relational Mapping) tools. Tools like Hibernate, Entity Framework, or Sequelize can automatically manage these patterns and handle much of the complexity associated with database interactions.
- **Understanding Implications**: Even if using commercial ORM tools, a solid understanding of these patterns is crucial. It helps you choose the right tool and configure it effectively to suit your application’s needs, ensuring optimal performance and maintainability.


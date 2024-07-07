![[swt4.png#invert|500]]
In Domain-Driven Design (DDD), building blocks like entities and value objects are essential for encoding domain concepts into the implementation of a software system. They serve to bring clarity and enforce the domain model within the code structure. 
## Entities
- **Definition**: An entity is an object that is not primarily defined by its attributes but by a "thread of continuity and identity."
- **Characteristics**:
  - **Identity**: Entities have a unique identifier that distinguishes them from other entities, even if other attributes are identical. This identity is established upon creation and is maintained across the life cycle of the entity.
  - **Continuity**: Entities are traceable and accountable throughout their lifecycle within the application, which means changes to an entity's attributes do not change its fundamental identity.
  - **Importance**: Entities are crucial for scenarios where the identity of objects matters in the domain, such as users, orders, or products in a webshop.
### Example in Implementation:
  - In Java, entities are often annotated with `@Entity` to signify their role in the domain model.
  - An identity field (like `id`), typically managed by the JPA with `@Id` and `@GeneratedValue`, ensures each entity instance can be uniquely identified.
  - Entities may have relationships with other entities or value objects, denoted with annotations like `@OneToOne`.
## Value Objects
- **Definition**: Value objects are objects that do not possess a conceptual identity and are defined only by their attributes.
- **Characteristics**:
  - **No Identity**: Value objects are interchangeable when their attributes are identical. They do not require a unique identity.
  - **Immutability**: Value objects should be immutable, meaning once they are created, their state cannot be altered. Any change requires the creation of a new value object.
  - **Usage**: They are used to describe characteristics of things, such as a quantity, monetary amount, or dimensions.
### Example in Implementation:
  - In Java, value objects can be annotated with `@Embeddable`, which allows them to be embedded directly in an entity's table, reflecting their intrinsic relationship to the entities they describe.
  - The immutability and non-null constraints ensure that the value object's integrity is maintained, typically enforced through constructor validation.
  - Operations on value objects (like arithmetic operations) return new value object instances rather than modifying existing instances.

## Services
- **Definition**: Services in DDD represent operations or functionalities that are important within the domain but do not logically belong to any particular entity or value object.
- **Characteristics**:
  - **Domain Functionality**: Services encapsulate important domain actions that don't fit neatly into the lifecycle or attributes of an entity or value object.
  - **Statelessness**: Services are stateless, meaning they do not hold any state between calls. This characteristic makes them ideal for operations that apply universally, regardless of the specific instance of an entity.
  - **Defined by Operations**: The interface of a service is defined by the operations it performs, often in terms of entities and value objects it manipulates.
  - **Distribution and Access**: Stateless services are easier to manage and distribute across different parts of an application or across different systems.
### Examples of Services
- **In a Webshop Application**:
  - **Customer Manager**: Manages customer details, registrations, updates, etc.
  - **Order Manager**: Handles the logic for order placements, cancellations, payments, and completions.
  - **Accountancy**: Manages financial transactions, billing, and invoice generation.
- **Implementation Considerations**:
  - **JavaEE**: Services can be implemented as stateless session beans annotated with `@Stateless`. This approach utilizes the JavaEE framework to handle the lifecycle and state management of service instances efficiently.
  - **Singleton Pattern**: In environments without the support of a container like JavaEE, services can be implemented using the Singleton design pattern, ensuring that only one instance of the service exists throughout the application.
## Practical Decisions in Modeling Services
- **Dependency on Domain Knowledge**: The decision to model a concept as a service or as another element in the domain model (like an entity or value object) heavily relies on domain knowledge and the specific requirements of the domain.
- **Performance Considerations**: Sometimes, managing complex operations across many entities can lead to performance penalties, and using a service can mitigate this by isolating heavy operations.
### Example: Address
- **Webshop Domain**: Address might be better modeled as a **value object** because multiple customers can share the same address without affecting the continuity or identity of the address itself.
- **Postal Service Domain**: Address might be modeled as an **entity** because it plays a crucial role in the management of postal zones and addresses. Each address has a unique identity within the postal system, and changes to the postal zone can affect all addresses within that zone.
## Aggregates
- **Definition and Purpose**: Aggregates define a cluster of associated objects that are treated as a single unit for data changes. They ensure the consistency of changes being made within the boundary of the aggregate by enforcing invariants that must hold for the aggregate to be in a valid state.
- **Components**:
  - **Root Entity**: This is the only member of the aggregate that outside objects are allowed to hold references to. It acts as the gatekeeper to the aggregate, ensuring that all interactions occur through it.
  - **Boundary**: The boundary of an aggregate defines what is included within the aggregate. Only objects within this boundary can be directly associated with the root entity.
  - **Invariants**: These are the rules that must always be true for the aggregate. The root entity is responsible for enforcing these invariants across the entire aggregate.
### Example: Purchase Order Aggregate
- **Root Entity**: The Purchase Order itself.
- **Boundary**: Includes entities such as Line Items and possibly references to Products.
- **Invariants**: For example, the sum of the amounts of the line items should not exceed the approved limit of the purchase order.
### Detailed Explanation of Your Example
In your diagram, the Purchase Order is shown as the root entity of the aggregate, with a constraint that the total cost represented by the Line Items must not exceed the Purchase Order's approved limit. This is an invariant of the aggregate. The Line Item, which references the Product for price and maintains its own quantity, is part of the aggregate because it directly contributes to this invariant.
![[Pasted image 20240707084609.png#invert|400]]
- **Line Item**: Represents the quantity of a specific product ordered. It calculates its total cost by multiplying the Product's price by its quantity.
- **Product**: Provides the price. Although part of the calculation for the Line Item, the product itself may not be included within the boundary of the Purchase Order aggregate if it is a global entity used by other parts of the system as well.
### Benefits and Strategies
- **Consistency**: Changes within the aggregate are controlled through the root entity, making it easier to ensure consistency.
- **Simplification of Complex Interactions**: By controlling how modifications are made within an aggregate, the system's overall design can be simplified since external objects only interact with the root.
- **Deletion Rules**: Deleting the root entity often cascades to delete all associated entities within the aggregate, simplifying the maintenance of database integrity.
### When to Use Aggregates
Aggregates are particularly useful when a cluster of objects is always used together, and when there are clear rules governing the state of those objects collectively. Deciding where to draw the boundary of an aggregate often involves understanding how the application is used and what consistencies must be enforced. This understanding helps in delineating which elements should be included within an aggregate to enforce the business rules effectively.

## Factories
- **Purpose**: Factories handle the instantiation of complex objects or aggregates without exposing the intricacies of their instantiation and initial state configuration to the client.
- **Encapsulation**: By encapsulating the construction logic, factories maintain the separation of concerns within the application. They allow the rest of the system to remain ignorant of the details involved in creating fully initialized, valid objects.
### Key Roles of Factories
1. **Simplifying Object Creation**: They simplify the client code that needs complex objects or aggregates by providing a clear and concise interface for their creation.
2. **Ensuring Invariants**: Factories ensure that created objects comply with business rules right from the moment they are created. This is crucial for maintaining the integrity of the domain model.
3. **Atomic Construction**: Factories often construct complex aggregates or related objects in a single, atomic operation, ensuring all parts of the system are consistent from the start.
### Example: Customer Factory in a Webshop Application
- **Context**: In a webshop, the creation of a `Customer` object might involve setting up associated objects like `UserAccount`, assigning roles, and ensuring that the customer's account details are correctly initialized.
- **Implementation**: This might be implemented in a Java Spring application as a `@Service` annotated class, where each method in the factory encapsulates the creation logic.

```java
@Service
@Stateless
public class CustomerFactory {
    public Customer createCustomer(RegistrationForm form) {
        var password = UnencryptedPassword.of(form.getPassword());
        var userAccount = createUserAccount(form.getName(), password, CUSTOMER_ROLE);
        return new Customer(userAccount, form.getAddress());
    }

    private UserAccount createUserAccount(String name, UnencryptedPassword password, Role role) {
        // Logic to create a UserAccount
    }
}
```
### Benefits of Using Factories
- **Decoupling**: Clients are decoupled from the creation logic of the objects they need, leading to cleaner, more maintainable code.
- **Flexibility**: It's easier to change the creation logic without affecting clients.
- **Consistency**: Factories can enforce business rules or invariants right at the object's point of creation, reducing bugs or inconsistencies in the application state.
### When to Use Factories
- **Complex Object Graphs**: When the creation of an object involves setting up an object graph with various interdependencies.
- **Detailed Setup**: When an object requires non-trivial initialization that should not be exposed outside its boundary.
- **Reusability**: When the creation logic needs to be reused across different parts of the application.

## Repositories
**Repositories** play a vital role in Domain-Driven Design (DDD) by managing the lifecycle of domain entities and value objects beyond their creation. They simulate a collection interface to domain objects that reside in the domain model but are often persisted externally, typically in a database.
### Key Roles of Repositories:
1. **Abstraction**: They abstract the underlying persistence mechanism to provide a collection-like interface for accessing domain entities.
2. **Collection Semantics**: Repositories allow you to treat the set of objects as a collection, providing methods to add, remove, and update entities.
3. **Query Capability**: They encapsulate complex querying capabilities, allowing for efficient searches and operations on the data set.
### Example: Repositories in a Webshop Application
In a webshop application, you might have several repositories such as:
- **Catalog Repository**: Manages all product catalog entries.
- **Order Repository**: Manages customer orders.
- **Accountancy Entry Repository**: Manages financial records related to transactions and balances.
```java
@Repository
public interface Catalog {
    @Query("select p from catalog p where :category member of p.categories")
    Streamable<Product> findByCategory(String category);

    @Query("select p from catalog p join p.categories c where c in :categories")
    Streamable<Product> findByAnyCategory(Collection<String> categories);

    @Query("...")
    Streamable<Product> findByAllCategories(Collection<String> categories);
}
```
This example uses Java Spring's `@Repository` annotation to denote that the `Catalog` interface is a repository, handling operations related to product catalog management. The queries are defined using the `@Query` annotation, simplifying the access to the database and ensuring that the domain logic remains separate from the data access logic.
## Modules
**Modules** are another core concept in DDD, used to structure the domain model into logical segments that reflect the ubiquity and integrity of the domain itself.
### Key Roles of Modules:
1. **Organization**: Modules help organize the domain model by dividing it into manageable segments that are relevant to specific aspects of the domain.
2. **Cohesion**: They ensure that elements within a module are closely related and functionally cohesive.
3. **Decoupling**: Modules are designed to be as decoupled as possible from one another, minimizing dependencies across different areas of the application.
### Example: Modules in a Webshop Application
Modules might correspond to major areas of functionality or bounded contexts in the application:
- **Catalog Module**: Manages all aspects of the product catalog.
- **Orders Module**: Handles order processing and management.
- **User Registration Module**: Manages user accounts and authentication.
These modules are often mapped to packages in an object-oriented programming environment like Java, although the domain-driven module organization goes beyond mere package structuring by emphasizing the relationship and roles of domain entities and services within the bounded context.
### Challenges with Modules
- **Technical Overhead**: Sometimes, the technical requirements or framework constraints can complicate the domain model with unnecessary technical details.
- **Evolution and Refactoring**: As the domain evolves, modules and packages might need to be refactored to reflect new insights or changes in the domain, which can be neglected due to the effort involved.

## Example: Classify each class as either Entity, Value Object or Service
![[Pasted image 20240707084914.png#invert|600]]
Let's classify each class in the diagram of the Conference Management example according to Domain-Driven Design concepts—whether they are an **Entity**, a **Value Object**, or a **Service**:
1. **Event**:
   - **Entity** - Because it has a distinct identity (an event is uniquely identifiable by its name, year, start, and end date).
2. **Venue**:
   - **Entity** - It has a unique identifier and important attributes that define its uniqueness (like name and address).
3. **Person**:
   - **Entity** - Identified by unique attributes such as ORCID, and serves as a key actor in the system with properties like name, affiliation, and email.
4. **Registration**:
   - **Entity** - As it involves a significant business process (registering for the event), it has a unique identity and life cycle within the domain.
5. **Bill**:
   - **Value Object** - It represents a set of attributes (billing address, description, amount, tax number) that define a monetary charge but does not have a distinct identity separate from the transaction it's associated with.
6. **Stay**:
   - **Value Object** - Represents a period of stay tied to a specific booking (arrival and departure dates). It is defined by its attributes and does not need a separate identity.
7. **RegistrationProcess**:
   - **Service** - Encapsulates the process of registration (such as `register(person, venue)`), acting as a transaction script that doesn't maintain state between executions and operates on Entities and Value Objects.
8. **Hotel**:
   - **Entity** or **Value Object** - This could be an Entity if each hotel needs to be distinctly identified and managed within the system. It could be treated as a Value Object if the system only needs to track hotels as part of accommodation packages without managing them as distinct entities.
9. **BookingService**:
   - **Service** - Manages the logic of booking accommodations, interacting with entities but maintaining no state of its own, focusing instead on operations like checking availability and booking rooms.

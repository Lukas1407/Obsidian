> [!abstract] Definition
> The Model-View-Controller (MVC) pattern is a widely used architectural pattern for developing user interfaces that divides an application into three interconnected components. This separation helps manage complexity in application development by enabling efficient code reuse and parallel development. 

![[Pasted image 20240701100351.png#invert|400]]
### 1. **Model**
- **Role**: The Model represents the data and the business logic of the application. It is the central component of the pattern, responsible for managing the rules, data, and logic of the application.
- **Responsibilities**:
    - **Data Management**: It stores data that is retrieved according to commands from the controller and displayed in the view.
    - **Data Update**: It updates data based on user actions and notifies views when data changes (in some implementations).
    - **Business Logic**: It includes computational logic and business rules that handle data exchange and updates based on application-specific operations.
- **Characteristics**: The model does not depend on the views or controllers. This separation allows the model to be independent and reusable without regard to how data is displayed or modified externally.
### 2. **View**
- **Role**: The View component is used for all the UI logic of the application. It presents data to the user and sends user commands (e.g., button clicks) to the controller.
- **Responsibilities**:
    - **Data Display**: It renders data from the model into a form suitable for interaction, typically a user interface.
    - **User Input**: It captures user inputs and actions and sends these to the controller for processing.
- **Characteristics**: A view is often designed as a template that is updated dynamically with data from the model. There can be multiple views for a single model for different data presentations.
### 3. **Controller**
- **Role**: The Controller acts as an intermediary between the model and the view. It listens to events triggered by the view (or another external source) and executes the appropriate response, which usually involves calling a method on the model.
- **Responsibilities**:
    - **Event Handling**: It receives input from users via the view, processes it (possibly updating the model), and returns the updated view or another appropriate response.
    - **Model Updates**: It decides what data of the model should be changed based on user inputs, then updates the model.
    - **View Selection**: It selects a view for response, and provides it with the model instance for rendering output. In some implementations, the controller updates the view directly (push model), while in others, the view requests new data from the model (pull model).
### Benefits of MVC
1. **Separation of Concerns**: It divides the application into three components with clear responsibilities, making it easier to manage complexity.
2. **Reusability and Scalability**: By decoupling data access and business logic from data presentation and user interaction, both the model and the view can be independently modified and reused, enhancing scalability.
3. **Flexibility**: Multiple views can be created for a model for different UI requirements. Changes made in the model are propagated automatically to these views, facilitating easy updates.
4. **Parallel Development**: Different developers can work on each component simultaneously, thus reducing development time.

## Separation of Concerns
- One of the core tenets of good software engineering is separating presentation and domain logic (i.e model-view separation), since;
	- they deal with different concerns 
	- use different libraries, skills etc. 
	- it allows to create different views for an application e.g. HTML, command line, WAP... 
	- testing UI objects is usually hard -> model-view separation facilitates testing of the application core 
- Separation of control logic and UI is also recommended -> Model-View-Controller 
	- although often not as easy (and obvious) with common UI frameworks 
	- e.g. Java seduces to have both in the same class which is fine, however, in most cases
## Architectural applications of MVC
![[Pasted image 20240701100619.png#invert|800]]
## Modelling Alternatives
### 1. One Controller Class Per Use Case
#### Description:
- This approach involves creating a separate controller class for each use case within the system. A use case here refers to a specific sequence of interactions between a user and the system that accomplishes a business goal (e.g., "Create Invoice," "Update Profile").
#### When It Works Best:
- **Scalability for Larger Systems**: This method is particularly effective in systems with many use cases because it isolates the logic for each use case into separate classes, making the system easier to manage, extend, and maintain.
- **Example in JEE**: In Java Enterprise Edition (JEE), this approach can be implemented using Session Beans, where each Session Bean handles the business logic for a specific CRUD (Create, Read, Update, Delete) operation.
### 2. One Controller Class Per Application/System
#### Description:
- Under this model, a single controller (often called a Façade controller) is used to manage all system operations. This controller acts as a central point of interaction for the application’s UI and the business logic.
#### When It Works Best:
- **Simplicity for Smaller Systems**: This is more suitable for smaller systems or applications with relatively few system operations (approximately less than 12). It simplifies the architecture by reducing the number of classes and centralizing control flow logic.
- **Façade Pattern**: Often, this approach is implemented using the façade design pattern, which provides a simplified interface to a more complex subsystem of controllers, thereby reducing complexity for the clients of the system.

### 3. Direct Access to Appropriate Domain Objects
#### Description:
- This strategy allows the presentation layer or client-side code to interact directly with the domain model, bypassing intermediate controllers for certain operations.
#### Advantages:
- **Efficiency**: Direct access can be more straightforward and efficient as it reduces the overhead of passing through multiple layers or classes.
- **Parameter Passing Reduction**: It minimizes the need for extensive parameter passing between layers which can simplify the code and reduce errors.
#### Drawbacks and Considerations:
- **Risk of Business Logic Leakage**: A major drawback of this approach is the potential for control flow logic to become intertwined with the domain model. This can lead to situations where business rules and data management logic are mixed with interaction control, making the domain model more complex and harder to maintain.
- **Pollution of Domain Model**: Ideally, the domain model should be agnostic of the specific interactions that occur in the application layers. Allowing direct access risks "polluting" the domain model with concerns that should be handled at higher layers.
The diagram you're referring to is a Use Case Diagram (UCD), which is a graphical representation of interactions among the elements of a system. A use case diagram provides a high-level overview of the system by depicting the actors (both human and systems), the goals they want to achieve, and how these goals are dependent on one another within the system. Here’s a breakdown of the elements and some diagramming recommendations:
## Elements of a UCD
![[Pasted image 20240707133539.png#invert|500]]

### Elements of the Use Case Diagram Explained
1. **Actor**:
   - **Customer and Clerk**: These are the primary actors who interact directly with the system to achieve their goals. They are represented by stick figures.
   - **CreditCard System**: This is a supporting actor, which is a system that interacts with the main software system to process credit card transactions. It is represented differently from human actors to distinguish system interactions.
2. **Use Cases**:
   - **Lend out video**, **Create customer account**, and **Retrieve customer data**: These are the functionalities or services provided by the system. Each circle or ellipse represents a different use case.
3. **Interactions**:
   - Arrows indicate interactions between the actors and the use cases, showing which actor initiates which use case.
4. **Relationships**:
   - **Include**: Indicated by `<<include>>`, this relationship means that one use case is essential to the functionality of another. In this diagram, both "Lend out video" and "Create customer account" include "Retrieve customer data", meaning this action is a necessary part of both use cases.
   - **Extend**: Indicated by `<<extend>>`, this is a conditional or optional inclusion, where the extended use case runs under certain specified conditions which are not necessarily always true.
## Use Case Diagramming Recommendations
- **Focus on User-Goal Level Use Cases**: Diagrams should primarily focus on user-goal level use cases, which represent significant user interactions with the system.
- **Use as a Table of Contents**: The diagram can serve as a "table of contents" for detailed textual use case descriptions. It provides a quick overview of all major functionalities and their relationships.
- **Simplify**: Avoid excessive diagramming. The primary value of a use case diagram is to outline relationships and provide a big-picture view of system interaction.
- **Actor Placement**: Place primary actors to the left and supporting actors to the right to maintain clarity and consistency across diagrams.
- **Correct Use of Relationships**: Properly apply `<<include>>` and `<<extend>>` relationships to clarify mandatory versus optional interactions.
- **Minimalism**: Do not get bogged down in detailing every possible relationship or actor interaction in the diagram. Focus on the most important and representative interactions to keep the diagram useful and manageable.
## Example
- Classify use cases as User Goal or Sub-Function?
![[Pasted image 20240707133655.png#invert|600]]
### Solution
![[Pasted image 20240707133717.png#invert|600]]
1. **User Goals**: These are primary activities that provide direct business value or fulfill significant user needs within the system.
   - **Make Decision**: This is a central activity within the system where a decision is made based on the information processed, significantly affecting the workflow.
   - **Review Reviews**: Involves evaluating the reviews provided, which is a critical step in understanding and improving the process or outcome.
   - **Invite Reviewer**: Directly relates to involving additional expertise or stakeholders, critical for enhancing the process or decision-making.
   - **Assign Reviewer**: Direct assignment of tasks to reviewers, a key part of the workflow.
   - **Submit Review**: Finalizes the input of the reviewer, marking the completion of their part in the process.
   - **Notify Author**: Directly communicates outcomes or necessary information back to the author, crucial for ongoing interactions and process continuation.
   - **Submit Paper**: Represents a significant milestone in the submission process, marking the completion of this phase.
2. **Sub-Functions**: These are secondary or supporting actions that facilitate the main activities but might not independently deliver complete business value.
   - **Set Paper State**: A setup or state adjustment action that prepares the paper for further actions.
   - **Select Paper**: A selection process that supports decisions or other actions but is part of a broader workflow.
   - **Select Reviewer**: Supports the reviewing process by choosing appropriate reviewers, important yet a part of a larger goal.
   - **Read Paper**: A reading step that, while necessary, supports the reviewing or decision-making process.
   - **Select Assigned Paper**: Another selection step that supports further actions.
   - **List Conferences**: Organizes or displays available options, supporting broader decision-making or submission processes.
   - **List Assigned Paper**: Similar to listing conferences, it organizes content for further action but doesn't deliver standalone business value.

## Key Use Case Terminology
### 1. **Stakeholder**
   - **Definition**: A stakeholder is any person or organization that has a direct or indirect influence on the requirements of the system because they have a vested interest in its success. This could include end-users, project sponsors, or external regulatory bodies, among others.
### 2. **Actor**
   - **Definition**: An actor represents an entity that interacts with the system from the outside. This can be a human user (in various roles), another computer system, or an organization.
   - **Roles**:
     - **Primary Actor**: Initiates the interaction with the system, driving the core functions for which the system is intended.
     - **Secondary Actor**: Provides a service or performs actions required by the system but does not initiate interactions.
### 3. Use Case Model
   - **Definition**: This is a comprehensive model that includes all use cases and possibly related diagrams that represent the interactions between actors and the system. It serves as a blueprint for understanding the system's functionality and behavior.
   - **Components**: May include various types of diagrams like activity diagrams to show the flow of interactions and processes within the use cases.
### 4. **Scenario**
   - **Definition**: A scenario is a specific sequence of actions and interactions between actors and the System under Development (SuD). It represents a single path or instance through a use case, detailing how specific tasks are performed and objectives achieved.
   - **Examples**: A scenario could be successfully withdrawing money from an ATM or booking a ticket online.
### 5. **Use Case**
   - **Definition**: A use case is a collection of related success and failure scenarios that describe how actors use the system to achieve a specific goal. It encompasses all the possible sequences within a given functionality.
   - **Purpose**: Use cases are used to capture and define the requirements of the system in terms that are understandable both to the stakeholders and the technical team. They focus on what the system should do and how it should react in various situations.
   - **Example**: The "Withdraw Money" use case for an ATM might include scenarios for successful withdrawal, insufficient funds, incorrect PIN, or a malfunctioning ATM.
## Relation between Goals and Scenarios
![[Pasted image 20240707134011.png#invert|600]]
## How to Find Use Cases
### 1. **Choose the System Boundary**
   - **Definition**: Setting the system boundary involves defining what is inside and outside of the system. This helps to focus on what the system should manage directly and what it interacts with.
   - **Example**: For a Point of Sale (POS) system, the system boundary includes the software and hardware directly interacting with the cashier and transactions.
### 2. **Identify Primary Actors**
   - **Definition**: Primary actors are those individuals or systems that interact directly with the system to achieve their goals.
   - **Examples**: In a POS system, primary actors could include the cashier who processes sales, the administrator who manages system settings, and the store owner who views sales reports.
### 3. **Identify User Goals for Each Actor**
   - **Definition**: User goals are the end objectives that each actor aims to achieve by using the system.
   - **Process**: Raise each identified goal to the highest user goal level that adheres to the Elementary Business Process (EBP) guidelines. This means ensuring that the goal is a task performed by one person, in one place, at one time, adding measurable business value.
   - **Examples**: User goals for a POS system might include processing a sale for a cashier or generating a financial report for the store owner.
### 4. **Define Use Cases**
   - **Definition**: Each use case is defined to fulfill the user goals. They should be named clearly in an imperative style.
   - **Examples**: Use cases for a POS system might be named "Process Sale," "Print Report," or "CRUD (Create/Retrieve/Update/Delete) Customer Accounts."
   - **CRUD Use Cases**: These are common for managing data entities like customer accounts, products, etc., encompassing basic operations that the system must handle.
## Iterative Use Case Elaboration
Use cases often need refinement to ensure they accurately capture the required functionality and user interactions. This elaboration should ideally follow a breadth-first approach:
1. **Identify Primary Actors and Their Goals**: List out all actors and their corresponding goals.
2. **Draft Use Case Briefs or Main Success Scenarios**: Start with broad strokes to outline the primary flow of interactions.
3. **Add Extension Conditions**: Include what might cause these use cases to fail (e.g., exceptions or special conditions).
4. **Define Extension Handling Steps**: Detail how the system should respond to each extension condition.
5. **Review and Refine**: As the system and requirements evolve, continuously refine the use cases. This may include extracting more detailed sub-use cases or merging overlapping ones.
Use cases remain dynamic throughout the development process, often undergoing modifications as new insights are gained and as stakeholders provide feedback. Formal change requests may be necessary to capture significant modifications or refinements in the use cases after initial approval.
## Example Use Case Breakdown
### Actor and Goal
- **Actor**: Cashier
- **Goal**: To process sales efficiently to generate revenue for the store.
This identifies who is using the system and for what purpose, which helps in defining the relevant processes and interactions.
### Main Success Scenario
This is a step-by-step description of the ideal interaction without any interruptions or errors. It typically includes:
1. **Input**: The cashier inputs item identifiers into the system.
2. **Processing**: The system retrieves item descriptions, calculates running totals, total amounts, and applicable taxes.
3. **Completion**: On receiving payment (e.g., cash), the system finalizes the transaction by storing sale data, updating inventory, and printing a receipt.
This scenario maps out the expected flow of events in a successful transaction, which is critical for understanding system behavior under normal conditions.
### Failure Conditions
Failure conditions address potential errors or exceptional situations that might occur during the use case execution. Examples include:
- Incorrectly recognized item identifiers.
- Cancellation of the sale process by the cashier.
- Removal of an item from the sale.
- Alternate payment methods like credit card use.
Identifying these helps in planning how the system should handle unexpected or irregular situations, ensuring robustness and user-friendliness.
## Writing Use Cases (Cockburn's Method)
**Steps describe interactions, validation, or internal changes**:
- Steps are sequentially listed to describe the interaction between the actor and the system.
- Each step should be clear and concise, detailing only one interaction or decision point.
- Validation steps should be included where the system needs to check or confirm data (e.g., validating a debit card).
- Changes within the system (like updating a balance) following an action should be clearly described.
### **Example for an ATM Transaction**:
1. **Insert Card**: Customer inserts his debit card.
2. **Validate Card**: The ATM checks if the card is valid.
3. **Select Transaction**: (Implicit step where the customer chooses a transaction type.)
4. **Process Transaction**: The ATM processes the requested transaction (e.g., cash withdrawal).
5. **Update Balance**: The ATM updates the customer's account balance.
6. **Return Card/Receipt**: The ATM returns the card and provides a receipt.
## Tips for Effective Use Cases:
- **Clarity**: Each step should be straightforward and unambiguous.
- **Completeness**: Cover all significant steps from start to finish, including exceptional and end conditions.
- **Consistency**: Use a consistent level of detail and terminology throughout the use case.
- **Actor-focused**: Keep the actor's interactions at the forefront, ensuring the system responses are aligned with user actions.
## Guidelines
The guidelines provided by Alistair Cockburn for writing use cases are designed to create clear and effective documentation that focuses on the user's needs and the system's responses without delving into user interface specifics. Here's a breakdown of these guidelines to explain their application and intent:
### Goal-Oriented Steps
- **Show (Sub) Goal Succeeding**: Each step in a use case should demonstrate progress towards the goal, illustrating how the process moves forward when successful.
- **Capture Actor's Intent**: Focus on what the actor intends to achieve rather than describing mechanical actions (e.g., clicking a button). This helps to maintain the purpose of the interaction clear and aligned with the user's goals.
- **Include an Actor in Each Step**: Ensure that every step involves an actor either passing information, controlling interaction, validating conditions, or updating the system state. This emphasizes the role of the user and their interaction with the system.
### Naming and Descriptions
- **Actor Names**: Start actor names with an uppercase letter to distinguish them clearly from other terms.
- **Avoid UI Details**: Keep user interface details (like specific GUI movements or interactions) out of the use case to avoid confining the design prematurely. This level of detail is better suited for UI design documentation.
- **Define Data Exchanged**: While detailed UI interactions are excluded, the data involved in the interactions needs to be clearly defined, such as specifying the types of data being entered or manipulated.
### Data Descriptions Levels
- **Precision Level 1: Data Nickname**: Use a simple, informal name that identifies the data.
- **Precision Level 2: Data Fields Associated with Nickname**: Describe what type of data is involved using a more technical description that includes data types.
- **Precision Level 3: Field Types, Lengths, and Validations**: Provide detailed specifications of the data, including constraints and validation rules. This might include formatting requirements or validation rules based on standards.
### Handling Conditions and Extensions
- **Main Scenario Free from If Statements**: The main success scenario should flow without conditional branches. This keeps the narrative straightforward and focused on the primary path of success.
- **Extensions for Conditional Handling**: Conditions and alternative paths (such as error handling or exceptional cases) should be documented separately in an extensions section. This helps to keep the main narrative uncluttered while providing a clear guide to handling different outcomes.
- **Use of ´include´ and Underlining**: These notations can be used to indicate included use cases or essential cross-references within the documentation, depending on the tools and notation standards being used.
### Practical Example
- If the main success scenario describes a cashier processing a sale, an extension might detail what happens if an item identifier is invalid:
  - **3a. Invalid Identifier**:
    1. The system signals an error and rejects the entry.
    2. Optionally, ´include´ "Find Product Help" where the cashier can get assistance to correct the identifier.
These guidelines help ensure that use cases are both comprehensive and clear, supporting effective development practices by focusing on user interactions and system responses without getting bogged down in implementation details. This approach helps bridge the gap between user needs and system design, ensuring that developers can build systems that meet actual user goals.
## Fully Dressed Use Case Sections
The "Fully Dressed Use Case Sections" is a methodical approach to documenting a use case comprehensively, ensuring all necessary details are captured to facilitate understanding and development. Here's an explanation of each section involved in creating a fully dressed use case:
### 1. Preface Elements
- **Purpose:** To set the stage by providing essential context such as the scope and goal level of the use case.
- **Primary Actor:** Identifies the main actor who interacts with the system to achieve a goal.
### 2. Stakeholders and Interest List
- **Purpose:** To outline who has an interest in the use case and what those interests are.
- **Function:** Helps define system responsibilities and tracks the source of each system requirement.
### 3. Preconditions
- **Purpose:** Specifies conditions that must be true before the use case begins.
- **Characteristics:** Assumed to be true and are not verified within the use case; often set by the completion of other use cases (e.g., user login).
### 4. Postconditions (Success Guarantees)
- **Purpose:** Defines what must be true after the use case successfully completes.
- **Detail:** Includes guarantees for main success and possible alternative paths, focusing on meeting all stakeholder needs.
### 5. Main Success Scenario
- **Purpose:** Describes the standard flow of events (often called the "happy path") for the use case.
- **Characteristics:** Linear sequence without branches or conditions, focusing on meeting the primary goal.
### 6. Extensions (Alternative Flows)
- **Purpose:** Captures all alternative paths, both successful and failing scenarios, diverging from the main flow.
- **Details:** Extensions are indexed by conditions that trigger them and include handling steps; they may merge back into the main flow unless explicitly terminated.
### 7. Special Requirements
- **Purpose:** Records specific non-functional requirements or constraints that relate directly to the use case.
- **Examples:** Performance, reliability, usability constraints, or other non-functional requirements that impact the use case execution.
### 8. Technology and Data Variations
- **Purpose:** Describes any required technical variations or constraints on how the use case should be implemented.
- **Examples:** Input/output device requirements, data format variations (e.g., using UPCs or EANs for item identifiers).
Each section of a fully dressed use case provides critical information that contributes to a complete understanding of how the use case fits within the system, the needs it meets, and the conditions under which it operates. This methodical documentation is crucial in complex systems where multiple use cases interact and where clarity and precision are necessary to ensure successful implementation.
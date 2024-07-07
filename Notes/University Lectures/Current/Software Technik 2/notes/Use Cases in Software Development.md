- **Definition**: Use cases in software development are narratives that detail an interaction between the user and the system, focusing on achieving user goals. They serve as a tool to capture functional (operational) requirements.
  
- **Black-Box User Goal Use Case**:
  - This type of use case focuses on what the system does from the user's perspective, without delving into how the system internally achieves these tasks.
  - It treats the system as a "black box," emphasizing the inputs and outputs related to user interactions.

- **Other Goal Levels and Scopes**:
  - Although use cases are versatile tools, their potential for modeling different levels of goals and scopes is often underutilized in practice.
  - Acknowledging different goal levels (such as strategic goals, user goals, and subfunction goals) and clarifying the scope of each use case can prevent confusion and enhance the effectiveness of requirements elicitation.
### Importance of Clarity and Scope
- **Clarifying Goals and Scope**: It is crucial to define the goals and scope of use cases clearly to ensure they meet the intended purpose and cover the necessary aspects of system functionality.
- **Avoiding Confusion**: Without clear goals and well-defined scopes, use cases can become sources of misunderstanding among stakeholders, potentially leading to incomplete or incorrect requirements.
## Goals, Actors and Scopes
![[swrt6.png#invert|400]]
The concept of **Goals, Actors, and Scopes** in use case modeling is essential for accurately capturing and representing the requirements within a system design. Here’s a breakdown of these components and how they contribute to effective use case development:
### Goals
- **Definition**: Goals are the intended outcomes that the system or its components aim to achieve. In the context of use case modeling, each use case is driven by a specific goal that justifies its existence.
- **Example**: In an insurance management system, possible goals could be "process claims efficiently" for a clerk or "obtain reimbursement for damages" for a customer.
### Actors
- **Definition**: Actors are the entities (usually people, other systems, or hardware) that interact with the system. They are external to the system and initiate or participate in the use cases.
- **Example**: In the same insurance management system, actors might include the insurance clerk, the customer, and perhaps a third-party claims adjuster.
### Scopes
- **System Boundary**:
    - **Definition**: This defines the border between the system under discussion and its environment. It helps in delineating what is inside the system (and thus within the scope of the system’s design) and what lies outside (the external elements with which the system interacts).
    - **Importance**: Identifying the system boundary is crucial for understanding what needs to be modeled and built within the system and what interfaces or interactions the system will have with external entities.
- **System Context**:
    - **Definition**: This includes parts of the environment that interact with the system and are relevant for defining and understanding the requirements. It helps in visualizing how the system fits within the larger operational environment.
    - **Modeling Tool**: Context diagrams are often used to visualize the system context, illustrating the system’s interfaces with external entities.
- **Context Boundary**:
    - **Definition**: This boundary separates the system context (relevant environment) from the broader, irrelevant environment. It helps focus the requirements elicitation on the aspects that directly impact the system.
    - **Development Impact**: Clarifying the context boundary is crucial during the early stages of requirements elicitation and tends to become more defined as the project progresses.

## What makes a Use Case good?
In the context of systems and software development, a **Use Case** is a specific methodology used to define the requirements of a system. A good use case effectively captures the interactions between a user (actor) and the system to achieve a goal, especially within the bounds of business operations. Understanding what makes a good use case can help guide the development process more smoothly and ensure the system meets its intended functionality and user interactions. 
### Emphasis on Goal Achievement
- **Core Idea**: A use case should be centered around a goal that is valuable from the business perspective. It describes an interaction necessary to achieve this goal.
- **Examples**: 
  - **Negotiate a supplier contract**: Could be a valid use case if the system includes features to support contract management or negotiations.
  - **Handle returns**: Appropriate for systems designed to manage inventory or customer service.
  - **Withdraw money**: A use case for banking systems where the ATM or banking software supports this financial transaction.
### Appropriate Level of Granularity
- **[[Elementary Business Processes (EBP)]]**: 
### Defining Use Cases at the Right Level
- **Avoid too low level**: Use cases should not be defined for very granular tasks unless these tasks themselves provide significant standalone business value.
- **Subfunctions and subtasks**: These are more detailed actions that are part of a larger use case but do not constitute standalone use cases unless they meet the criteria of an EBP.
### Practical Implications
- **Focus on EBPs**: For practical and effective requirements analysis, it is beneficial to focus on EBPs as they usually form good user goal use cases. This ensures that the use cases are meaningful and manageable within the scope of the project.
- **Creation of Sub Use Cases**: Similar to functions and subroutines in programming, sub use cases represent smaller, frequently performed tasks that support the main business process. They should be clearly defined only when they provide additional clarity or manage complexity within the main use case.
### Possible Use Case Goal Levels
![[Pasted image 20240707133230.png#invert|500]]
Defined by Alistair Cockburn, these levels help categorize use cases by their scope and complexity:
1. **High-Level Summary**
   - Often equates to an entire business process. This level ties together several user goals and subfunctions into a broad use case. 
   - Example: Managing a complete customer relationship lifecycle, from acquisition through service delivery to support.
2. **User Goal (Elementary Business Process, EBP)**
   - Directly describes how a user’s goal is achieved through interactions with the system. This level is typically the sweet spot for most use cases.
   - Example: Completing a sale transaction, where the user interacts with the system to select products, enter payment information, and confirm the purchase.
3. **Sub(function)**
   - Focuses on necessary subgoals that support a primary user goal. These are often technical or background tasks that don't deliver direct business value but are essential for the functionality.
   - Example: Verifying user credentials during a login process.
4. **Too Low (Feature or System Operation)**
   - Refers to tasks that are too granular to be considered a full use case, often just involving basic system operations.
   - Example: Clicking a button to refresh a page or sending a single API request.
### Examples
Identifying the correct goal level for each activity is essential for properly framing use cases in software development. Here's how each activity you listed could be categorized according to the use case goal levels:
1. **Process a claim in an insurance company**
   - **Goal Level:** User Goal
   - **Explanation:** Processing a claim involves several steps but represents a complete, valuable business process for the user (insurance agent) and aligns with the definition of an Elementary Business Process (EBP).
2. **Create customer account in a video store**
   - **Goal Level:** User Goal
   - **Explanation:** This is a direct interaction with the system that results in a significant change (creation of a new account), providing clear business value and user interaction.
3. **Book a flight ticket on an online platform**
   - **Goal Level:** User Goal
   - **Explanation:** Booking a flight is a complete activity that delivers a clear outcome and business value, involving several steps but focused around a single, significant goal.
4. **Login to your online banking account**
   - **Goal Level:** Sub Use Case or System Operation
   - **Explanation:** Login is a gateway to more substantive tasks and is generally a smaller component of a larger process. It could be seen as too granular for a standalone user goal.
5. **Retrieve a list of customer accounts**
   - **Goal Level:** Sub Use Case
   - **Explanation:** This function supports other processes (like managing customer accounts) and does not typically provide direct business value by itself but is essential for enabling those processes.
6. **Check credit card data in an ATM**
   - **Goal Level:** Sub Use Case
   - **Explanation:** This is a supporting task within a larger transactional process, such as withdrawing money or making a payment, and does not stand alone as a user goal.
7. **Withdraw money from ATM**
   - **Goal Level:** User Goal
   - **Explanation:** Withdrawing money is a complete and self-contained transaction that directly involves the user interacting with the system to achieve a specific and valuable outcome.
### Summary Table

| Activity | Use Case Level |
|----------|----------------|
| Process a claim in an insurance company | User Goal |
| Create customer account in a video store | User Goal |
| Book a flight ticket on an online platform | User Goal |
| Login to your online banking account | System Operation |
| Retrieve a list of customer accounts | Sub Use Case |
| Check credit card data in an ATM | Sub Use Case |
| Withdraw money from ATM | User Goal |


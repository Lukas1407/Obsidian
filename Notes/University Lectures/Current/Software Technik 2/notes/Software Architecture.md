> [!abstract] Definition
> Software architecture plays a crucial role in the development of systems, serving as the fundamental blueprint for both the system and the project. 
> A software architecture is the set of design decisions which are hard to revert or which have to be made early

- Design Decisions include the choice of technology stack, data storage solutions, interaction protocols, and the basic algorithms and processes that will be used throughout the application.
## Definitions
### ISO/IEC 42010 Definition
**"Architecture: fundamental concepts or properties of a system in its environment embodied in its elements, relationships, and in the principles of its design and evolution"**
- **Fundamental Concepts or Properties**: This suggests that architecture is about the core aspects that define the system's overall framework and operational characteristics within its environment. These are not just any properties but are fundamental, meaning they are essential and foundational to the system's function and identity.
- **Embodied in Elements, Relationships, and Principles**: Architecture is manifested through the system’s elements (components, modules, interfaces), their relationships (how these elements interact or connect with each other), and the design principles guiding the system’s development and future evolution. This embodiment is crucial as it dictates how well the system will function and adapt to changes over time.
### Taylor et al., 2009
**"A software system’s architecture is the set of principal design decisions made about the system"**
- **Principal Design Decisions**: This highlights that architecture involves decision-making at a high level, focusing on key aspects that determine the structure and behavior of the system. These decisions typically address the most critical issues regarding the system's functionality and performance.
- **Impact of Decisions**: The decisions taken are not trivial; they have a significant impact on the technical and operational characteristics of the system, influencing everything from system performance to user experience and maintenance.
### Bass et al., 2013
**"The software architecture of a system is the set of structures needed to reason about the system, which comprise software elements, relations among them, and properties of both"**
- **Set of Structures**: This definition emphasizes that architecture is structured; it is not an ad hoc assembly of software components. These structures provide a framework for thinking about the system logically and systematically.
- **Reason About the System**: The purpose of having a defined architecture is to enable reasoning about how the system works and how its various components interact. This is crucial for effective analysis, troubleshooting, and enhancement of the system.
- **Elements, Relations, and Properties**: Similar to the ISO definition, this highlights the importance of components, their interactions, and their characteristics, both individual and collective.
### Reussner 2016
**"A software architecture is the result of a set of design decisions comprising the structure of the system with components and their relationships and their mapping to execution environments."**
- **Result of Design Decisions**: Emphasizes that architecture is the outcome of deliberate and strategic planning, not something that emerges by chance.
- **Structure with Components and Relationships**: Reiterates that architecture involves a clear arrangement of parts — how they are organized and how they interact.
- **Mapping to Execution Environments**: Points to the practical aspect of architecture, involving the allocation of software components to hardware or virtualized environments, which is critical for the system’s actual operation in real-world scenarios.

- Architectural decisions are made at different stages of the software development lifecycle:
	- **Early Phases**: During the requirements gathering and analysis phases, decisions are primarily influenced by what the system is supposed to do (requirement-oriented).
	- **Later Phases**: As more information about the operating environment (hardware, network, etc.) becomes available, additional decisions are made that are influenced by these factors (code-oriented).
- **Deferring Decisions**: While some architectural decisions can be postponed, doing so is itself a strategic choice. Not all decisions can be delayed, and delaying too many decisions can lead to challenges later in the development process.
- **Documentation**: The architecture should be explicitly documented using dedicated architectural description languages (ADLs) rather than conventional programming languages, which are not designed to express architectural nuances effectively.
## Advantages of Explicit Software Architecture
1. **Stakeholder Communication**: A clearly defined architecture helps communicate the design and operation of the system to stakeholders, including developers, project managers, clients, and third-party partners. This clarity helps in aligning expectations and facilitating more effective collaboration.
2. **System Analysis**: It allows for a systematic analysis of how system components interact, identify potential bottlenecks, and evaluate performance and scalability. Architectural analysis can also help in assessing risks and planning for system evolution.
3. **Large-Scale Reuse**: Architecture can promote reuse at the large scale (like services, microservices architectures, or entire systems) by providing a library of reusable components or templates that can be employed in multiple projects. This reuse can significantly reduce development time and cost.
4. **Project Planning**: It aids in accurate project planning by providing a clear picture of the system’s requirements and complexity. This clarity helps in resource allocation, timeline estimation, and cost forecasting.
## Architectural Design
- Architectural design serves as a bridge between what the system needs to do (specified by requirements) and how the system will do it (detailed design). It translates requirements into a blueprint that outlines the system’s structure and behavior.
- Architectural design is described as a creative process that varies depending on the type of system being developed. It involves a series of common decision points:
	- **Generic Application Architecture**: Determining whether there is a pre-existing architectural pattern or framework that can be adapted for the new system can save time and reduce risk.
	- **Distribution**: Decisions need to be made regarding the distribution of components across different machines or networks. This includes considering which parts of the system need to communicate with others and how they will do so.
	- **Architectural Styles**: Choosing an architectural style (e.g., microservices, monolithic, layered, event-driven) that fits the project's needs is crucial. This choice will affect everything from performance to maintainability.
	- **Subsystem Decomposition**: How the system is broken down into smaller, manageable components or subsystems is a fundamental aspect of architectural design. This impacts not only development but also future scalability and adaptability.
	- **Component Sourcing**: Decisions regarding which components to buy off-the-shelf, which to develop in-house, and which can be reused from previous projects are important for budgeting and project timelines.
	- **Reusability and Future Proofing**: Consideration of how components designed and developed now can be reused in future projects or how they can be evolved as requirements change.
	- **Evaluation and Documentation**: Determining how the architectural design will be evaluated against requirements and constraints, how it will be documented, and what the realistic scenarios for evolution are, all form part of the architectural process.

![[Agile Architecture]]
## Recording Architecture Decisions
Recording architecture decisions is an essential practice that provides value not only during the initial development phases but also throughout the system’s life cycle. Here’s why it’s important and how to do it effectively:
- **Document Influential Factors**: Record the conditions and requirements that influence each architectural decision. This might include business rules, user needs, system requirements, or constraints.
- **Decisions and Rationales**: Clearly document what decisions were made and, importantly, why they were chosen. This helps in understanding the reasoning behind certain architectural choices.
- **Alternatives and Rejections**: It’s also beneficial to note what other options were considered and why they were not selected. This insight can be invaluable during future system upgrades or maintenance when those alternatives might be reconsidered.
- **Importance of Rationale**:
  - **Future Understanding**: Ensuring that the rationale behind decisions is documented helps future team members understand why the system is the way it is, which is crucial for effective maintenance and enhancement.
  - **Avoiding Redundancy**: Knowing why certain decisions were made can prevent future teams from revisiting already resolved issues or reconsidering ineffective solutions.
- **Efficiency**: Having a well-documented architecture decision log allows teams to quickly recall why certain paths were taken. This is particularly helpful during the later stages of design, system evolution, or when addressing unexpected issues.
- **Document Non-Decisions**: Sometimes, what is not done is as important as what is done. Recording non-decisions—potential decisions that were considered but explicitly decided against—is also crucial.

## Factors Influencing Architecture
1. **Requirements**
   - **Quality Requirements and Constraints**: These are critical because they directly influence the non-functional aspects of the system such as performance, security, and availability. Quality requirements often dictate the architectural decisions to ensure the system meets its intended service quality levels.
   - **Architecturally Significant Requirements**: Not all requirements have an equal impact on the architecture. Architecturally significant requirements are those that have a substantial effect on the architecture's structure, such as those affecting scalability, security, or maintainability.
2. **Re-Use**
   - **Architectures and Subsystems/Components**: Existing architectures, subsystems, and components that can be reused influence the new system's architecture. Reusing these elements can speed up development time and reduce costs.
   - **Styles, Patterns, Guidelines**: Architectural styles and patterns provide templates or best practices that can be adopted to solve common problems. For example, a microservices architecture pattern might be chosen for its scalability and maintainability benefits.
3. **Organization**
   - **Conway’s Law**: This adage posits that systems tend to mirror the communication structures of the organizations that create them. The organizational structure, therefore, indirectly shapes the architecture.
   - **Team Characteristics**: The size of the team, the number of teams involved, their experience, and the organizational structure also influence architectural decisions. Larger teams might lead to more modular architectures to simplify management and development.
### Influential Factor: Quality Requirements
Quality requirements are particularly significant as they directly impact the architecture's ability to meet its fundamental goals:
- **Performance**: To minimize subsystem communication, which can degrade performance, operations might be localized within large-grained components. This reduces the need for frequent interactions across components, enhancing performance.
- **Security**: A layered architecture can help in securing a system by placing critical assets in the inner layers, making it harder for external threats to reach sensitive information.
- **Safety**: The architecture might include strategies to analyze potential failures and design the system in a way that prevents faults from causing unsafe conditions.
- **Availability**: Including redundant components within the architecture to ensure that the system remains available even if some components fail.
- **Maintainability**: Using fine-grained, self-contained components can enhance maintainability. Each component can be updated or replaced independently without impacting others.
- **Scalability**: Considering how the system will handle increased load through strategies like distributing the system across multiple nodes to manage concurrency effectively.
### Balancing Conflicting Requirements
One of the critical challenges in architectural design is balancing these quality requirements, as they often conflict with each other. For example:
- Enhancing security might reduce performance due to additional security checks.
- Increasing redundancy to improve availability can increase costs and complexity, impacting maintainability.
Balancing these requires a careful analysis of the trade-offs involved and making informed decisions that align with the system's most critical needs and long-term objectives.
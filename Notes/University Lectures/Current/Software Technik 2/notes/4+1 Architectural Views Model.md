> [!abstract] Definition
> The "4+1" Architectural Views model, developed by Philippe Kruchten in 1995, is a framework for describing the architecture of software-intensive systems, based on multiple concurrent views. The views are designed to address different stakeholder concerns and are used for describing the system from different perspectives. 

## Components
### 1. **Logical View**
- **Purpose**: Focuses on the functionality the system provides to end-users. It's concerned with the system's capabilities, functionality, and user interactions.
- **UML Diagrams**:
    - **Class Diagrams**: Show how classes interact within the system.
    - **Communication Diagrams**: Detail how objects communicate with each other.
    - **Sequence Diagrams**: Describe how operations are carried out through a sequence of steps.
- **Key Concerns Addressed**: Usability, reusability, and robustness of the system architecture.
### 2. **Development View**
- **Purpose**: Provides a perspective from the programmer's viewpoint and is concerned with the software management aspect of the system.
- **UML Diagrams**:
    - **Component Diagrams**: Describe the organization of physical software components, including source code, binaries, libraries, and executables.
    - **Package Diagrams**: Depict how different components are grouped into packages, which helps organize the development environment.
- **Key Concerns Addressed**: Software modularity, understandability, and manageability.
### 3. **Process View**
- **Purpose**: Addresses the dynamic aspects of the system, focusing on runtime behavior, including processes, threads, and how they interact.
- **UML Diagrams**:
    - **Activity Diagrams**: Illustrate the flow from one activity to another in the system. They show concurrent processes and the synchronization and coordination of these processes.
- **Key Concerns Addressed**: Concurrency, distribution, integrators, performance, and scalability.
### 4. **Physical View**
- **Purpose**: Shows the system from a system engineer's perspective. It deals with the deployment of software components across hardware.
- **UML Diagrams**:
    - **Deployment Diagrams**: Focus on the physical configuration of software components on the hardware.
- **Key Concerns Addressed**: System topology, communication, and scalability.
### 5. **Scenarios (Use Case View)**
- **Purpose**: Uses a set of scenarios, or use cases, to illustrate and validate the design of the architecture. Scenarios help to explore the architecturally significant use cases that may influence both design and implementation strategies.
- **Key Roles**:
    - Help in identifying and addressing architectural drivers.
    - Serve as a basis for tests of the architectural integrity and capabilities.
    - Aid in discovering architectural elements and verifying the interactions and integrations within the system.
- **UML Diagrams**: While specific diagrams like sequence or activity diagrams might be used to represent scenarios, the primary focus is on the interaction patterns and processes described by the scenarios themselves.
### Integration in RUP (Rational Unified Process)
- The "4+1" model is extensively used in RUP for guiding the architecture of software projects. In RUP, these views help ensure that the architecture covers all angles and is communicated effectively among all stakeholders involved in the project.
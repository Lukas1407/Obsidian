> [!abstract] Definition
> The concept of **Architectural Styles** refers to a family of architectures that share certain characteristics. These styles provide a set of predefined constraints that guide the architecture of systems. Understanding and adhering to these styles helps in systematizing the design process, easing the maintenance, understanding, and evolution of the architecture. 
## Architectural Styles and Dimensions
Architectural styles are organized according to different "dimensions" of design decisions. Each dimension addresses a specific aspect of the system's architecture:
1. **Communication**
   - **Service-Oriented Architecture (SOA)**: Structures the architecture as a collection of services, which communicate with each other, often asynchronously, to perform functions. This style is highly modular and allows for easy scalability and flexibility.
   - **Message Bus**: An architecture style where components communicate via a common channel (the message bus) that handles message routing between components. It decouples the system components and enhances integration flexibility.
   - **REST (Representational State Transfer)**: An architectural style for distributed systems, particularly web services, which emphasizes stateless communication and leveraging standard web protocols and methods (HTTP).
2. **Deployment**
   - **Client/Server**: Segregates the system into two applications, where the client makes requests to the server that processes them and returns the results. This style simplifies network interactions and centralizes data management.
   - **N-Tier / 3-Tier**: Organizes components into separate tiers (usually three: presentation, logic, and data). It physically separates concerns across different servers or platforms enhancing manageability and scalability.
   - **Microservice**: Structures an application as a collection of loosely coupled services, which implement business capabilities. This style allows for independent deployment of parts of the system, enhancing agility and scalability.
3. **Structure: Primary / Secondary**
   - **Component-Based Architecture**: Divides the system into modular components, each with defined interfaces, allowing for high reusability and maintainability.
   - **Layered Architecture**: Organizes the system into layers with each layer performing a specific role. Typically, each layer depends only on the layer directly below it, reducing the coupling across the system.
   - **Microservice Architecture**: Similar to its role in deployment, it emphasizes structuring the application as small services with specific business functionalities.
## Practical Considerations and Mixing Styles
- **Do Not Mix Styles on the Same Dimension**: It's generally advised not to mix different architectural styles within the same dimension because it can lead to a design that is difficult to understand and maintain. For instance, using both SOA and REST within the communication dimension can complicate the communication strategy of the system.
- **Layer vs. Tier**: It’s important to differentiate between layers and tiers:
  - **Layers** are logical partitions of the architecture where each layer has a specific role and responsibility within the application. For example, presentation, business logic, and data access layers.
  - **Tiers**, on the other hand, refer to the physical distribution of those layers across multiple servers or systems.
  Despite their conceptual difference, "layers" and "tiers" are often used interchangeably in practice. This is especially common in n-tier architectures where the physical (tier) and logical (layer) architectures are directly mapped.

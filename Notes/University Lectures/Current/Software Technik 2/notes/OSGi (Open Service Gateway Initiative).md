Tne main critique of traditional CBSE approaches like CORBA, EJB, or COM is that they are more object-oriented rather than truly component-based. They often lack a robust mechanism for defining explicit required interfaces and managing dependencies.

**OSGi (Open Service Gateway Initiative)**:

- **Overview**: OSGi addresses some of these challenges by providing a more refined module system above the traditional package level in Java.
- **Bundles**: OSGi introduces the concept of bundles, which are like JAR files but with explicit declarations of provided and required interfaces in their manifest files.
- **Service Registry**: It features a dynamic service registry where services (or bundles) can be registered, discovered, and wired at runtime. This allows for a highly dynamic component system where services can be added or removed without shutting down the system.
- **Integration and Adoption**: OSGi has been widely integrated into environments like Eclipse, enhancing modularity and component management in development tools.
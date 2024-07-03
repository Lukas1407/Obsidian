The **Component Model** defines the practical application of CBSE principles by specifying how components should be designed, implemented, and interacted with. Key aspects covered by component models include:
![[Pasted image 20240701122423.png|500]]
- **Definition of a Component**: What qualifies as a component, usually a self-contained unit with a well-defined interface and encapsulated functionality.
- **Service Provision**: How components expose their functionalities to other components or the external world, typically via interfaces or through exposed methods and properties.
- **Composition and Connection**: Describes how components can be assembled together to form larger applications. This involves linking components via their interfaces and using connectors or mediators.
- **Communication**: Outlines how data and control information flow between components, whether synchronously or asynchronously, directly or through event mechanisms.
- **Discovery**: Explains how components can be discovered and utilized within a system, often supported by a component repository or registry.
- **Execution Infrastructure**: Some component models come with a supporting infrastructure that handles the execution, lifecycle management, and runtime configuration of components, commonly known as a component framework.

### Technical Realisation of CBSE

Over the years, various technology providers have developed their ecosystems and standards around component-based development:

- **OMG (Object Management Group)**: Introduced standards like CORBA (Common Object Request Broker Architecture) and CCM (CORBA Component Model), which focus on interoperability among distributed objects and components.
    
- **Sun Microsystems**: Developed technologies such as JavaBeans, Enterprise Java Beans (EJB), which provided a server-side component architecture for modular construction of enterprise applications.
    
- **Microsoft**: Created COM (Component Object Model) and later .NET CLR, which supports language interoperability and a unified environment for object and component development.
    

#### Similarities and Differences

- **Similarities**: These technologies share common object-oriented principles such as encapsulation, interface inheritance, and late binding, which allows runtime decisions on method calls.
    
- **Differences**: They differ in how they manage resources like memory, handle versioning and evolution of components, and integrate with various system environments.
## Research-based Component Models
![[Research-based Component Models]]
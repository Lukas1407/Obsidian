> [!abstract] Definition
> Cloud computing is a model for enabling ubiquitous, convenient, on-demand network access to a shared pool of configurable computing resources (e.g., networks, servers, storage, applications, and services) that can be rapidly provisioned and released with minimal management effort or service provider interaction. 
## Characteristics
- **[[Elasticity|Elastic]] [[Scalability]]**:
    - **Definition**: The ability to scale computing resources up or down easily, automatically adjusting to the level of demand. This is often referred to as "scaling on demand."
    - **Impact**: This allows businesses to handle increases or decreases in demand without the need for manual intervention, ensuring that the computing environment is always aligned with actual usage needs.
- **On-demand Self-Service**:
    - **Definition**: Users can provision computing capabilities, such as server time and network storage, as needed automatically without requiring human interaction with each service’s provider.
    - **Impact**: This feature supports agility and speed, as users can quickly deploy and decommission resources based on their requirements, which is particularly beneficial for projects with fluctuating workloads.
- **Ubiquitous Network Access**:
    - **Definition**: Capabilities are available over the network and accessed through standard mechanisms that promote use by heterogeneous thin or thick client platforms (e.g., mobile phones, tablets, laptops, and workstations).
    - **Impact**: Ensures that services can be accessed from anywhere at any time, which is crucial for remote work scenarios and global businesses.
- **Resource Pooling**:
    - **Definition**: The provider’s computing resources are pooled to serve multiple consumers using a multi-tenant model, with different physical and virtual resources dynamically assigned and reassigned according to consumer demand. The customer generally has no control or knowledge over the exact location of the provided resources but may be able to specify location at a higher level of abstraction (e.g., country, state, or datacenter).
    - **Impact**: Improves cost efficiency and resource utilization by ensuring that the physical and virtual resources can be used by multiple tenants without conflict.
- **Measured Service**:
    - **Definition**: Cloud systems automatically control and optimize resource use by leveraging a metering capability at some level of abstraction appropriate to the type of service (e.g., storage, processing, bandwidth, and active user accounts). Resource usage can be monitored, controlled, and reported, providing transparency for both the provider and consumer of the utilized service.
    - **Impact**: Enables pay-per-use or charge-per-use models, which means customers pay only for the resources they use, making cloud services cost-effective for varied types of workloads.
### Key Points of Cloud Computing
1. **Illusion of Infinite Resources**:
    - Cloud computing gives the appearance of infinite resources available to users, thanks to the underlying technologies that dynamically allocate resources based on demand.
2. **Always Available**:
    - Resources in the cloud are designed to be highly available and reliable, often distributed across multiple geographical locations to ensure continuous service even in the event of hardware failure or other disruptions.
3. **Network-Accessible**:
    - Cloud services are provided over the Internet, allowing users to access them from anywhere, which facilitates remote work and global operations.
4. **On-Demand Services**:
    - Users can obtain, use, and manage these resources without needing prior arrangements or long-term commitments. This flexibility is crucial for businesses that experience fluctuating workloads.
5. **No Requirement for Ownership**:
    - Cloud providers offer IT services through a subscription or pay-as-you-go model, eliminating the need for users to invest in owning and maintaining IT infrastructure.
6. **Pay for Use**:
    - This business model ensures that users pay only for the resources they consume, which optimizes cost efficiency and eliminates wasted resources.
## What are "Resources" in Computing?
In the realm of cloud computing, "resources" refers to a broad array of elements essential for IT operations:
1. **Compute Power**:
    - Includes CPUs, GPUs, and general processing capabilities necessary to execute applications.
2. **Storage**:
    - Encompasses persistent data storage solutions like databases, object storage, and file storage services.
3. **Network**:
    - Involves bandwidth and related capabilities to ensure data can be moved efficiently within and across networks.
4. **Energy**:
    - The power required to run the servers, storage, and networking equipment, which can be considerable in data centers.
5. **Real Estate**:
    - Physical space needed to house large data centers.
6. **Human Resources**:
    - Manpower and human intelligence necessary for managing, maintaining, and innovating within the IT infrastructure.
7. **Software Applications**:
    - The programs and operating systems that run on the hardware and are provided as services (Software as a Service - SaaS).
8. **Data**:
    - Could include datasets used for various applications, particularly in scenarios involving big data and analytics.
9. **Other Resources**:
    - This could extend to any other support services, technology stacks, or middleware that facilitate the operation and management of cloud resources.
## Efficient Use of Resources
![[Pasted image 20240704110319.png#invert|600]]
- **Predicted Demand (Dashed Line):**
    - Represents the anticipated or forecasted resource needs over time based on historical data, trend analysis, or predictive modeling.
- **Actual Demand (Red Line):**
    - Shows how the actual usage of resources fluctuates over time. It's common for actual demand to deviate significantly from predictions due to unpredictable factors like sudden spikes in user activity or unforeseen changes in requirements.
- **Traditional Hardware (Blue Line):**
    - Indicates the cost trajectory when using traditional hardware which often involves purchasing or leasing physical servers. Traditional infrastructure usually requires upfront capital expenditure and can lead to either over-provisioning (too much capacity) or under-provisioning (insufficient capacity), both of which are costly.
- **Automated Virtualization (Green Line):**
    - Shows the cost using cloud-based virtualization technologies, which enable dynamic allocation of computing resources. This method closely follows the actual demand curve, minimizing both over-provisioning and under-provisioning. This flexibility typically results in cost savings and greater efficiency.
### Key Issues Highlighted:
- **Over-Provisioning:**
    - This occurs when the capacity exceeds the actual demand. It leads to wasted resources and higher than necessary expenses.
- **Under-Provisioning:**
    - This happens when there isn't enough capacity to meet demand, potentially leading to lost customers and revenue due to poor performance and availability.
- **Resource Cost:**
    - Infrastructure cost varies over time with different provisioning strategies. Traditional hardware incurs a more static, often linear cost irrespective of the demand, while automated virtualization adjusts costs dynamically based on actual usage.
### Solution
-> Cloud Computing

## Conceptional View
![[Pasted image 20240704110708.png#invert|600]]
### Service Delivery Models
- [[Infrastructure as a Service (IaaS)]]
- [[Platform as a Service (PaaS)]]
- [[Software as a Service (SaaS)]]
- **IaaS** is most flexible, providing basic infrastructure services to users who want to have the most control over the software environment without the cost of physical hardware.
- **PaaS** offers a development environment to application developers who don't want to deal with the complexity of infrastructure management.
- **SaaS** is the most inclusive, providing ready-to-use applications to end-users who are not concerned with operating environments or hardware capabilities.
### Service Deployment Models
#### 1. Private Cloud
A **private cloud** is exclusive to one organization. This setup provides the organization full control over its cloud resources, including where data is stored and how it is secured, making it ideal for businesses with strict data security, regulatory, and compliance requirements.
#### 2. Public Cloud
In a **public cloud**, the cloud services and infrastructure are owned and operated by third-party cloud service providers and shared with multiple organizations through the internet. This is the most common type of cloud computing deployment.
#### 3. Hybrid Cloud
A **hybrid cloud** combines private and public clouds, allowing data and applications to be shared between them. This flexibility enables businesses to maintain a private infrastructure for sensitive assets while also leveraging the robust computational resources of a public cloud for less sensitive data and applications.
#### 4. Community Cloud
A **community cloud** is shared between organizations with similar backgrounds, concerns, and requirements, such as regulatory, compliance, jurisdiction, etc. This type of cloud can be managed internally or by third-party vendors.
## Guidelines for Cloud Applications:
1. **Scalability of Components**: Each component of the application should be designed to scale independently. This ensures that the application can handle increased load by scaling the most necessary parts without the need to scale the entire application.
2. **Loose Coupling**: Components should be loosely coupled. This means they interact with each other without tight dependencies, which enhances the ability to modify, replace, or scale them independently.
3. **Parallelization**: Design components to perform operations in parallel to optimize the use of resources and improve performance.
4. **Resilience**: Design with the assumption that components will fail. Ask "What if this fails?" and ensure there are mechanisms to handle failures gracefully.
5. **Cost-Effectiveness**: Utilize on-demand cloud resources wisely to keep costs down. Designing for effective scaling and de-scaling is key to managing expenses.
## Architecture Principles:
1. **Decentralization**: Avoid centralization to eliminate single points of failure and scaling bottlenecks.
2. **Asynchrony**: Ensure the system can operate effectively without waiting for responses, thus handling operations asynchronously.
3. **Autonomy**: Components should be autonomous, able to operate based on local information without needing constant communication with a central authority.
4. **Local Responsibility**: Each component is responsible for its consistency and state, reducing the complexity of distributed operations.
5. **Controlled Concurrency**: Design operations that require minimal concurrency controls to avoid complex lock management and increase performance.
6. **Failure Tolerance**: Design for failures as a norm, ensuring the system can continue operating smoothly when components fail.
7. **Controlled Parallelism**: Use abstractions that allow for effective parallel execution to enhance performance and robustness.
8. **Decomposition**: Build the system from small, well-understood building blocks that can be easily combined into larger systems, promoting reusability and simplicity.
9. **Symmetry**: Design nodes to be symmetrical in functionality, minimizing the need for unique configurations and simplifying management.
10. **Simplicity**: Aim for simplicity in system design to avoid unnecessary complexity, which can lead to errors and difficult maintenance.
> [!abstract] Definition
> The "Grep The Web" application is a scalable cloud application leveraging Amazon Web Services (AWS) to perform distributed grep (global regular expression print) operations on web-scale datasets. 

### Zoom Level 1
![[Pasted image 20240704111734.png#invert|400]]
This is the highest level of abstraction, where the "GrepTheWeb Application" takes a list of document URLs and a regular expression (RegEx). The application processes these URLs to filter and return a subset of URLs that match the regular expression.
### Zoom Level 2
![[Pasted image 20240704111751.png#invert|400]]
This level details the interaction with AWS services:
- **Amazon SQS (Simple Queue Service)**: Used for message queuing to manage the communication between different components of the application, such as sending requests to start grep operations or fetch status updates.
- **Controller**: Manages the phases of the application (launching, monitoring, and shutting down instances).
- **Amazon EC2 Cluster**: Executes the grep operation across a cluster of EC2 (Elastic Compute Cloud) instances.
- **Amazon S3 (Simple Storage Service)**: Stores input files and the output results of grep operations.
### Zoom Level 3
![[Pasted image 20240704111808.png#invert|600]]
Provides an even more detailed look at the architecture:

- **Controllers (Launch, Monitor, Shutdown)**: Handle different phases of the operation. The Launch Controller initiates the grep operation, the Monitor Controller oversees its progress, and the Shutdown Controller terminates the process once completed.
- **Amazon SQS Queues** (Launch, Monitor, Shutdown, Billing): Facilitate message passing between controllers for different operational phases and billing.
- **Hadoop Cluster on Amazon EC2**: Uses a Hadoop framework for distributed computing of grep operations.
- **Billing Service**: Manages billing information related to the use of computational resources.
- **Status DB (Database)** on Amazon SimpleDB: Stores status information about the job being processed, such as progress and completion status.

### Phases of GrepTheWeb Architecture
![[Pasted image 20240704111820.png#invert|600]]
- **Launch Phase**: Starts the operation by setting up necessary AWS resources.
- **Monitor Phase**: Continuously checks the progress of the grep operation.
- **Shutdown Phase**: Closes down resources after the operation completes.
- **Cleanup Phase**: Cleans up residual data and shuts down unnecessary resources to avoid extra costs.
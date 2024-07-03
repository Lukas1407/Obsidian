- [[Simple Object Access Protocol (SOAP)]]
- [[Web Service Description Language (WSDL)]]
- [[Universal Description, Discovery and Integration (UDDI)]]

## Interaction Stack
![[Pasted image 20240701145951.png#invert|150]]
This stack deals with the actual communication that takes place when web services are called. It includes:
- [[Simple Object Access Protocol (SOAP)]]
- **Data Encoding (XML)**: XML is used for encoding all communications to a web service. This includes SOAP messages, which consist of an envelope that encapsulates XML data representing the call and its parameters.
- **Network Protocol (HTTP)**: SOAP messages are typically transported using HTTP, making them compatible with the existing web infrastructure. HTTP serves as a request-response protocol in the client-server computing model.
## Description Stack
![[Pasted image 20240701150033.png#invert|150]]
This stack specifies the public interface for the web services. It includes:
- [[Web Service Description Language (WSDL)]]
- **Data Type Definition (XML Schema)**: XML Schema is used in conjunction with WSDL to define custom data types used by the web service and to ensure that the data conform to specific standards which allows for proper integration and interoperability among different systems.
## Discovery Stack
This stack is concerned with the discovery of web services. It includes:
- [[Universal Description, Discovery and Integration (UDDI)]]
## How These Technologies Interact
- **Publishing a Web Service**: A service provider describes their web service using WSDL and publishes these details in a UDDI registry.
- **Finding a Web Service**: Service requestors can search the UDDI registry to find service descriptions that match their needs.
- **Binding to a Web Service**: Once a suitable web service is found, the requestor can use the details provided in the WSDL file to understand how to interact with the service, including the data types expected and the protocol and network address to use.
- **Invoking a Web Service**: The requestor then uses SOAP to call the web service, encapsulating the invocation details in XML format and sending them over HTTP to the address specified in the WSDL.
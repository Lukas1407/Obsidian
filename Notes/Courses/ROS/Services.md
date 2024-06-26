<mark style="background: #FFB86CA6;">Services allow nodes to send a request and receive a response</mark>

[[rosservice]] can be used to interact with a service.
[[rossrv]] prints out the service description

- Services should be defined in a separate folder -> [[Messages#Define Messages , Services and Actions in a folder |define Messages, Services and Actions]]
## srv file
- builds upon the [[Messages]] format
- stored inside the srv/ sub directory of a [[Packages]]
- <mark style="background: #FFB86CA6;">consists of a request and response separated by "---"</mark>:
	- Here is a very simple example of a service that takes in a string and returns a string:
		```
		string str
		---
		string str
		```

## Necessary Changes to the CMake and xml
- The same as for the [[Messages#Necessary changes to the XML and CMakeLists|messages]] but instead of `add_message_files` change the:
```CMake
# Generate services in the 'srv' folder
add_service_files(
FILES
ComputeDiskArea.srv
)
```
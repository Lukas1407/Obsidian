## Without Fail-Safe State
### Homogeneous Redundancy
The **Safety & Reliability Pattern** known as **Homogeneous Redundancy** is a critical strategy in systems design, particularly aimed at enhancing system reliability and ensuring operational safety in the presence of potential faults. This pattern is widely used in critical applications, such as in aerospace and other fields where failure can have severe consequences.

Homogeneous redundancy involves the use of multiple identical components or systems (channels) that perform the same functions. The primary objective is to protect against random faults, especially when a fail-safe state cannot be guaranteed by a single system component.
#### Implementation: Switch to Backup
- **Redundant Components**: The system includes one or more backup components that are identical to the primary component. These backups remain idle or run in parallel with the primary component.
- **Automatic Switch-over**: If the primary component fails, the system automatically switches control to one of the backup components. This switch-over can happen without user intervention and is designed to be seamless to maintain continuous system operation.
#### Example: Flight Computers
![[Pasted image 20240707143659.png#invert|800]]
In aircraft systems, where reliability and safety are paramount, homogeneous redundancy is often employed with flight computers:
- **Separate Sensors/Actuators**: Each flight computer is connected to separate sensors and actuators. This segregation ensures that a fault in one set of sensors or actuators does not affect the others.
- **Automatic Switching**: The system is designed to automatically detect a fault in the primary flight computer. Upon detection, it switches control to a backup flight computer. This ensures that the aircraft can continue to operate safely, even if one computer fails.
### Triple Modular Redundancy (TMR)
**Triple Modular Redundancy (TMR)** and **Heterogeneous Redundancy** are two advanced strategies used in systems engineering to enhance the reliability and safety of critical systems. These methods are particularly relevant in environments where system failure could lead to significant consequences.

**Triple Modular Redundancy** is a form of fault-tolerant design used to ensure high levels of system reliability and availability. It involves three identical components performing the same operations simultaneously. Here’s how it works:

- **Three Parallel Components**: Three identical modules (e.g., flight computers) run the same operations simultaneously. Each module is typically connected to separate sensors and actuators to isolate potential failures.
- **Majority Voting System**: The outputs of these three modules are compared, and the majority decision (i.e., two out of three) determines the final output of the system. This approach effectively masks any single point of failure.
- **Continuity of Operation**: Because two modules must fail simultaneously for the output to be erroneous, TMR significantly enhances reliability and ensures continuity of operation without interruption.

**Applicability to Software Solutions**: TMR can be applied to software solutions, especially in critical real-time systems like aerospace control systems. However, the effectiveness in pure software solutions may depend on the nature of potential faults (e.g., bugs that could similarly affect all three systems).
#### Example: Use of Redundancy in Aviation
![[Pasted image 20240707143904.png#invert|600]]
- **Triple Modular Redundancy in MCAS**: In aviation, TMR might be used in critical flight control systems such as the Maneuvering Characteristics Augmentation System (MCAS). Three flight computers calculate control signals independently, reducing the risk of erroneous actions due to a faulty sensor or a failed computer.

### Heterogeneous Redundancy
**Heterogeneous Redundancy** takes a different approach compared to Homogeneous and Triple Modular Redundancy by using diverse components or systems:
- **Independent Design and Implementation**: Each channel or module is independently designed and implemented. This diversification aims to prevent a single systematic fault from affecting all channels simultaneously.
- **Protection Against Diverse Faults**: By using different designs and possibly different technologies or algorithms, the system can withstand both random faults and systematic errors that might arise from shared vulnerabilities in design or implementation.
- **Continuity of Operation**: Like TMR, this approach also aims at maintaining continuous operation without relying on a failsafe state, but it offers an additional layer of protection against design-related faults.
#### Example: Use of Redundancy in Aviation
- **Heterogeneous Redundancy**: This could be implemented by using flight control software developed by different teams, possibly in different programming languages or using different algorithms, to further guard against systematic errors.
## With Fail-Safe State
### Protected Single Channel Pattern
The **Protected Single Channel** strategy involves implementing safety features within a single channel to detect and handle errors, under the assumption that a fail-safe state is achievable:
![[Pasted image 20240707144025.png#invert|600]]
- **Error Detection**: This system incorporates mechanisms to detect errors within the channel, which may include checking sensor data validity or the integrity of data processing.
- **Handling Transient Faults**: The system can address transient faults, potentially through methods like retrying operations, resetting the state, or using error-correcting codes.
- **Fail-Safe State**: Critical to this approach is the assumption that the system can enter a safe state if an unrecoverable error occurs. For example, in the context of a vehicle's Lane Keeping Assist System (LKAS), the system might deactivate itself and signal the driver to take over, ensuring safety even if system components fail.
#### Example
In a vehicle's LKAS, the system might continually validate the integrity of sensor data and monitor the actual steering behavior compared to expected behavior. If discrepancies or errors are detected beyond a threshold, the LKAS would shut off, allowing the driver to resume manual control, thus serving as the fail-safe state.
![[Pasted image 20240707144103.png#invert|600]]
### Monitor-Actuator Pattern
![[Pasted image 20240707144124.png#invert|600]]
The **Monitor-Actuator Pattern** is a specialized form of safety strategy that includes an independent monitoring system to oversee both the control channel and the actuators:
- **Independent Monitoring**: This approach uses an independent sensor or monitoring system to supervise the main operational channel and the actuators. The monitor checks that the actuators are functioning as expected and that the control commands are correctly executed.
- **Fault Detection**: The monitoring system can detect both systematic and random faults, ensuring that any failure in the actuation process is identified quickly.
- **Fail-Safe State Activation**: Upon detecting a fault, the system can trigger a fail-safe state to prevent dangerous operations or system behavior.
#### Example Application:
![[Pasted image 20240707144143.png#invert|600]]
- In an enhanced LKAS, a monitoring system could be implemented to check the integrity of data sent to the steering controller and the subsequent actuator response. If a fault is detected (e.g., the steering does not adjust as commanded), the monitor can disable the LKAS and alert the driver, effectively serving as a safeguard against both random and systematic errors.
### Sanity Check Pattern
![[Pasted image 20240707144246.png#invert|600]]
The **Sanity Check Pattern** is a simplified version of the Monitor-Actuator pattern. It provides a basic level of protection against faults by approximating the results of operations rather than thoroughly analyzing every outcome:
- **Lightweight Protection**: This pattern focuses on basic checks that confirm whether the results of operations are within plausible limits.
- **Approximation of Results**: Instead of detailed monitoring or complex calculations, the sanity check verifies that outputs or states are reasonable, which helps in detecting glaring errors or system malfunctions.
- **Fail-Safe State**: If results are outside of expected bounds, the system can move into a predefined safe mode to prevent further error propagation or damage.
This pattern is particularly useful in systems where processing power or resources are limited, but there's still a need to ensure that operations do not produce completely erroneous results.
### Watchdog Pattern
![[Pasted image 20240707144258.png#invert|600]]
The **Watchdog Pattern** is aimed at protecting against system hangs or freezes, often caused by software faults such as deadlocks or infinite loops:

- **Time-Based Monitoring**: A watchdog timer regularly checks for "liveness" signals from the system (often called heartbeats). If the timer expires without receiving a heartbeat, it assumes a fault has occurred.
- **Simple and Effective**: The pattern does not involve complex monitoring of the system's internal state but rather ensures that the system is still running and responsive.
- **Fail-Safe Trigger**: On failure to receive a heartbeat, the watchdog can reset the system or switch it to a safe state, thereby preventing a non-responsive system from causing harm.
#### Example
In an LKAS, if the system is expected to send a heartbeat signal every fixed interval, the watchdog timer checks for these signals. If the LKAS fails to send a heartbeat within the expected timeframe, the watchdog could shut down the LKAS, prompting manual control to resume by the driver.
![[Pasted image 20240707144337.png#invert|600]]
### Safety Executive Pattern
![[Pasted image 20240707144352.png#invert|600]]
The **Safety Executive Pattern** is designed for complex systems where achieving a fail-safe state involves multiple steps or complex logic:
- **Complex Safety Management**: This pattern is used in systems where simply turning off or resetting isn't sufficient to ensure safety. It involves a series of actions or checks that must be performed to safely manage a fault condition.
- **Coordinated Fault Management**: The safety executive may need to coordinate responses across multiple subsystems or processes to achieve a safe state.
This pattern is essential in systems where safety is critical, and the processes to achieve safety are non-trivial, requiring careful coordination and execution of several safety-related tasks.
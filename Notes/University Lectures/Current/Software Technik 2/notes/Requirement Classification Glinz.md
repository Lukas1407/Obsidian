## **1. Requirement Classification by Kind**

**Glinz** divides requirements into three main **kinds**:

- **Functional Requirements** describe what the system should do.
- **Quality Requirements** (also called non-functional requirements) describe how the system should behave or perform.
- **Constraints** limit the design or implementation of the system.

### **1.1 Functional Requirements (What the system should do)**
Functional requirements describe the **functions** and behaviors the system must provide to fulfill its purpose. These can be subdivided into several subkinds.
#### Subkinds:
1. **Functions**  
    Specifies the core tasks or operations the system should perform.  
    **Example:**
    - A banking application must allow users to transfer money between accounts.
    - A messaging app must allow users to send and receive messages.
2. **Data**  
    Requirements specifying the data the system should store, process, or handle.  
    **Example:**
    - An e-commerce platform must store user profiles, product details, and order history.
    - A medical records system must securely store and retrieve patient information.
3. **Stimuli**  
    Inputs or events that trigger system behavior.  
    **Example:**
    - When a user presses the “Submit” button on a form, the system should process and validate the input data.
    - An IoT system should react to temperature changes from a sensor.
4. **Reactions**  
    The system’s responses to stimuli or user inputs.  
    **Example:**
    - Upon entering a wrong password, the system should display an error message.
    - If the stock price in a trading app hits a predefined limit, the system should notify the user.
5. **Behavior**  
    Defines how the system behaves under different conditions, such as normal use or failure scenarios.  
    **Example:**
    - If a file download is interrupted, the system should automatically retry.
    - In a game, the system should transition smoothly between levels.


### **1.2 Quality Requirements (How the system should behave)**
These requirements describe **non-functional aspects** that determine the quality of the system.
#### Subkinds:
1. **Performance**  
    Defines how quickly or efficiently the system performs under specific conditions.  
    **Example:**
    - The search results must be displayed within 2 seconds.
    - The system should handle 1,000 concurrent users without performance degradation.
2. **Reliability**  
    Describes the system’s ability to operate without failure for a specified period.  
    **Example:**
    - The system should have a 99.9% uptime per month.
    - Data integrity should be maintained during power outages.
3. **Usability**  
    Defines how user-friendly and easy to use the system is.  
    **Example:**
    - The system must provide clear error messages for incorrect user input.
    - A mobile app should have intuitive navigation with minimal clicks.
4. **Security**  
    Describes how the system protects against unauthorized access and threats.  
    **Example:**
    - All user data must be encrypted during storage and transmission.
    - Users should be locked out after 5 failed login attempts.
5. **Availability**  
    Describes how often the system should be available for use.  
    **Example:**
    - The system must be accessible 24/7 with a maximum downtime of 1 hour per month.
    - Critical services should be restored within 15 minutes of failure.
6. **Portability**  
    Describes how easily the system can be transferred or run in different environments.  
    **Example:**
    - The application must run on both Windows and Linux without requiring modifications.
    - A website must work across different browsers (Chrome, Firefox, Safari).
7. **Maintainability**  
    Defines how easily the system can be modified, updated, or fixed.  
    **Example:**
    - The system’s codebase should follow a modular structure to simplify updates.
    - Bug fixes should be applied and tested within 24 hours of detection.
### **1.3 Constraints (Restrictions on design or implementation)**
Constraints restrict the way the system is built, deployed, or operated. These are typically non-negotiable requirements.
#### Subkinds:
1. **Physical**  
    Constraints related to hardware, size, weight, or physical components.  
    **Example:**
    - The system must fit within a 10x10x5 cm hardware module.
    - A sensor must operate in temperatures between -10°C and 50°C.
2. **Legal**  
    Constraints related to legal regulations, standards, or compliance.  
    **Example:**
    - The system must comply with GDPR for user data privacy.
    - The financial system must meet regulatory audit requirements.
3. **Cultural**  
    Constraints due to social or cultural factors.  
    **Example:**
    - The interface should support right-to-left text for Arabic users.
    - Color choices should avoid culturally sensitive meanings.
4. **Environmental**  
    Restrictions related to external environmental factors like temperature, humidity, or vibration.  
    **Example:**
    - The system should operate reliably in high-humidity environments.
    - The device must withstand vibrations during transportation.
5. **Design & Implementation**  
    Restrictions on how the system should be designed or implemented.  
    **Example:**
    - The application must be built using Python and Django.
    - The database should be implemented using MySQL.
6. **Interface**  
    Defines specific constraints on how the system interacts with other systems.  
    **Example:**
    - The application must use REST APIs for external communications.
    - The payment system must integrate with PayPal and Stripe.
## **2. Requirement Classification by Representation**
This classification explains how requirements are specified.
### **2.1 Operational Requirements**
- Describes requirements related to operations or tasks the system should support. **Example:** The system must generate daily reports for sales data.
### **2.2 Quantitative Requirements**
- Requirements that can be measured or quantified. **Example:** The system should handle 1,000 transactions per second.
### **2.3 Qualitative Requirements**
- Requirements described using qualitative or descriptive terms. **Example:** The system should provide an intuitive and user-friendly interface.
### **2.4 Declarative Requirements**
- High-level statements of intent or goals without specifying how to achieve them. **Example:** The system should ensure data security.

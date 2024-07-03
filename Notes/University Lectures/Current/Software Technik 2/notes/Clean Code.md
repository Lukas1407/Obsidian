Clean code, which is simple, readable, and well-organized, is much easier to modify and extend in response to changing requirements.
## Motivation
- Bad code exponentially decreases productivity
- Readability of code is important
- -> You write code for the next human to read, not for the compiler/interpreter/computer!
- Software requirements often change, for example new customer needs arise, or additional features are requested.
	- This dynamic nature means that software must be adaptable and flexible.
	- ![[Lehman's Law#Lehman's First Law]]
	- ![[Lehman's Law#Lehman's Second Law]]
- The effort required for a change increases the later it occurs
	- -> making changes early in the development process is less costly and less complex than making them later
	- Early and continuous attention to maintaining clean code helps avoid the costly consequences of poor early design choices.
- Iterative and incremental (especially agile) development methods claim that this curve can be kept flat
	- Each iteration includes planning, design, coding, and testing, which allows for frequent reassessment and adaptation of the software.
## What makes Code clean?
### 1. „elegant and efficient“
- Clean code should not only solve problems effectively but do so in a way that is aesthetically pleasing and streamlined. Efficiency means the code performs its tasks without wasting resources, while elegance refers to the simplicity and clarity of the solution.
### 2. „straightforward logic“
- The logic in clean code should be clear and obvious to any programmer reading it. It shouldn't involve complex conditionals or tangled control structures. Straightforward logic aids in readability and maintainability.
### 3. „minimal dependencies“ - Bjarne Stroustrup
- The inventor of C++ emphasizes that clean code should minimize dependencies. This means that each part of the code should have little reliance on other separate parts, making it easier to update and maintain one part without affecting others.
### 4. „does one thing well“ - Bjarne Stroustrup
- This echoes the Unix philosophy of designing programs that do one thing and do it well. In the context of clean code, it means that a function, module, or class should have a single, well-defined responsibility and accomplish it effectively.
### 5. „simple and direct“
- Clean code should be straightforward without unnecessary complexity. This simplicity in design and implementation makes the code more understandable and less prone to errors.
### 6. „reads like well-written prose“ - Grady Booch
- Grady Booch, a co-inventor of the Unified Modeling Language (UML), suggests that clean code should be as readable and understandable as well-written prose. This means that someone reading the code can follow its logic and purpose as easily as if they were reading a book.
### 7. „never obscures the designer‘s intent“ - co-inventor of UML
- Clean code clearly communicates the intentions of its creator. The design choices and purpose of each part of the code should be apparent, preventing misunderstandings and errors during future maintenance or enhancement.
### 8. „code that has been taken care of“ - Michael Feathers
- Michael Feathers describes clean code as code that shows it has been handled with care. This implies not just initial creation with good practices but ongoing refinement, optimization, and cleaning up to ensure the code remains efficient and robust.
### 9. Follows a defined Structure
- If you’re thinking about clean code, you should also think about your code’s underlying structure
- -> [[Object-Oriented Design (OOD)]]
- 

> [!abstract] Definition
> Any function or class should implement the behaviors that another programmer could reasonably expect. 

## Key Aspects
- **Predictability**: Functions, methods, and classes should behave in a manner that matches their names and the expectations set by similar constructs in other parts of the application or in other commonly used frameworks and libraries.
- **Consistency**: Consistent behavior across similar functions and classes reduces the learning curve and the potential for errors because users and developers can apply their knowledge universally.
- **Intuitiveness**: Code should be straightforward such that other developers can understand its functionality without needing to delve into the implementation details. When the behavior of the code is intuitive, reliance on extensive documentation decreases.
## Example
```java
String dayName = "Monday";
Day day = DayDate.StringToDay(dayName);
```
In this case, based on the Principle of Least Surprise:
- **Expected Behavior**: The method `StringToDay` should reliably convert a string that represents the name of a day into the corresponding `Day` enum value. For example, "Monday" should convert to `Day.MONDAY`.
- **Handle Common Variations**: Since different users might input day names in various formats, the method should ideally handle:
    - **Common abbreviations**: Like "Mon" for "Monday".
    - **Case insensitivity**: "monday", "MONDAY", and "Monday" should all be interpreted as `Day.MONDAY`.
### Breaking the Principle
```java
public class Food {
	String name;
	Date expirationDate;
	Fridge fridge = null;
	public Food(String name, Date expirationDate) {
		this.name = name;
		this.expirationDate = expirationDate;
	}
	public Fridge getFridge() { return fridge; }
}
```
- The variables `name`, `expirationDate` and `fridge` are  package-private (no modifier) which makes them accessible to the entire package. However these should be private as the visibility should be as restrictive as possible
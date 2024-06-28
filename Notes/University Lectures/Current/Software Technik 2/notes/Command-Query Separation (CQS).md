> [!abstract] Definition
> Command-Query Separation (CQS) states that every method should either be a command that performs an action, or a query that returns data to the caller, but not both. In other words, methods should either change the state of an object (command) or return some information about the object (query), but never do both. 
## Why CQS?
1. **Clarity in Design**: Separating commands from queries clarifies the design by simplifying the understanding of what each method does. A method is either performing an action or answering a question about the state of the system.
2. **No Side Effects in Queries**: Queries should not change the state of an object. This ensures that queries are safe to use in any context without risking unintended modifications to the object's state.
3. **Easier to Test and Debug**: Testing commands and queries separately becomes straightforward. Queries can be tested for correctness based on returned values without worrying about the side effects of state changes, and commands can be tested for their effect on the state of the system.
## Example
```java
public class Dice {
	private int faveValue;
	...
	public void roll() {
		faceValue = (int) (Math.random() * 6 + 1);
	}
	public int getFaceValue() {
		return faceValue;
	} 
}
```
- -> Shows good application of CQS
- `roll()`: A command method that changes the internal state (`faceValue`).
- `getFaceValue()`: A query method that returns the current state without altering it
## Drawbacks and Violations
- **More Method Invocations**: To maintain separation, sometimes it's necessary to introduce more methods which can increase the number of method calls in the system.
- **Violations in Common Patterns**: The iterator pattern commonly seen in Java and C# often violates CQS because:
    - `next()`: Acts as both a command (advancing the iterator) and a query (returning the next element).
    - This is refined in newer designs like in "New Java" with `isCurrentValid()`, `current()`, and `moveNext()`, clearly separating the querying and command aspects, respectively.
## Relation to [[SOLID Principle]]
CQS complements the SOLID principles, particularly the Single Responsibility Principle (SRP) and the Interface Segregation Principle (ISP):
- **SRP (Single Responsibility Principle)**: By enforcing methods to either command or query, CQS aligns with SRP, which advocates for a class or method to have one reason to change. This keeps the system modular with each part having a clear responsibility.
- **ISP (Interface Segregation Principle)**: CQS encourages designing finer-grained interfaces that separate the behaviors of objects into commands and queries, which can help in adhering to ISP by ensuring that classes implementing interfaces are not forced to implement methods they don't use.
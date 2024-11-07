> [!abstract] Definition
> The Single Level of Abstraction principle ensures that a function, class, or component in a program operates at a single level of detail. This principle helps maintain a clean and understandable code structure, much like a well-written newspaper article.

## Details
- **Consistent Abstraction Levels**: Each function should operate at one level of abstraction. This means that all the statements within a function should be at the same level of detail. For instance, if a function is designed to outline high-level steps of a process, each of those steps should be similarly abstract, and any lower-level details should be abstracted into separate functions.
- **Enhance Readability**: Just like reading a newspaper where you expect a headline to give you a summary and the details to be fleshed out in the article body, SLA helps the reader of your code understand the high-level functionality quickly without getting bogged down by details initially.
- **Code Organization**: In a class, the methods should ideally be organized such that the abstraction levels decrease as you read down the file. This setup means that high-level methods are followed by their helper methods, providing a logical flow that mimics depth-first navigation through the code.
- **Practical Implementation**:
    - If a method is performing operations at varying levels of detail, consider extracting the lower-level operations into their own methods.
    - Arrange methods in a class so that the reader encounters the highest level of abstraction first, with subsequent methods providing supporting details.
## Example
```java
// High-level method using SLA
public void processOrder(Order order) {
    validateOrder(order);
    calculateShipping();
    finalizeOrder(order);
}

// Supporting methods at a lower level of abstraction
private void validateOrder(Order order) {
    // Details about validation
}

private void calculateShipping() {
    // Details about shipping calculations
}

private void finalizeOrder(Order order) {
    // Details about finalizing the order
}
```
- **[[You ain’t gonna need it (YAGNI)|YAGNI]]**: Each method does exactly what is necessary—nothing more. There are no speculative features.
- **SLA**: `processOrder` provides a high-level overview by calling other methods that encapsulate specific details, maintaining a consistent level of abstraction.
### Breaking the Principle
```java
public ArrayList < Food > getExpiredFoods () { 
	ArrayList < Food > expiredFoods = new ArrayList < >(); 
	Date now = Date.from(Instant.now());
	for ( Food food : foods ){
		if(food.expirationDate.before(now)){
			expiredFoods.add(food);
		}
	}
	foods.removeAll(expiredFoods);
	return expiredFoods;
}
```
- This function combines abstraction levels by also modifying the `foods` list
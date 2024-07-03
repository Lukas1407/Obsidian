The Observer pattern is a fundamental design pattern in software engineering, particularly useful in scenarios where an object, referred to as the subject, needs to notify a list of dependents, referred to as observers, about any state changes. It promotes a loose coupling between the subject and its observers, allowing for efficient communication between objects without making hard-coded dependencies. 

### Components of the Observer Pattern
![[Pasted image 20240701101809.jpg#invert|500]]
#### 1. **Subject**
- **Role**: Maintains a list of observers, adds or removes observers to this list, and notifies them of state changes.
- **Operations**:
  - `attach(Observer)`: Adds an observer to the list.
  - `detach(Observer)`: Removes an observer from the list.
  - `notify()`: Notifies all attached observers of a change in state.
#### 2. **Observer**
- **Role**: Provides an update interface that gets called by the subject to notify this observer of any state changes.
- **Operation**:
  - `update()`: The method through which observers respond to notifications from the subject.
#### 3. **ConcreteSubject**
- **Role**: Stores the state of interest to ConcreteObservers and sends a notification to its observers when its state changes.
- **Attributes and Operations**:
  - `subjectState`: The state of the ConcreteSubject.
  - `getState()`: Returns the current state.
  - `setState()`: Sets the subject's state and notifies observers.
#### 4. **ConcreteObserver**
- **Role**: Maintains a reference to a ConcreteSubject object, stores state that should stay consistent with the subject's, and implements the Observer updating interface to keep its state up to date.
- **Attributes and Operations**:
  - `observerState`: The state of the observer, which needs to be consistent with the subject's.
  - `update()`: Updates the observer state based on the subject's state.
### How It Works: Collaboration Details
- When the state of the `ConcreteSubject` changes (through `setState()`), it calls `notify()` to inform all its observers of the change.
- The `notify()` method iterates over the list of observers and calls `update()` on each observer.
- Each `ConcreteObserver`, upon receiving the `update()` call, may fetch new state data from the `ConcreteSubject` via `getState()`, ensuring that the observer's state remains consistent with the subject's state.
### Applicability and Use Cases
- **Decoupling of Components**: Useful when one part of an application changes frequently, and you have many other parts that need to be updated automatically. The observers can be added or removed dynamically, allowing for flexibility.
- **Event Management Systems**: In GUI frameworks or event-driven systems where changes in one component (like a button click) need to propagate updates to other components.
- **Publish-Subscribe Systems**: Implements a publish-subscribe model where publishers send data to subscribers without needing to know who those subscribers are, similar to newsletters or notifications systems.
### Real-World Example
Consider a stock market tracking application:
- **ConcreteSubject**: Stock Ticker that holds information about stock prices.
- **ConcreteObservers**: Displays or logs that need to show or act on these stock prices.
- **Functionality**: Whenever a stock price updates (state changes), all registered displays and logs are automatically updated to reflect the new price.

## [[Model-View-Controller (MVC)]] & Observer
![[Pasted image 20240701101916.png#invert|500]]
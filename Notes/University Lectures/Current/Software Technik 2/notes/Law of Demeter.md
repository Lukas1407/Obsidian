> [!abstract] Definition
> The Law of Demeter, often phrased as "don't talk to strangers," is a principle of object-oriented design that aims to promote loose coupling between software components. This principle restricts the number of classes a particular object or method interacts with and how it interacts with those classes. The intent is to reduce the dependencies an object has on the internal structure or operations of other objects it uses, leading to more modular and maintainable code. 
## Details
The Law of Demeter (LoD) suggests that a given object should only have direct knowledge of and interact with closely-related objects. More specifically, a method `m` of a class `C` should only directly call methods on:
1. **The object itself (`C`)**: The method can call other methods within the same object.
2. **Objects passed as arguments to `m`**: The method can call methods on objects that are passed to it.
3. **Objects created by `m`**: The method can call methods on objects that it creates.
4. **Objects held in instance variables of `C`**: The method can call methods on objects that the class owns as instance variables.
## Example
### Violation
![[Pasted image 20240628123115.png#invert|250]]

- The `Fahrer` class's `fahren()` method creates an instance of `Auto`.
- It then directly interacts with the `Auto`'s `Motor` instance by calling `meinAuto.motor.anlassen()`.
- This code violates the Law of Demeter because the `Fahrer` method directly calls a method on an object two levels deep (`motor`), which is part of the internal structure of `Auto`.
### Improved Design
![[Pasted image 20240628123133.png#invert|250]]
- The `Auto` class provides a method `fahrbereitmachen()`, which internally calls `motor.anlassen()`.
- The `Fahrer` class's `fahren()` method now only interacts with the `Auto` class by calling `meinAuto.fahrbereitmachen()`.
This refactoring adheres to the Law of Demeter as the `Fahrer` class no longer knows about the `Motor` inside the `Auto`. It only calls methods on the `Auto` object it directly creates. This change reduces the dependency of the `Fahrer` class on the internal structure of the `Auto` class, making the code more maintainable and flexible to changes in the `Auto` or `Motor` classes.
## Benefits of Following the Law of Demeter
1. **Reduced Coupling**: By limiting interactions to closely-associated objects, the code becomes less dependent on the internal structure of other objects, which in turn reduces the impact of changes within those objects.
2. **Increased Modularity**: Each class is more isolated and focused on its own responsibilities, leading to clearer, more modular code.
3. **Easier Maintenance**: Changes in one part of the system are less likely to require changes in a completely different part, as classes are less intertwined.
4. **Enhanced Testability**: Testing a class that adheres to the Law of Demeter is generally easier because it does not rely heavily on the internal behaviors of its dependencies.
## Naming Conventions
### Standardized Naming
- It is crucial for a team or project to establish and adhere to consistent naming conventions. 
- This consistency helps new team members quickly become productive and aids in the maintenance of the codebase.
### Meaningful and Intention-Revealing Names
- **Clear and Descriptive**: Names should clearly reflect what the variable, function, or class does. For instance, a method name like `getThem()` is vague and uninformative. On the other hand, `getFlaggedCells()` immediately suggests that it returns a list of cells that have been flagged, which is more intuitive and helpful.
- **Avoid Disinformation**: Names should not mislead the reader about what the code does. For example, using `list1` provides no information about what the list contains or its purpose. In contrast, `flaggedCells` clearly indicates that the list contains cells that have been flagged.
- **Make Meaningful Distinctions**: Different entities should have clearly distinguishable names. For example, `AccountName` vs. `Name` in an `Account` class should have a clear distinction to justify the more specific term.
- **Avoid Unnecessary Context**: Avoid embedding the type of the variable in its name (like `nameString` or `accountList`), which can clutter the code. Modern IDEs provide type information, making such prefixes generally unnecessary.
### Avoid Non-Informative Names
- Names like `n1` or `data` do not provide any clue about the content they hold. Even temporary variables used in loops should be given descriptive names if their role extends beyond simple iteration.

## Commenting Conventions
- **Rewrite Rather Than Commenting Bad Code**: As advised by Kernighan and Plaugher, instead of commenting on unclear or complicated code, it's better to rewrite it to be clearer. For example, replacing complex conditions with a well-named method can eliminate the need for explanatory comments.
### Useful Comments
- **Explaining Non-Obvious Code**: Sometimes, comments are necessary to explain why certain decisions were made, especially if they relate to legal issues, performance considerations, or specific algorithm choices.
- **Warnings and Important Notes**: Comments are also useful for highlighting potential issues or explaining why certain sections of code are critical. For instance, indicating that a particular object isn’t thread-safe can prevent future errors.
- **TODOs and Open Issues**: Comments are a good place to note improvements that need to be addressed, helping prioritize future development efforts.
### Avoid Redundant Comments
- **Javadoc and Documentation Comments**: While documentation for public APIs is crucial, avoid trivial comments in the code that simply restate what is evident from the code itself. For instance, commenting `/** The name. */` above a variable declared as `private String name;` is redundant.
### Avoid Commented-Out Code
- **Trust Version Control**: Instead of leaving old or unused code commented out in the codebase, it's better to remove it altogether. Version control systems like Git provide a way to retrieve previous versions of the code if needed, keeping the current codebase clean and manageable.

## Formatting Conventions
- Proper formatting is critical in making your code more readable and maintainable.
### Vertical Formatting
- **Visual Separation of Concepts**: Using vertical whitespace (blank lines) in code helps in separating concepts that are not closely related, thereby improving readability. For instance:
    - **After Imports**: A blank line after import statements separates them from the rest of the code.
    - **Between Methods**: Adding a blank line between methods makes it clear where one method ends and another begins.
    - **Inside Methods**: Strategic blank lines inside methods can separate steps in the logic, making the method easier to follow.
- **Closely Related Lines**: Lines of code that are tightly related should appear close together without intervening blank lines. This suggests a relationship or a progression in the logic, which helps in understanding the flow of the program. For example, variable declarations might be immediately followed by their first use
### Horizontal Formatting
- **Operator Spacing**: Spaces around operators can make the code easier to read and can help in understanding the precedence of operations. For example:
    - Correct: `x = a * (b + c)`
    - Less readable: `x=a*(b+c)`
- **Parameter Separation**: When defining or invoking methods, parameters should be separated by spaces after commas to improve readability. For example:
    - Correct: `print(sum, average, median)`
    - Less readable: `print(sum,average,median)`
- **Indentation**: Indentation is crucial for understanding the structure of code, particularly the scope of loops, conditionals, and other blocks. Consistent use of indentation (using spaces or tabs) across the team helps in maintaining a uniform structure in the codebase.
### Use of IDE Auto-Formatting
-> Makes formatting consistent, customizable and efficient for the team


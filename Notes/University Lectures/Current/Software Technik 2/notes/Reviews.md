Reviews in software development are a crucial method used to examine and improve the quality of a software artifact by identifying and removing errors before further stages of development, such as testing. 
## Types of Reviews
- **Inspection**: A formal, thorough review process where each part of a software artifact is examined in detail by a team to detect defects.
- **Team Review**: Involves the project team reviewing components together, often less formal than inspections.
- **Walkthrough**: A session where the author of a software artifact guides participants through the material to gather feedback and identify issues.
- **Pair Programming**: An agile development technique where two programmers work together at one workstation; one writes the code while the other reviews each line of code as it is typed in.
- **Pass-around**: Sending a document or code segment to multiple reviewers to get feedback asynchronously.
- **Ad-hoc Review**: An informal review without a structured process, often spontaneous.
## Effectiveness of Reviews
Reviews are recognized as highly effective in improving software quality:
- **Error Detection**: Rigorous inspections can identify and remove a significant percentage (60 – 90%) of errors before the testing phase begins. This preemptive error detection is critical in reducing the cost and time spent on later bug-fixing during testing and after deployment.
- **Cost Efficiency**: Despite the up-front investment in time and resources, reviews can significantly reduce downstream costs associated with fixing bugs post-development.
## Challenges and Underuse
Despite their proven effectiveness, reviews are not as widely adopted as one might expect:
- **Economic Reasons**: Consultants may not promote reviews because they do not offer recurring revenue like other more complex methodologies that require ongoing consulting services.
- **Lack of Novelty**: Because reviews are a well-established practice, they might be overlooked in favor of newer, more innovative technologies or methods.
- **Up-front Costs**: Reviews can be perceived as increasing the initial costs of the software development process. Organizations under tight budget constraints might opt to skip this step in favor of direct implementation.
- **Psychological Factors**: Reviews require careful handling of team dynamics and individual ego. They demand a culture where constructive criticism is valued over personal achievement, which can be challenging to maintain.
## Dangers of Reviews
1. **Complacency in Authoring**:
   - **Testing Omission**: A false sense of security might develop, leading teams to omit thorough testing with the assumption that reviews will catch all errors.
   - **Author Frustration**: Receiving constant critique can be demoralizing, especially if not handled sensitively, leading to reduced motivation or feelings of violation.
   - **Sloppy Practices**: Authors might become careless, relying too heavily on reviews to catch errors, which can decrease the overall quality of initial submissions.
2. **Obstacles to Effective Reviews**:
   - **Time Constraints**: Under tight deadlines, managers might reduce the allotted time for reviews, compromising their effectiveness.
   - **Poor Preparation**: If participants are not well-prepared, reviews can become unproductive and inefficient.
   - **Code Obfuscation**: Authors may intentionally make code or documents complex to shield their work from criticism or to reduce the likelihood of changes.
## Benefits of Reviews
1. **Quality and Correctness**:
   - As a primary verification technique, reviews significantly enhance the quality and correctness of software by identifying defects early in the development cycle.
2. **Improved Project Understanding**:
   - Reviews facilitate a deeper understanding of the project as they encourage the dissemination of knowledge across the team.
3. **Knowledge Sharing**:
   - They help distribute knowledge more broadly among team members, reducing dependency on any single individual and mitigating the risk associated with unavailable team members.
4. **Educational Value**:
   - Novices learn from more experienced developers about coding styles and best practices, which can accelerate their growth and integration into the team.
5. **Enhanced Readability and Maintenance**:
   - By promoting cleaner code and better documentation, reviews help ensure that the codebase is understandable and maintainable by anyone in the team, not just the original authors.
## Different Phases of a Review
1. **Planning**: Define the scope and objectives of the review, select participants, and schedule the review meeting.
2. **Overview**: Participants gain a common understanding of the review object and the review process.
3. **Preparation**: Participants individually examine the documents to be reviewed, noting potential issues according to their role.
4. **Meeting**: All participants gather to discuss the findings. The goal is to identify defects, not to solve them during this meeting.
5. **Rework**: Based on the meeting’s outcomes, the author revises the document to correct identified defects.
6. **Follow-Up**: The moderator (or a designated verifier) checks if the defects have been adequately addressed and that no new issues have been introduced.
7. **Causal Analysis**: After the review cycle, the team analyzes the defects found to determine their causes and improve future processes.
## Roles of a Review
- **Author**: The person who created the document or code being reviewed. Responsible for revising the work based on feedback.
- **Moderator**: Facilitates the review process, ensuring that it is efficient and effective. Acts as the chair of the review meeting.
- **Reader(s)**: Review the document or code critically to find potential defects. They represent different perspectives and expertise.
- **Recorder**: Documents issues, comments, and decisions made during the review meeting.
- **Verifier**: Confirms that the defects identified during the review have been corrected in the rework phase.
## Documents Involved in a Review
![[Pasted image 20240707111806.png#invert|500]]
- **Author Objectives**: Prepared by the author to guide the focus of the review.
- **Meeting Notice**: Sent by the moderator to inform participants about the review schedule and expectations.
- **Inspection Package**: Includes the document or code to be reviewed along with any relevant materials; prepared by the moderator or author.
- **Specification (or previous version)**: Provides context or benchmarks against which the document is reviewed.
- **Typo List**: A compilation of minor errors noted by inspectors, aimed at the author for correction.
- **Issue Log**: Maintained during the review to record defects, concerns, and comments for further action.
- **Corrected Deliverable**: The revised document post-rework, submitted to the verifier.
- **Inspection Summary Report**: Summarizes the outcome of the review and is usually communicated to management.
- **Lessons Learned**: Documented by the moderator to capture insights and improvements for future reviews.
- **Process Improvements**: Suggestions from inspectors on how to enhance the review process itself, typically directed at the process group responsible for quality standards.
## Different Review Types Explained
![[Pasted image 20240707111923.png#invert|500]]
1. **Inspection**:
   - **Planning**: Organizing the review process.
   - **Preparation**: Reviewers prepare by understanding the documents/code to be reviewed.
   - **Meeting**: Formal discussion where findings are presented and logged.
   - **Correction**: Issues found are addressed by the author.
   - **Verification**: Verification that all corrections are properly implemented.
2. **Team Review**:
   - Similar to inspections but may be less formal. Involves planning, preparation, a meeting, correction of defects, and verification.
3. **Walkthrough**:
   - A walkthrough is a more informal review that still involves planning, preparation, a meeting, and corrections. It’s often used for educational purposes or gaining feedback.
4. **Pair Programming**:
   - **Continuous**: Involves two programmers working together at one workstation, continuously reviewing each other’s code.
   - **Correction**: Immediate, as issues are addressed on-the-fly.
   - **Verification**: Also continuous as the work progresses.
5. **Peer Deskschecks**:
   - **Preparation**: Informal preparation, potentially reviewing the code before discussing it.
   - **Meeting**: This could be an informal discussion rather than a formal meeting.
   - **Correction**: Defects identified are corrected after the deskscheck.
6. **Ad hoc Pass Around**:
   - **Meeting**: The document or code is passed around among team members for ad hoc review.
   - **Correction**: Feedback is given for immediate correction.


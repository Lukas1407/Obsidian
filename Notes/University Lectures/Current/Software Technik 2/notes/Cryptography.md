Cryptography is essential for securing communications and data, especially where not all parties involved may be fully trustworthy. It addresses several core security goals through various cryptographic methods and protocols. Let’s explore the key concepts and practical applications in cryptography you’ve mentioned:

### Classical Goals of Cryptography

1. **Confidentiality**: Ensuring that information is accessible only to those authorized to have access.
2. **Integrity**: Assuring that information is accurate and unaltered from its original form.
3. **Authenticity**: Verifying that the data comes from a trusted source.
4. **Non-repudiability**: Ensuring that a party in a communication cannot deny the authenticity of their signature on a message or the sending of a message that they originated.

### Pseudo Randomness in Cryptography

Cryptographic security often hinges on the quality of randomness. Many cryptographic protocols require random numbers to ensure security, particularly for key generation, encryption algorithms, and digital signatures.

#### Issues with Pseudo Random Number Generators (PRNGs):
- **Non-cryptographic PRNGs** are unsuitable for cryptographic purposes because they produce predictable outputs that could be reproduced if the initial state is known.
- **Cryptographically Secure PRNGs (CSPRNGs)** are designed to withstand attempts to predict future outputs based on past or present ones, even if part of their internal state is compromised.

#### Characteristics of CSPRNGs:
1. **Unpredictability**: You cannot predict the next output bit with reasonable computational effort, even knowing all previous bits.
2. **Forward Secrecy**: If a CSPRNG’s state is compromised, it should not be possible to retroactively determine its previous outputs.
3. **Backward Security**: Knowledge of the current state and all previous outputs should not help in predicting future outputs.

#### Proper Seeding of CSPRNGs:
- **Seed Quality**: The seed for a CSPRNG must be unpredictable and contain sufficient entropy (randomness collected from various sources) to resist attacks.
- **Sources of Entropy**: Could include hardware inputs like mouse movements, keyboard timings, network traffic, or hardware dedicated to monitoring quantum phenomena like photon arrival timings.

### Practical Considerations:
- When implementing cryptography, ensure to use well-established libraries and frameworks that comply with current standards.
- Avoid creating custom cryptographic solutions, as these are likely to be less secure without extensive peer review and testing.
- Regularly update cryptographic libraries to incorporate fixes for known vulnerabilities.

Understanding and applying these cryptographic principles correctly is vital for building secure systems that protect against unauthorized access and data manipulation.
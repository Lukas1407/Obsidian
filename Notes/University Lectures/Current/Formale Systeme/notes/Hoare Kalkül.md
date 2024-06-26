> [!abstract] Definition
> Der Hoare-Kalkül ist ein formales System zur Verifikation von Programmen. Es ermöglicht, formale Beweise für die Korrektheit von Programmen zu führen, insbesondere in Bezug auf Vor- und Nachbedingungen. 
## Hoare-Kalkül und Substitutionslemma
### Grundbegriffe des Hoare-Kalküls
- **Hoare-Tripel**: Ein Hoare-Tripel hat die Form \(\{P\} \; C \; \{Q\}\), wobei:
  - $P$ eine Vorbedingung ist, die vor der Ausführung des Programms $C$ gilt.
  - $C$ ein Programm oder ein Programmausschnitt ist.
  - $Q$ eine Nachbedingung ist, die nach der Ausführung des Programms $C$ gelten soll.
- **Zuweisungsregel**: Diese Regel beschreibt, wie sich der Zustand eines Programms nach einer Zuweisung ändert.
### Substitutionslemma
- **Substitution**: Eine Substitution $\{x/s\}$ ersetzt eine Variable $x$ durch einen Term $s$. Dabei ist es wichtig, dass diese Substitution kollisionsfrei ist, d.h., sie führt nicht zu ungewollten Bindungen oder Verwechslungen von Variablen.
## Zuweisungsregel im Hoare-Kalkül
Die Zuweisungsregel im Hoare-Kalkül ist wie folgt definiert:$$\{ \{x/s\}A \} \; x := s \; \{ A \}$$
### Erklärung der Zuweisungsregel
- **Vorbedingung $\{x/s\}A$**: Die Vorbedingung $\{x/s\}A$ bedeutet, dass die Formel $A$ gilt, nachdem $x$ durch $s$ ersetzt wurde. Dies stellt sicher, dass $A$ unter der neuen Variablenbelegung wahr ist.
- **Zuweisung $x := s$**: Der Ausdruck $x := s$ bedeutet, dass die Variable $x$ den Wert des Terms $s$ zugewiesen bekommt. Dies verändert den Zustand des Programms.
- **Nachbedingung $A$**: Die Nachbedingung $A$ soll nach der Ausführung des Programms $x := s$ gelten. Das bedeutet, dass die Formel $A$ unter dem neuen Zustand des Programms wahr ist.
### Hintergrund-Interpretation
- **H**: Eine Interpretation, die die eingebauten Symbole der Programmiersprache interpretiert.
- **$\beta$**: Eine Variablenbelegung, die den aktuellen Zustand des Programms repräsentiert.
### Zustand nach der Zuweisung
- **Vor der Zuweisung**: Angenommen, die Formel $\{x/s\}A$ ist wahr im Zustand $\beta$:$$\text{val}_H,\beta(\{x/s\}A) = \text{wahr}$$
- **Nach der Zuweisung**: Nach der Ausführung der Zuweisung $x := s$ wird ein neuer Zustand $\beta'$ erreicht, bei dem:$$\beta'(x) := \text{val}_H,\beta(s)$$$$\beta'(y) := \beta(y) \text{ für } y \neq x$$
- **Nachbedingung**: Die Regel behauptet, dass die Nachbedingung $A$ im neuen Zustand $\beta'$ gilt:$$\text{val}_H,\beta'(A) = \text{wahr}$$
Das bedeutet, dass $A$ nach der Zuweisung wahr ist, wenn $\{x/s\}A$ vorher wahr war.
### Verbindung zum Substitutionslemma
Das Substitutionslemma besagt, dass die Bewertung einer Formel $A$ nach einer Substitution $\{x/s\}$ unter der neuen Belegung $\beta \{x/s\}$ gleich der Bewertung der Formel $A$ im neuen Zustand ist.
#### Beispiel zur Veranschaulichung
Stellen wir uns vor, wir haben die Formel $A$, die lautet: „$x$ ist größer als 5“, also $A = x > 5$.
- **Vorbedingung**: Die Substitution $\{x/s\}A$ könnte z.B. $\{x/7\}(x > 5)$ sein, was zu $7 > 5$ führt.
  - Wenn dies wahr ist, bedeutet es, dass $7 > 5$ stimmt.
- **Zuweisung**: $x := 7$. Dies bedeutet, dass $x$ den Wert 7 annimmt.
- **Nachbedingung**: Die Nachbedingung $A$ wäre $x > 5$. Im neuen Zustand ist $x$ gleich 7, und damit ist $7 > 5$ wahr.
Das zeigt, dass die Formel $A$ nach der Zuweisung $x := s$ im neuen Zustand wahr ist.


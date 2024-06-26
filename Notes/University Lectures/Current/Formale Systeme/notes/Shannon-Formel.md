> [!abstract] Definition
> Diese Formeln verwenden einen speziellen dreistelligen Operator, der als $\text{sh}$ bezeichnet wird, und ermöglichen die Darstellung von logischen Ausdrücken basierend auf bestimmten Bedingungen. 
## Shannon-Formeln: Aufbau und Semantik
### Bestandteile einer Shannon-Formel
- **Dreistelliger Operator $\text{sh}$**: Dies ist ein spezieller logischer Operator, der drei Argumente nimmt.
### Definition des $\text{sh}$-Operators
Der dreistellige Operator $\text{sh}(P_1, P_2, P_3)$ hat die folgende Semantik:
- **Interpretation**: Der Wert von $\text{sh}(P_1, P_2, P_3)$ hängt vom Wert der ersten Variablen $P_1$ ab:
  - Wenn $P_1$ „falsch“ (F) ist, dann nimmt $\text{sh}(P_1, P_2, P_3)$ den Wert von $P_2$ an.
  - Wenn $P_1$ „wahr“ (W) ist, dann nimmt $\text{sh}(P_1, P_2, P_3)$ den Wert von $P_3$ an.
  
-> Diese Funktionsweise kann als bedingte Auswahl zwischen $P_2$ und $P_3$ basierend auf dem Wert von $P_1$ verstanden werden.
### Tabellenform der Semantik
- Hier ist die Semantik des $\text{sh}$-Operators in Tabellenform dargestellt:

| valI(P1) | valI(P2) | valI(P3) | valI(sh(P1, P2, P3)) |
|----------|----------|----------|----------------------|
| W        | W        | W        | W                    |
| W        | W        | F        | F                    |
| W        | F        | W        | W                    |
| W        | F        | F        | F                    |
| F        | W        | W        | W                    |
| F        | W        | F        | W                    |
| F        | F        | W        | F                    |
| F        | F        | F        | F                    |

- Diese Tabelle zeigt, wie der Wert von $\text{sh}(P_1, P_2, P_3)$ von den Werten von $P_1$, $P_2$ und $P_3$ abhängt.
## Beispiele für Shannon-Formeln
### Beispiel 1: Einfache Shannon-Formel
- Betrachten wir eine einfache Shannon-Formel:$$\text{sh}(P, Q, R) $$
- Wenn $P$ „wahr“ ist (W), dann ist der Wert der Formel der Wert von $R$.
- Wenn $P$ „falsch“ ist (F), dann ist der Wert der Formel der Wert von $Q$.

**Interpretation**:
- Wenn $P = W$, $Q = W$, und $R = F$, dann:$$\text{valI}(\text{sh}(P, Q, R)) = F$$
- Wenn $P = F$, $Q = W$, und $R = F$, dann:$$\text{valI}(\text{sh}(P, Q, R)) = W$$
### Beispiel 2: Verschachtelte Shannon-Formel
- Betrachten wir eine verschachtelte Formel:$$\text{sh}(P, \text{sh}(Q, 0, 1), \text{sh}(R, 1, 0))$$
- Hierbei haben wir:
	- $\text{sh}(Q, 0, 1)$ nimmt den Wert 0, wenn $Q$ „falsch“ ist, und den Wert 1, wenn $Q$ „wahr“ ist.
	- $\text{sh}(R, 1, 0)$ nimmt den Wert 1, wenn $R$ „falsch“ ist, und den Wert 0, wenn $R$ „wahr“ ist.
**Interpretation**:
- Wenn $P = W$, $Q = W$, und $R = F$: $$\text{valI}(\text{sh}(P, \text{sh}(Q, 0, 1), \text{sh}(R, 1, 0))) = \text{valI}(\text{sh}(R, 1, 0)) = 1$$
- Wenn $P = F$, $Q = F$, und $R = W$: $$\text{valI}(\text{sh}(P, \text{sh}(Q, 0, 1), \text{sh}(R, 1, 0))) = \text{valI}(\text{sh}(Q, 0, 1)) = 0$$
## Eigenschaften
$$\text{sh}(P_{1},P_{2}, P_{3})\leftrightarrow(\neg P_{1}\land P_{2})\lor(P_{1}\land P_{3})$$
$$\text{sh}(P_{1},P_{2}, P_{3})\leftrightarrow(\neg P_{1}\lor P_{3})\land(P_{1}\lor P_{2})$$
$$\neg\text{sh}(A,B, C)\leftrightarrow\text{sh}(A,\neg B, \neg C)$$
## Normierte Shannon-Formel
> [!abstract] Definition
> Normierte Shannon-Formeln sind eine spezielle Form von Shannon-Formeln, die eine feste Ordnung auf der Menge der Aussagevariablen verwenden. Diese Form von Formeln ist besonders nützlich, da sie es ermöglicht, jede aussagenlogische Formel in eine strukturierte und normierte Form umzuwandeln, die eine systematische Analyse und Manipulation erleichtert. 
### Fixierte Ordnung
- Wir legen eine feste Ordnung auf der Menge der Umgebungsvariablen fest. Beispielsweise, wenn die Variablen $P_1, P_2, \ldots, P_n$ geordnet sind, dann ist $P_1 < P_2 < \ldots < P_n$.
### Definition von normierten Shannon-Formeln
1. **Konstanten als normierte sh-Formeln**:
   - Die Konstanten $0$ und $1$ sind normierte sh-Formeln.
     - $0$ repräsentiert „falsch“.
     - $1$ repräsentiert „wahr“.
2. **Shannon-Formel $\text{sh}(P_i, A, B)$**:
   - $\text{sh}(P_i, A, B)$ ist eine normierte sh-Formel, wenn:
     - **$A$ und $B$ sind normierte sh-Formeln**: Das bedeutet, dass $A$ und $B$ selbst in der Form $\text{sh}(P_j, C, D)$ vorliegen können, wobei diese ebenfalls die Ordnung der Variablen respektieren.
     - **Ordnung der Variablen**: Für jede in $A$ oder $B$ vorkommende Variable $P_j$ gilt, dass $i < j$. Das bedeutet, dass die Variable $P_i$ immer die höchste Priorität hat, bevor die Variablen in $A$ und $B$ betrachtet werden.
### Theorem
> [!note] Theorem
> Zu jeder aussagenlogischen Formel $F$ gibt es eine äquivalente normierte sh-Formel $F_{sh}$. 

- Das bedeutet, dass wir jede logische Formel in eine normierte Shannon-Formel umwandeln können, die die gleiche logische Bedeutung hat und dieselben Wahrheitswerte für alle Belegungen der Variablen liefert.
### Konstruktion normierter Shannon-Formeln
- Nehmen wir eine logische Formel $F$:$$F = (P_1 \land (P_2 \lor \neg P_3)) \lor \neg P_4$$
**Schritt 1: Fixierung der Ordnung**
- Wir ordnen die Variablen in der Reihenfolge $P_1 < P_2 < P_3 < P_4$.
**Schritt 2: Umwandlung in Shannon-Form**
- Wir betrachten $P_1$ als die erste Variable und teilen die Formel basierend auf dem Wert von $P_1$ auf:$$\text{sh}(P_1, F_{\neg P_1}, F_{P_1})$$
  - $F_{\neg P_1}$ ist die Formel, die $F$ beschreibt, wenn $P_1$ „falsch“ ist.
  - $F_{P_1}$ ist die Formel, die $F$ beschreibt, wenn $P_1$ „wahr“ ist.
**Schritt 3: Vereinfachung der Formeln für $F_{\neg P_1}$ und $F_{P_1}$**
- Wenn $P_1$ „falsch“ ist ($\neg P_1$), wird die Formel $F$ zu:$$F_{\neg P_1} = (P_2 \lor \neg P_3) \lor \neg P_4$$
- Wenn $P_1$ „wahr“ ist ($P_1$), wird die Formel $F$ zu:$$F_{P_1} = P_2 \lor \neg P_3$$
**Schritt 4: Wiederhole den Prozess für die restlichen Variablen in der Ordnung**
- Setze $P_2$ als die nächste Variable und verfeinere $F_{\neg P_1}$ und $F_{P_1}$ weiter:$$\text{sh}(P_2, F_{\neg P_1, \neg P_2}, F_{\neg P_1, P_2})$$
  $$\text{sh}(P_2, F_{P_1, \neg P_2}, F_{P_1, P_2})$$

- Setze diesen Prozess fort, bis alle Variablen durchgegangen sind.
**Schritt 5: Endgültige normierte Shannon-Formel**
- Am Ende haben wir eine normierte Shannon-Formel:$$F_{sh} = \text{sh}(P_1, \text{sh}(P_2, \ldots), \text{sh}(P_2, \ldots))$$
- Eine Formel ist in konjunktiver Normalform (KNF), wenn sie Konjunktion von Disjunktionen von [[Literalen]] ist.
- Eine Formel ist in KNF, wenn sie als Konjunktion (logisches „Und“) von Disjunktionen (logisches „Oder“) von Literalen geschrieben ist. Beispielsweise:
  $$(P \lor \neg Q) \land (Q \lor R) \land (\neg P \lor R)$$
- Jede Teildisjunktion ist ein **Maxterm**.
## Wahrheitstafel und Normalformen
- **Direktes Ablesen der KNF**:
  - Jede Zeile, in der die Formel falsch ist, entspricht einem Maxterm (Negation).
  - Diese Maxterme werden dann zu einer konjunktiven Normalform kombiniert.
### Beispiel für die Herstellung der Normalformen
Nehmen wir eine logische Formel: $$A = (P \land \neg Q) \lor R$$
#### Wahrheitstafel für $A$

| \( P \) | \( Q \) | \( R \) | \( A \)   |
|--------|--------|--------|---------|
| T      | T      | T      | T       |
| T      | T      | F      | F       |
| T      | F      | T      | T       |
| T      | F      | F      | T       |
| F      | T      | T      | T       |
| F      | T      | F      | F       |
| F      | F      | T      | T       |
| F      | F      | F      | F       |
**KNF aus der Wahrheitstafel**:
- $(\neg P \lor Q \lor R)$
- $(\neg P \lor \neg Q \lor R)$
- $(P \lor Q \lor \neg R)$
Diese werden kombiniert zu:
$$A = (\neg P \lor Q \lor R) \land (\neg P \lor \neg Q \lor R) \land (P \lor Q \lor \neg R)$$ 

## Kurze KNF
- Diese Methode verwendet neue Atome (Abkürzungen) und ermöglicht es, komplexe Formeln in eine vereinfachte KNF zu überführen. 
- Diese KNF ist äquivalent in Bezug auf eine bestimmte logische Eigenschaft, die später genauer erklärt wird.
### Schritte zur Konstruktion einer kurzen KNF
#### Schritt 1: Einführung von Abkürzungen
- **Teilformel und Abkürzung**: Für jede Teilformel, deren oberster Operator binär ist (z. B. AND, OR), führen wir eine neue Variable oder ein neues Atom ein, das diese Teilformel ersetzt.

- **Beispiel**: Betrachten wir die Formel $F = (P \land Q) \lor (R \land S)$.
  - Führe eine Abkürzung $A$ für $P \land Q$ ein.
  - Führe eine Abkürzung $B$ für $R \land S$ ein.
  - Die ursprüngliche Formel wird zu $F = A \lor B$.
#### Schritt 2: Definition der Abkürzungen
- **Definition aufstellen**: Für jede Abkürzung stellen wir eine Definition gemäß der ursprünglichen Teilformel auf.
  - $A \equiv P \land Q$
  - $B \equiv R \land S$
- **Berücksichtigung tieferer Abkürzungen**: Wenn eine Teilformel wiederum eine Teilformel enthält, die bereits eine Abkürzung hat, verwenden wir diese.
#### Schritt 3: Auflösen der Äquivalenzen
- **Äquivalenzen in Implikationen umwandeln**: Jede Definition $X \equiv Y$ wird in zwei Implikationen aufgelöst:
  - $X \rightarrow Y$
  - $Y \rightarrow X$
- **Beispiel**:
  - Für $A \equiv P \land Q$ wird dies aufgelöst in:
    $$A \rightarrow (P \land Q)$$
    $$(P \land Q) \rightarrow A$$

#### Schritt 4: Umwandeln in KNF
- **Umwandlung**: Jede der resultierenden Implikationen wird in eine konjunktive Normalform (KNF) umgewandelt.
- **Beispiel**:
  - $A \rightarrow (P \land Q)$ wird zu $\neg A \lor (P \land Q)$, und dies weiter in KNF:
    $$(\neg A \lor P) \land (\neg A \lor Q)$$
  - $(P \land Q) \rightarrow A$ wird zu $\neg(P \land Q) \lor A$:$$
    (\neg P \lor A) \land (\neg Q \lor A)$$




## Theorem: Konjunktive Normalform (KNF) für aussagenlogische Formeln
### Gegeben:
- Eine aussagenlogische Formel $A$ mit $n$ Literalvorkommen.
### Aussagen des Theorems:
1. **Erfüllbarkeit**:
   $A$ ist erfüllbar genau dann, wenn $A_{kknf}$ erfüllbar ist.
2. **Begrenzte Anzahl an Literalvorkommen**:
   $A_{kknf}$ enthält höchstens $c \cdot n$ Literalvorkommen, wobei $c$ eine von $n$ unabhängige Konstante ist
3. **Effiziente Konstruktion**:
   $A_{kknf}$  kann effektiv aus $A$ in polynomieller (sogar linearer) Zeit konstruiert werden.
### Erläuterung der Aussagen
#### 1. Erfüllbarkeit
- Die Aussage, dass $A$ erfüllbar ist, wenn und nur wenn $A_{kknf}$ erfüllbar ist, bedeutet, dass die Umwandlung in die konjunktive Normalform $A_{kknf}$ die Erfüllbarkeit der Formel nicht verändert. Beide Formeln sind also logisch äquivalent in Bezug auf die Eigenschaft der Erfüllbarkeit.
#### 2. Begrenzte Anzahl an Literalvorkommen
- $n$ ist die Anzahl der Literalvorkommen in der ursprünglichen Formel $A$. Ein Literal ist entweder eine Variable oder deren Negation (z. B. $P$ oder $\neg P$).
- Die Anzahl der Literalvorkommen in $A_{kknf}$ ist durch eine Konstante $c$ multipliziert, die unabhängig von $n$ ist. Das bedeutet, dass die resultierende KNF nicht exponentiell wächst, sondern nur linear in Bezug auf die Anzahl der Literale in der ursprünglichen Formel.
#### 3. Effiziente Konstruktion
- Die Aussage, dass $A_{kknf}$ in polynomieller (sogar linearer) Zeit konstruiert werden kann, bedeutet, dass es einen Algorithmus gibt, der $A$ effizient in $A_{kknf}$ umwandelt. Diese Transformation ist daher nicht zeitaufwändig und kann für große Formeln schnell durchgeführt werden.
### Beispiel für eine Konstruktion
- Angenommen, wir haben eine Formel $A$:$$A = (P \land \neg Q) \lor (R \land S)$$
#### Schritte zur Konstruktion von $A_{kknf}$
1. **Einführung von Abkürzungen**:
   - Für jede Teilformel, deren oberster Operator binär ist, führen wir Abkürzungen ein:
     $A_1 = P \land \neg Q$
     $A_2 = R \land S$
2. **Definition der Abkürzungen**:
   - Definiere die Abkürzungen:
     $C_1 \equiv P \land \neg Q$
     $C_2 \equiv R \land S$
3. **Auflösen der Äquivalenzen**:
   - Wandeln die Äquivalenzen in Implikationen um:
     $C_1 \rightarrow (P \land \neg Q)$
     $(P \land \neg Q) \rightarrow C_1$
     $C_2 \rightarrow (R \land S)$
     $(R \land S) \rightarrow C_2$
4. **Umwandeln in KNF**:
   - Jede Implikation wird in KNF umgewandelt:
     $\neg C_1 \lor (P \land \neg Q) \equiv (\neg C_1 \lor P) \land (\neg C_1 \lor \neg Q)$
     $(P \land \neg Q) \rightarrow C_1 \equiv \neg(P \land \neg Q) \lor C_1 \equiv (\neg P \lor C_1) \land (Q \lor C_1)$
5. **Kombinierte KNF**:
   - Kombiniere die KNF-Formeln der Implikationen und die Top-Level-Formel:
     $A_{kknf} = (\neg C_1 \lor P) \land (\neg C_1 \lor \neg Q) \land (\neg P \lor C_1) \land (Q \lor C_1) \land (\neg C_2 \lor R) \land (\neg C_2 \lor S) \land (\neg R \lor C_2) \land (\neg S \lor C_2) \land (C_1 \lor C_2)$

> [!abstract] Definition
> Eine Formel $A$ in Pränex-Normalform (PNF) hat die Form:$$Q_1 x_1 \; Q_2 x_2 \; \cdots \; Q_n x_n \; B$$
> wobei:
> - **$Q_i$**: Jeder $Q_i$ ist ein Quantor und gehört zur Menge $\{ \forall, \exists \}$.
> - **$x_i$**: Jede $x_i$ ist eine Variable.
> - **$B$**: Die Matrix $B$ ist eine quantorenfreie Formel. Das bedeutet, $B$ enthält keine Quantoren, sondern nur logische Verknüpfungen wie $\land$, $\lor$, $\neg$, etc., sowie atomare Formeln. 
## Theorem
Zu jeder logischen Formel $A$ gibt es eine logisch äquivalente Formel in Pränex-Normalform. Das bedeutet, jede Formel kann so umgeschrieben werden, dass alle Quantoren am Anfang der Formel stehen und der Rest der Formel (die Matrix) quantorenfrei ist.
### Umwandlung in Pränex-Normalform
Die Umwandlung einer Formel in Pränex-Normalform erfolgt durch die Anwendung bestimmter logischer Transformationen und Tautologien. Diese Transformationen verschieben die Quantoren in die Anfangsposition der Formel.
#### Wichtige Tautologien für die Umwandlung
1. **Konjunktion mit Quantoren**:$$A \land Qx B \leftrightarrow Qx (A \land B) \quad \text{wenn } x \notin \text{Frei}(A)$$
   - Wenn $x$ nicht frei in $A$ vorkommt, kann der Quantor nach vorne verschoben werden.
2. **Disjunktion mit Quantoren**:$$A \lor Qx B \leftrightarrow Qx (A \lor B) \quad \text{wenn } x \notin \text{Frei}(A)$$
   - Ähnlich wie bei der Konjunktion kann der Quantor nach vorne verschoben werden, wenn $x$ nicht frei in $A$ ist.
3. **Implikation mit Quantoren (Fall 1)**:$$A \rightarrow Qx B \leftrightarrow Qx (A \rightarrow B) \quad \text{wenn } x \notin \text{Frei}(A)$$
   - Der Quantor kann nach vorne verschoben werden, wenn $x$ nicht frei in $A$ ist.
4. **Implikation mit Quantoren (Fall 2)**:$$\exists x B \rightarrow A \leftrightarrow \forall x (B \rightarrow A) \quad \text{wenn } x \notin \text{Frei}(A)$$
   - Wenn $x$ nicht frei in $A$ vorkommt, kann die Implikation umgeschrieben werden, indem der Existenzquantor in einen Allquantor umgewandelt wird.
5. **Implikation mit Quantoren (Fall 3)**:$$\forall x B \rightarrow A \leftrightarrow \exists x (B \rightarrow A) \quad \text{wenn } x \notin \text{Frei}(A)$$
   - Umgekehrt zur vorherigen Regel: Der Allquantor wird in einen Existenzquantor umgewandelt.
### Beispiel für die Umwandlung in Pränex-Normalform
Betrachten wir die Formel:$$\forall y (\forall x (\forall y \; p(x, y)) \rightarrow \exists x \; r(x, y))$$
#### Schrittweise Umwandlung
1. **Erste Transformation**:
   - $y$ im innersten $\forall y$ stört. Ersetzen wir es durch eine andere Variable $z$:$$\forall y (\forall x (\forall z \; p(x, z)) \rightarrow \exists x \; r(x, y))$$
2. **Implikation umschreiben**:
   - Verschieben wir den $\exists x$ nach innen:$$\forall y \; (\exists x (\forall z \; p(x, z)) \rightarrow r(x, y))$$
3. **Implikation nach außen ziehen**:
   - Bringen wir den $\exists x$ nach vorne:$$\forall y \; \exists x (\forall z \; p(x, z) \rightarrow r(x, y))$$
4. **Verschieben des Quantors**:
   - Der $\forall z$-Quantor kann ebenfalls nach außen verschoben werden:$$\forall y \; \exists x \; \forall z \; (p(x, z) \rightarrow r(x, y))$$
5. **Abschluss**:
   - Jetzt haben wir die Formel fast in Pränex-Normalform. Noch den innersten Quantor $\exists u$ vor $r(u, y)$:$$\forall y \; \exists x \; \forall z \; \exists u \; (p(x, z) \rightarrow r(u, y))$$

## Eindeutigkeit der Pränex-Normalform
Die Umwandlung einer Formel in die Pränex-Normalform ist nicht eindeutig, da die Reihenfolge der angewandten Äquivalenzen und die Reihenfolge der Quantoren die resultierende Formel beeinflussen können. Das bedeutet, dass es mehrere äquivalente Pränex-Normalformen für eine einzige Formel geben kann.
### Beispiel: Unterschiedliche Pränex-Normalformen
Betrachten wir die Formel:$$\forall x \; p(x) \rightarrow \forall y \; q(y)$$
Diese kann in verschiedene Pränex-Normalformen umgewandelt werden, abhängig davon, wie die Quantoren verschoben werden:
1. **Erste Umwandlung**:
   $$\forall x \; p(x) \rightarrow \forall y \; q(y)$$
   - Diese kann umgeschrieben werden zu:
$$\exists x \; \forall y \; (p(x) \rightarrow q(y))$$
2. **Zweite Umwandlung**:
   $$\forall x \; p(x) \rightarrow \forall y \; q(y)$$
   - Diese kann auch umgeschrieben werden zu:
   $$\forall y \; \exists x \; (p(x) \rightarrow q(y))$$
Beide Formeln sind äquivalent, aber sie haben unterschiedliche Reihenfolgen der Quantoren. Dies zeigt, dass die Pränex-Normalform nicht eindeutig ist.
## Quantoren gegen Funktionszeichen
### Darstellung mit Existenzquantoren
Quantoren, insbesondere Existenzquantoren, geben an, dass es mindestens ein Element gibt, das eine bestimmte Bedingung erfüllt.
1. **$\forall x \exists y (y = x + x)$**:
   - Diese Formel sagt aus, dass für jedes $x$ ein $y$ existiert, das gleich $x + x$ ist.
2. **$\forall x \exists y (x < y)$**:
   - Für jedes $x$ gibt es ein $y$, das größer als $x$ ist.
3. **$\forall x \forall y \exists z (x < y \rightarrow x + z = y)$**:
   - Für alle $x$ und $y$, wenn $x < y$, dann gibt es ein $z$, so dass $x + z = y$.
### Darstellung mit Funktionszeichen
Statt Existenzquantoren können Funktionszeichen verwendet werden, um die Existenz eines bestimmten Wertes explizit anzugeben.
1. **$\forall x (do(x) = x + x)$**:
   - Diese Funktion $do(x)$ ist definiert als $x + x$.
2. **$\forall x (x < gr(x))$**:
   - Die Funktion $gr(x)$ gibt einen Wert zurück, der größer als $x$ ist, z.B. $gr(x) = x + 1$.
3. **$\forall x \forall y (x < y \rightarrow x + diff(x, y) = y)$**:
   - Die Funktion $diff(x, y)$ gibt einen Wert zurück, so dass $x + diff(x, y) = y$, wenn $x < y$.
### Interpretationen der Funktionszeichen
1. **$do_{N1}(d) = d + d$**:
   - Die einzige mögliche Interpretation für $do(x)$ ist $d + d$. Hier wird die Funktion $do(x)$ definiert als das Doppelte von $x$.
2. **$gr_{N2}(d) = d + 1$**:
   - Eine mögliche Interpretation für $gr(x)$ ist $d + 1$. Diese Funktion gibt den Nachfolger eines Wertes zurück.
3. **$diff_{N3}(d1, d2) = d2 - d1$ falls $d1 < d2$, sonst $0$**:
   - Die Funktion $diff(x, y)$ gibt die Differenz $d2 - d1$ zurück, wenn $d1 < d2$, andernfalls gibt sie 0 zurück. Dies ist eine mögliche Interpretation, um sicherzustellen, dass $x + diff(x, y) = y$.

> [!abstract] Definition
>  Die Skolem-Normalform ist eine spezielle Form der Pränex-Normalform, die für die Arbeit mit logischen Formeln in der Prädikatenlogik von großer Bedeutung ist, insbesondere für die automatisierte Theorembeweisung und die Modelltheorie.
Eine Formel ist in Skolem-Normalform, wenn sie die folgenden Eigenschaften hat:
1. **Geschlossenheit**:
   - Die Formel hat keine freien Variablen. Alle Variablen sind durch Quantoren gebunden.
2. **Form**:
   - Die Formel hat die Form:
     $\forall x_1 \; \forall x_2 \; \cdots \; \forall x_n \; B$
   - Dabei sind $x_1, x_2, \ldots, x_n$ alle Variablen, die durch Allquantoren gebunden sind, und $B$ ist eine quantorenfreie Formel.
3. **Matrix in KNF**:
   - Die Matrix $B$ ist in konjunktiver Normalform (KNF). Das bedeutet, $B$ ist eine Konjunktion von Disjunktionen von Literalen, wobei ein Literal entweder eine atomare Formel oder deren Negation ist.

## Theorem zur Skolem-Normalform
- Zu jeder Formel $A$ aus einer Menge $\text{For}_\Sigma$ (Formeln über einer Signatur $\Sigma$) gibt es eine endliche Erweiterung $\Sigma_{sk}$ von $\Sigma$ und eine Formel $A_{sk}$ aus $\text{For}_{\Sigma_{sk}}$ mit den folgenden Eigenschaften:
1. **Skolem-Normalform**:
   - Die Formel $A_{sk}$ ist in Skolem-Normalform. Das bedeutet, sie ist geschlossen, hat die Form $\forall x_1 \; \forall x_2 \; \cdots \; \forall x_n \; B$, und $B$ ist in konjunktiver Normalform.
2. **Modell-Äquivalenz**:
   - Die Formel $A_{sk}$ hat genau dann ein Modell, wenn $A$ ein Modell hat. Das bedeutet, wenn $A$ erfüllbar ist, dann ist auch $A_{sk}$ erfüllbar und umgekehrt.
3. **Algorithmische Erzeugung**:
   - $A_{sk}$ lässt sich aus $A$ algorithmisch erzeugen. Es gibt also einen systematischen Prozess, um $A$ in die Skolem-Normalform $A_{sk}$ zu überführen.
## Umwandlung in Skolem-Normalform
### Schrittweiser Prozess
1. **Umwandlung in Pränex-Normalform**:
   - Bringe die Formel in eine Form, bei der alle Quantoren am Anfang stehen, und der Rest der Formel (die Matrix) quantorenfrei ist.
2. **Eliminierung von Existenzquantoren**:
   - Ersetze alle Existenzquantoren durch Skolem-Funktionen. Dies wird als Skolemisierung bezeichnet.
   - Ein Existenzquantor $\exists y \; B(y)$ in der Pränex-Normalform $\forall x_1 \; \forall x_2 \; \cdots \; \forall x_n \; \exists y \; B(y, x_1, \ldots, x_n)$ wird ersetzt durch:
     $$\forall x_1 \; \forall x_2 \; \cdots \; \forall x_n \; B(f(x_1, \ldots, x_n), x_1, \ldots, x_n)$$
   - Hier ist $f$ eine neue Skolem-Funktion, die $y$ ersetzt.
3. **Eliminierung doppelter Quantoren**:
   - Wenn alle Existenzquantoren eliminiert sind, bleiben nur Allquantoren $\forall$. Diese Quantoren werden vor die quantorenfreie Matrix $B$ gestellt.
4. **Umwandlung in konjunktive Normalform (KNF)**:
   - Bringe die Matrix $B$ in eine konjunktive Normalform, falls dies noch nicht geschehen ist. Die KNF ist eine Konjunktion von Disjunktionen von Literalen.
### Beispiel zur Umwandlung in Skolem-Normalform
Nehmen wir an, wir haben die Formel:
$$\forall x \; \exists y \; \forall z \; (P(x, y) \land \neg Q(y, z))$$
1. **Pränex-Normalform**:
   - Die Formel ist bereits in Pränex-Normalform.
2. **Skolemisierung**:
   - Der Existenzquantor $\exists y$ wird durch eine Skolem-Funktion $f(x)$ ersetzt:
     $$\forall x \; \forall z \; (P(x, f(x)) \land \neg Q(f(x), z))$$
3. **Eliminierung doppelter Quantoren**:
   - Es bleiben nur Allquantoren übrig. Die Formel hat jetzt die Form:
     $$\forall x \; \forall z \; (P(x, f(x)) \land \neg Q(f(x), z))$$
4. **Umwandlung in KNF**:
   - Die Matrix $B = (P(x, f(x)) \land \neg Q(f(x), z))$ ist bereits in konjunktiver Normalform, da sie eine Konjunktion von Literalen ist.
Die resultierende Skolem-Normalform ist:
$$\forall x \; \forall z \; (P(x, f(x)) \land \neg Q(f(x), z))$$



## Konstruktionsvorschrift für die Skolem-Normalform
### Schritt 0: All-Abschluss der freien Variablen
- **All-Abschluss** bedeutet, dass alle freien Variablen der Formel durch Allquantoren gebunden werden.
- Beispiel: Wenn eine Formel $A$ die freie Variable $x$ enthält, wird sie umgeschrieben zu $\forall x \; A$.
### Schritt 1: Transformation in Pränex-Normalform
- **Pränex-Normalform** ist eine Form, in der alle Quantoren am Anfang der Formel stehen und die Matrix $B$ quantorenfrei ist.
- Die Formel wird in die Form:
  $Q_1 x_1 \; Q_2 x_2 \; \cdots \; Q_n x_n \; B$
  gebracht, wobei $Q_i \in \{\forall, \exists\}$ sind und $B$ keine Quantoren enthält.
### Schritt 2: Skolemisierung
Die Skolemisierung ersetzt Existenzquantoren durch Skolem-Funktionen. Dies geschieht in zwei Schritten:
#### Schritt 2(a): Signatur-Erweiterung von $\Sigma$ zu $\Sigma_{sk}$
- **Signatur-Erweiterung** bedeutet, dass die Menge der Symbole $\Sigma$ um neue Funktionssymbole $f_i$ erweitert wird.
- Für jedes $i$, wobei $1 \leq i \leq n$ und $Q_i = \exists$, wird ein neues $k$-stelliges Funktionssymbol $f_i$ hinzugefügt. Hierbei ist $k$ die Anzahl der Allquantoren $Q_j = \forall$ mit $j < i$.
  - Diese neuen Funktionssymbole werden verwendet, um die Existenzquantoren zu eliminieren.
#### Schritt 2(b): Für alle $Q_i = \exists$
- **Existenzquantoren entfernen**:
  - Lasse $\exists x_i$ weg.
  - Ersetze $x_i$ in der Matrix $B$ durch $f_i(x_1, x_2, \ldots, x_{i-1})$, wobei $x_j$ alle Variablen sind, die von vorherigen Allquantoren gebunden sind.
### Schritt 3: Transformation der Matrix der Formel in konjunktive Normalform (KNF)
- **KNF** (konjunktive Normalform) ist eine Form, in der die Matrix $B$ als eine Konjunktion von Disjunktionen von Literalen vorliegt.
- Die Matrix $B$ wird entsprechend umgeformt, sodass sie diese Struktur aufweist.
### Beispiele zur Umwandlung in Skolem-Normalform
#### Beispiel 1
Gegeben:
$$\forall x (\exists y \; p(y) \land \exists z \; q(x, z))$$
1. **All-Abschluss**: Alle Variablen sind bereits durch Quantoren gebunden.
2. **Pränex-Normalform**:
   $\forall x \; \exists y \; \exists z \; (p(y) \land q(x, z))$
3. **Skolemisierung**:
   - Für $\exists y$ wird $y$ durch $f_1(x)$ ersetzt.
   - Für $\exists z$ wird $z$ durch $f_2(x)$ ersetzt.
   $$\forall x \; (p(f_1(x)) \land q(x, f_2(x)))$$
4. **Matrix in KNF**: Die Matrix ist bereits in konjunktiver Normalform.
Skolem-Normalform:
$$\forall x \; (p(f_1(x)) \land q(x, f_2(x)))$$
#### Beispiel 2
Gegeben:
$$\exists x (p(w, x) \lor \forall y (q(w, x, y) \land \exists z \; r(y, z)))$$
1. **All-Abschluss**:
   $$\forall w \; \exists x (p(w, x) \lor \forall y (q(w, x, y) \land \exists z \; r(y, z)))$$
2. **Pränex-Normalform**:
   $$\forall w \; \exists x \; \forall y \; \exists z (p(w, x) \lor (q(w, x, y) \land r(y, z))$$
3. **Skolemisierung**:
   - Für $\exists x$ wird $x$ durch $f_1(w)$ ersetzt.
   - Für $\exists z$ wird $z$ durch $f_2(w, y)$ ersetzt.
   $$\forall w \; \forall y (p(w, f_1(w)) \lor (q(w, f_1(w), y) \land r(y, f_2(w, y))))$$
4. **Matrix in KNF**:
   $$p(w, f_1(w)) \lor (q(w, f_1(w), y) \land r(y, f_2(w, y)))$$
   - In KNF umgewandelt:
   $$(p(w, f_1(w)) \lor q(w, f_1(w), y)) \land (p(w, f_1(w)) \lor r(y, f_2(w, y)))$$
Skolem-Normalform:
$$\forall w \; \forall y ((p(w, f_1(w)) \lor q(w, f_1(w), y)) \land (p(w, f_1(w)) \lor r(y, f_2(w, y))))$$


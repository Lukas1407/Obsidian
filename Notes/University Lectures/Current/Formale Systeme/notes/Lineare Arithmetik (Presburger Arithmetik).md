### Lineare Arithmetik (Presburger Arithmetik)

**Presburger Arithmetik** ist ein formales System der Arithmetik der natürlichen Zahlen, das keine Multiplikation enthält. Es wurde von Mojżesz Presburger 1929 eingeführt und ist bekannt dafür, dass es vollständig und entscheidbar ist. 

#### Definition und Signatur

- **Signatur $\Sigma_P$:**
  $$ \Sigma_P = (\{0, 1, +\}, \{<\}) $$
  - **Operationen:** Addition $+$
  - **Konstanten:** $0$, $1$
  - **Relation:** $<$ (kleiner als)

- **Darstellbarkeit:** In der Presburger Arithmetik können alle natürlichen Zahlen und lineare Ausdrücke dargestellt werden, z.B.:
  - Die Zahl 3: $3 =^{def} 1 + 1 + 1$
  - Das Dreifache von $x$: $3x =^{def} x + x + x$

#### Theorem: Entscheidbarkeit

- **Aussage:** Die Theorie der Arithmetik der natürlichen Zahlen ohne Multiplikation ist entscheidbar. Das bedeutet, dass es einen Algorithmus gibt, der für jede gegebene Formel in dieser Theorie entscheiden kann, ob sie wahr oder falsch ist.

#### Bedeutung und Anwendung

- **Anwendungen:** Presburger Arithmetik wird oft in der formalen Verifikation und der automatisierten Beweisführung verwendet, insbesondere in Systemen, die Arithmetik ohne Multiplikation betreiben.

### Arithmetik der reellen Zahlen

**Arithmetik der reellen Zahlen** ist ein formales System, das die Standardarithmetik mit Addition, Subtraktion, Multiplikation und einer Ordnungsrelation umfasst.

#### Definition und Signatur

- **Signatur $\Sigma$:**
  $$ \Sigma = (\{+, -, \cdot, 0, 1\}, \{\leq\}) $$
  - **Operationen:** Addition $+$, Subtraktion $-$, Multiplikation $\cdot$
  - **Konstanten:** $0$, $1$
  - **Relation:** $\leq$ (kleiner gleich)

- **Standardmodell $\mathbb{R}$:**
  $$ \mathbb{R} = (\mathbb{R}, I_{\mathbb{R}}) $$
  - $\mathbb{R}$ ist die Menge der reellen Zahlen.
  - $I_{\mathbb{R}}$ ist die Standardinterpretation der Operationen und Relationen.

#### Tarski-Seidenberg-Theorem

- **Aussage:** Die Erfüllbarkeit prädikatenlogischer Formeln über den reellen Zahlen $\mathbb{R}$ ist entscheidbar.
- **Bedeutung:** Für jede prädikatenlogische Formel $\varphi$ in der Signatur $\Sigma$, die über den reellen Zahlen definiert ist, gibt es einen Algorithmus, der entscheiden kann, ob $\varphi$ in $\mathbb{R}$ wahr ist.

#### Axiome der Arithmetik der reellen Zahlen

1. **Abelsche Gruppe (Addition):**
   - Assoziativität:
     $$ \forall x \forall y \forall z \, ((x + y) + z = x + (y + z)) $$
   - Kommutativität:
     $$ \forall x \forall y \, (x + y = y + x) $$
   - Identitätselement:
     $$ \forall x \, (x + 0 = x) $$
   - Inverses Element:
     $$ \forall x \, (x + (-x) = 0) $$

2. **Abelsche Halbgruppe (Multiplikation):**
   - Assoziativität:
     $$ \forall x \forall y \forall z \, ((x \cdot y) \cdot z = x \cdot (y \cdot z)) $$
   - Kommutativität:
     $$ \forall x \forall y \, (x \cdot y = y \cdot x) $$
   - Identitätselement:
     $$ \forall x \, (x \cdot 1 = x) $$

3. **Distributivität:**
   $$ \forall x \forall y \forall z \, ((x + y) \cdot z = x \cdot z + y \cdot z \land z \cdot (x + y) = z \cdot x + z \cdot y) $$

4. **Ordnung:**
   - Additionserhaltung der Ordnung:
     $$ \forall x \forall y \forall z \, (x \leq y \rightarrow x + z \leq y + z) $$
   - Multiplikationserhaltung der Ordnung:
     $$ \forall x \forall y \, (0 \leq x \land 0 \leq y \rightarrow 0 \leq x \cdot y) $$

5. **Existenz von Wurzeln:**
   $$ \forall x \, (0 \leq x \rightarrow \exists y \, (x = y \cdot y)) $$
   - Es gibt für jede nichtnegative reelle Zahl $x$ eine reelle Zahl $y$, sodass $y^2 = x$.

6. **Existenz von Nullstellen (für ungerade $n$):**
   $$ \forall a_{n-1} \ldots \forall a_0 \, \exists x \, (x^n + a_{n-1} \cdot x^{n-1} + \ldots + a_1 \cdot x + a_0 = 0) $$
   - Für jedes Polynom ungeraden Grades gibt es eine reelle Nullstelle.

### Presburger-Arithmetik und Arithmetik reeller Zahlen
Diese Theorien beinhalten Aussagen über Zahlen und verwenden Quantoren, um Eigenschaften und Beziehungen zwischen den Zahlen auszudrücken. Die Entscheidbarkeit dieser Theorien hängt oft davon ab, ob Quantoren eliminiert werden können.
#### Was bedeutet Quantoren-Elimination?
Eine Theorie $T$ erlaubt Quantoren-Elimination, wenn für jede Formel mit Quantoren $Q_1x_1 \ldots Q_nx_n. \phi(x_1, \ldots, x_n, y_1, \ldots, y_m)$ eine dazu $T$-äquivalente quantorenfreie Formel $\psi(y_1, \ldots, y_m)$ existiert, die effektiv aus der ursprünglichen Formel berechnet werden kann.

### Beispiel für eine Formel mit Quantoren
Betrachten wir eine Formel der Form:
$$ \exists x (\phi_1(x, y_1, \ldots, y_m) \land \ldots \land \phi_n(x, y_1, \ldots, y_m)) $$
Die Quantoren-Elimination bedeutet, dass wir diese Formel so umschreiben können, dass der Existenzquantor $\exists x$ eliminiert wird und eine quantorenfreie Formel entsteht.

### Lemma zur Quantoren-Elimination
Falls eine Theorie $T$ Quantoren-Elimination für Formeln der obigen Form erlaubt, dann erlaubt $T$ Quantoren-Elimination für alle Formeln in der gleichen Sprache $Fml_\Sigma$.

### Anwendung auf SAT
Quantoren-Elimination ermöglicht es, das Problem der Erfüllbarkeit (SAT) für eine Theorie $T$ auf das Problem der Erfüllbarkeit quantorenfreier Formeln (QF-SAT) zurückzuführen. Wenn QF-SAT für $T$ entscheidbar ist, dann ist auch SAT für $T$ entscheidbar.

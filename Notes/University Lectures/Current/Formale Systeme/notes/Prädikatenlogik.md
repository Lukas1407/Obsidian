> [!abstract] Definition
> Die Prädikatenlogik erweitert die [[Aussagenlogik]] um Quantoren und Prädikate, die es ermöglichen, allgemeine Aussagen über Mengen von Objekten zu machen. 
## Beispiel: Alle Menschen sind sterblich
1. **Aussage**:
   - Alle Menschen sind sterblich.
   - Sokrates ist ein Mensch.
   - Also ist Sokrates sterblich.
2. **Prädikatenlogische Formalisierung**:
   - **Aussage in natürlicher Sprache**:
     - Alle Menschen sind sterblich.
     - Sokrates ist ein Mensch.
     - Also ist Sokrates sterblich.
   - **Logische Formalisierung**:
     - Wir verwenden die folgenden Prädikate:
       - $\text{Mensch}(x)$: $x$ ist ein Mensch.
       - $\text{sterblich}(x)$: $x$ ist sterblich.
     - Wir verwenden den spezifischen Namen „Sokrates“, um auf ein individuelles Objekt in unserem Universum der Diskurse zu verweisen.
3. **Formalisierte Aussagen**:
   - **Quantoren**:
     - $\forall x$: Für alle $x$ (Universalquantor).
   - **Implikation**:
     - $\rightarrow$: Wenn ..., dann ... (Implikation).
   - **Formel für die Aussage „Alle Menschen sind sterblich“**:$$\forall x (\text{Mensch}(x) \rightarrow \text{sterblich}(x))$$
     Das bedeutet: Für jedes $x$, wenn $x$ ein Mensch ist, dann ist $x$ sterblich.
   - **Formel für „Sokrates ist ein Mensch“**:$$\text{Mensch}(\text{Sokrates})$$
   - **Zu zeigende Aussage „Sokrates ist sterblich“**:$$\text{sterblich}(\text{Sokrates})$$
## Neue Syntax
- **$\forall$**: Der Allquantor wird verwendet, um Aussagen über alle Elemente einer bestimmten Menge zu treffen. Beispiel: $\forall x (P(x))$ bedeutet "Für alle $x$ gilt $P(x)$".
- **$\exists$**: Der Existenzquantor wird verwendet, um die Existenz von mindestens einem Element zu behaupten, für das eine Aussage gilt. Beispiel: $\exists x (P(x))$ bedeutet "Es existiert ein $x$, für das $P(x)$ gilt".
- **$\dot=$**: Das Gleichheitssymbol wird verwendet, um auszudrücken, dass zwei Individuen gleich sind. Beispiel: $x .= y$ bedeutet "x ist gleich y".
- **$v_i$**: Individuenvariablen, wobei $i \in \mathbb{N}$ (die Menge der natürlichen Zahlen)
- **Var**: Bezeichnet die Menge der zur Verfügung stehenden Variablen, die in den logischen Ausdrücken verwendet werden können.

## Definition der Signatur
**Signatur:** Ein Tripel $\Sigma = (F_\Sigma, P_\Sigma, \alpha_\Sigma)$
- **$F_\Sigma$**: Die Menge der Funktionssymbole. Diese Menge kann endlich oder abzählbar unendlich sein.
- **$P_\Sigma$**: Die Menge der Prädikatsymbole. Auch diese Menge kann endlich oder abzählbar unendlich sein.
- **$\alpha_\Sigma$**: Eine Abbildung, die den Funktions- und Prädikatsymbolen eine Stelligkeit (Anzahl der Argumente) zuordnet, also $\alpha_\Sigma : F_\Sigma \cup P_\Sigma \to \mathbb{N}$.
### Eigenschaften und Begriffe
- **Paarweise disjunkt:** Die Mengen $F_\Sigma$ (Funktionssymbole), $P_\Sigma$ (Prädikatsymbole) und die Menge der Sondersymbole sind paarweise disjunkt, d.h., sie haben keine gemeinsamen Elemente.
### Funktions- und Prädikatsymbole
- **Funktionssymbol ($f \in F_\Sigma$)**: Ein Symbol, das eine Funktion darstellt, die Objekte (Individuen) der Domäne nimmt und ein weiteres Objekt zurückgibt. Die Anzahl der Argumente, die ein Funktionssymbol akzeptiert, wird als Stelligkeit bezeichnet.
    - Ein $n$-stelliges Funktionssymbol $f$ hat $\alpha_\Sigma(f) = n$. Zum Beispiel ist $f(x, y)$ ein 2-stelliges Funktionssymbol.
    - Ein null-stelliges Funktionssymbol heißt auch **Konstantensymbol** oder kurz **Konstante**. Es repräsentiert ein einzelnes festes Objekt der Domäne.
- **Prädikatsymbol ($p \in P_\Sigma$)**: Ein Symbol, das eine Aussage über Objekte der Domäne macht. Die Anzahl der Argumente, die ein Prädikatsymbol akzeptiert, wird ebenfalls als Stelligkeit bezeichnet.
    - Ein $n$-stelliges Prädikatsymbol $p$ hat $\alpha_\Sigma(p) = n$. Zum Beispiel ist $P(x, y, z)$ ein 3-stelliges Prädikatsymbol.
    - Ein null-stelliges Prädikatsymbol ist eine Aussage, die ohne Argumente auskommt. Es wird auch als **aussagenlogisches Atom** bezeichnet, weil es eine eigenständige Aussage ist, die wahr oder falsch sein kann, wie eine gewöhnliche Aussage in der Aussagenlogik.

## Definition Terme
**Term ($\Sigma)$**, die Menge der Terme über $\Sigma$, ist induktiv definiert durch:
1. **Variablen als Terme:**$$\text{Var} \subseteq \text{Term}_\Sigma$$
   Jede Variable $x \in \text{Var}$ ist ein Term. Die Menge der Variablen, die als Terme gelten, ist also eine Teilmenge der Menge der Terme \(\text{Term}_\Sigma\).

2. **Funktionssymbole und Terme:**
  $$ \text{Sei } f \in F_\Sigma \text{ und } \alpha_\Sigma(f) = n$$, $$\text{und seien } t_1, t_2, \ldots, t_n \in \text{Term}_\Sigma$$, $$\text{dann ist } f(t_1, t_2, \ldots, t_n) \in \text{Term}_\Sigma$$
   Das bedeutet, wenn $f$ ein $n$-stelliges Funktionssymbol ist und $t_1, t_2, \ldots, t_n$ Terme sind, dann ist $f(t_1, t_2, \ldots, t_n)$ ebenfalls ein Term. Hier werden $t_1, t_2, \ldots, t_n$ als Argumente des Funktionssymbols $f$ verwendet.

### Grundterme
Ein Term wird als **Grundterm** bezeichnet, wenn er **keine Variablen** enthält. Grundterme sind also spezielle Terme, die ausschließlich aus Konstantensymbolen und Funktionssymbolen gebildet werden, ohne dass Variablen vorkommen. Ein Grundterm könnte beispielsweise eine Konstante wie $a$ oder eine Funktionsanwendung wie $f(a, b)$ sein, wobei $a$ und $b$ Konstanten sind.
#### Beispiele
1. **Variable als Term:**
   - $x$ ist ein Term, wenn $x$ eine Variable ist.
2. **Konstante als Term:**
   - Wenn $a$ eine Konstante ist, dann ist $a$ ein Grundterm.
3. **Funktionsanwendung als Term:**
   - Wenn $f$ ein zweistelliges Funktionssymbol ist und $x$ und $y$ Variablen sind, dann ist $f(x, y)$ ein Term.
   - Wenn $f$ ein einstelliges Funktionssymbol ist und $a$ eine Konstante ist, dann ist $f(a)$ ein Grundterm, weil keine Variablen enthalten sind.


In der Prädikatenlogik erster Stufe sind **Formeln** grundlegende Ausdrücke, die verwendet werden, um logische Aussagen über die Elemente der Domäne zu machen. Diese Formeln können komplexe logische Verbindungen und Quantoren enthalten. Hier ist die Definition von Formeln und atomaren Formeln im Kontext einer Signatur $\Sigma$:

## Definition Atomare Formeln
**$At_{\Sigma}$**, die Menge der atomaren Formeln über $\Sigma$, ist definiert durch:
1. **Gleichheit von Termen:**$$ \text{At}_\Sigma := \{ s .= t \mid s, t \in \text{Term}_\Sigma \}$$

   Eine atomare Formel kann eine Aussage über die Gleichheit zweier Terme sein. Zum Beispiel ist $x .= y$ eine atomare Formel, wenn $x$ und $y$ Terme sind.
2. **Prädikatsymbole mit Termen:$$\text{At}_\Sigma := \text{At}_\Sigma \cup \{ p(t_1, \ldots, t_n) \mid p \in P_\Sigma, \alpha_\Sigma(p) = n, t_i \in \text{Term}_\Sigma \}$$
   Eine atomare Formel kann auch eine Aussage sein, die ein Prädikatsymbol $p$ mit $n$ Argumenten (Termen) verwendet. Zum Beispiel ist $P(x, y)$ eine atomare Formel, wenn $P$ ein zweistelliges Prädikatsymbol und $x$, $y$ Terme sind.
### Beispiele
1. **Atomare Formeln:**
   - $x .= y$ (Gleichheit zweier Terme)
   - $P(a, b)$ (Prädikat mit Konstanten)

## Definition Formeln
**For$_\Sigma$**, die Menge der Formeln über $\Sigma$, ist induktiv definiert durch:
1. **Basisformeln:**$$\{ 1, 0 \} \cup \text{At}_\Sigma \subseteq \text{For}_\Sigma$$
   Die Konstanten $1$ und $0$ (die Wahrheitswerte "wahr" und "falsch") sowie die Menge der atomaren Formeln gehören zu den Formeln. Beispielsweise sind $x .= y$ und $P(a, b)$ atomare Formeln, die zur Menge der Formeln gehören.
2. **Komplexe Formeln:**
   Mit $x \in \text{Var}$ und $A, B \in \text{For}_\Sigma$ sind ebenfalls in $\text{For}_\Sigma$:
   - **Negation:** $\neg A$
   - **Konjunktion:** $(A \land B)$
   - **Disjunktion:** $(A \lor B)$
   - **Implikation:** $(A \rightarrow B)$
   - **Äquivalenz:** $(A \leftrightarrow B)$
   - **Allquantor:** $\forall x A$
   - **Existenzquantor:** $\exists x A$
   Das bedeutet, dass wir durch die Anwendung logischer Operatoren und Quantoren auf Formeln neue, komplexere Formeln erstellen können. Zum Beispiel ist $\neg P(x)$ eine Formel, die durch die Negation einer atomaren Formel $P(x)$ gebildet wird.
### Beispiele
2. **Komplexe Formeln:**
   - $\neg (x .= y)$ (Negation einer Gleichheit)
   - $(P(x) \land Q(y))$ (Konjunktion zweier Prädikate)
   - $\forall x (P(x) \rightarrow Q(x))$ (Allquantor mit Implikation)

## gebundene und freie Variablen
1. **Wirkungsbereich eines Präfixes:**
   - Wenn eine Formel $A$ die Form $\forall x B$ oder $\exists x B$ hat, wird $B$ als der **Wirkungsbereich** des Präfixes $\forall x$ bzw. $\exists x$ von $A$ bezeichnet. Das bedeutet, dass innerhalb von $B$ die Variable $x$ durch den entsprechenden Quantor quantifiziert wird.
2. **Gebundene Variable:**
   - Ein Auftreten einer Variablen $x$ in einer Formel $A$ heißt **gebunden**, wenn es innerhalb des Wirkungsbereichs eines Präfixes $\forall x$ oder $\exists x$ einer Teilformel von $A$ vorkommt. Das bedeutet, dass $x$ durch diesen Quantor eingeschränkt oder „gebunden“ ist.
3. **Freie Variable:**
   - Ein Auftreten einer Variablen $x$ in einer Formel $A$ heißt **frei**, wenn es nicht gebunden ist und nicht unmittelbar rechts neben einem Quantor steht. Freie Variablen sind in der Formel „offen“ und nicht durch Quantoren eingeschränkt.
### Beispiele
Gegeben ist die Formel:
$$\forall x (p_0(x, y) \rightarrow \forall z (\exists y \, p_1(y, z) \lor \forall x \, p_2(f(x), x)))$$
Wir untersuchen die gebundenen und freien Vorkommen der Variablen $x$, $y$ und $z$.
#### 1. Außere Formel: $\forall x (p_0(x, y) \rightarrow \forall z (\exists y \, p_1(y, z) \lor \forall x \, p_2(f(x), x)))$
- Hier wird $x$ durch den äußeren Allquantor $\forall x$ quantifiziert. Alle Vorkommen von $x$ in dieser Formel sind gebunden durch diesen Quantor, sofern sie nicht durch innere Quantoren neu gebunden werden.
#### 2. Innere Formel: $p_0(x, y)$
- In $p_0(x, y)$ ist $x$ gebunden durch den äußeren Quantor $\forall x$. $y$ ist frei, da es keinen übergeordneten Quantor gibt, der $y$ quantifiziert.
#### 3. Innere Formel: $\forall z (\exists y \, p_1(y, z) \lor \forall x \, p_2(f(x), x))$
- In $\forall z \ldots$ wird $z$ durch diesen Quantor gebunden. Alle Vorkommen von $z$ innerhalb dieser Teilformel sind gebunden durch diesen Quantor.
#### 4. Innere Formel: $\exists y \, p_1(y, z) \lor \forall x \, p_2(f(x), x)$
- $\exists y \, p_1(y, z)$: Hier ist $y$ durch $\exists y$ gebunden. $z$ ist durch den übergeordneten Quantor $\forall z$ gebunden.
- $\forall x \, p_2(f(x), x)$: Innerhalb dieses Ausdrucks ist $x$ durch den inneren Quantor $\forall x$ gebunden. Dieses Vorkommen von $x$ überschreibt den äußeren Quantor $\forall x$.
In der Prädikatenlogik erster Stufe ist es wichtig, die verschiedenen Arten von Variablen in einer Formel oder einem Term zu unterscheiden. Hier sind die Definitionen und Notationen, um gebundene und freie Variablen sowie die Gesamtmenge der Variablen in einer Formel oder einem Term zu identifizieren.

###  Notationen
1. **Gebundene Variablen (Bd(A))**
   - $\text{Bd}(A) := \{ x \mid x \in \text{Var}, x \text{ tritt gebunden in } A \text{ auf} \}$
   - **Bd(A)** ist die Menge aller Variablen, die in der Formel $A$ gebunden sind. Eine Variable $x$ ist gebunden, wenn sie innerhalb des Wirkungsbereichs eines Quantors $\forall x$ oder $\exists x$ in $A$ auftritt.
2. **Freie Variablen (Frei(A))**
   - $\text{Frei}(A) := \{ x \mid x \in \text{Var}, x \text{ tritt frei in } A \text{ auf} \}$
   - **Frei(A)** ist die Menge aller Variablen, die in der Formel $A$ frei sind. Eine Variable $x$ ist frei, wenn sie in $A$ vorkommt, aber nicht durch einen Quantor gebunden ist.
3. **Gesamtmenge der Variablen in einer Formel (Var(A))**
   - $\text{Var}(A) := \text{Frei}(A) \cup \text{Bd}(A)$
   - **Var(A)** ist die Menge aller Variablen, die in der Formel $A$ vorkommen, sowohl gebundene als auch freie Variablen.
4. **Gesamtmenge der Variablen in einem Term (Var(t))**
   - $\text{Var}(t) := \{ x \mid x \in \text{Var}, x \text{ kommt in } t \text{ vor} \}$
   - **Var(t)** ist die Menge aller Variablen, die im Term $t$ vorkommen.

## Abschlussoperationen für Formeln
> [!abstract] Definition
> In der Prädikatenlogik erster Stufe werden Abschlussoperationen für Formeln verwendet, um alle freien Variablen in einer Formel zu binden, wodurch die Formel **geschlossen** wird. 
1. **Geschlossene Formel (A)**
   - Eine Formel $A$ heißt **geschlossen**, wenn sie keine freien Variablen hat. Mathematisch ausgedrückt:
     $$\text{Frei}(A) = \{\}$$
   - Das bedeutet, dass jede Variable in der Formel entweder durch einen Allquantor $forall$ oder einen Existenzquantor $exists$ gebunden ist.
2. **Allabschluss (∀-Abschluss)**
   - Wenn die Menge der freien Variablen von $A$ $\text{Frei}(A) = \{ x_1, x_2, \ldots, x_n \}$ ist, dann ist der **Allabschluss** von $A$:
     $$\forall x_1 \, \forall x_2 \, \ldots \, \forall x_n \, A$$
   - Abkürzend schreiben wir:
    $$ \text{Cl}_\forall A$$
   - Der Allabschluss ist eine neue Formel, in der alle ursprünglich freien Variablen von $A$ durch Allquantoren gebunden werden.
3. **Existenzabschluss (∃-Abschluss)**
   - Wenn die Menge der freien Variablen von $A$ $\text{Frei}(A) = \{ x_1, x_2, \ldots, x_n \}$ ist, dann ist der **Existenzabschluss** von $A$:$$\exists x_1 \, \exists x_2 \, \ldots \, \exists x_n \, A$$
   - Abkürzend schreiben wir: $$ \text{Cl}_\exists A$$
   - Der Existenzabschluss ist eine neue Formel, in der alle ursprünglich freien Variablen von $A$ durch Existenzquantoren gebunden werden.
4. **Eigenschaft geschlossener Formeln**
   - Wenn $A$ bereits geschlossen ist, dann gilt:$$ \text{Cl}_\forall A = \text{Cl}_\exists A = A$$
   - Das bedeutet, dass bei einer geschlossenen Formel keine weiteren Quantoren notwendig sind, da keine freien Variablen vorhanden sind.
### Beispiele
#### Beispiel 1: Formel mit freien Variablen
Betrachten wir die Formel:
$$A = P(x, y) \land Q(z)$$

- **Freie Variablen:** $\text{Frei}(A) = \{ x, y, z \}$
- **Allabschluss:**
  $$\text{Cl}_\forall A = \forall x \, \forall y \, \forall z \, (P(x, y) \land Q(z))$$
  Diese Formel ist jetzt geschlossen, da alle Variablen $x$, $y$, und $z$ durch Allquantoren gebunden sind.
- **Existenzabschluss:**$$  \text{Cl}_\exists A = \exists x \, \exists y \, \exists z \, (P(x, y) \land Q(z))$$
    Auch diese Formel ist geschlossen, da alle Variablen $x$, $y$, und $z$ durch Existenzquantoren gebunden sind.

#### Beispiel 2: Geschlossene Formel
Betrachten wir die Formel:$$A = \forall x \, \exists y \, P(x, y)$$
- **Freie Variablen:** $\text{Frei}(A) = \{\}$
- **Allabschluss und Existenzabschluss:**
  Da $A$ bereits geschlossen ist, gilt: $$\text{Cl}_\forall A = \text{Cl}_\exists A = A$$
  Es sind keine zusätzlichen Quantoren notwendig, um $A$ zu schließen.


In der Prädikatenlogik erster Stufe ermöglicht die **Substitution** das Ersetzen von Variablen in Formeln oder Termen durch andere Terme. Dies ist eine wichtige Operation, um logische Ausdrücke zu manipulieren und zu vereinfachen. Hier sind die Definition und Notation für Substitutionen:

## Definition Substitutionen
1. **Substitution**
   - Eine **Substitution** ist eine Abbildung:$$
     \sigma : \text{Var} \to \text{Term}_\Sigma
     $$
     - Das bedeutet, dass jede Variable $x$ in der Menge der Variablen (Var) durch einen Term aus der Menge der Terme \(\text{Term}_\Sigma\) ersetzt wird.
   - Für fast alle $x \in \text{Var}$ gilt:$$
     \sigma(x) = x
     $$
     - Das heißt, dass die meisten Variablen $x$ unverändert bleiben. Nur eine endliche Anzahl von Variablen wird tatsächlich durch andere Terme ersetzt.

2. **Notation für Substitutionen**
   - Wenn gilt:$$
     \{ x \mid \sigma(x) \neq x \} \subseteq \{ x_1, \ldots, x_m \}
     $$
     - Das bedeutet, dass nur eine endliche Anzahl von Variablen $x_1, \ldots, x_m$ tatsächlich durch andere Terme $s_1, \ldots, s_m$ ersetzt wird.
   - Und wenn $\sigma(x_i) = s_i$ für $i = 1, \ldots, m$ ist, dann kann die Substitution auch wie folgt geschrieben werden:$$
     \{ x_1 / s_1, \ldots, x_m / s_m \}
     $$
     - Diese Notation bedeutet, dass die Variable $x_1$ durch den Term $s_1$, $x_2$ durch $s_2$, und so weiter, ersetzt wird.

3. **Grundsubstitution**
   - Eine Substitution $\sigma$ heißt **Grundsubstitution**, wenn für alle $x$, für die $\sigma(x) \neq x$ gilt, der Funktionswert $\sigma(x)$ ein Grundterm ist.
     - Ein **Grundterm** ist ein Term, der keine Variablen enthält. Das heißt, alle Ersetzungen erfolgen durch Terme, die aus Konstanten und Funktionssymbolen bestehen, aber keine Variablen enthalten.

4. **Identische Substitution (id)**
   - Mit $\text{id}$ bezeichnen wir die **identische Substitution** auf Var, d.h.:$$
     \text{id}(x) = x \text{ für alle } x \in \text{Var}
     $$
     - Das bedeutet, dass keine Variablen ersetzt werden. Jede Variable bleibt unverändert.

### Beispiele
1. **Einfache Substitution**
   - Betrachten wir die Substitution \($\sigma = \{ x / a, y / f(b) \}$\):
     - $\sigma(x) = a$
     - $\sigma(y) = f(b)$
     - Für alle anderen Variablen $z$ gilt $\sigma(z) = z$.

2. **Anwendung der Substitution**
   - Nehmen wir an, wir haben den Term $t = P(x, y, z)$. Durch Anwendung der Substitution \(\sigma\) erhalten wir:$$
     \sigma(P(x, y, z)) = P(\sigma(x), \sigma(y), \sigma(z)) = P(a, f(b), z)
     $$
     - Hier wird $x$ durch $a$ und $y$ durch $f(b)$ ersetzt. $z$ bleibt unverändert, da $\sigma(z) = z$.

3. **Grundsubstitution**
   - Eine Substitution \(\sigma\) ist eine Grundsubstitution, wenn alle Ersetzungen Grundterme sind, z.B.:$$
     \sigma = \{ x / a, y / b \}
     $$
     - Hier sind $a$ und $b$ Konstanten, sodass keine Variablen in den Ersetzungen vorkommen.

4. **Identische Substitution**
   - Die identische Substitution $\text{id}$ würde angewendet auf $P(x, y, z)$ den Term unverändert lassen:$$
     \text{id}(P(x, y, z)) = P(x, y, z)
     $$


### Anwendung von Substitutionen
1. **Anwendung einer Substitution auf einen Term ($\sigma(t)$)**
   - Sei $\sigma$ eine Substitution und $t$ ein Term. Die Anwendung der Substitution $\sigma$ auf den Term $t$ wird durch simultane Ersetzung aller Variablenvorkommen $x$ durch $\sigma(x)$ durchgeführt.
   - Mathematisch ausgedrückt:$$
     \sigma(t)
     $$
   - Das bedeutet, dass jeder Vorkommensort einer Variablen $x$ im Term $t$ durch den entsprechenden Term $\sigma(x)$ ersetzt wird.

2. **Anwendung einer Substitution auf eine Formel ($\sigma(\phi)$)**
   - Sei $\sigma$ eine Substitution und $\phi$ eine Formel. Die Anwendung der Substitution $\sigma$ auf die Formel $\phi$ erfolgt durch simultane Ersetzung aller freien Variablenvorkommen $x$ in $\phi$ durch $\sigma(x)$.
   - Mathematisch ausgedrückt:$$
     \sigma(\phi)
     $$
   - Hierbei werden nur die freien Variablen in der Formel $\phi$ ersetzt, gebundene Variablen bleiben unverändert.

### Beispiele
#### Anwendung auf einen Term
**Beispiel 1:**
- Sei der Term:$$
  t = f(x, g(y, z))
  $$
- Sei die Substitution:$$
  \sigma = \{ x / a, y / b \}
  $$
- Anwendung der Substitution $\sigma$ auf den Term $t$:$$
  \sigma(t) = f(\sigma(x), g(\sigma(y), \sigma(z))) = f(a, g(b, z))
  $$
- Erklärung:
  - $x$ wird durch $a$ ersetzt.
  - $y$ wird durch $b$ ersetzt.
  - $z$ bleibt unverändert, da $\sigma(z) = z$.
#### Anwendung auf eine Formel
**Beispiel 2:**
- Sei die Formel:$$
  \phi = P(x) \land Q(y, z)
  $$
- Sei die Substitution:$$
  \sigma = \{ x / a, y / f(b) \}
  $$
- Anwendung der Substitution $\sigma$ auf die Formel $\phi$:$$
  \sigma(\phi) = P(\sigma(x)) \land Q(\sigma(y), \sigma(z)) = P(a) \land Q(f(b), z)
  $$
- Erklärung:
  - $x$ wird durch $a$ ersetzt.
  - $y$ wird durch $f(b)$ ersetzt.
  - $z$ bleibt unverändert, da $\sigma(z) = z$.

#### Beispiel mit gebundenen und freien Variablen
**Beispiel 3:**
- Sei die Formel:$$
  \phi = \forall z \, (P(x) \rightarrow \exists y \, Q(x, y, z))
  $$
- Sei die Substitution:$$
  \sigma = \{ x / a, y / b \}
  $$
- Anwendung der Substitution $\sigma$ auf die Formel $\phi$:$$
  \sigma(\phi) = \forall z \, (P(\sigma(x)) \rightarrow \exists y \, Q(\sigma(x), y, z)) = \forall z \, (P(a) \rightarrow \exists y \, Q(a, y, z))
  $$
- Erklärung:
  - $x$ wird durch $a$ ersetzt, sowohl in $P(x)$ als auch in $Q(x, y, z)$.
  - $y$ bleibt in der Existenzquantifikation gebunden und wird nicht durch $b$ ersetzt, da nur freie Variablen ersetzt werden.

## Definition: Kollisionsfreie Substitutionen
> [!abstract] Definition
> In der Prädikatenlogik erster Stufe sind **kollisionsfreie Substitutionen** wichtig, um sicherzustellen, dass die Anwendung einer Substitution auf eine Formel nicht zu unbeabsichtigten Änderungen in der Bedeutung der Formel führt. Insbesondere sollen keine gebundenen Variablen durch die Substitution in Konflikt mit freien Variablen geraten. 

-> Eine Substitution $\sigma$ heißt **kollisionsfrei** für eine Formel $A$, wenn für jede Variable $z$ und jede Stelle des freien Auftretens von $z$ in $A$ gilt:
Diese Stelle liegt nicht im Wirkungsbereich eines Präfixes $\forall x$ oder $\exists x$, bei dem $x$ eine Variable in $\sigma(z)$ ist.
### Erklärung
  - Eine **Kollision** tritt auf, wenn eine freie Variable durch eine Substitution ersetzt wird, deren Ergebnis eine Variable enthält, die innerhalb eines Wirkungsbereichs eines Quantors derselben Variable liegt. Dies kann die logische Struktur und Bedeutung der Formel ungewollt verändern.
### Beispiel zur Verdeutlichung
#### Beispiel: Nicht kollisionsfreie Substitution
- Gegebene Formel:$$
  \forall y \, p(x, y)
  $$
- Gegebene Substitution:$$
  \mu_1 = \{ x / y \}
  $$
- Anwendung der Substitution $\mu_1$ auf die Formel:$$
  \mu_1(\forall y \, p(x, y)) = \forall y \, p(y, y)
  $$
- Problem:
  - Hierbei wird $x$ durch $y$ ersetzt. Allerdings ist $y$ bereits eine gebundene Variable im Wirkungsbereich des Quantors $\forall y$.
  - Dies führt zu einer unerwünschten Veränderung der Formel, da nun eine Kollision mit der gebundenen Variable $y$ vorliegt.

- Ergebnis:$$
  \mu_1 = \{ x / y \} \text{ ist nicht kollisionsfrei für } \forall y \, p(x, y).
  $$

#### Beispiel: Kollisionsfreie Substitution
- Gegebene Formel:$$
  \forall y \, p(x, y)
  $$
- Gegebene Substitution:$$
  \mu_2 = \{ x / z \}
  $$
- Anwendung der Substitution $\mu_2$ auf die Formel:$$
  \mu_2(\forall y \, p(x, y)) = \forall y \, p(z, y)
  $$
- Hier ist $z$ eine neue Variable, die nicht im Wirkungsbereich des Quantors $\forall y$ liegt und somit keine Kollision verursacht.

- Ergebnis:$$
  \mu_2 = \{ x / z \} \text{ ist kollisionsfrei für } \forall y \, p(x, y).
  $$

### Kriterien für kollisionsfreie Substitutionen
- Bei einer Substitution $\sigma = \{ x_1 / t_1, x_2 / t_2, \ldots, x_n / t_n \}$ muss für jede Variable $x_i$ und jeden Term $t_i$ sichergestellt werden, dass die Ersetzung $x_i$ durch $t_i$ keine Kollisionen mit bestehenden gebundenen Variablen in der Formel $A$ verursacht.
- Es darf keine Variable $x$ in $t_i$ geben, die innerhalb eines Wirkungsbereichs eines Quantors auftritt, der $x$ bindet, da dies die logische Struktur der Formel verändern würde.

In der Prädikatenlogik erster Stufe beschreibt die **Komposition von Substitutionen** das aufeinanderfolgende Anwenden zweier Substitutionen auf eine Variable. Dabei wird eine Substitution zuerst angewendet, und das Ergebnis wird dann der zweiten Substitution unterzogen. Hier ist die Definition und eine Erklärung der Komposition von Substitutionen:

## Definition: Komposition von Substitutionen
Sind $\sigma$ und $\tau$ Substitutionen, dann definieren wir die **Komposition von $\tau$ mit $\sigma$** durch:$$
(\tau \circ \sigma)(x) = \tau(\sigma(x))
$$

- Dies bedeutet, dass zunächst die Substitution $\sigma$ auf die Variable $x$ angewendet wird, und das Ergebnis wird dann der Substitution $\tau$ unterzogen.
- Auf der rechten Seite ist $\tau$ als die Anwendung der Substitution $\tau$ auf den Term $\sigma(x)$ zu verstehen. Es wird also das Ergebnis der Substitution $\sigma(x)$ erneut einer Substitution unterzogen.
### Erklärung
- **Substitution $\sigma$**: Diese erste Substitution ersetzt die Variable $x$ durch einen Term $t = \sigma(x)$.
- **Substitution $\tau$**: Die zweite Substitution $\tau$ wird dann auf den Term $\sigma(x)$ angewendet, sodass das Ergebnis $\tau(\sigma(x))$ die endgültige Ersetzung für $x$ darstellt.
Die Komposition ermöglicht eine flexiblere und effektivere Handhabung von Variablenersetzungen, indem sie es erlaubt, komplexere Ersetzungsregeln durch die Kombination einfacherer Substitutionen zu definieren.
### Beispiele
#### Beispiel 1: Einfache Komposition
- **Gegebene Substitutionen**:$$
  \sigma = \{ x / y \}
  $$$$
  \tau = \{ y / z \}
  $$
- **Komposition**:$$
  (\tau \circ \sigma)(x) = \tau(\sigma(x)) = \tau(y) = z
  $$
- **Ergebnis**: $$
  (\tau \circ \sigma)(x) = z
  $$
  Das heißt, $x$ wird durch $y$ und dann $y$ durch $z$ ersetzt, was insgesamt zu $z$ führt.

#### Beispiel 2: Komplexere Substitution
- **Gegebene Substitutionen**:$$
  \sigma = \{ x / f(a, b) \}
  $$$$
  \tau = \{ a / c, b / d \}
  $$
- **Komposition**:$$
  (\tau \circ \sigma)(x) = \tau(\sigma(x)) = \tau(f(a, b)) = f(\tau(a), \tau(b)) = f(c, d)
  $$
- **Ergebnis**:$$
  (\tau \circ \sigma)(x) = f(c, d)
  $$
  Das bedeutet, $x$ wird zuerst durch $f(a, b)$ und dann $a$ durch $c$ und $b$ durch $d$ ersetzt.

### Wichtige Hinweise
- **Reihenfolge der Substitutionen**:
  - Die Reihenfolge der Substitutionen ist entscheidend. Die Substitution $\sigma$ wird zuerst angewendet, gefolgt von $\tau$.
  - $\tau \circ \sigma$ ist nicht notwendigerweise dasselbe wie $\sigma \circ \tau$.
- **Interpretation der Komposition**:
  - Die Komposition wird wie eine Kettenregel angewendet. Zuerst wird die innere Substitution angewendet, dann die äußere.
- **Vermeidung von Kollisionen**:
  - Bei der Anwendung von Substitutionen sollte man sicherstellen, dass keine Kollisionen entstehen, d.h., dass durch die Ersetzung keine unbeabsichtigten Bindungen von Variablen entstehen.

## Theorem zur Anwendung struktureller Induktion auf Terme

1. **Aussage**: Wenn für einen Term $t \in \text{Term}_\Sigma$ und zwei Substitutionen $\sigma$ und $\tau$ die Gleichung $\sigma(t) = \tau(t)$ gilt, dann gilt $\sigma(s) = \tau(s)$ für jeden Teilterm $s$ von $t$.
### Beweis
Der Beweis erfolgt mittels struktureller Induktion nach dem Aufbau des Terms $t$.
#### Basisfall: $t \in \text{Var}$
- **Annahme**: $t$ ist eine Variable.
- **Teilterm**: Der einzige Teilterm von $t$ ist $t$ selbst.
- **Substitution**: Da $t$ eine Variable ist, gilt:$$
  \sigma(t) = \tau(t)
  $$
  Das bedeutet, die Substitutionen $\sigma$ und $\tau$ müssen für die Variable $t$ denselben Term liefern.
- **Ergebnis**: Da keine weiteren Teilterme existieren, folgt direkt, dass $\sigma(t) = \tau(t)$ für $t$.
#### Induktionsschritt: $t = f(t_1, t_2, \ldots, t_n)$
- **Annahme**: Der Term $t$ ist von der Form $t = f(t_1, t_2, \ldots, t_n)$, wobei $f$ ein $n$-stelliges Funktionssymbol ist und $t_1, t_2, \ldots, t_n$ Terme sind.
- **Substitutionen**: Wir wenden die Substitutionen $\sigma$ und $\tau$ auf $t$ an:$$
  \sigma(t) = \sigma(f(t_1, t_2, \ldots, t_n)) = f(\sigma(t_1), \sigma(t_2), \ldots, \sigma(t_n))
  $$$$
  \tau(t) = \tau(f(t_1, t_2, \ldots, t_n)) = f(\tau(t_1), \tau(t_2), \ldots, \tau(t_n))
  $$
- **Gleichheit der Substitutionen**: Da $\sigma(t) = \tau(t)$ vorausgesetzt ist, folgt:$$
  f(\sigma(t_1), \sigma(t_2), \ldots, \sigma(t_n)) = f(\tau(t_1), \tau(t_2), \ldots, \tau(t_n))
  $$
- **Induktionsvoraussetzung**: Wir nehmen an, dass für jeden Teilterm $t_i$ von $t$ gilt:$$
  \sigma(t_i) = \tau(t_i)
  $$
- **Folgerung für alle Teilterme**: Jeder Teilterm $s$ von $t$ ist entweder $t$ selbst oder ein Teilterm eines der $t_i$. Da die Induktionsvoraussetzung besagt, dass $\sigma(s) = \tau(s)$ für alle Teilterme $s$ der $t_i$ gilt, folgt daraus:$$
  \sigma(s) = \tau(s)
  $$
  für jeden Teilterm $s$ von $t$.

## Theorem Variablenumbenennung
- Variablenumbenennung eine spezielle Art der Substitution, bei der Variablen durch andere Variablen ersetzt werden, ohne die Struktur oder Bedeutung der Formel zu ändern.
- **Aussage**: Gilt für die Substitutionen $\sigma$ und $\tau$, dass $\tau \circ \sigma = \text{id}$, dann ist $\sigma$ eine Variablenumbenennung.
### Beweis
Der Beweis erfolgt durch Analyse der Substitutionen und zeigt, dass $\sigma$ nur Variablen in andere Variablen umbenennt.
#### Annahmen und Ziel
- Es ist gegeben, dass $\tau \circ \sigma = \text{id}$, was bedeutet, dass die Komposition der Substitutionen $\sigma$ und $\tau$ die Identität liefert.
- Ziel ist es zu zeigen, dass $\sigma$ eine Variablenumbenennung ist, d.h., $\sigma$ ersetzt jede Variable nur durch eine andere Variable und nicht durch einen komplexen Term.
#### Schritte des Beweises
1. **Identität der Komposition**:
   - Die Annahme $\tau \circ \sigma = \text{id}$ bedeutet:$$
     (\tau \circ \sigma)(x) = \text{id}(x) = x \text{ für jedes } x \in \text{Var}
     $$
   - Das heißt:$$
     \tau(\sigma(x)) = x \text{ für jedes } x \in \text{Var}
     $$

2. **Ersetzung durch Variablen**:
   - Aus $\tau(\sigma(x)) = x$ folgt, dass $\sigma(x)$ eine Variable sein muss, weil $\tau$ nur auf Variablen definiert ist und $\tau$ den Term $\sigma(x)$ zurück zu $x$ abbildet.
   - Daher gilt:$$
     \sigma(x) \in \text{Var}
     $$
3. **Eindeutigkeit der Ersetzung**:
   - Nehmen wir an, $\sigma(x) = \sigma(y)$ für zwei verschiedene Variablen $x$ und $y$.
   - Dann gilt:$$
     \tau(\sigma(x)) = \tau(\sigma(y))
     $$
   - Da aber $\tau(\sigma(x)) = x$ und $\tau(\sigma(y)) = y$ ist, folgt:$$
     x = y
     $$
   - Dies zeigt, dass die Substitution $\sigma$ bijektiv ist, d.h., sie bildet jede Variable auf eine eindeutige andere Variable ab, ohne Mehrdeutigkeiten.

4. **Schlussfolgerung**:
   - Da $\sigma$ jede Variable nur durch eine andere Variable ersetzt und dies auf eindeutige Weise geschieht, ist $\sigma$ eine Variablenumbenennung.
   - Somit ist $\sigma$ eine Substitution, die Variablen lediglich in andere Variablen umbenennt, und zwar in einer Weise, dass die Rücksubstitution durch $\tau$ die ursprüngliche Variable wiederherstellt.
### Beispiel zur Verdeutlichung
**Beispiel**:
- Sei $\sigma = \{ x / y, y / z, z / x \}$, also eine Zyklus-Substitution, die jede Variable in eine andere Variable umbenennt.
- Sei $\tau = \{ y / x, z / y, x / z \}$, die Umkehrung von $\sigma$.
- Dann gilt:$$
  \tau(\sigma(x)) = \tau(y) = x
  $$$$
  \tau(\sigma(y)) = \tau(z) = y
  $$$$
  \tau(\sigma(z)) = \tau(x) = z
  $$
- Hierbei ist klar, dass die Komposition $\tau \circ \sigma$ jede Variable wieder auf sich selbst abbildet, also die Identität $\text{id}$ ist.
- Dies bestätigt, dass $\sigma$ eine Variablenumbenennung ist.

## Definition: Unifikation
- Die **Unifikation** beschreibt den Prozess des Findens einer Substitution, die eine Menge von Termen gleich macht.
- **Unifikator**: Eine Substitution $\sigma$ über einer Signatur $\Sigma$ heißt **Unifikator** für eine Menge von Termen $T \subseteq \text{Term}_\Sigma$, wenn durch Anwendung von $\sigma$ alle Terme in $T$ gleich werden, d.h.:$$
  \#\sigma(T) = 1
  $$
  - Das bedeutet, dass nach der Substitution $\sigma$ alle Terme in $T$ auf einen einzigen Term reduziert werden.

- **Unifizierbare Terme**: Eine Menge von Termen $T$ heißt **unifizierbar**, wenn es eine Substitution $\sigma$ gibt, die $T$ unifiziert, d.h., alle Terme in $T$ gleich macht.

- **Unifizierbare Terme s und t**: Zwei Terme $s$ und $t$ heißen unifizierbar, wenn es eine Substitution $\sigma$ gibt, so dass:$$
  \sigma(s) = \sigma(t)
  $$

### Beispiel zur Unifikation
Gegeben ist die Menge von Termen:$$
T = \{ f(g(a, x), g(y, b)), f(z, g(v, w)), f(g(x, a), g(v, b)) \}
$$

- Ziel: Finde eine Substitution $\sigma$, so dass alle Terme in $T$ gleich werden.
- Eine mögliche Substitution, die $T$ unifiziert, ist:$$
  \sigma = \{ x / a, y / v, z / g(a, a), w / b \}
  $$

- Anwendung der Substitution $\sigma$ auf die Terme in $T$:
  1. $\sigma(f(g(a, x), g(y, b)))$:     $$
     = f(g(a, \sigma(x)), g(\sigma(y), b))
     = f(g(a, a), g(v, b))
     $$
  
  2. $\sigma(f(z, g(v, w)))$:     $$
     = f(\sigma(z), g(v, \sigma(w)))
     = f(g(a, a), g(v, b))
     $$
  
  3. $\sigma(f(g(x, a), g(v, b)))$:     $$
     = f(g(\sigma(x), a), g(v, b))
     = f(g(a, a), g(v, b))
     $$

- Ergebnis:$$
  \sigma(T) = \{ f(g(a, a), g(v, b)), f(g(a, a), g(v, b)), f(g(a, a), g(v, b)) \}
  $$

- Da alle Terme nach der Anwendung der Substitution $\sigma$ gleich sind, unifiziert $\sigma$ die Menge $T$.


### Elementare Fakten zur Unifikation
1. **Jeder Term ist mit sich selbst unifizierbar mittels der identischen Substitution ($\text{id}$).**
2. **Zwei Terme mit unterschiedlichen Funktionssymbolen (Köpfen) sind nicht unifizierbar.**
3. **Zwei Terme mit demselben Funktionssymbol (Kopf) sind genau dann unifizierbar, wenn eine Substitution existiert, die ihre Argumente paarweise gleich macht.**
4. **Eine Variable und ein Term sind genau dann unifizierbar, wenn die Variable nicht im Term vorkommt.**
#### Fakt 1: Jeder Term ist mit sich selbst unifizierbar mittels der identischen Substitution ($\text{id}$).
- **Erklärung**: 
  - Die Identitätssubstitution $\text{id}$ ändert keine Variablen oder Terme. Für jeden Term $t$ gilt:
    $$
    \text{id}(t) = t
    $$
  - Dies bedeutet, dass die Substitution $\text{id}$ den Term $t$ unverändert lässt.
  - Da jeder Term $t$ identisch mit sich selbst ist, wird $t$ durch $\text{id}$ unifiziert.
- **Beispiel**:
  - Sei $t = f(x, g(y))$. Dann gilt:$$
    \text{id}(t) = f(x, g(y)) = t
    $$
  - Der Term $t$ ist somit mit sich selbst unifizierbar.
#### Fakt 2: Zwei Terme mit unterschiedlichen Funktionssymbolen (Köpfen) sind nicht unifizierbar.
- **Erklärung**:
  - Zwei Terme $f(s_1, \ldots, s_n)$ und $g(t_1, \ldots, t_n)$, die unterschiedliche Funktionssymbole $f$ und $g$ haben, können nicht unifiziert werden, weil die Funktionssymbole die Struktur der Terme bestimmen und nicht durch Substitutionen verändert werden können.
  - Die Struktur der Terme ist zu unterschiedlich, um eine gemeinsame Substitution zu finden.
- **Beispiel**:
  - Die Terme $f(a)$ und $g(b)$ sind nicht unifizierbar, da $f$ und $g$ unterschiedliche Funktionssymbole sind.
#### Fakt 3: Zwei Terme mit demselben Funktionssymbol (Kopf) sind genau dann unifizierbar, wenn eine Substitution existiert, die ihre Argumente paarweise gleich macht.
- **Erklärung**:
  - Zwei Terme $f(s_1, \ldots, s_n)$ und $f(t_1, \ldots, t_n)$, die dasselbe Funktionssymbol $f$ haben, können unifiziert werden, wenn es eine Substitution $\sigma$ gibt, die dafür sorgt, dass:$$
    \sigma(s_i) = \sigma(t_i) \text{ für alle } i = 1, \ldots, n
    $$
  - Die Unifikation erfolgt paarweise auf den Argumenten der Terme.
- **Beispiel**:
  - Die Terme $f(a, x)$ und $f(y, b)$ sind unifizierbar durch die Substitution $\{x / b, y / a\}$:$$
    \sigma(f(a, x)) = f(a, b)
    $$$$
    \sigma(f(y, b)) = f(a, b)
    $$

#### Fakt 4: Eine Variable und ein Term sind genau dann unifizierbar, wenn die Variable nicht im Term vorkommt.
- **Erklärung**:
  - Eine Variable $x$ und ein Term $t$ sind unifizierbar, wenn $x$ in $t$ nicht vorkommt. In diesem Fall kann $x$ durch $t$ ersetzt werden, ohne dass ein Zirkelschluss oder eine Rekursion entsteht.
  - Wenn $x$ in $t$ vorkommt, würde die Substitution $x \rightarrow t$ zu einer endlosen Verschachtelung führen.
- **Beispiel**:
  - Die Variable $x$ und der Term $f(a, b)$ sind unifizierbar durch die Substitution $\{x / f(a, b)\}$.
  - Die Variable $x$ und der Term $f(x, a)$ sind nicht unifizierbar, da $x$ in $f(x, a)$ vorkommt.

## Definition: Allgemeinster Unifikator (mgu)
- Ein **allgemeinster Unifikator (most general unifier, mgu)** ist eine Substitution, die eine Menge von Termen so vereinheitlicht, dass jede andere Substitution, die dieselbe Menge unifiziert, eine Erweiterung dieser allgemeinsten Substitution ist.
- Sei $T \subseteq \text{Term}_\Sigma$ eine Menge von Termen über einer Signatur $\Sigma$. Eine Substitution $\mu$ heißt **allgemeinster Unifikator** oder **mgu** von $T$, wenn die folgenden Bedingungen erfüllt sind:
1. **Unifikation von $T$**:$$
   \mu \text{ unifiziert } T
   $$
   - Das bedeutet, dass die Substitution $\mu$ alle Terme in $T$ gleich macht, d.h., nach Anwendung von $\mu$ sind alle Terme in $T$ identisch.
2. **Allgemeinheit**:
   - Zu jedem Unifikator $\sigma$ von $T$ gibt es eine Substitution $\sigma'$ mit:$$
     \sigma = \sigma' \circ \mu
     $$
   - Das bedeutet, dass $\sigma$ als eine Komposition von $\mu$ und einer weiteren Substitution $\sigma'$ geschrieben werden kann. $\mu$ ist somit die allgemeinste Substitution, da jede andere Substitution $\sigma$, die $T$ unifiziert, durch Anwendung einer weiteren Substitution $\sigma'$ auf $\mu$ erhalten werden kann.
### Beispiel für einen allgemeinsten Unifikator
Betrachten wir die Menge von Termen:$$
T = \{ f(x, g(y)), f(a, g(b)) \}
$$
- Ziel: Finde einen allgemeinsten Unifikator $\mu$ für $T$.
#### Schritt-für-Schritt-Unifikation
1. **Terme identifizieren**:$$
   s = f(x, g(y))
   $$$$
   t = f(a, g(b))
   $$
2. **Strukturvergleich**:
   - Beide Terme haben das gleiche Funktionssymbol $f$ mit zwei Argumenten.
   - Vergleiche die Argumente paarweise:$$
     x \text{ und } a
     $$$$
     g(y) \text{ und } g(b)
     $$

3. **Unifikation der Argumente**:
   - Für die erste Position: $\{ x / a \}$
     - Ersetze $x$ durch $a$.
   - Für die zweite Position: $\{ y / b \}$
     - Ersetze $y$ durch $b$.
4. **Kombination der Substitutionen**:$$
   \mu = \{ x / a, y / b \}
   $$
5. **Anwendung der Substitution**:$$
   \mu(s) = f(a, g(b))
   $$$$
   \mu(t) = f(a, g(b))
   $$

- Beide Terme werden durch die Substitution $\mu$ gleich. Somit unifiziert $\mu$ die Menge $T$.
6. **Allgemeinheit prüfen**:
   - Jede andere Substitution $\sigma$, die $T$ unifiziert, muss eine Erweiterung von $\mu$ sein. Zum Beispiel, wenn $\sigma$ eine zusätzliche Substitution $\{ z / c \}$ enthält, gilt:$$
     \sigma = \{ z / c \} \circ \mu
     $$

## Theorem: Eindeutigkeit des allgemeinsten Unifikators
Sei $T$ eine unifizierbare, nichtleere Menge von Termen. Dann ist der allgemeinste Unifikator von $T$ bis auf Variablenumbenennung eindeutig bestimmt. Das bedeutet:
- Sind $\mu$ und $\mu'$ zwei allgemeinste Unifikatoren von $T$ mit $\mu(T) = \{t\}$ und $\mu'(T) = \{t'\}$, dann gibt es eine Umbenennung $\pi$ der Variablen von $t$, so dass:$$
  t' = \pi(t)
  $$
### Beweis
Der Beweis erfolgt in mehreren Schritten und zeigt, dass die Terme, die durch die beiden Unifikatoren erzeugt werden, nur durch eine Variablenumbenennung voneinander abweichen können.
1. **Existenz von Substitutionen**:
   - Da $\mu$ und $\mu'$ beide allgemeinste Unifikatoren von $T$ sind, gibt es Substitutionen $\sigma$ und $\sigma'$, so dass:$$
     \mu' = \sigma \circ \mu
     $$$$
     \mu = \sigma' \circ \mu'
     $$

1. **Anwendung auf $T$**:
   - Durch Anwendung der Unifikatoren auf $T$ erhalten wir:$$
     \mu(T) = t
     $$$$
     \mu'(T) = t'
     $$
   - Da $\mu$ und $\mu'$ beide Unifikatoren von $T$ sind, gilt:$$
     t = \sigma'(\mu'(T)) = \sigma'(\{t'\}) = \sigma'(t')
     $$$$
     t' = \sigma(\mu(T)) = \sigma(\{t\}) = \sigma(t)
     $$
2. **Eindeutigkeit der Terme**:
   - Setzen wir diese Gleichungen in die Definition der Unifikatoren ein, ergibt sich:$$
     t = \sigma'(\sigma(t)) = \sigma'(\sigma(t))
     $$
3. **Ergebnis der Substitution**:
   - Da $\sigma' \circ \sigma$ eine Substitution ist, die $t$ unverändert lässt, muss für jede Variable $x$ in $t$ gelten:$$
     \sigma'(\sigma(x)) = x
     $$
   - Dies bedeutet, dass $\sigma(x)$ eine Variable ist und keine weitergehende Ersetzung erfährt.
4. **Variablenumbenennung**:
   - Folglich ist $\sigma$ eine Umbenennung der Variablen, und $\sigma'(x) = x$, was darauf hinweist, dass $\sigma$ und $\sigma'$ Umbenennungen sind.
   - Insbesondere gilt für jede Variable $x \in \text{Var}(t)$:$$
     \sigma(x) \neq \sigma(y) \text{ für } x \neq y
     $$
   - Somit muss $t'$ eine Umbenennung von $t$ durch eine geeignete Variablenumbenennung $\pi$ sein, die sicherstellt:$$
     t' = \pi(t)
     $$
## Unifikationsalgorithmus von Robinson
Der Unifikationsalgorithmus von Robinson ist ein Verfahren, das schrittweise versucht, zwei Terme durch geeignete Substitutionen gleich zu machen. Dabei werden Unvereinbarkeiten erkannt und entsprechend behandelt. Der Algorithmus besteht aus der Bestimmung von Differenzen zwischen Termen und der schrittweisen Reduktion dieser Differenzen.
### Definitionen
1. **Position in einem Term** ($t(i)$):
   - Für einen Term $t \in \text{Term}_\Sigma$ und eine natürliche Zahl $i \in \mathbb{N}$, ist $t(i)$ der Teilterm von $t$, der an der Position $i$ beginnt (beim Lesen von links nach rechts), wenn dort eine Variable oder ein Funktionssymbol steht.
   - Ist dort kein Funktionssymbol oder keine Variable, so ist $t(i)$ undefiniert.
2. **Differenz einer Termmenge $T$**:
   - Für eine Menge von Termen $T \subseteq \text{Term}_\Sigma$ ist die Differenz $D(T)$ wie folgt definiert:
     1. **Ein Term** ($\#T \leq 1$):$$
        D(T) := T
        $$
        - Wenn $T$ nur einen Term enthält oder leer ist, bleibt $D(T)$ gleich $T$.
     2. **Mehrere Terme** ($\#T \geq 2$):
        - Sei $j$ die kleinste Zahl, so dass sich zwei Terme aus $T$ an der Position $j$ unterscheiden.
        - Setze:$$
          D(T) := \{ t(j) \mid t \in T \}
          $$
### Beispiel für die Differenzbildung
Gegeben ist die Menge von Termen:$$
T = \{ f(g(a, x), g(y, b)), f(z, g(v, w)), f(g(x, a), g(v, b)) \}
$$
- Bestimme die Position, an der sich die Terme unterscheiden:
  - Vergleiche die Terme von links nach rechts:
    - An der Position 1 haben alle Terme das gleiche Funktionssymbol $f$.
    - An der Position 2 beginnen die Terme $g(a, x)$, $z$, und $g(x, a)$, die sich unterscheiden.
- Die Differenz $D(T)$ ist somit:$$
  D(T) = \{ g(a, x), z, g(x, a) \}
  $$

### Unifikationsalgorithmus von Robinson: Schritt-für-Schritt
1. **Initialisierung**:
   - Beginne mit einer Menge von Termpaaren, die unifiziert werden sollen.
   - Setze die initiale Substitution als leere Substitution ($\sigma = \{ \}$).
2. **Schrittweise Verarbeitung der Paare**:
   - Für jedes Paar von Termen $(s, t)$:
     1. **Wenn $s$ und $t$ Variablen sind**:
        - Wenn $s = t$, ignoriere das Paar.
        - Wenn $s \neq t$, wende die Substitution $\{ s / t \}$ an und füge sie zur laufenden Substitution hinzu.
     2. **Wenn einer der Terme eine Variable ist**:
        - Substituiere die Variable durch den anderen Term.
     3. **Wenn beide Terme Funktionssymbole haben**:
        - Wenn die Funktionssymbole gleich sind, zerlege die Terme in ihre Argumente und bearbeite die Paare der Argumente.
        - Wenn die Funktionssymbole unterschiedlich sind, ist die Unifikation nicht möglich.
3. **Anwendung der Substitutionen**:
   - Wende die laufenden Substitutionen auf die verbleibenden Paare an und iteriere, bis keine Paare mehr vorhanden sind oder eine Unifikation nicht möglich ist.
4. **Ergebnis**:
   - Wenn alle Paare erfolgreich verarbeitet wurden, ist die endgültige Substitution der allgemeinste Unifikator.
   - Wenn es eine Unvereinbarkeit gibt, ist die Unifikation nicht möglich.
### Detailliertes Beispiel für den Algorithmus
#### Beispiel: Unifikation von $f(a, x)$ und $f(y, g(b))$

1. **Initialisierung**:$$
   \text{Paare} = \{ (f(a, x), f(y, g(b))) \}
   $$$$
   \sigma = \{ \}
   $$
2. **Verarbeitung des Paares**:$$
   \{ f(a, x) = f(y, g(b)) \}
   $$
   - Funktionssymbole $f$ sind gleich.
   - Zerlege in die Paare:$$
     \text{Paare} = \{ (a, y), (x, g(b)) \}
     $$
3. **Bearbeite das Paar $(a, y)$**:
   - $a$ und $y$ sind unterschiedlich.
   - Substituiere $y$ durch $a$:$$
     \sigma = \{ y / a \}
     $$
4. **Bearbeite das Paar $(x, g(b))$**:
   - $x$ ist eine Variable.
   - Substituiere $x$ durch $g(b)$:$$
     \sigma = \{ y / a, x / g(b) \}
     $$
5. **Endergebnis**:
   - Die endgültige Substitution $\sigma$ ist der allgemeinste Unifikator:$$
     \sigma = \{ y / a, x / g(b) \}
     $$

- Diese Substitution macht die Terme $f(a, x)$ und $f(y, g(b))$ gleich, indem $x$ durch $g(b)$ und $y$ durch $a$ ersetzt wird.

## Theorem: Unifikationstheorem
1. **Terminierung**: Der Algorithmus von Robinson terminiert für jede endliche, nicht-leere Menge $T \subseteq \text{Term}_\Sigma$.
2. **Allgemeinster Unifikator**: Wenn $T$ unifizierbar ist, liefert der Algorithmus einen allgemeinsten Unifikator (mgu) für $T$.
3. **Nicht-Unifizierbarkeit**: Wenn $T$ nicht unifizierbar ist, gibt der Algorithmus die Ausgabe „$T$ nicht unifizierbar“.
### Beweis
#### 1. Der Algorithmus terminiert
- **Strukturelle Argumentation**:
  - Der Unifikationsalgorithmus verarbeitet Terme, indem er sie schrittweise in einfachere Terme zerlegt.
  - In jedem Schritt wird entweder eine Variable durch einen Term ersetzt oder Funktionssymbole und deren Argumente werden verglichen und weiter zerlegt.
  - Da $T$ endlich ist und der Algorithmus in jedem Schritt Terme reduziert, kann dieser Prozess nur eine endliche Anzahl von Schritten durchführen, bevor er entweder einen Unifikator findet oder feststellt, dass die Terme nicht unifizierbar sind.
- **Beispiel**:
  - Nehmen wir eine Menge von Termen $T = \{ f(x), f(a) \}$:
    - Der Algorithmus würde erkennen, dass $x$ durch $a$ ersetzt werden muss, was in einem Schritt möglich ist.
    - Dies zeigt, dass der Prozess schnell zum Ende kommt, da die Menge der zu verarbeitenden Terme abnimmt.
#### 2. Wenn der Algorithmus eine Substitution $\mu$ ausgibt, dann ist $\mu$ Unifikator von $T$
- **Unifikator**:
  - Wenn der Algorithmus eine Substitution $\mu$ findet, bedeutet das, dass $\mu$ alle Terme in $T$ gleich macht. Dies ist die Definition eines Unifikators.
  - Der Algorithmus prüft jede Paarung von Termen, um sicherzustellen, dass sie durch $\mu$ gleich gemacht werden können.
- **Beispiel**:
  - Für $T = \{ f(x, a), f(b, y) \}$:
    - Der Algorithmus gibt die Substitution $\mu = \{ x / b, y / a \}$ aus, die beide Terme gleich macht, da:      $$
      \mu(f(x, a)) = f(b, a) \text{ und } \mu(f(b, y)) = f(b, a)
     $$
#### 3. Ist $\sigma$ ein beliebiger Unifikator von $T$, dann gibt es $\sigma'$ so, dass der Algorithmus mit Ausgabe $\mu$ terminiert und $\sigma = \sigma' \circ \mu$
- **Eindeutigkeit des mgu**:
  - Wenn $\sigma$ ein Unifikator für $T$ ist, dann kann $\sigma$ als eine Komposition $\sigma' \circ \mu$ geschrieben werden, wobei $\mu$ der allgemeinste Unifikator ist, den der Algorithmus gefunden hat.
  - Das bedeutet, dass $\mu$ die allgemeinste Lösung ist und jede andere Lösung durch zusätzliche Substitutionen (Variablenumbenennungen) von $\mu$ erhalten werden kann.
- **Beispiel**:
  - Angenommen, $T = \{ f(x, y), f(a, b) \}$ und $\sigma = \{ x / a, y / b \}$:
    - Der Algorithmus gibt $\mu = \{ x / a, y / b \}$ aus.
    - Jede andere Substitution, die $T$ unifiziert, kann als Erweiterung dieser $\mu$ angesehen werden.
#### 4. Wenn der Algorithmus ausgibt „$T$ nicht unifizierbar“, dann ist $T$ nicht unifizierbar
- **Korrektheit der Unifizierung**:
  - Der Algorithmus von Robinson prüft systematisch jede Möglichkeit der Unifikation. Wenn er feststellt, dass zwei Terme nicht vereinbar sind (z.B. unterschiedliche Funktionssymbole an derselben Position), kann keine Substitution diese Terme gleich machen.
  - Daher ist die Aussage „$T$ nicht unifizierbar“ korrekt, wenn der Algorithmus diese Ausgabe gibt.
- **Beispiel**:
  - Nehmen wir $T = \{ f(x), g(y) \}$:
    - Der Algorithmus erkennt, dass $f$ und $g$ unterschiedliche Funktionssymbole sind, was die Unifikation unmöglich macht, und gibt daher korrekt „nicht unifizierbar“ aus.

## Signatur Σ der Prädikatenlogik 1. Ordnung (PL1)
Eine **Signatur** in der Prädikatenlogik 1. Ordnung (PL1) ist eine Sammlung von Symbolen, die in den Formeln verwendet werden. Sie umfasst:
- Konstanten (z.B. $a, b, c$)
- Funktionssymbole (z.B. $f, g$)
- Prädikatsymbole (z.B. $P, Q$)
Diese Symbole werden verwendet, um Aussagen und Ausdrücke zu formulieren.
## Interpretation
Eine **Interpretation** in der Prädikatenlogik gibt diesen Symbolen eine konkrete Bedeutung. Sie bestimmt, wie die Symbole auf einer Menge von Objekten (genannt Universum oder Trägerbereich) interpretiert werden.
### Definition einer Interpretation
Eine Interpretation $D$ der Signatur $\Sigma$ ist ein Paar $(D, I)$, wobei:
1. **D ist eine nicht leere Menge**:
   - **D** wird als Universum oder Trägerbereich bezeichnet. Es ist die Menge von Objekten, auf die sich die Aussagen der Logik beziehen. Zum Beispiel könnte $D$ die Menge aller Menschen, aller Zahlen oder aller Tiere sein.
2. **I ist eine Abbildung der Signatursymbole**:
   - **I** (Interpretationsfunktion) ordnet jedem Symbol der Signatur eine spezifische Bedeutung in Bezug auf die Menge $D$ zu. Die Abbildung $I$ gibt an, was die Symbole der Signatur in der konkreten Welt $D$ bedeuten.
### Details zur Abbildung $I$:
#### Konstanten:
- Jeder **Konstanten** $c$ wird ein spezifisches Element in $D$ zugewiesen.
  - **I(c)** ist das Element in $D$, das durch die Konstante $c$ repräsentiert wird.
  - Beispiel: Wenn $c$ eine Konstante für „die Zahl 5“ ist, dann könnte $I(c) = 5$ sein, wenn $D$ die Menge der Zahlen ist.
#### Funktionssymbole:
- Für $n \geq 1$, jedem **n-stelligen Funktionssymbol** $f$ wird eine Funktion zugeordnet, die $n$ Elemente aus $D$ auf ein Element in $D$ abbildet.
  - **I(f)** ist eine Funktion, die eine $n$-stellige Eingabe aus $D$ auf ein Element in $D$ abbildet.
  - Beispiel: Wenn $f$ das Funktionssymbol für „Addition“ ist, dann könnte $I(f)$ eine Funktion sein, die zwei Zahlen addiert, wie $I(f)(x, y) = x + y$.
#### Prädikatsymbole:
- Für **null-stellige Prädikatsymbole** (die wie Konstanten sind), wird ein Wahrheitswert (Wahr oder Falsch) zugewiesen.
  - **I(P)** gibt den Wahrheitswert des Prädikatsymbols $P$ an.
  - Beispiel: Wenn $P$ ein Symbol für „es regnet“ ist, könnte $I(P)$ wahr (W) oder falsch (F) sein.
- Für $n \geq 1$, jedem **n-stelligen Prädikatsymbol** $p$ wird eine Relation auf $D$ zugeordnet.
  - **I(p)** ist eine $n$-stellige Relation, das heißt, eine Menge von $n$-Tupeln, die angibt, welche Kombinationen von $n$ Objekten aus $D$ die Beziehung $p$ erfüllen.
  - Beispiel: Wenn $p$ ein Symbol für „ist größer als“ ist, könnte $I(p)$ die Menge aller Paare $(x, y)$ sein, für die $x > y$ gilt.
### Beispiel für eine Interpretation
Nehmen wir eine Signatur $\Sigma$ mit den folgenden Symbolen:
- Konstanten: $a, b$
- Funktionssymbole: $f$ (1-stellig)
- Prädikatsymbole: $P$ (0-stellig), $Q$ (2-stellig)
Eine mögliche Interpretation könnte sein:
- $D = \{ 1, 2, 3 \}$
- $I(a) = 1$
- $I(b) = 2$
- $I(f)$ ist eine Funktion, die eine Zahl $x$ auf $x+1$ abbildet.
- $I(P)$ ist wahr.
- $I(Q)$ ist die Relation $\{ (1, 2), (2, 3) \}$, die bedeutet, dass $Q(1, 2)$ und $Q(2, 3)$ wahr sind.
### Beispiel: Tarski’s World
![[Pasted image 20240612131929.png#invert|400]]
#### Signatur $\Sigma$
Die Signatur $\Sigma$ besteht aus folgenden Symbolen:
- $k( )$ – Funktion oder Prädikat, das Kreise beschreibt.
- $q( )$ – Funktion oder Prädikat, das Quadrate beschreibt.
- $d( )$ – Funktion oder Prädikat, das Dreiecke beschreibt.
- $kl( )$ – Funktion oder Prädikat, das „klein“ beschreibt.
- $gr( )$ – Funktion oder Prädikat, das „groß“ beschreibt.
- $in( , )$ – Relation, die eine „Enthaltensein“-Beziehung zwischen zwei Objekten beschreibt.
#### Interpretation
Eine Interpretation $D$ der Signatur $\Sigma$ wird durch die Menge der Objekte $D_{Bsp}$ und eine Zuordnung $I_{Bsp}$ der Signatursymbole definiert.
#### Objekte $D_{Bsp}$
Die Menge der Objekte $D_{Bsp}$ in dieser Welt besteht aus:
- Quadrate: $\{ Q_1, Q_2, Q_3, Q_4, Q_5, Q_6 \}$
- Kreise: $\{ K_1, K_2, K_3 \}$
- Dreiecke: $\{ D_1, D_2, D_3 \}$
#### Zuordnungen $I_{Bsp}$
Die Zuordnung $I_{Bsp}$ der Signatursymbole definiert die Bedeutungen der Symbole in Bezug auf die Objekte in $D_{Bsp}$.
1. **Quadrate $q$**:
   - $I_{Bsp}(q) = \{ Q_1, Q_2, Q_3, Q_4, Q_5, Q_6 \}$
   - Alle Quadrate $Q_1$ bis $Q_6$ werden durch das Symbol $q$ repräsentiert.
2. **Kreise $k$**:
   - $I_{Bsp}(k) = \{ K_1, K_2, K_3 \}$
   - Alle Kreise $K_1$ bis $K_3$ werden durch das Symbol $k$ repräsentiert.
3. **Dreiecke $d$**:
   - $I_{Bsp}(d) = \{ D_1, D_2, D_3 \}$
   - Alle Dreiecke $D_1$ bis $D_3$ werden durch das Symbol $d$ repräsentiert.
4. **Relation $in$**:
   - $I_{Bsp}(in)$ definiert, welche Objekte in welchen anderen Objekten enthalten sind. Die Relation enthält Paare der Form $(x, y)$, wobei $x$ in $y$ enthalten ist.
   - $I_{Bsp}(in) = \{ (K_1, Q_1), (K_1, Q_3), (K_2, Q_1), (K_2, Q_2), (K_3, Q_2), (K_3, Q_4), (D_3, D_1), (Q_5, D_2) \}$
5. **„Klein“ $kl$** und „Groß“ $gr$**:
   - Diese Symbole sind nicht direkt in der Abbildung beschrieben, könnten jedoch ebenfalls Relationen oder Funktionen definieren, die angeben, welche Objekte als „klein“ oder „groß“ gelten.

Die Definition bezieht sich auf die Prädikatenlogik und erklärt, wie Variablenbelegungen (auch als Substitutionen oder Zuordnungen bezeichnet) funktionieren, wenn sie auf einer bestimmten Interpretation basieren. Lass uns das Schritt für Schritt durchgehen.

## Variablenbelegungen
Eine **Interpretation** $(D, I)$ in der Prädikatenlogik besteht aus:
- $D$: Eine nichtleere Menge, die als Trägerbereich oder Universum dient. Dies ist die Menge der Objekte, auf die sich die Aussagen der Logik beziehen.
- $I$: Eine Funktion, die den Symbolen der Signatur $\Sigma$ eine Bedeutung in Bezug auf $D$ zuweist.
- Eine **Variablenbelegung** oder kurz **Belegung** $\beta$ ist eine Funktion, die Variablen (Symbole, die Werte repräsentieren) auf Objekte in $D$ abbildet.
	- **Notation**: $\beta : \text{Var} \to D$
	  - **Var** ist die Menge der Variablen.
	  - **D** ist der Trägerbereich der Interpretation.
Die Funktion $\beta$ weist jeder Variable $x$ einen bestimmten Wert in $D$ zu. Zum Beispiel könnte $\beta(x) = 5$ bedeuten, dass die Variable $x$ den Wert 5 im Kontext der Interpretation $(D, I)$ hat.
### Modifikation der Variablenbelegung $\beta$
Die Modifikation einer Variablenbelegung $\beta$ an der Stelle $x$ zu einem neuen Wert $d \in D$ wird wie folgt definiert:
#### Definition:
Für $\beta, x \in \text{Var}$ und $d \in D$ definieren wir die modifizierte Belegung $\beta^d_x$, die sich wie folgt verhält:
$$
\beta^d_x(y) = 
\begin{cases} 
d & \text{falls } y = x \\
\beta(y) & \text{falls } y \neq x 
\end{cases}
$$

- **Falls $y = x$**: Die modifizierte Belegung $\beta^d_x$ weist der Variable $x$ den Wert $d$ zu.
- **Falls $y \neq x$**: Die modifizierte Belegung $\beta^d_x$ behält die gleiche Zuordnung wie die ursprüngliche Belegung $\beta$.

### Beispiel
Nehmen wir an, $\beta$ sei eine Variablenbelegung in einem Trägerbereich $D = \{1, 2, 3\}$ mit:
- $\beta(x) = 1$
- $\beta(y) = 2$
Wenn wir die Belegung $\beta$ an der Stelle $x$ auf den Wert 3 ändern wollen, erhalten wir eine neue Belegung $\beta^3_x$, die folgendermaßen aussieht:
- $\beta^3_x(x) = 3$ (weil $x = x$)
- $\beta^3_x(y) = \beta(y) = 2$ (weil $y \neq x$)

Die Auswertung von Termen und Formeln in der Prädikatenlogik erfolgt mithilfe einer Auswertungsfunktion, die Werte aus dem Trägerbereich einer Interpretation $D$ und eine Variablenbelegung $\beta$ nutzt. Diese Funktion weist jedem Term und jeder Formel einen Wert oder Wahrheitswert zu. Lass uns die Definitionen und Prozesse im Detail durchgehen.

## Auswertung von Funktionen $\text{val}_{D,I,\beta}$
Die **Auswertungsfunktion** $\text{val}_{D,I,\beta}$ ist eine Funktion, die auf Basis einer Interpretation $(D, I)$ und einer Variablenbelegung $\beta$ definiert ist. Sie gibt an, wie Terme und Formeln ausgewertet werden, um entweder einen Wert in $D$ oder einen Wahrheitswert (wahr oder falsch) zu ergeben.
- **Für Terme** $t \in \text{Term}_{\Sigma}$:
 $\text{val}_{D,I,\beta}(t) \in D$
  Das Ergebnis ist ein Wert im Trägerbereich $D$.
- **Für Formeln** $A \in \text{For}_{\Sigma}$:
 $\text{val}_{D,I,\beta}(A) \in \{ W, F \}$
  Das Ergebnis ist ein Wahrheitswert: $W$ (wahr) oder $F$ (falsch).
### Auswertung von Termen
1. **Auswertung einer Variablen**:
  $\text{val}_{D,I,\beta}(x) = \beta(x) \, \text{für} \, x \in \text{Var}$
   Die Auswertung einer Variablen $x$ ergibt den Wert, der der Variablen $x$ in der Belegung $\beta$ zugewiesen ist.
2. **Auswertung eines Funktionssymbols**:
  $\text{val}_{D,I,\beta}(f(t_1, \ldots, t_n)) = I(f)(\text{val}_{D,I,\beta}(t_1), \ldots, \text{val}_{D,I,\beta}(t_n))$
   Die Auswertung eines Funktionssymbols $f$, das auf die Terme $t_1$ bis $t_n$ angewendet wird, ergibt den Wert, der durch die Interpretation $I$ der Funktion $f$ auf die ausgewerteten Werte der Terme $t_1$ bis $t_n$ berechnet wird.
### Beispiel zur Auswertung von Termen
Angenommen, wir haben folgende Elemente:
- **Trägerbereich** $D = \{1, 2, 3\}$
- **Interpretation** $I$:
  - $I(f)$ ist eine Funktion, die zwei Zahlen addiert, also $I(f)(x, y) = x + y$.
- **Belegung** $\beta$:
  - $\beta(x) = 2$
  - $\beta(y) = 3$
Nun wollen wir den Term $f(x, y)$ auswerten.
1. Auswertung der Variablen:
  $\text{val}_{D,I,\beta}(x) = \beta(x) = 2$
  $\text{val}_{D,I,\beta}(y) = \beta(y) = 3$
2. Auswertung des Funktionssymbols:
  $\text{val}_{D,I,\beta}(f(x, y)) = I(f)(\text{val}_{D,I,\beta}(x), \text{val}_{D,I,\beta}(y))$
  $= I(f)(2, 3) = 2 + 3 = 5$
Der Term $f(x, y)$ wird also zu 5 ausgewertet.
### Auswertung von Formeln
Die Auswertung von Formeln in der Prädikatenlogik folgt bestimmten Regeln, die die Zuordnung von Wahrheitswerten zu den Formeln auf Basis einer Interpretation $(D, I)$ und einer Variablenbelegung $\beta$ ermöglichen. 
#### 1. Atomare Formeln
Atomare Formeln sind die grundlegendsten logischen Ausdrücke und umfassen Konstanten, Gleichungen und Prädikate.
1. **Konstanten**:
   - $\text{val}_{D,I,\beta}(1) = W$ (wahr)
   - $\text{val}_{D,I,\beta}(0) = F$ (falsch)
2. **Gleichungen**:
   $$\text{val}_{D,I,\beta}(s \mathrel{.=} t) = 
   \begin{cases} 
   W & \text{falls } \text{val}_{D,I,\beta}(s) = \text{val}_{D,I,\beta}(t) \\
   F & \text{sonst}
   \end{cases}
  $$
   Hier wird überprüft, ob die Auswertung der Terme $s$ und $t$ gleich sind. Wenn ja, ist die Formel wahr, andernfalls falsch.
3. **0-stellige Prädikate** (Prädikate ohne Argumente, ähnlich wie Aussagenlogische Variablen):
   $\text{val}_{D,I,\beta}(P) = I(P)$
   Das bedeutet, der Wahrheitswert des 0-stelligen Prädikats $P$ wird direkt aus der Interpretation $I$ genommen.
4. **n-stellige Prädikate** (mit Argumenten):
   $$\text{val}_{D,I,\beta}(p(t_1, \ldots, t_n)) = 
   \begin{cases} 
   W & \text{falls } (\text{val}_{D,I,\beta}(t_1), \ldots, \text{val}_{D,I,\beta}(t_n)) \in I(p) \\
   F & \text{sonst}
   \end{cases}
  $$
   Hier wird überprüft, ob das Tupel der Auswertungen der Terme $t_1, \ldots, t_n$ in der Relation $I(p)$ enthalten ist. Wenn ja, ist die Formel wahr, andernfalls falsch.
#### 2. Logische Operatoren (wie in der Aussagenlogik)
Für die folgenden komplexeren Formeln wird die Auswertung wie in der Aussagenlogik durchgeführt:
- **Negation** ($\neg A$):
  $$\text{val}_{D,I,\beta}(\neg A) = 
  \begin{cases} 
  W & \text{falls } \text{val}_{D,I,\beta}(A) = F \\
  F & \text{sonst}
  \end{cases}
 $$
- **Und** ($A \land B$):
 $$\text{val}_{D,I,\beta}(A \land B) = 
  \begin{cases} 
  W & \text{falls } \text{val}_{D,I,\beta}(A) = W \text{ und } \text{val}_{D,I,\beta}(B) = W \\
  F & \text{sonst}
  \end{cases}
 $$
- **Oder** ($A \lor B$):
  $$\text{val}_{D,I,\beta}(A \lor B) = 
  \begin{cases} 
  W & \text{falls } \text{val}_{D,I,\beta}(A) = W \text{ oder } \text{val}_{D,I,\beta}(B) = W \\
  F & \text{sonst}
  \end{cases}
 $$

- **Implikation** ($A \rightarrow B$):
  $$\text{val}_{D,I,\beta}(A \rightarrow B) = 
  \begin{cases} 
  F & \text{falls } \text{val}_{D,I,\beta}(A) = W \text{ und } \text{val}_{D,I,\beta}(B) = F \\
  W & \text{sonst}
  \end{cases}
 $$
- **Äquivalenz** ($A \leftrightarrow B$):
  $$\text{val}_{D,I,\beta}(A \leftrightarrow B) = 
  \begin{cases} 
  W & \text{falls } \text{val}_{D,I,\beta}(A) = \text{val}_{D,I,\beta}(B) \\
  F & \text{sonst}
  \end{cases}
 $$
#### 3. Universeller Quantor ($\forall$)
Für eine Formel $\forall x A$ (für alle $x$ gilt $A$):
$$\text{val}_{D,I,\beta}(\forall x A) = 
\begin{cases} 
W & \text{falls für alle } d \in D: \text{val}_{D,I,\beta^d_x}(A) = W \\
F & \text{sonst}
\end{cases}
$$
Das bedeutet, die Formel ist wahr, wenn $A$ für alle möglichen Werte $d$ im Trägerbereich $D$ wahr ist, wobei $\beta^d_x$ die Belegung $\beta$ modifiziert, indem $x$ auf $d$ gesetzt wird.
#### 4. Existentieller Quantor ($\exists$)
Für eine Formel $\exists x A$ (es existiert ein $x$, für das $A$ gilt):
$$\text{val}_{D,I,\beta}(\exists x A) = 
\begin{cases} 
W & \text{falls es ein } d \in D \text{ gibt mit } \text{val}_{D,I,\beta^d_x}(A) = W \\
F & \text{sonst}
\end{cases}$$
Die Formel ist wahr, wenn es mindestens einen Wert $d$ im Trägerbereich $D$ gibt, für den $A$ wahr ist, wobei $\beta^d_x$ wieder die Belegung $\beta$ modifiziert, indem $x$ auf $d$ gesetzt wird.
### Beispiel zur Auswertung von Formeln
Nehmen wir eine einfache Formel $\forall x (x \geq 0)$ in einer Welt, wo $D$ die Menge der natürlichen Zahlen $\{0, 1, 2, \ldots\}$ ist.
- **Trägerbereich**: $D = \{0, 1, 2, \ldots\}$
- **Interpretation**: $I(\geq)$ ist die übliche „größer-gleich“-Relation.
- **Belegung**: $\beta$ ist hier irrelevant, da wir $\forall x$ haben, das für alle $x$ in $D$ gilt.
Die Auswertung erfolgt wie folgt:
1. **Überprüfung für jeden $d$ in $D$**:
   $\text{val}_{D,I,\beta^d_x}(x \geq 0) = W \, \text{für alle } d \in D$
Da $x \geq 0$ für alle $x$ in $D$ wahr ist, ist die Formel $\forall x (x \geq 0)$ wahr.
### Beispiel 2
Lass uns die Auswertung der Formel $q(x) \rightarrow \exists y \, (\text{in}(y, x) \land \text{kl}(y))$ Schritt für Schritt durchgehen, basierend auf der gegebenen Interpretation $D_{Bsp}$ und der Variablenbelegung $\beta$.
#### Gegebene Interpretation $D_{Bsp}$ und Zuordnung $I_{Bsp}$
- **Objekte**: $D_{Bsp} = \{ Q_1, Q_2, Q_3, Q_4, Q_5, Q_6, K_1, K_2, K_3, D_1, D_2, D_3 \}$
- **Zuordnung**:
  - $I_{Bsp}(q) = \{ Q_1, Q_2, Q_3, Q_4, Q_5, Q_6 \}$
  - $I_{Bsp}(k) = \{ K_1, K_2, K_3 \}$
  - $I_{Bsp}(d) = \{ D_1, D_2, D_3 \}$
  - $I_{Bsp}(in) = \{ (K_1, Q_1), (K_1, Q_3), (K_2, Q_1), (K_2, Q_2), (K_3, Q_2), (K_3, Q_4), (D_3, D_1), (Q_5, D_2) \}$
  - $I_{Bsp}(kl) = \{ K_1, K_2, K_3 \}$
#### Gegebene Variablenbelegung $\beta$
- $\beta(x) = Q_1$
#### Auswertung der Formel
Die Formel lautet:
$$ q(x) \rightarrow \exists y \, (\text{in}(y, x) \land \text{kl}(y)) $$
1. **Auswertung des Terms $x$**:
   $$ \text{val}_{D_{Bsp}, \beta}(x) = \beta(x) = Q_1 $$
2. **Auswertung von $q(x)$**:
   $$ \text{val}_{D_{Bsp}, \beta}(q(x)) = 
   \begin{cases} 
   W & \text{falls } Q_1 \in I_{Bsp}(q) \\
   F & \text{sonst}
   \end{cases}
   $$
   Da $Q_1 \in \{ Q_1, Q_2, Q_3, Q_4, Q_5, Q_6 \}$, ist:
   $$ \text{val}_{D_{Bsp}, \beta}(q(x)) = W $$
3. **Wahl einer Belegung für $y$**:
   - Wählen wir $K_1$ als Belegung für $y$, also $\beta_{K_1}^y(y) = K_1$.
4. **Auswertung von $\text{in}(y, x)$**:
   $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{in}(y, x)) = 
   \begin{cases} 
   W & \text{falls } (K_1, Q_1) \in I_{Bsp}(\text{in}) \\
   F & \text{sonst}
   \end{cases}
   $$
   Da $(K_1, Q_1) \in \{ (K_1, Q_1), (K_1, Q_3), (K_2, Q_1), (K_2, Q_2), (K_3, Q_2), (K_3, Q_4), (D_3, D_1), (Q_5, D_2) \}$, ist:
  $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{in}(y, x)) = W $$
5. **Auswertung von $\text{kl}(y)$**:
   $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{kl}(y)) = 
   \begin{cases} 
   W & \text{falls } K_1 \in I_{Bsp}(\text{kl}) \\
   F & \text{sonst}
   \end{cases}
   $$
   Da $K_1 \in \{ K_1, K_2, K_3 \}$, ist:
   $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{kl}(y)) = W $$
6. **Kombination der beiden Teile $\text{in}(y, x) \land \text{kl}(y)$**:
   $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{in}(y, x) \land \text{kl}(y)) = 
   \begin{cases} 
   W & \text{falls } \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{in}(y, x)) = W \text{ und } \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{kl}(y)) = W \\
   F & \text{sonst}
   \end{cases}
   $$
   Beide Bedingungen sind wahr:
   $$ \text{val}_{D_{Bsp}, \beta_{K_1}^y}(\text{in}(y, x) \land \text{kl}(y)) = W $$

7. **Auswertung des existentiellen Quantors $\exists y \, (\text{in}(y, x) \land \text{kl}(y))$**:
   $$ \text{val}_{D_{Bsp}, \beta}(\exists y \, (\text{in}(y, x) \land \text{kl}(y))) = 
   \begin{cases} 
   W & \text{falls es ein } d \in D \text{ gibt, so dass } \text{val}_{D_{Bsp}, \beta^d_y}(\text{in}(y, x) \land \text{kl}(y)) = W \\
   F & \text{sonst}
   \end{cases}
   $$
   Da $K_1$ diese Bedingung erfüllt:
   $$ \text{val}_{D_{Bsp}, \beta}(\exists y \, (\text{in}(y, x) \land \text{kl}(y))) = W $$

8. **Auswertung der gesamten Formel $q(x) \rightarrow \exists y \, (\text{in}(y, x) \land \text{kl}(y))$**:
   $$ \text{val}_{D_{Bsp}, \beta}(q(x) \rightarrow \exists y \, (\text{in}(y, x) \land \text{kl}(y))) = 
   \begin{cases} 
   W & \text{falls } \text{val}_{D_{Bsp}, \beta}(q(x)) = F \text{ oder } \text{val}_{D_{Bsp}, \beta}(\exists y \, (\text{in}(y, x) \land \text{kl}(y))) = W \\
   F & \text{sonst}
   \end{cases}
   $$
   Da $\text{val}_{D_{Bsp}, \beta}(q(x)) = W$ und $\text{val}_{D_{Bsp}, \beta}(\exists y \, (\text{in}(y, x) \land \text{kl}(y))) = W$, ist:
   $$ \text{val}_{D_{Bsp}, \beta}(q(x) \rightarrow \exists y \, (\text{in}(y, x) \land \text{kl}(y))) = W $$


## Koinzidenzlemma: Theorem
Das Theorem besagt im Wesentlichen, dass die Auswertung eines Terms oder einer Formel unabhängig von der spezifischen Variablenbelegung ist, solange diese Belegung für die relevanten Variablen (d.h. die Variablen, die tatsächlich in dem Term oder der Formel vorkommen) übereinstimmt.
### 1. Koinzidenz für Terme
**Aussage**:
- Für einen Term $t$ gilt: Wenn $\beta(x) = \gamma(x)$ für alle $x \in \text{Var}(t)$, dann ist $\text{val}_D, \beta(t) = \text{val}_D, \gamma(t)$.
**Erklärung**:
- **Terme** sind Ausdrücke, die aus Variablen, Konstanten und Funktionssymbolen bestehen.
- **$\text{Var}(t)$** bezeichnet die Menge der Variablen, die in dem Term $t$ vorkommen.
- Wenn zwei Variablenbelegungen $\beta$ und $\gamma$ für alle Variablen, die in $t$ vorkommen, dieselben Werte zuordnen, dann wird der Term $t$ unter beiden Belegungen denselben Wert haben.
**Beweisidee**:
- Der Beweis erfolgt durch strukturelle Induktion über die Struktur des Terms $t$.
- Für jede Art von Term (z.B. Variable, Anwendung eines Funktionssymbols auf andere Terme) zeigt man, dass die Auswertung nur von den Variablenbelegungen der in $t$ vorkommenden Variablen abhängt.
### 2. Koinzidenz für Formeln
**Aussage**:
- Für eine Formel $A$ gilt: Wenn $\beta(x) = \gamma(x)$ für alle $x \in \text{Frei}(A)$, dann ist $\text{val}_D, \beta(A) = \text{val}_D, \gamma(A)$.
**Erklärung**:
- **Formeln** sind logische Ausdrücke, die Prädikate, logische Operatoren und Quantoren enthalten.
- **$\text{Frei}(A)$** bezeichnet die Menge der freien Variablen in der Formel $A$.
- Wenn zwei Variablenbelegungen $\beta$ und $\gamma$ für alle freien Variablen in $A$ dieselben Werte zuordnen, dann hat die Formel $A$ unter beiden Belegungen denselben Wahrheitswert.
**Beweisidee**:
- Der Beweis erfolgt ebenfalls durch strukturelle Induktion, indem man die Formel $A$ in ihre Bestandteile zerlegt und zeigt, dass die Auswertung der Bestandteile nur von den freien Variablen abhängt.
### 3. Koinzidenz für geschlossene Formeln
**Aussage**:
- Ist $A$ eine geschlossene Formel, dann gilt $\text{val}_D, \beta(A) = \text{val}_D, \gamma(A)$.
**Erklärung**:
- Eine **geschlossene Formel** ist eine Formel ohne freie Variablen.
- Da es keine freien Variablen gibt, ist die Auswertung der Formel unabhängig von der Variablenbelegung $\beta$ oder $\gamma$.
**Beweisidee**:
- Da geschlossene Formeln keine freien Variablen enthalten, hat jede Änderung der Variablenbelegung keinen Einfluss auf die Auswertung der Formel. Daher ist die Auswertung unabhängig von der gewählten Belegung.
### Konsequenzen
1. **Notationsvereinfachung für geschlossene Formeln**:
   - Für geschlossene Formeln $A$ schreibt man $\text{val}_D(A)$ statt $\text{val}_D, \beta(A)$, weil die Auswertung unabhängig von der Variablenbelegung ist.
2. **Modellrelation**:
   - Die Notation $D \models A$ bedeutet, dass $\text{val}_D(A) = W$ (die Formel $A$ ist in der Interpretation $D$ wahr).
### Beispiel zur Veranschaulichung
Angenommen, wir haben die Interpretation $D = \{1, 2, 3\}$ und die Zuordnung $I$, die $P(x)$ als „$x$ ist gerade“ definiert, und die Belegungen:
- $\beta(x) = 2$
- $\gamma(x) = 2$
Für den Term $t = x + 1$ und die Formel $A = P(x)$, können wir zeigen, dass die Auswertung unter beiden Belegungen gleich ist, weil $\beta$ und $\gamma$ für $x$ dieselben Werte zuweisen.

## Arithmetische Strukturen in der Prädikatenlogik
Wir betrachten zwei arithmetische Strukturen unter einer gegebenen Signatur $\Sigma_{\text{arith}}$, die die üblichen arithmetischen Operationen und Relationen umfasst. Die Strukturen sind:
1. **Die mathematischen ganzen Zahlen $\mathbb{Z}$**
2. **Die ganzen Zahlen in Java $\mathbb{Z}_{\text{Jint}}$**
Die Signatur $\Sigma_{\text{arith}}$ enthält:
- Konstanten: $0, 1$
- Funktionen: $+, *$
- Relationen: $<$
### Struktur 1: Mathematische ganze Zahlen $\mathbb{Z}$
Die mathematischen ganzen Zahlen $\mathbb{Z}$ sind wie folgt definiert:
- **Trägerbereich**: $\mathbb{Z}$, die Menge aller ganzen Zahlen.
- **Addition**: $+_{\mathbb{Z}}$, die übliche Addition von ganzen Zahlen.
- **Multiplikation**: $*_{\mathbb{Z}}$, die übliche Multiplikation von ganzen Zahlen.
- **Vergleich**: $<_{\mathbb{Z}}$, die übliche Kleiner-als-Relation auf ganzen Zahlen.
Diese Struktur repräsentiert die üblichen arithmetischen Operationen und Vergleiche, die wir aus der Mathematik kennen.
### Struktur 2: Ganze Zahlen in Java $\mathbb{Z}_{\text{Jint}}$
Die ganzen Zahlen in Java (int) haben eine feste Größe und arbeiten innerhalb eines bestimmten Wertebereichs, der durch den Datentyp $int$ definiert ist. Die Struktur $\mathbb{Z}_{\text{Jint}}$ wird wie folgt definiert:
- **Trägerbereich**: $\mathbb{Z}_{\text{Jint}}$, die Menge der ganzen Zahlen im Bereich von $\text{int MIN}$ bis $\text{int MAX}$, also $[-\text{2}^{31}, \text{2}^{31} - 1]$.
- **Addition**: $+_{\text{J}}$, die Addition von ganzen Zahlen in Java, die nach bestimmten Regeln definiert ist, um Überlauf zu berücksichtigen.
- **Multiplikation**: $*_{\text{J}}$, die Multiplikation von ganzen Zahlen in Java, ebenfalls mit Berücksichtigung von Überlauf.
- **Vergleich**: $<_{\text{J}}$, die übliche Kleiner-als-Relation für ganze Zahlen in Java.
### Definition der Operationen in $\mathbb{Z}_{\text{Jint}}$
#### 1. Trägerbereich
Der Trägerbereich $\mathbb{Z}_{\text{Jint}}$ ist definiert als:
$$ \mathbb{Z}_{\text{Jint}} := [\text{int MIN}, \text{int MAX}] = [-2^{31}, 2^{31} - 1] $$
#### 2. Addition $+_{\text{J}}$
Die Addition $n +_{\text{J}} m$ ist definiert durch:
$$ n +_{\text{J}} m := \text{int MIN} +_{\mathbb{Z}} \left( \text{int HALFRANGE} +_{\mathbb{Z}} (n +_{\mathbb{Z}} m) \right) \mod \text{int RANGE} $$
- **$\text{int MIN}$**: Minimalwert des $int$-Bereichs, $-2^{31}$.
- **$\text{int HALFRANGE}$**: Hälfte des Wertebereichs, $\text{int HALFRANGE} = 2^{31}$.
- **$\text{int RANGE}$**: Größe des Wertebereichs, $\text{int RANGE} = 2^{32}$.
Diese Definition berücksichtigt den Überlauf, der in Java bei der Addition von ganzen Zahlen auftritt.
#### 3. Multiplikation $*_{\text{J}}$
Die Multiplikation $n *_{\text{J}} m$ ist definiert durch:
$$ n *_{\text{J}} m := \text{int MIN} +_{\mathbb{Z}} \left( \text{int HALFRANGE} +_{\mathbb{Z}} (n *_{\mathbb{Z}} m) \right) \mod \text{int RANGE} $$
Auch hier wird der Überlauf bei der Multiplikation von ganzen Zahlen in Java berücksichtigt.
#### 4. Vergleich $<_{\text{J}}$
Der Vergleich $n <_{\text{J}} m$ ist definiert durch:
$$ n <_{\text{J}} m \Leftrightarrow n <_{\mathbb{Z}} m $$
Der Vergleich ist derselbe wie in der Mathematik, d.h., er basiert auf der üblichen Kleiner-als-Relation.
### Beispiel zur Verdeutlichung
Nehmen wir an, wir haben die Zahlen $n = 2^{31} - 1$ und $m = 1$ in Java.
#### Addition:
- In Java: $2^{31} - 1 + 1 = -2^{31}$ (wegen Überlauf).
- Mit der definierten Funktion: 
  $ n +_{\text{J}} m := \text{int MIN} +_{\mathbb{Z}} \left( \text{int HALFRANGE} +_{\mathbb{Z}} ((2^{31} - 1) +_{\mathbb{Z}} 1) \right) \mod \text{int RANGE} $
  $ = -2^{31} +_{\mathbb{Z}} \left( 2^{31} +_{\mathbb{Z}} 2^{31} \right) \mod 2^{32} $
  $ = -2^{31} +_{\mathbb{Z}} 2^{32} \mod 2^{32} $
  $ = -2^{31} $

#### Multiplikation:

- In Java: $2^{31} * 2 = 2^{32} = 0$ (wegen Überlauf).
- Mit der definierten Funktion:
  $$ n *_{\text{J}} m := \text{int MIN} +_{\mathbb{Z}} \left( \text{int HALFRANGE} +_{\mathbb{Z}} ((2^{31} - 1) *_{\mathbb{Z}} 2) \right) \mod \text{int RANGE} $$
 $$ = -2^{31} +_{\mathbb{Z}} \left( 2^{31} +_{\mathbb{Z}} (2^{32} - 2) \right) \mod 2^{32} $$
  $$ = -2^{31} +_{\mathbb{Z}} 2^{31} = 0 $$


## Der Modellbegriff
Ein Modell ist in der Logik eine Interpretation, die den Wahrheitsgehalt von Aussagen bestimmt. Hier wird speziell der Modellbegriff für Formeln ohne freie Variablen definiert.
### Definition:
- **Interpretation $D$**: Eine Interpretation ist eine Zuordnung von Bedeutungen zu den Symbolen in einer Formel oder Formelmenge. $D$ steht für eine bestimmte Interpretation über einem Symbolvorrat $Σ$.
- **Formel $A$**: Eine Formel ist eine logische Aussage, die wahr oder falsch sein kann. Hier geht es um Formeln ohne freie Variablen, das heißt, alle Variablen in der Formel sind gebunden (durch Quantoren wie $\forall$ oder $\exists$).
- **Formelmenge $M$**: Eine Menge von Formeln, die alle die oben genannten Eigenschaften teilen.
### Modell für eine Formel
- Eine Interpretation $D$ über $Σ$ ist ein Modell für eine Formel $A$ ohne freie Variablen, wenn die Bewertung der Formel $A$ unter der Interpretation $D$ wahr ist, also $\text{val}_D(A) = \text{wahr}$ (kurz: $\text{val}_D(A) = W$).
### Modell für eine Formelmenge
- Eine Interpretation $D$ ist ein Modell für eine Formelmenge $M$ ohne freie Variablen, wenn für jede Formel $B$ in $M$ gilt, dass $\text{val}_D(B) = W$, also dass jede Formel in $M$ unter $D$ wahr ist.
### Beispiel zur Veranschaulichung
Stell dir vor, wir haben eine Formel $A$, die lautet: "Alle Katzen sind Tiere". 
- **Interpretation $D$**: Eine mögliche Interpretation könnte sein, dass wir für jede Katze in unserer Welt prüfen, ob sie tatsächlich ein Tier ist.
- **Formel ohne freie Variablen**: In unserem Beispiel sind alle Variablen gebunden, da wir sagen "alle Katzen", und es gibt keine Variable, die wir frei wählen können
Wenn in unserer Interpretation $D$ alle Katzen tatsächlich Tiere sind, dann ist $D$ ein Modell für die Formel $A$, weil $\text{val}_D(A) = W$ (wahr).

## Der logische Folgerungsbegriff
### Definition
- **M**: Eine Menge von Formeln ohne freie Variablen. Diese Formeln sind in einer logischen Sprache $\Sigma$ definiert.
- **A**: Eine einzelne Formel in der gleichen logischen Sprache $\Sigma$, ebenfalls ohne freie Variablen.
Die Aussage „$M \models_\Sigma A$” bedeutet:
- Jedes Modell, das die Formeln in $M$ erfüllt, erfüllt auch die Formel $A$.
- Anders gesagt: Wenn eine Interpretation $D$ ein Modell für $M$ ist, dann ist $D$ auch ein Modell für $A$.
### Kurznotationen
- **$|=$**: Anstatt $|=\Sigma$ wird häufig einfach nur $|=$ geschrieben, wenn klar ist, in welcher Sprache gearbeitet wird.
- **$|= A$**: Bedeuten, dass die Formel $A$ allgemeingültig ist, das heißt, sie ist in jeder möglichen Interpretation wahr.
- **$\{B\} \models A$**: Wenn eine Formel $B$ in einer Menge betrachtet wird, die nur aus $B$ besteht, und daraus $A$ folgt, dann kann man auch einfach $B \models A$ schreiben.
### Bemerkungen zum Modellbegriff
- **$M \models A$**: Dies ist äquivalent zu der Aussage, dass die Vereinigung von $M$ und der Negation von $A$ kein Modell hat.
  - **$\mathrm{M} \cup \{\neg A\}$**: Wenn es kein Modell für die Formelmenge $M$ gibt, das die Negation von $A$ erfüllt, bedeutet das, dass $A$ aus $M$ folgt.
### Definitionen
- **Allgemeingültig**: Eine Formel $A$ heißt allgemeingültig (oder tautologisch), wenn sie in jeder möglichen Interpretation wahr ist. Dies wird mit $|= A$ notiert.
- **Erfüllbar**: Eine Formel $A$ ist erfüllbar, wenn ihre Negation $\neg A$ nicht allgemeingültig ist. Das heißt, es gibt mindestens ein Modell, in dem $A$ wahr ist.
### Beispiel zur Veranschaulichung
Stell dir vor, $M$ sei eine Menge von Formeln, die besagt: „Alle Menschen sind sterblich“ und „Sokrates ist ein Mensch“. Wenn $A$ die Formel „Sokrates ist sterblich“ ist, dann können wir sagen:
- **$M \models A$**: Jedes Modell, das die Formeln in $M$ erfüllt (also, dass alle Menschen sterblich sind und Sokrates ein Mensch ist), erfüllt auch die Formel $A$ (Sokrates ist sterblich).

Gerne erkläre ich das Theorem und die logischen Äquivalenzen, die hier behandelt werden! Diese Begriffe und Regeln sind wichtig, um die formale Logik besser zu verstehen.

## Theorem zur Äquivalenz der Aussagen zur Allgemeingültigkeit
1. **A allgemeingültig**:
   - Eine Formel $A$ ist allgemeingültig, wenn sie in jeder möglichen Interpretation wahr ist. Das bedeutet, dass es keine Interpretation gibt, in der $A$ falsch wäre.
2. **Jede Interpretation $D$ ist Modell von $A$**:
   - Dies bedeutet, dass für jede mögliche Interpretation $D$, die den Symbolen in $A$ Bedeutung verleiht, $A$ wahr ist. Das ist eine andere Formulierung dafür, dass $A$ in jeder Interpretation wahr ist.
3. **$\text{val}_D(A) = W$ für alle $D$**:
   - Hier steht $\text{val}_D(A)$ für den Wahrheitswert von $A$ unter der Interpretation $D$. Wenn $A$ für jede mögliche Interpretation $D$ wahr ist (also $W$ bedeutet „wahr“), ist $A$ allgemeingültig.
Diese drei Aussagen sind äquivalent. Das bedeutet, sie sind verschiedene Weisen, dasselbe Konzept zu beschreiben: die Allgemeingültigkeit von $A$.

## Theorem zur Äquivalenz der Aussagen zur Erfüllbarkeit
1. **$A$ erfüllbar**:
   - Eine Formel $A$ ist erfüllbar, wenn es mindestens eine Interpretation $D$ gibt, unter der $A$ wahr ist.
2. **Es gibt eine Interpretation $D$ mit $\text{val}_D(A) = W$**:
   - Dies bedeutet, dass es mindestens eine Interpretation $D$ gibt, bei der $A$ wahr ist. Das ist genau die Definition von Erfüllbarkeit.
Auch diese beiden Aussagen sind äquivalent und beschreiben dasselbe: die Erfüllbarkeit von $A$.
### Wichtige allgemeingültige Äquivalenzen
#### 1. Quantoren-Negationen
- **$\neg \forall x A$ ist äquivalent zu $\exists x \neg A$**:
  - Die Aussage „Es ist nicht wahr, dass $A$ für alle $x$ gilt“ ist dasselbe wie „Es gibt ein $x$, für das $A$ nicht gilt“.
- **$\neg \exists x A$ ist äquivalent zu $\forall x \neg A$**:
  - Die Aussage „Es ist nicht wahr, dass es ein $x$ gibt, für das $A$ gilt“ ist dasselbe wie „Für alle $x$ gilt $A$ nicht“.
#### 2. Vertauschbarkeit der Quantoren
- **$\forall x \forall y A$ ist äquivalent zu $\forall y \forall x A$**:
  - Ob man zuerst über $x$ und dann über $y$ oder umgekehrt quantifiziert, spielt keine Rolle, wenn beide Quantoren universell sind.
- **$\exists x \exists y A$ ist äquivalent zu $\exists y \exists x A$**:
  - Ähnlich wie bei den universellen Quantoren, kann man auch bei existenziellen Quantoren die Reihenfolge vertauschen.
#### 3. Verteilung der Quantoren über logische Operatoren
- **$\forall x (A \land B)$ ist äquivalent zu $\forall x A \land \forall x B$**:
  - Wenn $A$ und $B$ für alle $x$ wahr sind, dann sind sowohl $A$ als auch $B$ einzeln für alle $x$ wahr.
- **$\exists x (A \lor B)$ ist äquivalent zu $\exists x A \lor \exists x B$**:
  - Wenn es ein $x$ gibt, für das $A$ oder $B$ wahr ist, dann gibt es entweder ein $x$, für das $A$ wahr ist, oder ein $x$, für das $B$ wahr ist.
#### 4. Verteilung der Quantoren in Bezug auf logische Operatoren, wenn $x$ nicht frei in $A$ ist
Wenn $x$ nicht frei in $A$ vorkommt (das heißt, $A$ ist von $x$ unabhängig):
- **$(A \land \forall x B)$ ist äquivalent zu $\forall x (A \land B)$**:
  - Wenn $A$ und $B$ für alle $x$ wahr sind, dann ist $A$ unabhängig von $x$ und man kann den Quantor verteilen.
- **$(A \land \exists x B)$ ist äquivalent zu $\exists x (A \land B)$**:
  - Wenn $A$ wahr ist und es ein $x$ gibt, für das $B$ wahr ist, dann gibt es ein $x$, für das $A$ und $B$ wahr sind.
- **$(A \lor \forall x B)$ ist äquivalent zu $\forall x (A \lor B)$**:
  - Wenn $A$ oder $B$ für alle $x$ wahr ist, dann ist $A$ unabhängig von $x$ und der Quantor kann verteilt werden.
- **$(A \lor \exists x B)$ ist äquivalent zu $\exists x (A \lor B)$**:
  - Wenn $A$ oder $B$ für ein $x$ wahr ist, dann gibt es ein $x$, für das $A$ oder $B$ wahr ist.

## Folgerbarkeitsprobleme
### 1. Beispiel: Transitivität, Symmetrie und Endlosigkeit führen zur Reflexivität
#### Gegebene Aussagen:
1. **∀x∀y∀z (r(x, y) ∧ r(y, z) → r(x, z))**
   - Das ist die Transitivität der Relation $r$. Wenn $x$ in Relation zu $y$ steht und $y$ in Relation zu $z$, dann steht auch $x$ in Relation zu $z$.
2. **∀x∀y (r(x, y) → r(y, x))**
   - Das ist die Symmetrie der Relation $r$. Wenn $x$ in Relation zu $y$ steht, dann steht auch $y$ in Relation zu $x$.
3. **∀x∃y (r(x, y))**
   - Das bedeutet, dass für jedes $x$ ein $y$ existiert, das in Relation zu $x$ steht. Dies beschreibt eine Art von „Endlosigkeit“, da es immer ein $y$ gibt, das in Relation zu $x$ steht.
#### Zu zeigende Aussage:
- **∀x r(x, x)**
   - Das ist die Reflexivität der Relation $r$. Jeder Wert $x$ steht in Relation zu sich selbst.
#### Erklärung:
Um zu zeigen, dass diese drei Eigenschaften zur Reflexivität führen, betrachten wir die gegebenen Bedingungen:
1. **Transitivität** bedeutet, dass, wenn eine Kette von Relationen existiert, diese zu einer direkten Relation führen kann.
2. **Symmetrie** bedeutet, dass, wenn eine Relation in eine Richtung besteht, sie auch in die entgegengesetzte Richtung besteht.
3. **Endlosigkeit** bedeutet, dass es immer eine weitere Beziehung gibt.
Wir können folgendermaßen argumentieren:
- Nehmen wir an, $r(x, y)$ ist wahr für ein $x$ und ein $y$.
- Da $r$ symmetrisch ist, ist auch $r(y, x)$ wahr.
- Da $r$ transitiv ist, ergibt $r(x, y)$ und $r(y, x)$, dass $r(x, x)$ wahr ist.
Da wir für jedes $x$ ein solches $y$ finden können (wegen der Endlosigkeit), zeigt dies, dass $r(x, x)$ für jedes $x$ gilt, also ist $r$ reflexiv.
### 2. Beispiel: Nicht-Existenz einer bestimmten Bedingung führt zur Existenz der Negation
#### Gegebene Aussage:
- **¬∃x (a < x ∧ c(x) ∧ ∀y (a ≤ y < x → b(y)))**
   - Es gibt kein $x$, das größer als $a$ ist und die Bedingung $c(x)$ sowie $∀y(a ≤ y < x → b(y))$ erfüllt.
#### Zu zeigende Aussage:
- **∃x (a < x ∧ ¬c(x) ∧ ∀y (a ≤ y < x → ¬b(y)))**
   - Es gibt ein $x$, das größer als $a$ ist, die Bedingung $¬c(x)$ erfüllt und $∀y(a ≤ y < x → ¬b(y))$ wahr ist.
#### Erklärung:
Um diese logische Folgerung zu überprüfen, verwenden wir eine Art indirekten Beweis:
1. **Gegebene Aussage** besagt, dass es kein $x$ gibt, das alle drei Bedingungen (größer als $a$, $c(x)$ wahr und $∀y (a ≤ y < x → b(y))$ wahr) gleichzeitig erfüllt.
2. **Zu zeigende Aussage** besagt, dass es ein $x$ gibt, das größer als $a$ ist, $¬c(x)$ erfüllt und für das gilt, dass $∀y (a ≤ y < x → ¬b(y))$ wahr ist.
Wenn kein $x$ die Bedingung $c(x)$ und $∀y (a ≤ y < x → b(y))$ erfüllt, dann muss es ein $x$ geben, das diese Bedingungen in der negierten Form erfüllt. 
Wir führen dies Schritt für Schritt durch:
- Die Nicht-Existenz eines $x$ mit $c(x)$ und $b(y)$ impliziert, dass $c(x)$ und $b(y)$ nicht gleichzeitig wahr sein können.
- Dies führt zu der Existenz eines $x$, bei dem $¬c(x)$ gilt, und für alle $y$, die kleiner als $x$ sind, gilt $¬b(y)$.
Das bedeutet, dass die Aussage **JA** korrekt ist: Die Nicht-Existenz einer solchen Bedingung führt zur Existenz der Negation.




## Substitutionslemma für Terme
- **$\sigma$**: Eine Substitution, die Variablen durch Terme ersetzt.
- **$t$**: Ein Term aus der Termmenge $\text{Term}_\Sigma$. Ein Term ist eine Kombination von Variablen und Funktionen, die gemäß der Signatur $\Sigma$ gebildet wird.
### Aussage des Theorems
- Das Theorem besagt, dass die Bewertung eines Terms $t$ nach einer Substitution $\sigma$ und unter einer Interpretation $D$ und einer Variablenbelegung $\beta$ gleich der Bewertung des Terms $t$ unter der zusammengesetzten Belegung $\beta \sigma$ ist. Formal ausgedrückt:$$\text{val}_{D,\beta}(\sigma(t)) = \text{val}_{D,\beta \sigma}(t)$$wobei $\beta \sigma$ definiert ist als:$$\beta \sigma (x) := \text{val}_{D,\beta}(\sigma(x)) \text{ für alle } x \in \text{Var}$$
- Das bedeutet, dass die Bewertung eines Terms nach der Substitution und Belegung der Bewertung des Terms unter der zusammengesetzten Belegung entspricht.
### Beweis durch strukturelle Induktion
#### Induktionsanfang
1. **Basisfall (Variable)**:
   - $t$ ist eine Variable $t = x$.
   - Die Substitution $\sigma$ ersetzt $x$ durch einen Term $\sigma(x)$.
   - Die Bewertung $\text{val}_{D,\beta}(\sigma(x))$ ist gleich der Belegung $\beta \sigma(x)$, also:
     $\text{val}_{D,\beta}(\sigma(x)) = \beta \sigma(x) \text{ (Definition von } \beta \sigma)$
     $= \text{val}_{D,\beta \sigma}(x) \text{ (Definition der Bewertung von } x)$
#### Induktionsschritt
1. **Induktionsschritt (Term aus Funktion)**:
   - Sei $t$ ein Term der Form $t = f(t_1, \ldots, t_n)$.
   - Wir müssen zeigen, dass:$$\text{val}_{D,\beta}(\sigma(f(t_1, \ldots, t_n))) = \text{val}_{D,\beta \sigma}(f(t_1, \ldots, t_n))$$
2. **Transformation der Funktion**:
   - Die Substitution $\sigma$ wirkt auf jeden Teilterm $t_i$:$$\sigma(f(t_1, \ldots, t_n)) = f(\sigma(t_1), \ldots, \sigma(t_n))$$
   - Die Bewertung $\text{val}_{D,\beta}$ von $\sigma(f(t_1, \ldots, t_n))$ ist:$$\text{val}_{D,\beta}(\sigma(f(t_1, \ldots, t_n))) = \text{val}_{D,\beta}(f(\sigma(t_1), \ldots, \sigma(t_n)))$$
3. **Bewertung der Funktion**:
   - Die Interpretation $I(f)$ bewertet die Funktion $f$:$$\text{val}_{D,\beta}(f(\sigma(t_1), \ldots, \sigma(t_n))) = I(f)(\text{val}_{D,\beta}(\sigma(t_1)), \ldots, \text{val}_{D,\beta}(\sigma(t_n)))$$
5. **Induktionsannahme**:
   - Unter Verwendung der Induktionsannahme gilt:$$\text{val}_{D,\beta}(\sigma(t_i)) = \text{val}_{D,\beta \sigma}(t_i) \text{ für alle } i$$
   - Daher:$$I(f)(\text{val}_{D,\beta \sigma}(t_1), \ldots, \text{val}_{D,\beta \sigma}(t_n)) = \text{val}_{D,\beta \sigma}(f(t_1, \ldots, t_n))$$
6. **Schlussfolgerung**:
   - Somit haben wir gezeigt:$$\text{val}_{D,\beta}(\sigma(f(t_1, \ldots, t_n))) = \text{val}_{D,\beta \sigma}(f(t_1, \ldots, t_n))$$
Gerne erkläre ich das Substitutionslemma für Formeln und dessen Anwendung! Dieses Lemma ist in der formalen Logik von großer Bedeutung, um sicherzustellen, dass die semantische Bedeutung einer Formel durch Substitutionen nicht verändert wird, sofern diese kollisionsfrei sind.

## Substitutionslemma für Formeln
### Aussage des Theorems
- Das Substitutionslemma besagt, dass die Bewertung einer Formel $A$ nach einer Substitution $\sigma$ unter einer Interpretation $D$ und einer Variablenbelegung $\beta$ gleich der Bewertung von $A$ unter der zusammengesetzten Belegung $\beta \sigma$ ist.
- Formal ausgedrückt:$$\text{val}_{D,\beta}(\sigma(A)) = \text{val}_{D,\beta \sigma}(A)$$wobei $\beta \sigma$ definiert ist als:$$\beta \sigma (x) := \text{val}_{D,\beta}(\sigma(x)) \text{ für alle } x \in \text{Var}$$
### Anwendung des Substitutionslemmas
#### Gegebene Voraussetzungen
- **Kollisionsfreie Substitution**: Die Substitution $\sigma$ ist kollisionsfrei, das bedeutet, sie ersetzt Variablen nur in einer Weise, die keine Konflikte oder ungewollte Bindungen verursacht.
- **$\sigma(y) = y$ für alle Variablen $y \neq x$**: Die Substitution $\sigma$ ändert nur die Variable $x$ und lässt alle anderen Variablen $y$ unverändert.
#### Aussage des Theorems
Es wird gezeigt, dass unter den gegebenen Voraussetzungen folgende Aussagen wahr sind:
1. **$∀xA → σ(A)$**:
   - Die Aussage besagt, dass die Implikation $\forall x A \rightarrow \sigma(A)$ unter jeder Interpretation und Belegung wahr ist.
   - Das bedeutet, wenn $A$ für alle $x$ gilt, dann gilt $\sigma(A)$ auch.
2. **$σ(A) → ∃xA$**:
   - Die Aussage besagt, dass die Implikation $\sigma(A) \rightarrow \exists x A$ unter jeder Interpretation und Belegung wahr ist.
   - Das bedeutet, wenn $\sigma(A)$ gilt, dann gibt es mindestens ein $x$, für das $A$ gilt.
### Erklärung und Beweisidee
#### Substitutionslemma für Formeln
- **Ziel des Substitutionslemmas**: Es stellt sicher, dass das Ersetzen von Variablen durch Terme in einer Formel $A$ unter Beibehaltung der semantischen Bedeutung erfolgt, wenn die Substitution $\sigma$ kollisionsfrei ist.
- **Strukturelle Induktion**: Der Beweis des Substitutionslemmas erfolgt üblicherweise durch Induktion über die Struktur der Formel $A$. Dabei werden Basisfälle (z.B. atomare Formeln) und Induktionsschritte (z.B. für logische Operatoren und Quantoren) betrachtet.
#### Anwendung des Substitutionslemmas
1. **$∀xA → σ(A)$**:
    - **Interpretation**: Wenn $A$ für alle $x$ wahr ist, dann ist auch $\sigma(A)$ wahr, da $\sigma$ eine gültige Substitution ist und keine zusätzlichen Einschränkungen einführt.
    - **Beweisidee**: Nehmen wir an, $\forall x A$ ist wahr. Dann ist $A$ unter jeder möglichen Belegung für $x$ wahr. Da $\sigma(A)$ keine neuen Variablen einführt, die $x$ ersetzen, bleibt die Wahrheitsbedingung von $A$ erhalten.
2. **$σ(A) → ∃xA$**:
    - **Interpretation**: Wenn $\sigma(A)$ wahr ist, dann muss es ein $x$ geben, für das $A$ wahr ist. Das liegt daran, dass $\sigma$ eine spezifische Substitution darstellt, die auf ein bestimmtes $x$ angewendet wird, das $A$ erfüllt.
    - **Beweisidee**: Wenn $\sigma(A)$ wahr ist, dann bedeutet dies, dass es eine Belegung für die Variablen gibt, die $A$ wahr macht. Insbesondere gibt es ein $x$, das diese Bedingung erfüllt, sodass $\exists x A$ ebenfalls wahr ist.

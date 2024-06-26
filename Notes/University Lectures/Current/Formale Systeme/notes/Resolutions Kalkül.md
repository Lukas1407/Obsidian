ass uns die Merkmale des Resolutionskalküls genauer ansehen und sie Schritt für Schritt erklären.

## Merkmale
### Widerlegungskalkül
- **Was bedeutet das?**
  - Der Resolutionskalkül ist ein **Widerlegungskalkül**. Das bedeutet, dass er verwendet wird, um zu zeigen, dass eine bestimmte Aussage unerfüllbar ist. Anstatt direkt zu beweisen, dass eine Aussage wahr ist, zeigt der Widerlegungskalkül, dass die Negation dieser Aussage nicht wahr sein kann. Wenn die Negation nicht erfüllbar ist, muss die ursprüngliche Aussage wahr sein.
### Voraussetzung: Konjunktive Normalform
- **Was ist das?**
  - Alle Formeln, die im Resolutionskalkül verwendet werden, müssen in **konjunktiver Normalform** (KNF) vorliegen. Das bedeutet, dass die Formel als eine Konjunktion (und-Verknüpfung) von Disjunktionen (oder-Verknüpfungen) von Literalen dargestellt ist.
  - Ein Literal ist entweder eine Variable oder die Negation einer Variable.
### Die Resolutionsregel
- **Wie funktioniert sie?**
  - Der Resolutionskalkül hat nur eine einzige Regel, die **Resolutionsregel**. Diese Regel besagt, wie aus zwei Klauseln eine neue Klausel abgeleitet werden kann.
  - Wenn du zwei Klauseln hast, z. B. $(A \lor B)$ und $(\neg A \lor C)$, dann kannst du daraus eine neue Klausel ableiten, indem du das widersprüchliche Paar $A$ und $(\neg A)$ entfernst. Die resultierende Klausel wäre $(B \lor C)$
- Die **Resolutionsregel** in der Aussagenlogik ist eine Methode, um aus zwei Klauseln eine neue Klausel abzuleiten. Die Regel lautet:
$$\frac{C_1 \cup \{P\}, C_2 \cup \{\neg P\}}{C_1 \cup C_2}$$
- **$P$** ist eine Aussagenlogische Variable (AL-Variable).
- **$C_1$** und **$C_2$** sind Klauseln (Mengen von Literalen).
- $C_1 \cup C_2$ ist die **Resolvente** von $C_1 \cup \{P\}$ und $C_2 \cup \{\neg P\}$.
Diese Regel bedeutet, dass wenn du eine Klausel hast, die die Variable $P$ enthält, und eine andere Klausel, die $\neg P$ enthält, du eine neue Klausel erstellen kannst, die alle Literale aus den beiden Klauseln enthält, außer $P$ und $\neg P$.
#### Beispiel
Die gegebene Klauselmenge $M$ ist:
$$M = \{\{P_1, P_2\}, \{P_1, \neg P_2\}, \{\neg P_1, P_2\}, \{\neg P_1, \neg P_2\}\}$$
##### Anwendung der Resolutionsregel
Schritt für Schritt:
1. **Klauseln:**
   - $\{P_1, P_2\}$ und $\{P_1, \neg P_2\}$
   - Gemeinsame Variable: $P_2$
   - Resolvente: $\{P_1\}$ (weil $P_2$ und $\neg P_2$ sich aufheben)
2. **Klauseln:**
   - $\{\neg P_1, P_2\}$ und $\{\neg P_1, \neg P_2\}$
   - Gemeinsame Variable: $P_2$
   - Resolvente: $\{\neg P_1\}$
3. **Klauseln:**
   - $\{P_1\}$ und $\{\neg P_1\}$
   - Gemeinsame Variable: $P_1$
   - Resolvente: $\{\}$ (die leere Klausel, weil $P_1$ und $\neg P_1$ sich aufheben)
Die leere Klausel $\{\}$ zeigt, dass die Klauselmenge unerfüllbar ist, da sie einen Widerspruch enthält.
##### Widerlegung
Die leere Klausel zeigt, dass $M$ einen Widerspruch enthält, d.h. $M$ ist unerfüllbar. Daher:
$$M \vdash_{\text{Res}} \{\}$$
#### Beweis einer Tautologie
Die zu prüfende Formel:
$$(A \rightarrow B) \rightarrow ((B \rightarrow C) \rightarrow (A \rightarrow C))$$
soll als Tautologie gezeigt werden.
##### Negation der Formel
Zuerst negieren wir die Formel, um zu zeigen, dass diese Negation unerfüllbar ist:
$$\neg((A \rightarrow B) \rightarrow ((B \rightarrow C) \rightarrow (A \rightarrow C)))$$
##### Umwandlung in Klauselnormalform
Die Negation der Formel wird in konjunktive Normalform (KNF) umgewandelt:
- $A \rightarrow B$ wird zu $\neg A \lor B$
- $B \rightarrow C$ wird zu $\neg B \lor C$
- $A \rightarrow C$ wird zu $\neg A \lor C$
Die Negation $\neg((A \rightarrow B) \rightarrow ((B \rightarrow C) \rightarrow (A \rightarrow C)))$ wird dann zu:
$$\{ \{\neg A, B\}, \{\neg B, C\}, \{A\}, \{\neg C\} \}$$
##### Ableitung der leeren Klausel
Schritte zur Ableitung der leeren Klausel:
1. **Klauseln:**
   - $\{\neg A, B\}$
   - $\{\neg B, C\}$
   - $\{A\}$
   - $\{\neg C\}$
2. **Resolution:**
   - $\{\neg A, B\}$ und $\{A\}$ geben $\{B\}$
   - $\{\neg B, C\}$ und $\{B\}$ geben $\{C\}$
   - $\{C\}$ und $\{\neg C\}$ geben $\{\}$
Schritte im Detail:
1. $\{\neg A, B\}$ und $\{A\}$ ergeben:
   - Gemeinsame Variable: $A$
   - Resolvente: $\{B\}$
2. $\{\neg B, C\}$ und $\{B\}$ ergeben:
   - Gemeinsame Variable: $B$
   - Resolvente: $\{C\}$
3. $\{C\}$ und $\{\neg C\}$ ergeben:
   - Gemeinsame Variable: $C$
   - Resolvente: $\{\}$
Die leere Klausel zeigt, dass die Klauselmenge unerfüllbar ist.
##### Fazit
Da die Negation der ursprünglichen Formel unerfüllbar ist, ist die ursprüngliche Formel eine Tautologie. Das bedeutet, die Aussage ist in allen möglichen Fällen wahr.
Gibt es dazu noch Fragen oder möchtest du einen bestimmten Teil noch genauer erläutert haben?

#### Theorem
- Für eine Menge $M$ von Klauseln gilt:
$$M \text{ ist unerfüllbar genau dann, wenn } M \vdash_{R_0} \square$$

- **$\vdash_{R_0}$**: Dies bedeutet, dass die Menge $M$ nach den Regeln des Resolutionskalküls $R_0$ die leere Klausel $\square$ ableiten kann.

- **$\square$**: Die leere Klausel repräsentiert einen logischen Widerspruch. Wenn sie abgeleitet wird, zeigt das, dass die Menge $M$ einen Widerspruch enthält und daher unerfüllbar ist.
- Das Theorem besagt, dass es eine Äquivalenz zwischen der Unerfüllbarkeit einer Menge von Klauseln $M$ und der Ableitung der leeren Klausel $\square$ aus $M$ mittels des Resolutionskalküls gibt. Das heißt:
	- Wenn $M$ unerfüllbar ist, dann kann man aus $M$ die leere Klausel $\square$ ableiten.
	- Umgekehrt: Wenn man aus $M$ die leere Klausel $\square$ ableiten kann, dann ist $M$ unerfüllbar.
##### Beispiel zur Veranschaulichung
Nehmen wir an, wir haben die Klauselmenge:
$$M = \{\{A\}, \{\neg A\}\}$$
- Diese Klauselmenge enthält die Klauseln $\{A\}$ und $\{\neg A\}$.
#### Unerfüllbarkeit prüfen
- Es gibt keine Zuweisung, die beide Klauseln gleichzeitig erfüllen kann, weil $A$ und $\neg A$ sich gegenseitig ausschließen. Daher ist $M$ unerfüllbar.
#### Ableitung der leeren Klausel
1. Wende die Resolutionsregel an:
   - Klauseln: $\{A\}$ und $\{\neg A\}$
   - Gemeinsame Variable: $A$
   - Resolvente: $\square$ (leere Klausel)
- Da wir die leere Klausel ableiten können, zeigt das, dass die Klauselmenge $M$ einen Widerspruch enthält und somit unerfüllbar ist.
### Verallgemeinerung der DPLL-Simplifikation
- **Was bedeutet das?**
  - Der Resolutionskalkül verallgemeinert den **DPLL-Algorithmus** (Davis-Putnam-Logemann-Loveland). Der DPLL-Algorithmus ist ein Entscheidungsalgorithmus, der verwendet wird, um die Erfüllbarkeit von Aussagenlogik-Formeln zu bestimmen.
  - Während der DPLL-Algorithmus eine Mischung aus systematischen und heuristischen Methoden verwendet, basiert der Resolutionskalkül auf einer klaren Regel und kann auf eine größere Klasse von Problemen angewendet werden.
### Keine logischen Axiome notwendig
- **Was bedeutet das?**
  - Im Resolutionskalkül sind keine **logischen Axiome** erforderlich. Das bedeutet, dass du keine vordefinierten Wahrheiten oder Grundannahmen brauchst, um den Kalkül zu verwenden.
  - Stattdessen suchst du direkt nach passenden **Instantiierungen** (Konkretisierungen) für die Variablen in den Formeln. Die Notwendigkeit, allgemeine Axiomenschemata zu durchsuchen und zu spezifizieren, entfällt.
### Indeterminismus bei der Auswahl der Formeln
- **Was bedeutet das?**
  - Der einzige **Indeterminismus** im Resolutionskalkül liegt in der Auswahl der Formeln, auf die die Regel angewendet wird. Du hast also Freiheit in der Entscheidung, welche Formeln du kombinierst, um neue abzuleiten.
  - Diese Freiheit kann dazu führen, dass unterschiedliche Wege zur Lösung führen oder auch, dass man sich auf eine ineffiziente Weise in eine Sackgasse bewegt.
## 1-Resolution
Die 1-Resolutionsregel ist eine spezialisierte Form der allgemeinen Resolutionsregel. Sie bezieht sich auf Klauseln, die nur ein einziges Literal enthalten. Diese Regel wird auch als „Unit Propagation“ bezeichnet, besonders im Kontext des DPLL-Algorithmus (Davis-Putnam-Logemann-Loveland).

Es gibt zwei mögliche Fälle der 1-Resolutionsregel:
1. **Erster Fall**:
   - Wenn eine Klausel nur die Variable $P$ enthält und eine andere Klausel $C_2$ die Negation dieser Variable $\neg P$ enthält, dann kannst du $P$ aus der ersten Klausel und $\neg P$ aus der zweiten Klausel eliminieren, um die neue Klausel $C_2$ zu erzeugen.
   - Formal: $$\{P\}, \quad C_2 \cup \{\neg P\} \quad \Rightarrow \quad C_2$$
2. **Zweiter Fall**:
   - Umgekehrt, wenn eine Klausel nur die Negation der Variable $\neg P$ enthält und eine andere Klausel $C_2$ die Variable $P$, dann kannst du $\neg P$ aus der ersten Klausel und $P$ aus der zweiten Klausel eliminieren, um die neue Klausel $C_2$ zu erzeugen.
   - Formal: $$\{\neg P\}, \quad C_2 \cup \{P\} \quad \Rightarrow \quad C_2$$
### Beispiel zur 1-Resolution
Nehmen wir an, wir haben folgende Klauseln:
- $\{P\}$
- $\{\neg P, Q\}$
Anwendung der 1-Resolutionsregel:
$$\{P\}, \quad \{\neg P, Q\} \quad \Rightarrow \quad \{Q\}$$
Hierbei wird die Klausel $\{Q\}$ abgeleitet, indem das widersprüchliche Literal $P$ entfernt wird.
### Vergleich zur allgemeinen Resolutionsregel
- **Allgemeine Resolutionsregel**: Diese erlaubt die Ableitung neuer Klauseln, indem eine Variable und ihre Negation aus beliebigen Klauseln entfernt werden, nicht nur aus Klauseln mit einem einzigen Literal.
- **1-Resolution**: Begrenzt sich auf Klauseln mit einem einzigen Literal (sogenannte „Unit“-Klauseln).
### Warum ist die 1-Resolution nicht vollständig?
#### Vollständigkeit
- Ein Kalkül ist vollständig, wenn er in der Lage ist, jede ableitbare Wahrheit oder jeden Widerspruch innerhalb eines Systems zu finden.
- Die allgemeine Resolutionsregel ist vollständig, weil sie es ermöglicht, jeden möglichen logischen Widerspruch zu entdecken und die leere Klausel abzuleiten.
#### 1-Resolution und Unvollständigkeit
Die 1-Resolution ist nicht vollständig, weil sie nur auf „Unit“-Klauseln anwendbar ist und nicht alle möglichen Widersprüche entdecken kann. Sie kann daher nicht immer die leere Klausel ableiten, selbst wenn ein Widerspruch vorliegt.
### Beispiel für Unvollständigkeit
Betrachten wir die Klauselmenge:
$$E = \{\{P_1, P_2\}, \{P_1, \neg P_2\}, \{\neg P_1, P_2\}, \{\neg P_1, \neg P_2\}\}$$
- Diese Klauselmenge ist unerfüllbar, weil keine Belegung existiert, die alle Klauseln wahr macht.
#### Anwendung der 1-Resolution
- Es gibt keine Klauseln mit nur einem Literal (Unit-Klauseln) in $E$, auf die die 1-Resolution angewendet werden kann.
- Daher kann aus $E$ mit 1-Resolution keine neue Klausel abgeleitet werden, einschließlich der leeren Klausel.
#### Was bedeutet das?
- Die 1-Resolution kann die Unerfüllbarkeit dieser Klauselmenge nicht erkennen, weil sie auf komplexere Klauseln nicht anwendbar ist.
- Deshalb ist sie nicht vollständig: Sie kann nicht immer den Widerspruch, also die leere Klausel, ableiten.

## Einschränkung der Resolutionsregel: Geordnete Resolution
- Die geordnete Resolution ist eine Variante der Resolutionsregel, die eine bestimmte Reihenfolge für die Anwendung der Regel vorschreibt. Diese Reihenfolge basiert auf einer festen Reihenfolge der Aussagenlogischen Atome (Variablen).

- Angenommen, wir haben eine Aufzählung der Atome $P_1, P_2, \ldots, P_n$. Diese Reihenfolge wird für die geordnete Resolution verwendet.
- Die geordnete Resolutionsregel lautet:

$$\frac{C_1 \cup \{P_j\}, \quad C_2 \cup \{\neg P_j\}}{C_1 \cup C_2}$$
**Bedingung:**
Die Regel darf nur angewendet werden, wenn für alle Atome $P_i$ in den Klauseln $C_1$ oder $C_2$ gilt, dass $i < j$. Das bedeutet, dass das Atom $P_j$, welches für die Auflösung verwendet wird, das höchste (letzte) Atom in der geordneten Reihenfolge der beiden Klauseln sein muss.
### Beispiel
Stellen wir uns vor, wir haben die Atome in der Reihenfolge $P_1, P_2, P_3$:
- Klausel $C_1$ könnte $\{P_2, P_3\}$ sein.
- Klausel $C_2$ könnte $\{\neg P_3, P_1\}$ sein.
Hier könnten wir die geordnete Resolutionsregel nicht direkt anwenden, weil in $C_2$ das Atom $P_1$ vorkommt, welches vor $P_3$ in der Reihenfolge steht.
Wenn jedoch $C_1 = \{P_3\}$ und $C_2 = \{\neg P_3, P_2\}$ wären, dann könnten wir $P_3$ eliminieren, weil alle anderen Atome in der Reihenfolge vor $P_3$ kommen.
### Vollständigkeit des geordneten Resolutionskalküls
- Die geordnete Resolution ist tatsächlich vollständig. Das bedeutet, sie kann jeden Widerspruch (die leere Klausel $\square$) in einer Menge von Klauseln ableiten, wenn diese Menge unerfüllbar ist. Der Grund dafür ist, dass trotz der Einschränkungen in der Reihenfolge der Atome die Regel in Kombination mit den restlichen Klauseln immer noch alle notwendigen Ableitungen machen kann, um einen Widerspruch zu finden.
**Warum ist die geordnete Resolution vollständig?**
- Auch wenn die Anwendung der Regel eingeschränkt ist, stellt die geordnete Resolution sicher, dass alle möglichen Atome der Reihe nach geprüft werden, sodass alle relevanten Kombinationen von Klauseln zur Ableitung der leeren Klausel betrachtet werden können.
- Die Einschränkung in der Reihenfolge sorgt dafür, dass eine systematische und geordnete Durchsuchung des Lösungsraumes stattfindet, was letztlich dazu führt, dass alle möglichen Widersprüche gefunden werden.
#### Frage: Ist der geordnete Resolutionskalkül vollständig?
**Antwort: Ja, der geordnete Resolutionskalkül ist vollständig.** Er ist in der Lage, jede logische Konsequenz abzuleiten und jeden Widerspruch in einer Menge von Klauseln zu finden, die unerfüllbar ist.

Schauen wir uns den Resolutionskalkül genauer an, insbesondere die Syntax und die Notation. Ich werde die Begriffe und Konzepte Schritt für Schritt erklären.

## Syntax des Resolutionskalküls
### Literale
- **Definition**: Ein **Literal** ist entweder eine atomare Formel oder die Negation einer atomaren Formel.
  - Atomare Formel: Eine einfache Aussage ohne Negation, z.B. $P$.
  - Negierte atomare Formel: Eine Aussage, die negiert ist, z.B. $\neg P$.
### Klauseln
- **Definition**: Eine **Klausel** ist eine endliche Menge von Literalen.
  - Beispiele: $\{P, \neg Q, R\}$, $\{\neg P\}$.
- **Leere Klausel**: Die leere Klausel wird durch das Symbol $\square$ bezeichnet. Sie stellt einen logischen Widerspruch dar, da sie keine Literale enthält und daher nie erfüllbar ist.
- **Interpretation**: Eine Klausel wird als die Disjunktion (oder-Verknüpfung) ihrer Literale interpretiert.
  - Beispiel: Die Klausel $\{P, \neg Q, R\}$ wird interpretiert als $P \lor \neg Q \lor R$.
### Menge von Klauseln
- **Interpretation**: Eine Menge von Klauseln wird als die Konjunktion (und-Verknüpfung) ihrer Klauseln interpretiert.
  - Beispiel: Die Klauselmenge $\{\{P, \neg Q\}, \{\neg P, R\}\}$ wird interpretiert als $(P \lor \neg Q) \land (\neg P \lor R)$.
### Quantoren
- **Keine expliziten Quantoren**: Im Resolutionskalkül gibt es keine expliziten Quantoren wie „für alle“ ($\forall$) oder „es existiert“ ($\exists$).
- **Universelle Quantifizierung**: Klauseln sind implizit universell quantifiziert. Das bedeutet, dass sie für alle möglichen Belegungen ihrer Variablen gelten.
### Fazit
- Der Resolutionskalkül arbeitet nur mit Formeln in **Skolemnormalform**. Das bedeutet, dass die Formeln in einer speziellen Form vorliegen, die keine existenziellen Quantoren enthält und nur universelle Quantoren verwendet, was durch die Struktur der Klauseln automatisch gegeben ist.
### Notation
#### Negation eines Literals
- Zu einem Literal $L$ sei $\sim L$ das Literal, das $L$ negiert.
  - Wenn $L$ ein Atom ist, z.B. $P$, dann ist $\sim L = \neg P$.
  - Wenn $L$ die Negation eines Atoms ist, z.B. $\neg P$, dann ist $\sim L = P$.
  - Formal:
    - $$\sim L := \begin{cases} 
      \neg L & \text{wenn } L \text{ ein Atom ist} \\
      L' & \text{wenn } L = \neg L'
      \end{cases}$$
#### Negation einer Klausel
- Zu einer Klausel $C$ sei $\sim C$ die Menge der negierten Literale in $C$.
  - Beispiel: Wenn $C = \{P, \neg Q, R\}$, dann ist $\sim C = \{\neg P, Q, \neg R\}$.Formal:
    - $$\sim C := \{\sim L \mid L \in C\}$$

## Die Resolutionsregel
1. **Gegebene Klauseln und Literale**:
   - $C_1$ und $C_2$ sind Klauseln.
   - $p(t_1)$ ist ein Literal in $C_1$.
   - $\neg p(t_2)$ ist ein Literal in $C_2$.
2. **Bedingung für die Variablenmengen**:
   - $\text{Var}(C_1 \cup \{p(t_1)\}) \cap \text{Var}(C_2 \cup \{\neg p(t_2)\}) = \emptyset$
   - Das bedeutet, die Variablen in den Klauseln $C_1$ und $C_2$ dürfen sich nicht überschneiden. Sie müssen disjunkt sein.
3. **Unifikator**:
   - $\mu$ ist der allgemeinste Unifikator von $p(t_1)$ und $p(t_2)$.
   - Unifikation ist der Prozess, bei dem zwei Ausdrücke durch Substitution so angeglichen werden, dass sie identisch werden. Der allgemeinste Unifikator (MGU, Most General Unifier) ist die allgemeinste Substitution, die dies ermöglicht.
4. **Resolutionsregel**:
   $$
   \frac{C_1 \cup \{p(t_1)\}, \quad C_2 \cup \{\neg p(t_2)\}}{\mu(C_1 \cup C_2)}
   $$
   - Die Klausel $\mu(C_1 \cup C_2)$ ist die **Resolvente** der Eingabeklauseln $C_1 \cup \{p(t_1)\}$ und $C_2 \cup \{\neg p(t_2)\}$.
### Beispiel zur Verdeutlichung
Angenommen, wir haben zwei Klauseln:
- $C_1 = \{P(x), Q(y)\}$
- $C_2 = \{\neg P(f(z)), R(w)\}$
Hier sind die Literale:
- $p(t_1) = P(x)$
- $\neg p(t_2) = \neg P(f(z))$
Die Variablenmenge:
- $\text{Var}(C_1 \cup \{P(x)\}) = \{x, y\}$
- $\text{Var}(C_2 \cup \{\neg P(f(z))\}) = \{z, w\}$
Da $\{x, y\} \cap \{z, w\} = \emptyset$, sind die Variablenmengen disjunkt.
Der Unifikator $\mu$:
- $\mu$ könnte sein $x = f(z)$.
Die Resolvente $\mu(C_1 \cup C_2)$:
- $\mu(C_1) = \{P(f(z)), Q(y)\}$
- $\mu(C_2) = \{\neg P(f(z)), R(w)\}$
- Resolvente: $\mu(C_1 \cup C_2) = \{Q(y), R(w)\}$.
### Erweiterte Definition
1. **Klauseln und Literale**:
   - $C_1, C_2, K_1, K_2$ sind Klauseln.
   - $K_1$ und $K_2$ sind nicht die leere Klausel (also $K_1, K_2 \neq \square$).
2. **Bedingung für die Variablenmengen**:
   - $\text{Var}(C_1 \cup K_1) \cap \text{Var}(C_2 \cup K_2) = \emptyset$.
   - Die Variablenmengen in den Klauseln $C_1 \cup K_1$ und $C_2 \cup K_2$ müssen disjunkt sein.
3. **Unifikator**:
   - $\mu$ ist der allgemeinste Unifikator von $K_1 \cup \sim K_2$.
   - $\sim K_2$ bezeichnet die Menge der negierten Literale in $K_2$.
4. **Resolutionsregel**:
   $$
   \frac{C_1 \cup K_1, \quad C_2 \cup K_2}{\mu(C_1 \cup C_2)}
   $$
   - Die Klausel $\mu(C_1 \cup C_2)$ ist die Resolvente der Eingabeklauseln $C_1 \cup K_1$ und $C_2 \cup K_2$.
Schauen wir uns die Anwendung der Resolutionsregel anhand des gegebenen Beispiels und der nachfolgenden Erklärungen an. Wir gehen dabei schrittweise vor.
### Anwendung der Resolutionsregel
Gegeben sein die Klauseln:
- $C_1 = \{p(x), q(f(x)), q(f(g(c)))\}$
- $C_2 = \{r(y, z), \neg q(z), \neg q(f(y))\}$
#### Regelanwendung
1. **Klauseln aufteilen**:
   - Wir teilen die Klauseln in zwei Teile:
     - $C_1 \setminus \{q(f(x)), q(f(g(c)))\} = \{p(x)\}$
     - $C_2 \setminus \{\neg q(z), \neg q(f(y))\} = \{r(y, z)\}$
   - Die Literale, die wir auflösen wollen, sind:
     - In $C_1$: $q(f(x)), q(f(g(c)))$
     - In $C_2$: $\neg q(z), \neg q(f(y))$
2. **Gemeinsame Literale finden**:
   - Wir suchen Literale in $C_1$, die die Negation von Literalen in $C_2$ sind:
     - $q(f(x))$ in $C_1$ und $\neg q(z)$ in $C_2$.
     - $q(f(g(c)))$ in $C_1$ und $\neg q(f(y))$ in $C_2$.
3. **Unifikator $\mu$**:
   - Um die Literale aufzulösen, finden wir den allgemeinsten Unifikator $\mu$, der diese Literale gleich macht.
   - Die zu unifizierenden Literale sind:
     - $q(f(x))$ und $q(z)$
     - $q(f(g(c)))$ und $q(f(y))$
   - Die Unifikation führt zu:
     - $\mu = \{x \rightarrow g(c), y \rightarrow g(c), z \rightarrow f(g(c))\}$
4. **Resolvente finden**:
   - Wende $\mu$ auf die Klauseln an:
     - $\mu(C_1) = \{p(g(c)), q(f(g(c))), q(f(g(c)))\}$
     - $\mu(C_2) = \{r(g(c), f(g(c))), \neg q(f(g(c))), \neg q(f(g(c)))\}$
   - Entferne die übereinstimmenden Literale $q(f(g(c)))$ und $\neg q(f(g(c)))$:
     - Übrig bleiben die Literale $p(g(c))$ und $r(g(c), f(g(c)))$.
   - Die Resolvente ist:
     $\mu(C_1 \cup C_2) = \{p(g(c)), r(g(c), f(g(c)))\}$
### Definitionen

1. **Resolventenmenge $\text{Res}(M)$**:
   - $\text{Res}(M)$ ist die Menge aller möglichen Resolventen, die aus den Klauseln der Menge $M$ gebildet werden können.
   - Formal: 
     $\text{Res}(M) = \{B \mid \text{es gibt Varianten } C_1, C_2 \text{ von Klauseln aus } M, \text{ sodass } B \text{ eine Resolvente von } C_1, C_2 \text{ ist}\}$
2. **Klauselmenge $R_0(M)$**:
   - $R_0(M) = M$
   - Das bedeutet, $R_0$ ist einfach die ursprüngliche Klauselmenge.
3. **Rekursive Definition $R_{n+1}(M)$**:
   - $R_{n+1}(M) = \text{Res}(R_n(M)) \cup R_n(M)$
   - Das bedeutet, $R_{n+1}(M)$ ist die Vereinigung der Menge aller Resolventen von $R_n(M)$ und $R_n(M)$ selbst.
   - Rekursiv angewendet, wird so jede neue Klauselmenge um die möglichen neuen Resolventen erweitert.
### Korrektheit und Vollständigkeit
**Theorem**: Eine Klauselmenge $M$ ist unerfüllbar genau dann, wenn die leere Klausel $\square$ in der Vereinigung aller $R_n(M)$-Mengen enthalten ist.
- **Formal**:
  $M \text{ ist unerfüllbar genau dann, wenn } \square \in \bigcup_{n \in \mathbb{N}} R_n(M)$
- **Bedeutung**:
  - Wenn $M$ unerfüllbar ist, dann kann man durch sukzessive Anwendung der Resolutionsregel (und der Unifikation) irgendwann die leere Klausel $\square$ ableiten.
  - Die sukzessive Erweiterung von $R_n(M)$ umfasst schließlich alle möglichen Resolventen, wodurch jeder mögliche Widerspruch erkannt wird.
Lassen Sie uns das Beispiel zur Transitivität der Teilmengenbeziehung in der Mengenlehre Schritt für Schritt durchgehen. Dabei werden wir die logische Folgerung beweisen, die zu einer leeren Klausel führt, was zeigt, dass die Folgerung korrekt ist.

### Beispiel
Wir wollen beweisen:
$$\forall x \forall y (x \subseteq y \leftrightarrow \forall u (u \in x \rightarrow u \in y)) \models \forall x \forall y \forall z (x \subseteq y \land y \subseteq z \rightarrow x \subseteq z)$$
Dies ist eine Aussage zur Transitivität der Teilmengenbeziehung $\subseteq$. Sie besagt, dass wenn $x \subseteq y$ und $y \subseteq z$, dann folgt $x \subseteq z$.
#### Transformation in Klauselnormalform für die Prämisse
1. **Zerlegung der Prämisse**:
   Die Prämisse besteht aus zwei Teilen:
   - $\forall x \forall y (x \subseteq y \rightarrow \forall u (u \in x \rightarrow u \in y))$
   - $\forall x \forall y (\forall u (u \in x \rightarrow u \in y) \rightarrow x \subseteq y)$
2. **Umwandlung der ersten Formel**:
   - Ursprüngliche Formel:
     $\forall x \forall y (x \subseteq y \rightarrow \forall u (u \in x \rightarrow u \in y))$
   - Übersetzung in Klauseln: Wir ersetzen die Relationen $\subseteq$ und $\in$ durch Prädikate $\text{conteq}$ und $\text{memb}$.
     $\{\neg \text{conteq}(x, y), \neg \text{memb}(u, x), \text{memb}(u, y)\}$
3. **Umwandlung der zweiten Formel**:
   - Ursprüngliche Formel:
     $\forall x \forall y (\forall u (u \in x \rightarrow u \in y) \rightarrow x \subseteq y)$
   - Elimination des Implikators:
     $\forall x \forall y (\exists u (u \in x \land \neg u \in y) \lor x \subseteq y)$
   - Skolemisierung: Wir führen eine Skolem-Funktion $f(x, y)$ ein, um die Existenzquantoren zu eliminieren.
     $\forall x \forall y ((f(x, y) \in x \land \neg f(x, y) \in y) \lor x \subseteq y)$
   - Umformung in Klauselnormalform (KNF):
     $\{\text{memb}(f(x, y), x), \text{conteq}(x, y)\}, \quad \{\neg \text{memb}(f(x, y), y), \text{conteq}(x, y)\}$

#### Transformation in Klauselnormalform für die Behauptung
1. **Negation der Behauptung**:
   - Ursprüngliche Behauptung:
     $\forall x \forall y \forall z (x \subseteq y \land y \subseteq z \rightarrow x \subseteq z)$
   - Negation der Behauptung:
     $\exists x \exists y \exists z (x \subseteq y \land y \subseteq z \land \neg x \subseteq z)$
   - Skolemisierung: Wir führen Skolemkonstanten $a, b, c$ ein.
     $\text{conteq}(a, b)$
     $\text{conteq}(b, c)$
     $\neg \text{conteq}(a, c)$
#### Resolutionsbeweis
1. **Vorbedingungen** (Prämissen):
   - (1) $\{\neg \text{conteq}(x, y), \neg \text{memb}(u, x), \text{memb}(u, y)\}$
   - (2) $\{\text{memb}(f(x, y), x), \text{conteq}(x, y)\}$
   - (3) $\{\neg \text{memb}(f(x, y), y), \text{conteq}(x, y)\}$
2. **Negation der Behauptung**:
   - (4) $\text{conteq}(a, b)$
   - (5) $\text{conteq}(b, c)$
   - (6) $\neg \text{conteq}(a, c)$
3. **Ableitungsschritte**:
   - (7) $\{\neg \text{memb}(u, a), \text{memb}(u, b)\}$ aus (1) und (4) durch Unifikation $x = a$, $y = b$
   - (8) $\{\neg \text{memb}(u, b), \text{memb}(u, c)\}$ aus (1) und (5) durch Unifikation $x = b$, $y = c$
   - (9) $\{\neg \text{memb}(f(a, c), c)\}$ aus (3) und (6) durch Unifikation $x = a$, $y = c$
   - (10) $\{\text{memb}(f(a, c), a)\}$ aus (2) und (6) durch Unifikation $x = a$, $y = c$
   - (13) $\{\text{memb}(f(a, c), b)\}$ aus (7) und (10)
   - (19) $\{\text{memb}(f(a, c), c)\}$ aus (8) und (13)
   - (20) $\{\}$ (leere Klausel) aus (19) und (9)
#### Schlussfolgerung
Die leere Klausel $\{\}$ zeigt, dass die Negation der Behauptung zu einem Widerspruch führt. Daher ist die ursprüngliche Behauptung wahr: Die Transitivität der Teilmengenbeziehung kann direkt aus der Definition der Teilmengenrelation gefolgert werden.

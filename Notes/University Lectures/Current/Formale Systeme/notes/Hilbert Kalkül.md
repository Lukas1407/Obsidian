> [!abstract] Definition
> Der Hilbertkalkül ist ein System der formalen Logik, das auf Axiomen und Ableitungsregeln basiert. Er wird verwendet, um Aussagen und deren logische Konsequenzen zu analysieren und zu beweisen. Die Axiome sind in diesem System als Schemata definiert, was bedeutet, dass sie allgemeine Muster darstellen, die durch spezifische Ausdrücke ersetzt werden können. 

## Hilbertkalkül Überblick
- **Axiome**: Grundlegende, immer wahre logische Aussagen.
- **Axiom-Schemata**: Allgemeine Formen von Aussagen, die durch Einsetzen von Variablen oder Formeln spezifiziert werden.
- **Operatoren**: Logische Verknüpfungen wie „nicht“ (¬), „impliziert“ (→), „für alle“ (∀).
- **Ableitungsregeln**: Regeln, wie man aus gegebenen Aussagen neue Aussagen ableiten kann.
## Axiome des Hilbertkalküls
### Ax1: $\alpha \rightarrow (\beta \rightarrow \alpha)$ (Abschwächung)
- **Bedeutung**: Wenn eine Aussage $\alpha$ wahr ist, dann ist $\alpha$ auch wahr, selbst wenn eine andere Aussage $\beta$ wahr ist.
- **Interpretation**: Das Hinzufügen zusätzlicher Bedingungen $\beta$ ändert nichts an der Gültigkeit von $\alpha$.
### Ax2: $(\alpha \rightarrow (\beta \rightarrow \gamma)) \rightarrow ((\alpha \rightarrow \beta) \rightarrow (\alpha \rightarrow \gamma))$ (Verteilung von →)
- **Bedeutung**: Wenn $\alpha$ zu $\beta \rightarrow \gamma$ führt und $\alpha$ zu $\beta$ führt, dann führt $\alpha$ direkt zu $\gamma$.
- **Interpretation**: Die Implikation kann verteilt werden, um direkt von $\alpha$ auf $\gamma$ zu schließen, wenn die Zwischenschritte erfüllt sind.
### Ax3: $(\neg \alpha \rightarrow \neg \beta) \rightarrow (\beta \rightarrow \alpha)$ (Kontraposition)
- **Bedeutung**: Wenn $\alpha$ aus $\beta$ folgt, folgt auch die Negation von $\beta$ aus der Negation von $\alpha$.
- **Interpretation**: Dies ist die Regel der Kontraposition, die besagt, dass die Negation einer Aussage die Negation ihrer Konsequenz umkehrt.
### Ax4: $\forall x \alpha \rightarrow \{x/t\}(\alpha)$ (Instantiierung)
- **Bedeutung**: Wenn eine Aussage für alle $x$ gilt, dann gilt sie auch für jede spezifische Instanz $t$ von $x$, solange es keine Konflikte gibt.
- **Interpretation**: Aus einer universellen Aussage kann eine spezifische Aussage abgeleitet werden.
### Ax5: $\forall x(\alpha \rightarrow \beta) \rightarrow (\alpha \rightarrow \forall x \beta)$ (∀-Verschiebung)
- **Bedeutung**: Wenn $\alpha$ zu $\beta$ führt, und dies für alle $x$ gilt, dann führt $\alpha$ auch zu $\beta$ für alle $x$, sofern $x$ nicht in $\alpha$ vorkommt.
- **Interpretation**: Dies ermöglicht es, die Allquantifizierung in der Aussage zu verschieben.
## Ableitungsregeln
### Modus Ponens (Mp):
- **Schema**: $\alpha, \alpha \rightarrow \beta$ ⇒ $\beta$
- **Bedeutung**: Wenn $\alpha$ wahr ist und $\alpha \rightarrow \beta$ wahr ist, dann ist auch $\beta$ wahr.
- **Interpretation**: Aus einer Implikation und der Wahrheit des Antezedens folgt die Wahrheit des Konsequens.
### Generalisierung (Gen):
- **Schema**: $\alpha$ ⇒ $\forall x \alpha$
- **Bedeutung**: Wenn $\alpha$ wahr ist, dann ist $\alpha$ für alle $x$ wahr.
- **Interpretation**: Eine wahre Aussage kann auf alle $x$ verallgemeinert werden.
## Hilbertkalkül $H_0$
- Die Axiome Ax1, Ax2 und Ax3 zusammen mit dem Modus Ponens bilden den sogenannten aussagenlogischen Hilbertkalkül $H_0$.
- **Aussagenlogischer Hilbertkalkül $H_0$**: 
  - Ein System, das sich ausschließlich mit Aussagenlogik beschäftigt und es ermöglicht, aus gegebenen Aussagen durch Anwendung von Regeln neue Aussagen abzuleiten.
### Beispiel: Schrittweise Ableitung von $A \rightarrow A$
Wir möchten zeigen, dass $A \rightarrow A$ unter Verwendung der Axiome und des Modus Ponens im Hilbertkalkül $H_0$ abgeleitet werden kann.
#### 1. Zuerst: Das Axiom $\text{Ax2}$
$$
\text{Ax2:} \quad (\alpha \rightarrow (\beta \rightarrow \gamma)) \rightarrow ((\alpha \rightarrow \beta) \rightarrow (\alpha \rightarrow \gamma))
$$
Wir setzen für $\alpha, \beta, \gamma$:
$$
\alpha = A, \quad \beta = A \rightarrow A, \quad \gamma = A
$$
$$
\text{Also:} \quad (A \rightarrow ((A \rightarrow A) \rightarrow A)) \rightarrow ((A \rightarrow (A \rightarrow A)) \rightarrow (A \rightarrow A))
$$
#### 2. Zweitens: Das Axiom $\text{Ax1}$
$$
\text{Ax1:} \quad A \rightarrow (B \rightarrow A)
$$
Setze $A = A$ und $B = A \rightarrow A$:
$$
\text{Also:} \quad A \rightarrow ((A \rightarrow A) \rightarrow A)
$$
#### 3. Modus Ponens auf (1) und (2)
Der Modus Ponens (Mp) lautet:
$$
\text{Mp:} \quad \frac{\alpha \quad \alpha \rightarrow \beta}{\beta}
$$
Wende Mp auf die Ergebnisse von (1) und (2) an:
$$
\text{1:} \quad (A \rightarrow ((A \rightarrow A) \rightarrow A)) \rightarrow ((A \rightarrow (A \rightarrow A)) \rightarrow (A \rightarrow A))
$$

$$
\text{2:} \quad A \rightarrow ((A \rightarrow A) \rightarrow A)
$$
$$
\text{3:} \quad (A \rightarrow (A \rightarrow A)) \rightarrow (A \rightarrow A)
$$

#### 4. Viertens: Das Axiom $\text{Ax1}$ nochmal
Verwende $\text{Ax1}$ noch einmal:
$$
\text{Ax1:} \quad A \rightarrow (B \rightarrow A)
$$
Setze $A = A$ und $B = A$:
$$
\text{Also:} \quad A \rightarrow (A \rightarrow A)
$$

#### 5. Modus Ponens auf (3) und (4)
Wende Mp auf die Ergebnisse von (3) und (4) an:

$$
\text{3:} \quad (A \rightarrow (A \rightarrow A)) \rightarrow (A \rightarrow A)
$$

$$
\text{4:} \quad A \rightarrow (A \rightarrow A)
$$
$$
\text{5:} \quad A \rightarrow A
$$

### Erklärung der Schritte

1. **Schritt 1**:
   - Wir beginnen mit Axiom $\text{Ax2}$, das uns ermöglicht, Implikationen zu verketten und komplexe logische Zusammenhänge auszudrücken.
2. **Schritt 2**:
   - Wir wenden $\text{Ax1}$ an, das besagt, dass eine Aussage $A$ immer dann wahr bleibt, wenn eine andere Aussage hinzukommt, die $A$ nicht ändert.
3. **Schritt 3**:
   - Durch Anwendung des Modus Ponens (Mp) auf die Ergebnisse von Schritt 1 und 2 leiten wir eine neue Aussage ab.
4. **Schritt 4**:
   - Wir verwenden $\text{Ax1}$ erneut, um die Wahrheit der Aussage $A \rightarrow (A \rightarrow A)$ zu bestätigen.
5. **Schritt 5**:
   - Schließlich wenden wir erneut Mp an, um die endgültige Aussage $A \rightarrow A$ zu erhalten, die beweist, dass eine Aussage $A$ impliziert, dass $A$ wahr ist.

## Deduktionstheorem
Das Deduktionstheorem besagt Folgendes:
- **Theorem**: Sei $M$ eine Menge von Formeln, und $A$ und $B$ seien Formeln, wobei $A$ keine freien Variablen enthält. Dann gilt:
$$
M \vdash_H A \rightarrow B \quad \Leftrightarrow \quad M \cup \{A\} \vdash_H B
$$
- **Bedeutung**: Wenn $A$ keine freien Variablen enthält, dann können wir $A \rightarrow B$ aus der Menge $M$ ableiten, wenn und nur wenn wir $B$ aus der Menge $M$ und $A$ ableiten können.

### Beweis des Deduktionstheorems
#### Richtung: $\Rightarrow$
- **Annahme**: $M \vdash A \rightarrow B$ (Das bedeutet, dass $A \rightarrow B$ aus $M$ ableitbar ist.)
- **Zu zeigen**: $M \cup \{A\} \vdash B$
1. **$M \cup \{A\} \vdash A \rightarrow B$**:
   - Wenn $M \vdash A \rightarrow B$, dann gilt erst recht $M \cup \{A\} \vdash A \rightarrow B$, weil $M \cup \{A\}$ alle Formeln aus $M$ enthält und zusätzlich $A$.
2. **$M \cup \{A\} \vdash A$**:
   - $A$ ist triviale Ableitung in $M \cup \{A\}$, weil $A$ direkt in der Menge enthalten ist.
3. **$M \cup \{A\} \vdash B$**:
   - Aus $A \rightarrow B$ und $A$ können wir $B$ mit Modus Ponens ableiten: $\frac{A, A \rightarrow B}{B}$.
#### Richtung: $\Leftarrow$
- Diese Richtung wird in der typischen Beweisskizze oft auf das Skriptum oder weiterführende Literatur verwiesen, weil sie komplexer ist. Im Wesentlichen zeigt man hier, dass, wenn $M \cup \{A\} \vdash B$, dies bedeutet, dass $A \rightarrow B$ durch eine schrittweise Ableitung aus $M$ herausgearbeitet werden kann.
### Beispiel einer Ableitung mit dem Deduktionstheorem
Zeige, dass:
$$
(A \rightarrow B) \rightarrow ((B \rightarrow C) \rightarrow (A \rightarrow C))
$$
eine Tautologie ist.
#### Schrittweise Ableitung
1. **Menge der Formeln**: $\{A \rightarrow B, B \rightarrow C, A\}$
2. **Ableitung von $A$**: 
   - $\{A \rightarrow B, B \rightarrow C, A\} \vdash A$ 
   - Trivialerweise ist $A$ aus der Menge ableitbar, weil es in der Menge enthalten ist.
3. **Ableitung von $A \rightarrow B$**: 
   - $\{A \rightarrow B, B \rightarrow C, A\} \vdash A \rightarrow B$
   - Dies ist ebenfalls trivial, weil $A \rightarrow B$ in der Menge enthalten ist.
4. **Ableitung von $B$**: 
   - $\{A \rightarrow B, B \rightarrow C, A\} \vdash B$
   - Aus $A$ und $A \rightarrow B$ kann mit Modus Ponens $B$ abgeleitet werden: $\frac{A, A \rightarrow B}{B}$.
5. **Ableitung von $B \rightarrow C$**: 
   - $\{A \rightarrow B, B \rightarrow C, A\} \vdash B \rightarrow C$
   - Dies ist trivial, weil $B \rightarrow C$ in der Menge enthalten ist.
6. **Ableitung von $C$**: 
   - $\{A \rightarrow B, B \rightarrow C, A\} \vdash C$
   - Aus $B$ und $B \rightarrow C$ kann mit Modus Ponens $C$ abgeleitet werden: $\frac{B, B \rightarrow C}{C}$.
7. **Ableitung von $A \rightarrow C$**: 
   - $\{A \rightarrow B, B \rightarrow C\} \vdash A \rightarrow C$
   - Nach dem Deduktionstheorem folgt: $A \rightarrow C$ ist ableitbar aus der Menge $\{A \rightarrow B, B \rightarrow C\}$, weil $C$ aus $A$ ableitbar ist.
8. **Ableitung der Tautologie**:
   - $\{A \rightarrow B\} \vdash (B \rightarrow C) \rightarrow (A \rightarrow C)$
   - Wiederum durch Anwendung des Deduktionstheorems.
9. **Ableitung der endgültigen Tautologie**:
   - $\vdash (A \rightarrow B) \rightarrow ((B \rightarrow C) \rightarrow (A \rightarrow C))$
   - Schließlich können wir durch das Deduktionstheorem die ursprüngliche Aussage als allgemeingültige Tautologie ableiten.

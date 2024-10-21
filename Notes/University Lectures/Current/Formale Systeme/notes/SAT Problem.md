## Problemstellung
- **Instanz**: Eine aussagenlogische Formel $F$ in der [[Konjunktive Normalform (KNF)]]
  - Die Formel besteht aus Variablen, die durch logische Operatoren (wie UND, ODER, NICHT) verknüpft sind.
- **Frage**: Ist die Formel $F$ erfüllbar?
  - Gibt es eine Interpretation $I$ der Variablen, bei der die Formel $F $wahr wird, d.h. $\text{val}_I(F) = W$?
## NP-Vollständigkeit des SAT-Problems
### NP-Vollständigkeit
- **NP**: Die Klasse der Probleme, die in nichtdeterministischer polynomialer Zeit lösbar sind. Das bedeutet, dass es eine polynomial lange Verifikation für Lösungen gibt.
- **NP-vollständig**: Ein Problem ist NP-vollständig, wenn es in NP liegt und jedes Problem in NP auf dieses Problem in polynomialer Zeit reduziert werden kann.
### Bedeutung für das SAT-Problem
- Das SAT-Problem ist das erste bekannte NP-vollständige Problem.
- Wenn es einen deterministischen polynomialen Algorithmus gäbe, der das SAT-Problem löst, würde das bedeuten, dass NP gleich P ist (d.h., alle Probleme in NP könnten in polynomialer Zeit gelöst werden).
### Beispiel: Einfache logische Formel
- **Formel $F$**: $(x_1 \lor \neg x_2) \land (x_2 \lor x_3) \land (\neg x_1 \lor \neg x_3)$
  - Diese Formel besteht aus drei Klauseln:
    - $(x_1 \lor \neg x_2)$: $x_1$ oder nicht $x_{2}$ müssen wahr sein.
    - $(x_2 \lor x_3)$: $x_2$ oder $x_3$ müssen wahr sein.
    - $(\neg x_1 \lor \neg x_3)$: nicht $x_1$ oder nicht $x_3$ müssen wahr sein.
- **Frage**: Ist $F$ erfüllbar?
  - Wir suchen eine Zuweisung der Variablen $x_1, x_2, x_3$, die die Formel wahr macht.
#### Zuweisung testen
- **Mögliche Zuweisung**:
  - $x_1 = \text{wahr}, x_2 = \text{falsch}, x_3 = \text{wahr}$
- **Überprüfung**:
  - $(x_1 \lor \neg x_2) = \text{wahr} \lor \text{wahr} = \text{wahr}$
  - $(x_2 \lor x_3) = \text{falsch} \lor \text{wahr} = \text{wahr}$
  - $(\neg x_1 \lor \neg x_3) = \text{falsch} \lor \text{falsch} = \text{falsch}$
- Diese Zuweisung erfüllt die dritte Klausel nicht, daher ist die Formel mit dieser Zuweisung nicht wahr.
- **Andere Zuweisung**:
  - $x_1 = \text{wahr}, x_2 = \text{wahr}, x_3 = \text{falsch}$
- **Überprüfung**:
  - $(x_1 \lor \neg x_2) = \text{wahr} \lor \text{falsch} = \text{wahr}$
  - $(x_2 \lor x_3) = \text{wahr} \lor \text{falsch} = \text{wahr}$
  - $(\neg x_1 \lor \neg x_3) = \text{falsch} \lor \text{wahr} = \text{wahr}$
- Diese Zuweisung erfüllt alle Klauseln, daher ist die Formel erfüllbar.

## Erfüllbarkeitsprobleme für spezielle Formeln
1. **KNF (Konjunktive Normalform)**:
   - **Definition**: Eine Formel in KNF ist eine Konjunktion (UND-Verknüpfung) von Disjunktionen (ODER-Verknüpfungen) von Literalen.
   - **Beispiel**: $F = (x \lor \neg y) \land (y \lor z) \land (\neg x \lor \neg z)$.
   - **Komplexität**: Das Erfüllbarkeitsproblem für KNF-Formeln ist NP-vollständig.
2. **3-KNF**:
   - **Definition**: Eine 3-KNF-Formel ist eine spezielle Form der KNF, bei der jede Disjunktion höchstens drei Literale enthält.
   - **Beispiel**: $F = (x \lor \neg y \lor z) \land (y \lor \neg z \lor w) \land (\neg x \lor y \lor \neg w)$.
   - **Komplexität**: Das Erfüllbarkeitsproblem für 3-KNF-Formeln ist ebenfalls NP-vollständig.
3. **2-KNF (Krom-Formeln)**:
   - **Definition**: Eine 2-KNF-Formel ist eine spezielle Form der KNF, bei der jede Disjunktion höchstens zwei Literale enthält.
   - **Beispiel**: $F = (x \lor \neg y) \land (y \lor z) \land (\neg x \lor z)$.
   - **Komplexität**: Das <mark style="background: #FFB86CA6;">Erfüllbarkeitsproblem für 2-KNF-Formeln ist polynomial entscheidbar</mark>, da es effiziente Algorithmen gibt, die in polynomialer Zeit feststellen können, ob eine solche Formel erfüllbar ist.
4. **DNF (Disjunktive Normalform)**:
   - **Definition**: Eine Formel in DNF ist eine Disjunktion (ODER-Verknüpfung) von Konjunktionen (UND-Verknüpfungen) von Literalen.
   - **Beispiel**: $F = (x \land \neg y) \lor (y \land z) \lor (\neg x \land \neg z)$.
   - **Komplexität**: Das <mark style="background: #FFB86CA6;">Erfüllbarkeitsproblem für DNF-Formeln ist polynomial entscheidbar</mark>. Da jede Klausel eine Belegung direkt erfüllt, ist die Entscheidung, ob die Formel wahr ist, einfach.
5. **[[Horn-Formel|Horn-Formeln]]**:
   - **Definition**: Eine [[Horn-Formel]] ist eine spezielle Form der KNF, bei der jede Klausel höchstens ein positives Literal enthält.
   - **Beispiel**: $F = (\neg x \lor \neg y \lor z) \land (\neg z \lor w) \land (\neg w)$.
   - **Komplexität**: Das Erfüllbarkeitsproblem für [[Horn-Formel|Horn-Formeln]] ist ebenfalls polynomial entscheidbar. Es gibt effiziente u noch weitere Fragen hast oder tiefer in ein bestimmtes Thema eintauchen möchtest, lass es mich wissen!
## [[Disjunktive Normalform (DNF)]]
## [[Konjunktive Normalform (KNF)]]

Lass uns diese Aussagen genauer untersuchen, um zu verstehen, warum die Erfüllbarkeit einer Formel in disjunktiver Normalform (DNF) und die Allgemeingültigkeit einer Formel in konjunktiver Normalform (KNF) in polynomieller Zeit überprüfbar sind und warum die umgekehrten Probleme schwieriger sind.

## Erfüllbarkeit einer Formel in DNF
- Um die Erfüllbarkeit einer Formel in DNF zu überprüfen, genügt es, eine Konjunktion zu finden, die erfüllbar ist, da die gesamte Disjunktion wahr wird, wenn mindestens eine der Konjunktionen wahr ist.
- **Schritte zur Überprüfung**:
  1. Gehe jede Konjunktion in der DNF durch.
  2. Prüfe, ob es eine Konjunktion gibt, die keine widersprüchlichen Literale enthält (wie $P$ und $\neg P$ in derselben Konjunktion).
- **Polynomielle Zeit**:
  - Jede Konjunktion in der DNF kann in linearer Zeit in Bezug auf die Anzahl der Variablen überprüft werden.
  - <mark style="background: #FFB86CA6;">Da die DNF eine polynomielle Anzahl von Konjunktionen hat</mark>, ist die Überprüfung der Erfüllbarkeit in polynomieller Zeit möglich.
### Beispiel
- Für die Formel $D = (P \land \neg Q) \lor (Q \land R) \lor (\neg P \land R)$:
- Wir prüfen jede Konjunktion:
  - $P \land \neg Q$ ist erfüllbar (z.B. $P = \text{wahr}, Q = \text{falsch}$).
  - Da $P \land \neg Q$ erfüllbar ist, ist $D$ erfüllbar.
## Allgemeingültigkeit einer Formel in KNF
- Um die Allgemeingültigkeit einer Formel in KNF zu überprüfen, genügt es, jede Disjunktion zu prüfen und sicherzustellen, dass jede Disjunktion wahr ist, unabhängig von der Belegung der Variablen.
- **Schritte zur Überprüfung**:
  1. Gehe jede Disjunktion in der KNF durch.
  2. Prüfe, ob jede Disjunktion mindestens ein Literal enthält, das unter jeder möglichen Belegung der Variablen wahr ist.
- **Polynomielle Zeit**:
  - Jede Disjunktion kann in linearer Zeit in Bezug auf die Anzahl der Literale überprüft werden.
  - Da die KNF eine polynomielle Anzahl von Disjunktionen hat, ist die Überprüfung der Allgemeingültigkeit in polynomieller Zeit möglich.
### Beispiel
Für die Formel $K = (P \lor \neg Q) \land (Q \lor R) \land (\neg P \lor R)$:
- Wir prüfen jede Disjunktion:
  - $P \lor \neg Q$: Für alle Belegungen von $P$ oder $Q$ ist mindestens ein Literal wahr.
  - $Q \lor R$: Für alle Belegungen von $Q$ oder $R$ ist mindestens ein Literal wahr.
  - $\neg P \lor R$: Für alle Belegungen von $P$ oder $R$ ist mindestens ein Literal wahr.
- Da jede Disjunktion wahr ist, ist $K$ allgemein gültig.
## Umgekehrte Probleme: Erfüllbarkeit von KNF und Allgemeingültigkeit von DNF
### Erfüllbarkeit von KNF
- **Schwierig**: Die Erfüllbarkeit von KNF-Formeln zu überprüfen ist <mark style="background: #FF5582A6;">NP-vollständig</mark>. Das bedeutet, dass die Überprüfung, ob eine KNF-Formel erfüllbar ist, im Allgemeinen schwieriger und zeitaufwändiger ist.
- **Grund**: Man muss potenziell eine große Anzahl von Variablenkombinationen durchprobieren, um eine zu finden, die die Formel wahr macht.
### Allgemeingültigkeit von DNF
- **Schwierig**: Die Allgemeingültigkeit von DNF-Formeln zu überprüfen ist ebenfalls <mark style="background: #FF5582A6;">NP-vollständig</mark>.
- **Grund**: Man müsste sicherstellen, dass jede mögliche Variablenkombination in mindestens einer Konjunktion der DNF zu wahr führt, was eine sehr große Anzahl von Prüfungen erfordert.

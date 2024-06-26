## Was ist eine Sequenz?
Eine **Sequenz** ist ein formales Konstrukt, das in der logischen Beweisführung verwendet wird. Es besteht aus zwei endlichen Mengen von Formeln und wird in der Form$\Gamma \Rightarrow \Delta$ notiert, wobei:
- **$\Gamma$** (Antezedent) ist die Menge von Formeln auf der linken Seite des Sequenzzeichens$\Rightarrow$.
- **$\Delta$** (Sukzedent) ist die Menge von Formeln auf der rechten Seite des Sequenzzeichens$\Rightarrow$.

Sowohl $\Gamma$ als auch $\Delta$ können die leere Menge sein.

#### Interpretation von Sequenzen

In einer prädikatenlogischen Struktur$D$ mit einer Variablenbelegung$\beta$ wird eine Sequenz$\Gamma \Rightarrow \Delta$ wie folgt interpretiert:

$$ \text{val}_{D, \beta}(\Gamma \Rightarrow \Delta) = \text{val}_{D, \beta} \left( \bigwedge \Gamma \rightarrow \bigvee \Delta \right) $$

Hierbei bedeuten:
- **$\bigwedge \Gamma$**: Konjunktion aller Formeln in $\Gamma$.
- **$\bigvee \Delta$**: Disjunktion aller Formeln in $\Delta$.
#### Vereinbarungen für leere Mengen
- **Leere Konjunktion ($\bigwedge \emptyset$)**: Diese wird als wahr ($ \top$) betrachtet. Das bedeutet, dass eine leere Menge von Formeln, die alle konjunktiv verbunden sind, immer als wahr gilt, da es keine widersprüchlichen Aussagen gibt.
- **Leere Disjunktion ($\bigvee \emptyset$)**: Diese wird als falsch ($ \bot$) betrachtet. Das bedeutet, dass eine leere Menge von Formeln, die alle disjunktiv verbunden sind, als falsch gilt, da keine der Formeln erfüllt ist.
#### Bedeutung der Sequenznotation
- Eine Sequenz $\Gamma \Rightarrow \Delta$ besagt, dass die Konjunktion der Formeln in$\Gamma$ die Disjunktion der Formeln in$\Delta$ impliziert.
- Wenn $\Gamma$ leer ist, bedeutet das$\top \Rightarrow \Delta$, was äquivalent ist zu$\bigvee \Delta$ (die Formeln in$\Delta$ müssen erfüllt sein, um die Sequenz wahr zu machen).
- Wenn $\Delta$ leer ist, bedeutet das$\Gamma \Rightarrow \bot$, was äquivalent ist zu$\bigwedge \Gamma$ (die Konjunktion der Formeln in$\Gamma$ muss falsch sein, um die Sequenz wahr zu machen).
#### Beispiele
1. **Einfaches Beispiel:**
   -$\Gamma = \{P, Q\}$
   -$\Delta = \{R, S\}$
  $\Gamma \Rightarrow \Delta$ bedeutet$P \land Q \rightarrow R \lor S$.
2. **Leeres Antezedent:**
   -$\Gamma = \emptyset$
   -$\Delta = \{P\}$
  $\emptyset \Rightarrow \{P\}$ bedeutet$\top \rightarrow P$, was einfach$P$ bedeutet.
3. **Leeres Sukzedent:**
   -$\Gamma = \{P\}$
   -$\Delta = \emptyset$
  $\{P\} \Rightarrow \emptyset$ bedeutet$P \rightarrow \bot$, was einfach$\neg P$ bedeutet.

Die Theoreme zur **Korrektheit und Vollständigkeit des Sequenzenkalküls** stellen sicher, dass das Kalkül sowohl zuverlässige Beweise ermöglicht als auch vollständig ist. Das bedeutet, dass man mit dem Kalkül alle wahren Aussagen beweisen kann. Lass uns die beiden Theoreme und ihre Bedeutung genauer betrachten:

## Korrektheit des Sequenzenkalküls
**Theorem:**
Es seien$M \subseteq \text{For}_{\Sigma}$ und$A \in \text{For}_{\Sigma}$. Dann gilt:

$$ \text{Aus } M \vdash_S A \text{ folgt } M \models A $$
#### Bedeutung:
- **$M \vdash_S A$:** Das bedeutet, dass$A$ aus der Menge von Formeln$M$ im Sequenzenkalkül$S$ ableitbar ist. Es gibt also eine formale Ableitung im Kalkül$S$, die zeigt, dass$A$ aus$M$ folgt.
- **$M \models A$:** Das bedeutet, dass$A$ in jeder Interpretation, die alle Formeln in$M$ wahr macht, ebenfalls wahr ist. Es ist die semantische Folgebeziehung.
#### Erklärung:
Die Korrektheit besagt, dass jeder Beweis, der im Sequenzenkalkül$S$ geführt werden kann, auch semantisch gültig ist. Mit anderen Worten: Wenn man$A$ aus$M$ beweisen kann, dann ist$A$ in jedem Modell, das$M$ erfüllt, ebenfalls erfüllt. Das verhindert, dass man im Kalkül zu falschen Schlussfolgerungen gelangt.
#### Beispiel:
Angenommen, $M = \{P \rightarrow Q, P\}$ und$A = Q$. 
- **Ableitbar im Kalkül:** Man kann $Q$ aus $P \rightarrow Q$ und$P$ ableiten.
- **Semantische Gültigkeit:** Wenn $P \rightarrow Q$ und$P$ wahr sind, muss$Q$ auch wahr sein.

### Vollständigkeit des Sequenzenkalküls
**Theorem:**
Es seien$M \subseteq \text{For}_{\Sigma}$ und$A \in \text{For}_{\Sigma}$. Dann gilt:

$$ \text{Aus } M \models A \text{ folgt } M \vdash_S A $$

#### Bedeutung:
- **$M \models A$:** Das bedeutet, dass$A$ in jeder Interpretation, die$M$ erfüllt, ebenfalls wahr ist.
- **$M \vdash_S A$:** Das bedeutet, dass$A$ im Sequenzenkalkül$S$ aus$M$ ableitbar ist.
#### Erklärung:
Die Vollständigkeit besagt, dass alle semantisch gültigen Aussagen auch im Kalkül ableitbar sind. Das bedeutet, wenn$A$ eine logische Folge von$M$ ist, dann kann man dies auch im Sequenzenkalkül$S$ beweisen. Das stellt sicher, dass der Kalkül stark genug ist, um alle wahren Aussagen abzuleiten.
#### Beispiel:
Angenommen,$M = \{P \rightarrow Q, P\}$ und$A = Q$.

- **Semantische Gültigkeit:** Wenn in jeder Interpretation, in der$P \rightarrow Q$ und$P$ wahr sind, auch$Q$ wahr ist, muss dies auch formell im Kalkül ableitbar sein.

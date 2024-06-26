## Definition (Grundinstanzen)
### Gegeben
Eine geschlossene Formel $A$ hat die Form:
$$A := \forall x_1 \; \forall x_2 \; \cdots \; \forall x_n \; B$$
wobei $B$ eine quantorenfreie Formel ist. Das bedeutet, $A$ ist eine universell quantifizierte Formel, bei der alle Variablen $x_1, x_2, \ldots, x_n$ durch Allquantoren gebunden sind, und $B$ enthält keine Quantoren mehr.
### Grundinstanz
Eine Grundinstanz einer Formel $A$ ist eine Formel, die durch Ersetzen der Variablen $x_1, x_2, \ldots, x_n$ mit Grundtermen $t_1, t_2, \ldots, t_n$ in der quantorenfreien Matrix $B$ entsteht. Ein Grundterm ist ein term ohne Variablen, also ein spezifischer Wert oder eine Konstante.
Formell ausgedrückt ist eine Grundinstanz:
$$\{ x_1 / t_1, \; x_2 / t_2, \; \cdots, \; x_n / t_n \} (B)$$
Das bedeutet:
- Wir ersetzen $x_1$ durch $t_1$, $x_2$ durch $t_2$, und so weiter, in der Formel $B$.
- Die resultierende Formel ist frei von Quantoren und enthält keine Variablen mehr, nur noch die Grundterme.
### Grundinstanzen von Mengen geschlossener, universell quantifizierter Formeln
Wenn $M$ eine Menge geschlossener, universell quantifizierter Formeln ist, dann ist:
$$\text{Grundinstanzen}(M)$$die Menge aller Grundinstanzen aller Formeln in $M$.
Das bedeutet, für jede Formel in $M$ wird eine Menge von Grundinstanzen erzeugt, indem alle möglichen Kombinationen von Grundtermen für die Variablen in den quantorenfreien Teil der Formel eingesetzt werden.
### Beispiel zur Veranschaulichung
#### Beispiel 1: Einfache universell quantifizierte Formel
Nehmen wir an, wir haben die Formel:
$$\forall x \; P(x)$$
- Die quantorenfreie Matrix $B$ ist $P(x)$.
- Eine mögliche Grundinstanz wäre:
  - Ersetze $x$ durch einen Grundterm $t_1$, z.B. $a$:
  $$P(a)$$
Diese Grundinstanz $P(a)$ ist eine spezifische Aussage, die keinen Quantor mehr enthält und nur den Grundterm $a$.
#### Beispiel 2: Komplexere universell quantifizierte Formel
Betrachten wir die Formel:
$$\forall x \; \forall y \; (P(x, y) \rightarrow Q(y))$$
- Die quantorenfreie Matrix $B$ ist $P(x, y) \rightarrow Q(y)$.
- Mögliche Grundinstanzen könnten sein:
  - Ersetze $x$ durch $a$ und $y$ durch $b$:
  $$P(a, b) \rightarrow Q(b)$$
  - Ersetze $x$ durch $c$ und $y$ durch $d$:
  $$P(c, d) \rightarrow Q(d)$$
Diese Grundinstanzen sind spezifische Aussagen, die keinen Quantor mehr enthalten und nur die Grundterme $a$, $b$, $c$, und $d$ verwenden.


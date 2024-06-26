- Eine Formel ist in disjunktiver Normalform (DNF), wenn sie Disjunktion von Konjunktionen von [[Literalen]] ist.
- Eine Formel ist in DNF, wenn sie als Disjunktion (logisches „Oder“) von Konjunktionen (logisches „Und“) von Literalen geschrieben ist. Beispielsweise:$$(P \land \neg Q) \lor (Q \land R) \lor (\neg P \land R)$$
- Jede Teilkonjunktion ist eine **Minterm**.

## Wahrheitstafel und Normalformen
- **Direktes Ablesen der DNF**: 
  - Jede Zeile, in der die Formel wahr ist, entspricht einem Minterm.
  - Diese Minterme werden dann zu einer disjunktiven Normalform kombiniert.

- **Direktes Ablesen der KNF**:
  - Jede Zeile, in der die Formel falsch ist, entspricht einem Maxterm (Negation).
  - Diese Maxterme werden dann zu einer konjunktiven Normalform kombiniert.

### Beispiel für die Herstellung der Normalformen
Nehmen wir eine logische Formel: $$A = (P \land \neg Q) \lor R$$
#### Wahrheitstafel für $A$

| \( P \) | \( Q \) | \( R \) | \( A \)   |
|--------|--------|--------|---------|
| T      | T      | T      | T       |
| T      | T      | F      | F       |
| T      | F      | T      | T       |
| T      | F      | F      | T       |
| F      | T      | T      | T       |
| F      | T      | F      | F       |
| F      | F      | T      | T       |
| F      | F      | F      | F       |

**DNF aus der Wahrheitstafel**:
- $(P \land \neg Q \land \neg R)$ 
- $(P \land \neg Q \land R)$
- $(P \land Q \land R)$
- $(\neg P \land \neg Q \land R)$
- $(\neg P \land Q \land R)$ 
Diese werden kombiniert zu:
$$A = (P \land \neg Q \land \neg R) \lor (P \land \neg Q \land R) \lor (P \land Q \land R) \lor (\neg P \land \neg Q \land R) \lor (\neg P \land Q \land R)$$ 

Lass uns diese Konzepte Schritt für Schritt durchgehen. Sie betreffen spezifische Eigenschaften und Kriterien für disjunktive Normalformen (DNF) in der Logik.

## Vollständige disjunktive Normalform
> [!abstract] Definition
> Eine disjunktive Normalform (DNF) der Form $D = \bigvee_{K \in K} K$ in einer Signatur $\Sigma$ heißt **vollständig**, wenn für jede [[Klausel]] $K \in K$ und jede atomare Aussage $P \in \Sigma$ entweder das Literal $P$ oder das Literal $\neg P$ in $K$ vorkommt. 
### Beispiel:
- Betrachten wir eine Formel mit der Signatur $\Sigma = \{P, Q\}$:$$D = (P \land \neg Q) \lor (\neg P \land Q) \lor (P \land Q) \lor (\neg P \land \neg Q)$$
- Hier ist die DNF vollständig, weil jede [[Klausel]] $K$ für jede atomare Aussage $P$ und $Q$ entweder $P$ oder $\neg P$ und $Q$ oder $\neg Q$ enthält.
### Bedeutung:
- Eine vollständige DNF stellt sicher, dass alle möglichen Kombinationen der atomaren Aussagen in der Signatur berücksichtigt werden, was wichtig für die eindeutige Charakterisierung der Formel ist.
## Eindeutigkeit von vollständigen Normalformen
- Vollständige disjunktive Normalformen sind eindeutig bis auf die Reihenfolge der [[Klausel|Klauseln]].
- **Eindeutigkeit**: Zwei vollständige DNFs, die dieselbe logische Aussage darstellen, sind bis auf die Reihenfolge der Klauseln identisch.
- **Umordnung**: Die Reihenfolge, in der die Klauseln aufgelistet sind, kann variieren, ohne die logische Bedeutung der Formel zu ändern.
## Minimale disjunktive Normalform
> [!abstract] Definition
> Eine disjunktive Normalform $D = \bigvee_{K \in K} K$ heißt **minimal**, wenn es keine kürzere disjunktive Normalform $D'$ gibt, die logisch äquivalent zu $D$ ist. 

- **Kürzere Formel**: Eine Formel $D' = \bigvee_{K' \in K'} K'$ ist kürzer als $D$, wenn für jede [[Klausel]] $K'$ in $D'$ eine [[Klausel]] $K$ in $D$ existiert, sodass $K'$ eine Teilmenge (Teilformel) von $K$ ist.
### Beispiel
- Nehmen wir eine DNF:
$$D = (P \land \neg Q) \lor (P \land Q)$$
- Hier könnte eine kürzere Formel $D'$ sein:$$D' = P$$

- Da $P$ sowohl $(P \land \neg Q)$ als auch $(P \land Q)$ impliziert, ist $D'$ kürzer und logisch äquivalent zu $D$.
### Bedeutung
- Eine minimale DNF enthält keine überflüssigen Klauseln oder Literale. Sie ist die einfachste Form, die immer noch dieselbe logische Aussage trifft.
## Nicht-Eindeutigkeit minimaler Normalformen
> [!abstract] Definition
> Minimale disjunktive und konjunktive Normalformen einer Formel sind nicht eindeutig. 

- -> Es kann mehrere verschiedene minimale Normalformen für dieselbe logische Formel geben, die zwar unterschiedlich aussehen, aber logisch äquivalent sind.
### Beispiel
- Für eine Formel:$$F = P \land Q \lor \neg P \land R$$
- Könnten zwei verschiedene minimale DNFs sein:
	1. $D_1 = (P \land Q) \lor (\neg P \land R)$
	2. $D_2 = (\neg P \land R) \lor (P \land Q)$
- -> Beide sind minimal, weil sie keine überflüssigen Klauseln enthalten, aber sie sind nicht eindeutig.
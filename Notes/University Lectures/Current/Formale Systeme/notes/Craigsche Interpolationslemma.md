Seien $A$ und $B$ zwei aussagenlogische Formeln, und es gelte $A \models B$. Das bedeutet, dass$A$ die Aussage $B$ impliziert (wenn $A$ wahr ist, dann ist auch $B$ wahr). 

Das Lemma besagt, dass es eine Formel $C$ gibt, die zwischen $A$ und $B$ „vermittelt“, so dass:
1. $A \models C$ (wenn $A$ wahr ist, dann ist auch $C$ wahr)
2. $C \models B$ (wenn $C$ wahr ist, dann ist auch $B$ wahr)

Zusätzlich hat die Formel $C$ nur solche Variablen (auch **Atome** genannt), die sowohl in $A$ als auch in $B$ vorkommen. 
## Beispiele
#### Beispiel 1: $P_1 \land P_2 \rightarrow P_1 \lor P_3$
- **Formeln**:
  - $A = P_1 \land P_2$
  - $B = P_1 \lor P_3$
- **Implikation**: $P_1 \land P_2 \models P_1 \lor P_3$
- **Interpolante $C$**: 
  - Eine mögliche Interpolante ist $P_1$, da $P_1$ sowohl in $A$ als auch in $B$ vorkommt.
  - Hier ist $P_1 \models P_1 \lor P_3$ weil $P_1$ wahr ist, ist $P_1 \lor P_3$ auch wahr.
  - Und $P_1 \land P_2 \models P_1$ wenn $P_1 \land P_2$ wahr ist, dann ist auch $P_1$ wahr.
#### Beispiel 2:  $P_1 \land P_2 \rightarrow P_1$
- Formeln:
  -  $A = P_1 \land P_2$
  -  $B = P_1$
- **Interpolante $C$** :
  - Eine mögliche Interpolante ist $P_1$, da $P_1$ sowohl in $A$ als auch in $B$ vorkommt.
  - Hier ist $P_1 \land P_2 \models P_1$  und $P_1 \models P_1$

#### Beispiel 3:  $P_1 \rightarrow P_1 \lor P_3$
- Formeln:
  -  $A = P_1$
  -  $B = P_1 \lor P_3$ 
- **Interpolante  $C$** :
  - Eine mögliche Interpolante ist $P_1$, da $P_1$ sowohl in $A$ als auch in B vorkommt.
  - Hier ist $P_1 \models P_1 \lor P_3$ und $P_1 \models P_1$
## Konstruktion
### Gegeben:
- Formeln$A$ und $B$: Wir haben zwei aussagenlogische Formeln $A$ und $B$, wobei $A \models B$.
- Atome $P_1, \ldots, P_n$: Dies sind alle in $A$ vorkommenden aussagenlogischen Atome, die nicht in $B$ vorkommen.
### Ziel:
Wir wollen eine Formel $C$ finden, die nur solche Atome enthält, die sowohl in$A$als auch in$B$vorkommen und für die $A \models C$ und $C \models B$ gilt.
### Konstruktion:
1. Ersetzen der Atome:
   - Wir betrachten alle Kombinationen der Atome $P_1, \ldots, P_n$, die in $A$ vorkommen, aber nicht in $B$.
   - Für jede Kombination von Wahrheitswerten $c_i \in \{1, 0\}$ ersetzen wir jedes Atom $P_i$ in $A$ durch $c_i$.
   - Diese Ersetzung erzeugt Formeln $A[c_1, \ldots, c_n]$, die Versionen von $A$ sind, bei denen die Atome $P_1, \ldots, P_n$ durch feste Werte ersetzt wurden.
2. Disjunktion:
   - Wir konstruieren $C$ als die Disjunktion (logisches Oder) aller dieser Formeln $A[c_1, \ldots, c_n]$:$$C \equiv \bigvee_{(c_1, \ldots, c_n) \in \{1, 0\}^n} A[c_1, \ldots, c_n]$$
   - Das bedeutet, $C$ ist wahr, wenn mindestens eine der Formeln $A[c_1, \ldots, c_n]$ wahr ist.
### Beispiel
Angenommen, wir haben $A = P_1 \land P_2 \land Q$ und $B = Q \lor R$. Die Atome $P_1$ und $P_2$ kommen in $A$, aber nicht in $B$ vor.
- Wir betrachten die Kombinationen von $P_1$ und $P_2$:
	- $(1, 1)$
	- $(1, 0)$
	- $(0, 1)$
	- $(0, 0)$
- Wir ersetzen diese in $A$:
	- $A[1, 1] = 1 \land 1 \land Q = Q$
	- $A[1, 0] = 1 \land 0 \land Q = 0$
	- $A[0, 1] = 0 \land 1 \land Q = 0$
	- $A[0, 0] = 0 \land 0 \land Q = 0$
- Die Formel $C$ ist:
  $$C = Q \lor 0 \lor 0 \lor 0 = Q$$

### Korrektheit der Konstruktion (nicht in den Folien)
Der Beweis, warum diese Konstruktion korrekt ist, folgt typischerweise aus der Eigenschaft der Implikation und der Tatsache, dass $C$ nur solche Atome enthält, die in beiden Formeln vorkommen:
1. $A \models C$:
   - $A \models C$, weil $C$ aus $A$ durch Ersetzen der nicht relevanten Atome durch alle möglichen Wahrheitswerte entsteht. Das heißt, jede mögliche Belegung der$P_i$führt zu einer Formel, die Teil von $C$ ist.
2. $C \models B$:
   - $C \models B$, weil jede der Formeln $A[c_1, \ldots, c_n]$, die Teil von $C$ ist, bereits eine Version von $A$ ist, die $B$ impliziert.

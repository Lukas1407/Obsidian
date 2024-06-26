> [!abstract] Definition
> Eine Funktion $f$ von $\{W, F\}^n$ nach $\{W, F\}$ für $n \in \mathbb{N}$ heißt eine $n$-stellige Boolesche Funktion. 

## Erklärung:
- Eine Boolesche Funktion nimmt $n$ Wahrheitswerte als Eingabe und gibt einen Wahrheitswert als Ausgabe zurück.
- $\{W, F\}^n$ bedeutet, dass die Eingaben Tupel aus $n$ Wahrheitswerten sind.
- Das Ergebnis ist ein einzelner Wahrheitswert \( $W$ \) oder \( $F$ \).

## Beispiele:
- Eine 1-stellige Boolesche Funktion könnte die Negation sein: $f(\{W\}) = F$ und $f(\{F\}) = W$
- Eine 2-stellige Boolesche Funktion könnte die logische Konjunktion (UND) sein: $f(\{W, W\}) = W$, $f(\{W, F\}) = F$, $f(\{F, W\}) = F$, $f(\{F, F\}) = F$

## Boolesche Funktion einer Formel
- Sei $A \in \text{For}^0_\Sigma$, $\Sigma = \{P_1, \ldots, P_n\}$, dann wird die Boolesche Funktion$f_A$ von $A$ definiert durch:$$  f_A(\bar{b}) = \text{val}_I(A)$$wobei $\bar{b} \in \{W, F\}^n$ und $I(P_i) = b_i$

### Erklärung
- $A$ ist eine Formel über der Signatur $\Sigma$
- $\bar{b} = (b_1, b_2, \ldots, b_n)$ ist ein Tupel von Wahrheitswerten, das $n$ Elemente enthält.
- $I$ ist eine Interpretation, die jeder atomaren Aussage $P_{i}$ einen Wahrheitswert $b_{i}$ zuordnet.
- Die Boolesche Funktion $f_A$ nimmt also $n$ Wahrheitswerte als Eingabe und gibt den Wahrheitswert der Formel $A$ unter der Interpretation $I$ zurück.
### Beispiel
![[Pasted image 20240611171601.png#invert|500]]
## Satz der Funktionalen Vollständigkeit
> [!note] Funktionale Vollstandigkeit
>  Zu jeder Boolesche Funktion $f:{W,F}\rightarrow{W,F}$ gibt es eine Formel $A\in\text{For}^0_\Sigma$ mit $$f=f_{A}$$

- $f_A$: Die Funktion, die durch die Formel $A$ repräsentiert wird.
### Beweis
#### Schritt 1: Identifizierung der Tupel
- Sei $b_1, \ldots, b_k$ die n-Tupel aus $\{W, F\}^n$, für die $f(b_i) = W$
- Das heißt, $f$ gibt $W$ genau für diese Tupel zurück.
#### Schritt 2: Definition der Formel $A$
- Wir konstruieren die Formel $A$ als Disjunktion (logisches Oder) von Teilformeln $A_i$:$$A = A_1 \lor A_2 \lor \ldots \lor A_k$$
  
- Jede Teilformel $A_i$ ist eine Konjunktion (logisches Und) von [[Literalen]] $A_{i,1}, \ldots, A_{i,n}$:$$A_i = A_{i,1} \land A_{i,2} \land \ldots \land A_{i,n}$$ 
- Jedes Literal $A_{i,j}$ hängt davon ab, ob die $j$-te Komponente des Tupels $b_i$ wahr oder falsch ist:
  - Wenn die $j$-te Komponente $b_{i,j} = W$ ist, dann $A_{i,j} = P_j$
  - Wenn die $j$-te Komponente $b_{i,j} = F$ ist, dann $A_{i,j} = \neg P_j$
### Beispiel zur Veranschaulichung
Nehmen wir an, wir haben eine Boolesche Funktion $f$, die für $n=2$ wie folgt definiert ist:
$$f( W, F ) = W$$
$$f( F, W ) = W$$
$$f( W, W ) = F$$
$$f( F, F ) = F$$

- Die Tupel, für die $f$ wahr ist, sind:
  - $b_1 = (W, F)$
  - $b_2 = (F, W)$
- Die entsprechenden Formeln $A_1$ und $A_2$ sind:$$A_1 = P_1 \land \neg P_2$$$$A_2 = \neg P_1 \land P_2$$

- Die endgültige Formel $A$ ist:$$A = (P_1 \land \neg P_2) \lor (\neg P_1 \land P_2)$$
## Basis
> [!abstract] Definition
> Eine Menge $\text{KOp}$ von aussagenlogischen Konstanten und Operatoren heißt eine Basis, wenn für jede Boolesche Funktion $f : \{W, F\}^n \rightarrow \{W, F\}$) eine Formel $A$ existiert, die nur mit Konstanten und Operatoren aus $\text{KOp}$ aufgebaut ist, mit $f_A = f$ 

Eine Menge $\text{KOp}$ ist eine Sammlung von:
- **Aussagenlogischen Konstanten**: Dies sind feste Werte wie „wahr“ (W) und „falsch“ (F).
- **Operatoren**: Logische Verknüpfungen wie UND ($∧$), ODER ($∨$), NICHT ($¬$), etc.
#### Was bedeutet Basis?
Eine Menge $\text{KOp}$ wird als **Basis** bezeichnet, wenn sie ausreicht, um jede Boolesche Funktion $f$ mit Hilfe einer Formel $A$ darzustellen. Das bedeutet, dass man durch Kombination der Konstanten und Operatoren in $\text{KOp}$ jede denkbare logische Funktion nachbilden kann.
#### Beispiel einer Basis
Eine typische Basis für die Aussagenlogik ist die Menge der Operatoren:$$\{ \neg, \land, \lor \}$$Mit diesen Operatoren kann man jede Boolesche Funktion darstellen. Zum Beispiel:
#### Keine Minimalität gefordert
Der Satz sagt auch, dass keine **Minimalität** für $\text{KOp}$ gefordert wird. Das bedeutet, die Menge $\text{KOp}$ muss nicht minimal sein, um als Basis zu gelten. Sie könnte zusätzliche, nicht unbedingt notwendige Operatoren enthalten. 

> [!abstract] Definition
> Ein **Shannon-Graph** ist ein spezieller gerichteter, binärer und zusammenhängender Graph, der eine strukturierte und effiziente Methode zur Darstellung von Booleschen Funktionen bietet. Der Shannon-Graph verwendet Shannon-Formeln und repräsentiert diese als Graphen. 

![[Pasted image 20240612084335.png#invert|400]]

## Eigenschaften und Struktur eines Shannon-Graphen
1. **Gerichteter, binärer, zusammenhängender Graph**:
   - Der Graph ist **gerichtet**, d. h., die Kanten haben eine Richtung.
   - Der Graph ist **binär**, d. h., jeder Knoten hat höchstens zwei ausgehende Kanten.
   - Der Graph ist **zusammenhängend**, d. h., es gibt einen Pfad zwischen jedem Paar von Knoten.
2. **Knoten**:
   - Es gibt zwei Arten von Knoten:
     - **Nichtterminale Knoten**: Diese Knoten haben zwei ausgehende Kanten und sind mit einer natürlichen Zahl, dem Index, versehen.
     - **Terminale Knoten**: Diese Knoten haben keine ausgehenden Kanten und sind mit einem Wert versehen, entweder 0 (entspricht „falsch“) oder 1 (entspricht „wahr“).
3. **Kanten**:
   - Von jedem nichtterminalen Knoten gehen genau zwei Kanten aus:
     - Eine Kante ist mit 0 gekennzeichnet und repräsentiert den Fall, wenn die entsprechende Variable „falsch“ ist.
     - Eine Kante ist mit 1 gekennzeichnet und repräsentiert den Fall, wenn die entsprechende Variable „wahr“ ist.
4. **Indexregel**:
   - Der **Index** eines nichtterminalen Knotens ist eine natürliche Zahl $\text{index}(v)$.
   - Für jeden nichtterminalen Knoten $v$ gilt, dass der Index $\text{index}(v)$ kleiner ist als der Index jedes seiner unmittelbaren Nachfolger $w$, d. h., $\text{index}(v) < \text{index}(w)$.
   - Dies stellt sicher, dass die Variablen in einer festen Reihenfolge geprüft werden.
5. **Wurzelknoten**:
   - Es gibt genau einen **Wurzelknoten**, der den Ausgangspunkt des Graphen bildet.
## Wertzuweisung in Shannon-Graphen
- **Terminale Knoten**:
  - Jeder terminale Knoten ist mit einem Wert versehen:
    - $\text{wert}(0) = \text{F}$ (falsch)
    - $\text{wert}(1) = \text{W}$ (wahr)
- **Nichtterminale Knoten**:
  - Ein nichtterminaler Knoten $v$ repräsentiert eine Variable $P_{\text{index}(v)}$.
  - Die Kante, die mit 0 gekennzeichnet ist, führt zum Knoten für den Fall, dass $P_{\text{index}(v)}$ „falsch“ ist.
  - Die Kante, die mit 1 gekennzeichnet ist, führt zum Knoten für den Fall, dass $P_{\text{index}(v)}$ „wahr“ ist.
### Pfad im Shannon-Graphen
![[Pasted image 20240612084722.png#invert|400]]
## Beispiel einer Funktion $f(P_{1},P_{2},P_{3})$
![[Pasted image 20240612085145.png#invert|600]]
## Korrespondenz zwischen Shannon-Graphen und normierten Shannon-Formeln
1. **Variablen als Knoten**:
   - Jede Variable $P_i$ in einer normierten Shannon-Formel entspricht einem Knoten im Shannon-Graphen mit dem Index $i$.
   - Der Knoten $P_i$ hat zwei ausgehende Kanten, eine für den Fall $P_i = \text{F}$ und eine für den Fall $P_i = \text{W}$.
2. **Logische Struktur**:
   - Die rekursive Struktur der normierten Shannon-Formeln spiegelt sich in der Baumstruktur der Shannon-Graphen wider.
   - Der Operator $\text{sh}(P_i, A, B)$ entspricht einem Knoten $P_i$ mit zwei ausgehenden Kanten, die zu den Subgraphen für $A$ und $B$ führen.
3. **Terminale Knoten**:
   - Die terminalen Knoten in einem Shannon-Graphen (0 oder 1) entsprechen den logischen Endwerten der Booleschen Funktion in der normierten Shannon-Formel.
4. **Pfadwahl und Rekursion**:
   - <mark style="background: #FFB86CA6;">Die Wahl des Pfades in einem Shannon-Graphen entspricht der rekursiven Auflösung der normierten Shannon-Formel</mark>.
   - Jeder Pfad durch den Graphen entspricht einer Folge von logischen Entscheidungen, die in der normierten Shannon-Formel durch den $\text{sh}$-Operator dargestellt werden.

## Reduzierter Shannon Graph (ordered binary decision diagram: (O)BDD)
### Bedingungen für einen reduzierten Shannon-Graphen
1. **Isomorphie von Teilgraphen vermeiden:**
   - Ein reduzierter Shannon-Graph darf keine zwei Knoten $v$ und $w$ haben, bei denen der Teilgraph, der in $v$ verwurzelt ist, isomorph zu dem Teilgraphen ist, der in $w$ verwurzelt ist.
   - **Isomorphie** bedeutet hier, dass die Teilgraphen von $v$ und $w$ strukturell gleich sind, das heißt, sie haben dieselben Verbindungen und Anordnungen der Knoten, nur die Namen der Knoten könnten unterschiedlich sein.
   - **Frage**: Warum denkst du, ist es wichtig, dass zwei Teilgraphen nicht isomorph sind?
2. **Unterschiedliche Nachfolgerknoten:**
   - Es darf keinen Knoten $v$ geben, von dem aus zwei Kanten zu demselben Nachfolgerknoten führen.
   - Diese Bedingung stellt sicher, dass jede Entscheidung, die vom Knoten $v$ ausgeht, zu einer eindeutigen und unterscheidbaren Verzweigung im Graphen führt.
   - **Frage**: Was könnte passieren, wenn von einem Knoten zwei Kanten zu demselben Nachfolger führen?
### Was bedeutet das für den Graphen?
- Ein reduzierter Shannon-Graph ist sehr effizient, da er keine überflüssigen oder redundanten Strukturen enthält.
- Er stellt eine klare und eindeutige Darstellung von Entscheidungsprozessen dar, was besonders bei der Implementierung von Algorithmen und Entscheidungsbäumen hilfreich ist.
### Beispiel
![[reduzierter_shannon_graph-ezgif.com-speed.gif#invert|600]]

## Isomorphie von Shannon Graphen
> [!abstract] Definition
> Zwei Shannon-Graphen $H$ und $G$ sind isomorph, wenn es eine bijektive Abbildung $\pi$ von der Knotenmengen $V1$ (von $H$) nach $V2$ (von $G$) gibt. Das bedeutet, dass $\pi$ eine Eins-zu-eins-Zuordnung zwischen den Knoten der beiden Graphen ist. Die Isomorphie-Bedingungen sind wie folgt: 
1. **Index-Gleichheit für Nichtterminalknoten:**
   - $\text{index}(k) = \text{index}(\pi(k))$ für jeden Nichtterminalknoten $k \in V1$.
   - Das bedeutet, dass der Index jedes Nichtterminalknotens $k$ in $H$ gleich dem Index des entsprechenden Knotens $\pi(k)$ in $G$ sein muss. Der Index könnte beispielsweise eine Variablenzuordnung oder eine bestimmte Kennzeichnung im Graphen sein.
2. **Wert-Gleichheit für Terminalknoten:**
   - $\text{wert}(k) = \text{wert}(\pi(k))$ für jeden Terminalknoten $k \in V1$.
   - Jeder Terminalknoten $k$ in $H$ muss denselben Wert wie sein zugehöriger Knoten $\pi(k)$ in $G$ haben. Terminalknoten repräsentieren oft Endzustände oder Ergebnisse im Graphen.
3. **Erhaltung der Kantenstruktur:**
   - Für jeden Nichtterminalknoten $k \in V1$, dessen 0-Kante/1-Kante zu dem Knoten $k_0/k_1$ führt, gilt: die 0-Kante von $\pi(k)$ führt zu $\pi(k_0)$, die 1-Kante zu $\pi(k_1)$.
   - Das bedeutet, dass die Verbindungen (Kanten) zwischen den Knoten in $H$ und $G$ entsprechend den Abbildungen $\pi$ gleich bleiben. Wenn $k$ in $H$ eine Kante zu $k_0$ und $k_1$ hat, dann muss der entsprechende Knoten $\pi(k)$ in $G$ dieselben Kantenverbindungen zu $\pi(k_0)$ und $\pi(k_1)$ aufweisen.

## Theorem zur Reduziertheit
### Voraussetzungen des Theorems
1. **Gleiche Indizes und Nachfolger bei Nichtterminalknoten:**
   - Angenommen, zwei Nichtterminalknoten $v$ und $w$ haben denselben Index (d.h. sie repräsentieren dieselbe Entscheidung oder Variable).
   - Wenn die 1-Nachfolger von $v$ und $w$ identisch sind und die 0-Nachfolger von $v$ und $w$ ebenfalls identisch sind, dann sind $v$ und $w$ tatsächlich derselbe Knoten, also $v = w$.
2. **Gleiche Werte bei Terminalknoten:**
   - Wenn zwei Terminalknoten $u$ und $s$ denselben Wert haben (d.h. sie repräsentieren dasselbe Ergebnis), dann sind $u$ und $s$ tatsächlich derselbe Knoten, also $u = s$.
### Bedingung für die Reduziertheit
- Das Theorem besagt, dass unter den genannten Voraussetzungen der Graph $G$ die Bedingung (1) für die Definition eines reduzierten Shannon-Graphen erfüllt:
  - **Bedingung (1):** Es gibt keine zwei Knoten $x$ und $y$ (mit $x \ne y$), bei denen die in $x$ und $y$ verwurzelten Teilgraphen $G_x$ und $G_y$ isomorph sind.
### Implikation des Theorems
- **Einzigartigkeit der Knoten:**
  - Die Implikation ist, dass in einem reduzierten Shannon-Graphen jeder Knoten eindeutig durch seine Struktur bestimmt ist. Es gibt keine zwei verschiedenen Knoten, die dieselbe Struktur oder dieselben Nachfolger haben.
### Bedeutung des Theorems
- **Vermeidung von Redundanz:**
  - Das Theorem stellt sicher, dass der Graph keine redundanten Knoten enthält, die die gleiche Rolle oder Funktion im Graphen einnehmen. Dies führt zu einer effizienteren und klareren Darstellung des Entscheidungsprozesses.
- **Eindeutigkeit der Struktur:**
  - Es wird sichergestellt, dass jede Entscheidung oder jedes Ergebnis im Graphen durch genau einen Knoten dargestellt wird, was die Interpretation und Analyse des Graphen erleichtert.
### Beispiel
![[reduzier_2-ezgif.com-speed.gif#invert|600]]



## Theorem: Eindeutigkeit reduzierter Shannon-Graphen
### Aussage des Theorems
1. **Gleiche Boolesche Funktion, gleiche Graphstruktur:**
   - Wenn $G$ und $H$ reduzierte Shannon-Graphen über dem Alphabet $\Sigma = \{P_1, ..., P_n\}$ sind und sie dieselbe Boolesche Funktion $f$ darstellen ($f_G = f_H$), dann sind $G$ und $H$ isomorph ($G \sim= H$).
   - Das bedeutet, dass $G$ und $H$ zwar unterschiedlich aussehen können, aber strukturell identisch sind, wenn sie dieselbe Funktion repräsentieren.
2. **Eindeutigkeit des reduzierten Shannon-Graphen:**
   - Für jede Boolesche Funktion $f$ existiert bis auf Isomorphie genau ein reduzierter Shannon-Graph $H$, der diese Funktion $f$ darstellt ($f = f_H$).
   - Das bedeutet, dass jede Boolesche Funktion eine eindeutige minimale Graphrepräsentation hat, wenn man redundante Knoten und Kanten entfernt.
### Bedeutung des Theorems
- **Eindeutige Repräsentation:**
  - Das Theorem garantiert, dass jede Boolesche Funktion durch genau einen reduzierten Shannon-Graphen repräsentiert werden kann, wodurch die Darstellung von Funktionen konsistent und eindeutig wird.
- **Reduktion auf Isomorphie:**
  - Auch wenn zwei reduzierte Shannon-Graphen unterschiedlich aussehen können, sind sie isomorph, wenn sie dieselbe Funktion darstellen. Das bedeutet, dass sie durch eine Umbenennung der Knoten in denselben Graphen überführt werden können.
- **Effizienz und Klarheit:**
  - Da jede Funktion eine eindeutige Graphstruktur hat, können Optimierungsprozesse und Vergleiche effizient durchgeführt werden, ohne dass redundante Strukturen berücksichtigt werden müssen.
### Beispiel: Unterschiedliche Darstellungen derselben Funktion
- **Boolesche Funktion $f(x) = \neg x$**: Die Funktion gibt den negativen Wert von \$x$ zurück.
- **Graph $G$**: Ein reduzierter Shannon-Graph für diese Funktion könnte so aussehen:
  ```
    x
   / \
  1   0
  ```
- **Graph $H$: Ein anderer Graph für dieselbe Funktion könnte so aussehen:
  ```
     x
    / \
   0   1
  ```
  Aber dieser Graph ist nicht korrekt reduziert, da er die falsche Funktion darstellt. Nur der erste Graph $G$ stellt $\neg x$ korrekt dar.

## Auswirkungen der Variablenordnung auf die BDD-Größe
![[Pasted image 20240612091243.png#invert|400]]
1. **Optimierung durch passende Variablenordnung:**
   - Die linke Ordnung $x_1 < x_2 < x_3 < x_4 < x_5 < x_6 < x_7 < x_8$ führt zu einem kleineren und effizienteren BDD.
   - Die rechte Ordnung $x_1 < x_3 < x_5 < x_7 < x_2 < x_4 < x_6 < x_8$ führt zu einem größeren BDD, was mehr Speicherplatz und Rechenleistung erfordert.
2. **Wichtige Erkenntnis:**
   - Eine geschickte Wahl der Variablenordnung kann die Größe eines BDDs erheblich reduzieren und damit die Effizienz bei der Verarbeitung und Analyse von Booleschen Funktionen erhöhen.
-> Die Wahl der Variablenordnung hat einen erheblichen Einfluss auf die Größe und Komplexität eines BDDs. Durch eine geeignete Ordnung kann man die Effizienz bei der Darstellung und Verarbeitung Boolescher Funktionen maximieren. Ein schlanker, gut strukturierter BDD ist leichter zu analysieren und erfordert weniger Speicherplatz und Rechenzeit.

## Ausgangssituation: BDDs für Multiplikation
### Variablen und Binärzahlen
- **Variablen $X$**: Das Set $X$ enthält $2k$ Variablen $\{x_0, \ldots, x_{k-1}, y_0, \ldots, y_{k-1}\}$
- **Binärzahlen $x$ und $y$**: Diese Variablen repräsentieren die Binärzahlen $x$ und $y$, wobei jede Zahl $k$-stellige Binärzahlen darstellt.
  - $x = x_0 x_1 \ldots x_{k-1}$
  - $y = y_0 y_1 \ldots y_{k-1}$
- **Funktion Multi**: Diese boolesche Funktion beschreibt das $i$-te Bit des Produkts von $x$ und $y$.
### Theorem zur Komplexität von BDDs
- **Theoremaussage**:
  - Für jede Ordnung $<$ der Variablen in $X$ existiert mindestens ein Index $0 \leq i < 2k$, so dass der BDD $B_{\text{Multi}, <}$ mindestens $2^{k/8}$ Knoten hat.
  - Das bedeutet, dass unabhängig davon, wie man die Variablen anordnet, immer mindestens ein BDD für die Multiplikation eine hohe Anzahl an Knoten besitzen wird, was die Darstellung und Verarbeitung komplex macht.
### Beispiel zur Verdeutlichung
#### Multiplikation zweier 2-Bit-Zahlen
- **Binärzahlen**: 
  - $x$ wird durch die Variablen $x_0$ und $x_1$ repräsentiert.
  - $y$ wird durch die Variablen $y_0$ und $y_1$ repräsentiert.
  - $x$ und $y$ sind jeweils 2-Bit-Zahlen.
- **Multiplikation**:
  - Das Produkt $P$ von $x$ und $y$ ist eine 4-Bit-Zahl: $P = x \cdot y = p_0 p_1 p_2 p_3$.
- **Darstellung des Produkts**:
  - $p_0$: Das niederwertigste Bit des Produkts.
  - $p_1$, $p_2$, $p_3$: Weitere Bits des Produkts.
#### BDD-Darstellung
- **BDD für $p_0$**:
  - Dies kann relativ einfach sein, da es nur von den niedrigsten Bits der beiden Zahlen abhängt.
- **BDD für $p_3$**:
  - Die Darstellung wird komplexer, da es von den höchsten Bits sowie von deren Kombinationen abhängt.
- **Komplexität**:
  - Selbst bei einer optimalen Variablenordnung ist es notwendig, eine hohe Anzahl an Knoten zu haben, um das korrekte Bit $p_i$ des Produkts darzustellen.
  - Der BDD für $p_3$ hat tendenziell mehr Knoten als der BDD für $p_0$.

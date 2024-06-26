- [[Deterministische endliche Automaten (DEA)]]
- [[Nichtdeterministische endliche Automaten (NEA)]]
- [[Vollständige endliche Automaten]]

### Spontane Übergänge

Ein endlicher Automat mit spontanen Übergängen erlaubt es dem Automaten, Zustandswechsel durchzuführen, ohne ein Eingabesymbol zu konsumieren. Diese Übergänge werden als $\varepsilon$-Übergänge bezeichnet.

#### Definition

Ein endlicher Automat mit spontanen Übergängen wird definiert durch:

1. **Endliche Menge von Zuständen (S)**:
   - $S$ ist die Menge der Zustände, die der Automat haben kann.

2. **Alphabet (V)**:
   - $V$ ist die Menge der terminalen Zeichen, die der Automat als Eingabe akzeptieren kann.

3. **Anfangszustand (s_0)**:
   - $s_0 \in S$ ist der Zustand, in dem der Automat startet.

4. **Menge von Endzuständen (F)**:
   - $F \subseteq S$ ist die Menge der Endzustände. Wenn der Automat in einem dieser Zustände endet, wird das Eingabewort akzeptiert.

5. **Übergangsfunktion (δ)**:
   - $\delta : S \times (V \cup \{\varepsilon\}) \rightarrow \mathcal{P}(S)$ ist eine Funktion, die für ein Paar aus Zustand und Eingabesymbol (einschließlich $\varepsilon$) eine Menge von Zuständen zurückgibt. Hierbei ist $\mathcal{P}(S)$ die Potenzmenge von $S$, also die Menge aller Teilmengen von $S$.

#### $\varepsilon$-Abschluss (ε-cl)

Der $\varepsilon$-Abschluss einer Menge von Zuständen ist die Menge aller Zustände, die durch beliebig viele $\varepsilon$-Übergänge von den Zuständen in der ursprünglichen Menge erreicht werden können.

Sei $A = (S, V, s_0, \delta, F)$ ein endlicher Automat mit spontanen Übergängen, dann ist die Funktion $\varepsilon$-cl definiert als:

$$
\varepsilon\text{-cl}(I) = \text{die kleinste Teilmenge } J \subseteq S \text{ mit }
$$

1. $I \subseteq J$
2. Für alle $s \in J$ gilt $\delta(s, \varepsilon) \subseteq J$.

#### Erweiterte Übergangsfunktion

Die erweiterte Übergangsfunktion $\delta : S \times (V \cup \{\varepsilon\}) \rightarrow \mathcal{P}(S)$ wird zu $\bar{\delta} : S \times V^* \rightarrow \mathcal{P}(S)$ wie folgt definiert:

1. Für das leere Wort $\varepsilon$:

$$
\bar{\delta}(s, \varepsilon) = \varepsilon\text{-cl}(\{s\})
$$

Das bedeutet, dass der $\varepsilon$-Abschluss des Anfangszustands alle Zustände umfasst, die durch $\varepsilon$-Übergänge vom Anfangszustand aus erreichbar sind.

2. Für ein Wort der Form $aw_1$ (wo $a$ das erste Zeichen ist und $w_1$ der Rest des Wortes):

$$
\bar{\delta}(s, aw_1) = \{s' \mid \text{es gibt } s_1, s_2 \text{ mit } s_1 \in \varepsilon\text{-cl}(\{s\}), s_2 \in \delta(s_1, a), s' \in \bar{\delta}(s_2, w_1)\}
$$

Das bedeutet, der Automat liest das erste Zeichen $a$, wechselt zu einem der Zustände $s_1$ im $\varepsilon$-Abschluss von $s$, verarbeitet dann das restliche Wort $w_1$ und bewegt sich durch Zustände $s_2$ entsprechend der Übergangsfunktion $\delta$.

#### Akzeptierte Sprache $L(NEA)$

Die von einem NEA mit spontanen Übergängen akzeptierte Sprache $L(NEA)$ ist die Menge der Wörter, für die es mindestens einen Verarbeitungsweg gibt, der das Wort zu einem Endzustand führt:

$$
L(NEA) = \{ w \in V^* \mid \bar{\delta}(s_0, w) \cap F \neq \emptyset \}
$$

Das bedeutet, $L(NEA)$ besteht aus allen Wörtern $w$ aus dem Alphabet $V$, für die gilt, dass der Automat, wenn er im Anfangszustand $s_0$ startet und das Wort $w$ liest, mindestens einen Endzustand in der Menge $F$ erreicht.

### Zusammenfassung

Ein endlicher Automat mit spontanen Übergängen (auch $\varepsilon$-Übergänge genannt) erlaubt Zustandswechsel ohne Eingabesymbole. Der $\varepsilon$-Abschluss ist eine wichtige Funktion, die alle Zustände umfasst, die durch $\varepsilon$-Übergänge erreicht werden können. Die erweiterte Übergangsfunktion berücksichtigt diese $\varepsilon$-Übergänge, um die akzeptierte Sprache zu definieren. Ein Wort wird akzeptiert, wenn mindestens ein Verarbeitungsweg das Wort zu einem Endzustand führt.

Natürlich, ich erkläre den Satz von Myhill und Büchi sowie die Operationen mit Wortmengen im Detail.

### Satz von Myhill und Büchi

#### Aussage des Satzes

Der Satz von Myhill und Büchi besagt, dass zu jedem nichtdeterministischen endlichen Automaten (NEA) ein äquivalenter deterministischer endlicher Automat (DEA) existiert, der dieselbe Sprache akzeptiert.

#### Formaler Satz

Zu jedem nichtdeterministischen endlichen Automaten $A = (S, V, s_0, \delta, F)$ gibt es einen deterministischen endlichen Automaten $B = (Q, V, q_0, \Delta, G)$ mit $L(A) = L(B)$.

Dabei kann $A$ spontane Übergänge ($\varepsilon$-Übergänge) enthalten und muss auch nicht vollständig sein.

#### Bedeutung

Das bedeutet, dass jede Sprache, die von einem NEA erkannt wird, auch von einem DEA erkannt werden kann. Der DEA ist in der Lage, die gleiche Sprache zu akzeptieren wie der NEA, obwohl der NEA nichtdeterministische Übergänge und spontane Übergänge verwenden kann.

#### Konstruktion des DEA

Die Konstruktion des äquivalenten DEA aus einem NEA erfolgt durch die Methode der **Potenzmengenkonstruktion** oder **Subset-Konstruktion**. Hier sind die Schritte grob umrissen:

1. **Zustandsmenge $Q$**: Die Zustände des DEA entsprechen den Teilmengen der Zustände des NEA. Wenn der NEA $S$ Zustände hat, dann hat der DEA $2^S$ Zustände.
2. **Alphabet $V$**: Das Alphabet bleibt gleich.
3. **Startzustand $q_0$**: Der Startzustand des DEA ist die $\varepsilon$-Abschlussmenge des Startzustands des NEA.
4. **Übergangsfunktion $\Delta$**: Für jede Teilmenge von Zuständen und jedes Symbol aus dem Alphabet wird die $\varepsilon$-Abschlussmenge der Zustände berechnet, zu denen der NEA wechseln kann.
5. **Endzustände $G$**: Eine Teilmenge von Zuständen ist ein Endzustand im DEA, wenn sie mindestens einen Endzustand des NEA enthält.

Diese Konstruktion stellt sicher, dass der DEA jede Sprache akzeptiert, die auch der NEA akzeptiert.

### Operationen mit Wortmengen

In der formalen Sprachtheorie können verschiedene Operationen auf Sprachen (Mengen von Wörtern) durchgeführt werden. Hier sind die wichtigsten Operationen:

1. **Konkatenation (L1L2)**:
   - Definition: $L1L2 = \{w1w2 \mid w1 \in L1, w2 \in L2\}$
   - Beschreibung: Die Sprache besteht aus allen Wörtern, die durch die Konkatenation eines Wortes aus $L1$ und eines Wortes aus $L2$ entstehen.

2. **Kleenesche Hülle (L\*)**:
   - Definition: $L\* = \{w1 w2 \ldots w_n \mid n \geq 0, w_i \in L\}$
   - Beschreibung: Die Sprache besteht aus allen möglichen Konkatenationen von null oder mehr Wörtern aus $L$. Dazu gehört auch das leere Wort $\varepsilon$, wenn $n = 0$.

3. **Vereinigung (L1 ∪ L2)**:
   - Definition: $L1 ∪ L2 = \{w \mid w \in L1 \text{ oder } w \in L2\}$
   - Beschreibung: Die Sprache besteht aus allen Wörtern, die entweder in $L1$ oder in $L2$ enthalten sind.

4. **Durchschnitt (L1 ∩ L2)**:
   - Definition: $L1 ∩ L2 = \{w \mid w \in L1 \text{ und } w \in L2\}$
   - Beschreibung: Die Sprache besteht aus allen Wörtern, die sowohl in $L1$ als auch in $L2$ enthalten sind.

5. **Differenz (L1 \ L2)**:
   - Definition: $L1 \ L2 = \{w \mid w \in L1 \text{ und } w \notin L2\}$
   - Beschreibung: Die Sprache besteht aus allen Wörtern, die in $L1$ enthalten, aber nicht in $L2$ enthalten sind.

Diese Operationen sind grundlegend für die Manipulation und Kombination von formalen Sprachen in der Theorie der Automaten und formalen Sprachen.

### Zusammenfassung

- **Satz von Myhill und Büchi**: Jeder NEA hat einen äquivalenten DEA, der dieselbe Sprache akzeptiert.
- **Operationen mit Wortmengen**: Diese umfassen Konkatenation, Kleenesche Hülle, Vereinigung, Durchschnitt und Differenz und sind grundlegende Werkzeuge in der formalen Sprachtheorie.


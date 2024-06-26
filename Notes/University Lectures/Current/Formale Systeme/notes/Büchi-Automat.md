Natürlich, lassen Sie uns Büchi-Automaten und die Beispiele aus den Bildern im Detail besprechen.

### Büchi-Automaten

Ein Büchi-Automat ist eine spezielle Art von nichtdeterministischem endlichen Automaten (NEA), der zum Erkennen von $\omega$-regulären Sprachen verwendet wird, also von Sprachen über unendlichen Wörtern.

#### Definition eines Büchi-Automaten

Ein Büchi-Automat $A$ ist ein 5-Tupel:
$$ A = (S, V, s_0, \delta, F) $$
wobei:
- $S$ eine endliche Menge von Zuständen ist,
- $V$ ein Alphabet ist,
- $s_0 \in S$ der Anfangszustand ist,
- $\delta : S \times V \rightarrow \mathcal{P}(S)$ die Übergangsfunktion ist,
- $F \subseteq S$ die Menge der Endzustände ist.

#### Berechnungsfolge (Run)

Für ein $\omega$-Wort $w \in V^\omega$ nennen wir eine Folge von Zuständen $s_0, s_1, s_2, \ldots$ eine Berechnungsfolge (Englisch "run") für $w$, wenn für alle $n \geq 0$ gilt:
$$ s_{n+1} \in \delta(s_n, w(n)) $$

#### Akzeptanzbedingung

Ein $\omega$-Wort $w$ wird von einem Büchi-Automaten $A$ akzeptiert, wenn es eine Berechnungsfolge $s_0, s_1, s_2, \ldots$ für $w$ gibt, die unendlich oft Zustände aus $F$ besucht.

Die von $A$ akzeptierte $\omega$-Sprache ist also:
$$ L_\omega(A) = \{ w \in V^\omega \mid \text{es gibt eine Berechnungsfolge für } w \text{ mit unendlich vielen Zuständen aus } F \} $$

### Unterschiede zu endlichen Automaten

Der Hauptunterschied zwischen Büchi-Automaten und normalen endlichen Automaten liegt in der Akzeptanzbedingung:
- **Normale endliche Automaten**: Ein Wort wird akzeptiert, wenn die Berechnungsfolge in einem Endzustand endet.
- **Büchi-Automaten**: Ein $\omega$-Wort wird akzeptiert, wenn die Berechnungsfolge unendlich oft durch Endzustände läuft.

### Beispiel 1
![[Pasted image 20240626110358.png#invert|400]]
#### Beschreibung

- Der Automat hat zwei Zustände. Der linke Zustand ist sowohl der Anfangszustand als auch ein Endzustand.
- Übergänge:
  - Vom Anfangszustand gibt es Selbstübergänge für $a$ und $b$.
  - Ein Übergang von $a$ führt zum rechten Zustand, der auch ein Endzustand ist.

#### Akzeptierte Sprache

Die akzeptierte Sprache ist:
$$ \{a, b\}^* a^\omega $$

Das bedeutet, der Automat akzeptiert alle unendlichen Wörter, die mit beliebigen Kombinationen von $a$ und $b$ beginnen und dann unendlich viele $a$ enthalten.

### Beispiel 2
![[Pasted image 20240626110416.png#invert|400]]
#### Beschreibung

- Der Automat hat zwei Zustände, $a$ und $b$, die beide Endzustände sind.
- Übergänge:
  - Vom Anfangszustand gibt es einen Übergang mit $a$ zu sich selbst und einen Übergang mit $b$ zum anderen Zustand.
  - Vom zweiten Zustand gibt es einen Übergang mit $b$ zu sich selbst und einen Übergang mit $a$ zurück zum ersten Zustand.

#### Akzeptierte Sprache

Die akzeptierte Sprache ist:
$$ (a^* b)^\omega $$
$$ \{ w \in \{a, b\}^\omega \mid b \text{ kommt unendlich oft vor in } w \} $$

Das bedeutet, der Automat akzeptiert alle unendlichen Wörter, in denen das Symbol $b$ unendlich oft vorkommt.

### Zusammenfassung

- **Büchi-Automaten** erkennen $\omega$-reguläre Sprachen.
- Ein Wort wird von einem Büchi-Automaten akzeptiert, wenn es eine Berechnungsfolge gibt, die unendlich oft durch Endzustände läuft.
- Beispiel 1 akzeptiert unendliche Wörter, die mit beliebigen Kombinationen von $a$ und $b$ beginnen und dann unendlich viele $a$ enthalten.
- Beispiel 2 akzeptiert unendliche Wörter, in denen das Symbol $b$ unendlich oft vorkommt.

### Entscheidbarkeit bei Büchi-Automaten

#### Fragestellung

Die Frage lautet, ob für einen Büchi-Automaten $B$ die Menge der akzeptierten Wörter nicht leer ist, also ob:
$$ L_\omega(B) \neq \emptyset $$

#### Beweis der Entscheidbarkeit

Um zu zeigen, dass $L_\omega(B) \neq \emptyset$ ist, muss man einen erreichbaren Endzustand $q_f \in F$ finden, der auf einer Schleife liegt. Der Beweis verläuft wie folgt:

1. **Erreichbarkeit**:
   - Man überprüft, ob der Endzustand $q_f \in F$ vom Startzustand $s_0$ aus erreichbar ist.
2. **Schleifenbildung**:
   - Man überprüft, ob es vom Endzustand $q_f$ aus einen Weg gibt, der wieder zu $q_f$ führt (d.h., es gibt eine Schleife).

Wenn ein solcher Endzustand $q_f$ gefunden wird, der eine Schleife bildet, bedeutet dies, dass es ein unendliches Wort gibt, das unendlich oft durch diesen Endzustand läuft, und somit $L_\omega(B) \neq \emptyset$.

### ω-reguläre Sprachen

Eine Menge $L$ von $\omega$-Wörtern heißt $\omega$-regulär, wenn es einen Büchi-Automaten $A$ gibt, so dass $L_\omega(A) = L$.

### Endliche und unendliche Akzeptanz

#### Lemma

Sei $A$ ein endlicher Automat und $K = L(A)$. Dann gilt:

1. $L_\omega(A) \subseteq K⃗$
2. Falls $A$ deterministisch ist, gilt sogar $L_\omega(A) = K⃗$

#### Beweis zu 1: $L_\omega(A) \subseteq K⃗$

Für $w \in L_\omega(A)$ gibt es eine Berechnungsfolge:
$$ \rho_w = s_0, s_1, \ldots, s_n, \ldots $$
so dass $F_w = \{ n \in \mathbb{N} \mid s_n \in F \}$ unendlich ist.

Für alle $n \in F_w$ gilt $s_n \in F$, also:
$$ w \downarrow (n) \in K $$

Daraus folgt:
$$ w \in K⃗ $$

#### Beweis zu 2: $L_\omega(A) = K⃗$ (für deterministische Automaten)

Für $w \in K⃗$ ist $R_w = \{ n \in \mathbb{N} \mid w \downarrow (n) \in K \}$ unendlich.

Für jedes $n \in R_w$ gibt es eine Berechnungsfolge:
$$ s_n = s_{n,1}, s_{n,2}, \ldots, s_{n,n} $$
für $w \downarrow (n)$.

Da $A$ deterministisch ist, ist für jedes Paar $n, m \in R_w$ mit $n < m$:
$$ s_n \text{ Anfangsstück von } s_m $$

Zusammengesetzt ergibt dies eine unendliche Berechnungsfolge $s$ für $w$, die unendlich oft einen Endzustand durchläuft:
$$ w \in L_\omega(A) $$

### Zusammenfassung

- **Entscheidbarkeit**: Es ist entscheidbar, ob die Menge der akzeptierten Wörter eines Büchi-Automaten nicht leer ist.
- **$\omega$-reguläre Sprachen**: Eine Sprache ist $\omega$-regulär, wenn sie von einem Büchi-Automaten akzeptiert wird.
- **Lemma über endliche und unendliche Akzeptanz**: Dieses Lemma zeigt die Beziehung zwischen den von einem Automaten akzeptierten endlichen und unendlichen Wörtern. Wenn der Automat deterministisch ist, entspricht die Menge der von ihm akzeptierten unendlichen Wörter genau der Menge der unendlichen Wörter, deren endliche Anfangsstücke unendlich oft in der Sprache des Automaten enthalten sind ($K⃗$).

Diese Konzepte sind grundlegend für das Verständnis der Theorie der formalen Sprachen und der Eigenschaften von Automaten bei der Erkennung unendlicher Wörter.

### Deterministische Büchi-Automaten

#### Korollar

Für eine $\omega$-Sprache $L \subseteq V^\omega$ sind äquivalent:
- $L = L_\omega(A)$ für einen deterministischen Büchi-Automaten $A$.
- Es gibt eine reguläre Sprache $K \subseteq V^*$ mit $L = K \⃗$.

#### Beweis

Dieser folgt direkt aus der Tatsache, dass für deterministische Automaten $A$ gilt:
$$ L_\omega(A) = \lim(L(A)) $$
(wie im vorherigen Lemma gezeigt).

### Beispielautomat $N_{bfin}$

#### Beschreibung

- **Zustände**: Der Automat hat zwei Zustände $s_0$ und $s_1$.
- **Startzustand**: $s_0$.
- **Endzustände**: $s_1$.
- **Übergänge**:
  - Vom Startzustand $s_0$ gibt es Übergänge nach $s_0$ für $a$ und $b$.
  - Es gibt einen Übergang von $s_0$ nach $s_1$ für $a$.
  - Vom Zustand $s_1$ gibt es einen Selbstübergang für $a$.

#### Akzeptierte Sprachen

1. **$\omega$-Sprache $L_\omega(N_{bfin})$**:
   $$ L_\omega(N_{bfin}) = \{ w \in \{a, b\}^\omega \mid b \text{ kommt nur endlich oft vor} \} $$
   Dies bedeutet, dass $w$ ein unendliches Wort ist, in dem $b$ nur endlich oft vorkommt, und danach nur noch $a$ vorkommt.

2. **Endliche Sprache $L(N_{bfin})$**:
   $$ L(N_{bfin}) = \{ w \in \{a, b\}^* \mid w \text{ endet auf } a \} $$
   Dies bedeutet, dass $w$ ein endliches Wort ist, das auf $a$ endet.

3. **Limes von $L(N_{bfin})$**:
   $$ \lim(L(N_{bfin})) = \{ w \in \{a, b\}^\omega \mid a \text{ kommt unendlich oft vor} \} $$
   Dies bedeutet, dass $w$ ein unendliches Wort ist, in dem $a$ unendlich oft vorkommt.

Man sieht leicht, dass:
$$ L_\omega(N_{bfin}) \neq \lim(L(N_{bfin})) $$

### Beweis des Korollars

Der Beweis des Korollars basiert auf dem vorherigen Lemma, das zeigt, dass für deterministische Automaten $A$ gilt:
$$ L_\omega(A) = \lim(L(A)) $$

Das bedeutet, wenn $L$ eine $\omega$-Sprache ist, die von einem deterministischen Büchi-Automaten akzeptiert wird, dann gibt es eine reguläre Sprache $K$, sodass:
$$ L = K \⃗ $$
Das ist der Limes der Sprache $K$, was genau die unendlichen Wörter beschreibt, deren endliche Anfangsstücke unendlich oft in $K$ vorkommen.

### Zusammenfassung

- **Deterministische Büchi-Automaten** erkennen $\omega$-Sprachen.
- Eine $\omega$-Sprache $L \subseteq V^\omega$ kann durch einen deterministischen Büchi-Automaten erkannt werden, wenn es eine reguläre Sprache $K \subseteq V^*$ gibt, sodass $L = K \⃗$.
- Das Korollar zeigt die Äquivalenz zwischen $\omega$-Sprachen, die von deterministischen Büchi-Automaten erkannt werden, und regulären Sprachen.

Dieses Korollar ist ein mächtiges Werkzeug, um die Eigenschaften von $\omega$-Sprachen und Büchi-Automaten zu verstehen, insbesondere bei der Umwandlung zwischen verschiedenen Darstellungen von Sprachen.

### Korollar: Existenz nicht-deterministischer Büchi-Automaten für bestimmte Sprachen

#### Aussage

Es gibt Sprachen $L \subseteq V^\omega$, die von einem nicht-deterministischen Büchi-Automaten akzeptiert werden, aber von keinem deterministischen Büchi-Automaten.

#### Beweis

Wir wählen $V = \{a, b\}$ und
$$ L = L_\omega(N_{bfin}) = \{ w \in V^\omega \mid w(n) = b \text{ nur für endlich viele } n \} $$

Angenommen, $L = K⃗$ für eine reguläre Menge $K \subseteq V^*$.

1. Es gibt ein $k_1 > 0$ mit $a^{k_1} \in K$, da $a^\omega \in L$.
2. Dann gibt es auch ein $k_2 > 0$ mit $a^{k_1} b a^{k_2} \in K$, weil $a^{k_1} b a^\omega \in L$.

So fortfahrend gibt es $k_i > 0$ für alle $i$ mit $a^{k_1} b a^{k_2} b \ldots a^{k_i} \in K$.

Wegen $L = K⃗$ folgt daraus auch:
$$ a^{k_1} b a^{k_2} b \ldots a^{k_i} b \ldots \in L $$

Dies steht im Widerspruch zur Definition von $L$, da in $L$ das Symbol $b$ nur endlich oft vorkommen darf.

### Abschlusseigenschaften ω-regulärer Sprachen

Wenn $L_1, L_2$ ω-reguläre Sprachen sind und $K$ eine reguläre Sprache ist, dann ist auch:

1. $L_1 \cup L_2$ ω-regulär,
2. $K^\omega$ ω-regulär, falls $\varepsilon \notin K$,
3. $KL_1$ ω-regulär,
4. $V^\omega \setminus L_1$ ω-regulär,
5. $L_1 \cap L_2$ ω-regulär.

#### Beweis

Seien $A_i = (Q_i, V, s_{i0}, \delta_i, F_i)$ für $i = 1, 2$ Büchi-Automaten und $L_i = L_\omega(A_i)$.

Ohne Beschränkung der Allgemeinheit können wir annehmen, dass $Q_1 \cap Q_2 = \emptyset$.

Wir konstruieren einen Büchi-Automaten $A = (Q, V, s_0, \delta, F)$, wobei $s_0$ ein neuer Zustand ist, der weder in $Q_1$ noch in $Q_2$ vorkommt.

- $Q = Q_1 \cup Q_2 \cup \{s_0\}$
- $\delta(q, a) = \delta_i(q, a)$, falls $q \in Q_i$
- $\delta(s_0, a) = \delta_1(s_{10}, a) \cup \delta_2(s_{20}, a)$
- $F = F_1 \cup F_2$

Man zeigt leicht, dass $L_\omega(A) = L_1 \cup L_2$.

### Zusammenfassung

- **Existenz von Sprachen**: Es gibt Sprachen, die von nicht-deterministischen Büchi-Automaten akzeptiert werden, aber nicht von deterministischen.
- **ω-reguläre Sprachen**: Diese Sprachen besitzen bestimmte Abschluss-Eigenschaften unter den Operationen Vereinigung, Konkatenation, Differenz und Durchschnitt.
- **Kontruktion**: Wir haben gezeigt, wie man einen Büchi-Automaten für die Vereinigung zweier ω-regulärer Sprachen konstruieren kann.

Diese Konzepte sind grundlegend für das Verständnis der Theorie der $\omega$-regulären Sprachen und deren Erkennung durch Büchi-Automaten.

### Abgeschlossenheit unter Iteration und der Zerlegungssatz

#### Abgeschlossenheit unter Iteration

Um zu zeigen, dass eine ω-reguläre Sprache unter Iteration abgeschlossen ist, definieren wir den Automaten $B = (Q_B, V, s_{B0}, \delta_B, F_B)$ wie folgt:

1. **Zustandsmenge**:
   $$
   Q_B = Q_A
   $$
   Die Zustandsmenge von $B$ ist die gleiche wie die Zustandsmenge von $A$.

2. **Startzustand**:
   $$
   s_{B0} = s_{A0}
   $$
   Der Startzustand von $B$ ist der gleiche wie der Startzustand von $A$.

3. **Übergangsfunktion**:
   $$
   \delta_B(q, a) = \delta_A(q, a) \quad \text{falls } q \in Q_B
   $$
   $$
   \delta_B(q, \varepsilon) = \{s_{B0}\} \quad \text{falls } q \in F_A
   $$
   Die Übergangsfunktion von $B$ ist die gleiche wie die Übergangsfunktion von $A$, außer dass es $\varepsilon$-Übergänge vom Endzustand $F_A$ zum Startzustand $s_{B0}$ gibt.

4. **Endzustände**:
   $$
   F_B = \{s_{B0}\}
   $$
   Der einzige Endzustand von $B$ ist der Startzustand $s_{B0}$.

#### Beweis der Abgeschlossenheit unter Iteration

Wir nehmen an, dass für alle $q \in F_A$ und alle $x \in V$ gilt:
$$ s_{A0} \notin \delta_A(q, x) $$

Dies bedeutet, dass der Startzustand $s_{A0}$ von $A$ nicht direkt von einem Endzustand $F_A$ aus durch ein Eingabesymbol $x$ erreicht werden kann, sondern nur durch einen $\varepsilon$-Übergang in $B$.

Der Automat $B$ akzeptiert nun Wörter, die durch wiederholte Anwendungen der Sprache von $A$ entstehen, indem er nach jedem Akzeptieren eines Wortes aus $A$ wieder in den Startzustand zurückkehrt, um den nächsten Abschnitt zu akzeptieren.

### Zerlegungssatz

#### Aussage

Eine Sprache $L \subseteq V^\omega$ ist ω-regulär, genau dann, wenn $L$ eine endliche Vereinigung von Mengen der Form $J K^\omega$ für reguläre Mengen $J, K \subseteq V^*$ ist, wobei $\varepsilon \notin K$.

#### Bedeutung

Dies bedeutet, dass jede ω-reguläre Sprache als eine endliche Vereinigung von unendlichen Wiederholungen regulärer Muster beschrieben werden kann.

### Beispiel zur Komplementbildung
![[Pasted image 20240626111721.png#invert|400]]
Das Diagramm zeigt den Automat $N_{ba}$ und sein Komplement $\text{co}N_{ba}$.

- **Automat $N_{ba}$**:
  - Zustände: $s_0, s_1$
  - Übergänge:
    - $s_0$ hat einen Übergang nach sich selbst für $b, c$ und einen Übergang zu $s_1$ für $a$.
    - $s_1$ hat einen Übergang nach sich selbst für $a, c$.

  - Akzeptierte Sprache:
    $$
    L^\omega(N_{ba}) = \{ w \in \{a, b, c\}^\omega \mid nach jedem a kommt ein b \}
    $$

- **Komplement $\text{co}N_{ba}$**:
  - Zustände: $s_0, s_1$
  - Übergänge:
    - $s_0$ hat einen Übergang nach sich selbst für $a, b, c$.
    - $s_1$ hat einen Übergang nach sich selbst für $a, c$.

  - Akzeptierte Sprache:
    $$
    L^\omega(\text{co}N_{ba}) = \{ w \in \{a, b, c\}^\omega \mid nicht nach jedem a kommt ein b \}
    $$

### Zusammenfassung

- **Abgeschlossenheit unter Iteration**: ω-reguläre Sprachen sind unter Iteration abgeschlossen, was bedeutet, dass eine Sprache, die durch Wiederholung einer ω-regulären Sprache entsteht, ebenfalls ω-regulär ist.
- **Zerlegungssatz**: Eine ω-reguläre Sprache kann als endliche Vereinigung von Mengen der Form $J K^\omega$ für reguläre Mengen $J$ und $K$ beschrieben werden, wobei $\varepsilon \notin K$ ist.
- **Komplementbildung**: Das Beispiel zeigt, wie man das Komplement einer ω-regulären Sprache durch einen entsprechenden Automaten darstellen kann.


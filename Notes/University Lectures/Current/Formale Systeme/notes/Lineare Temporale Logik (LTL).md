### Lineare Temporale Logik (LTL)

#### LTL-Formeln (LTLFor)

LTL ist eine Erweiterung der klassischen Aussagenlogik mit temporalen Modaloperatoren, die es ermöglichen, zeitabhängige Aussagen zu formulieren. Hier sind die Bestandteile von LTL-Formeln:

1. **Grundlagen**:
   - $\Sigma$ ist die Menge aller Aussagenlogik-Atome (AL-Atome).
   - $\Sigma \subseteq \text{LTLFor}$: Jedes Element aus $\Sigma$ ist eine LTL-Formel.
   - $1, 0 \in \text{LTLFor}$: Die Konstanten 1 (wahr) und 0 (falsch) sind ebenfalls LTL-Formeln.

2. **Aussagenlogische Kombinationen**:
   - Wenn $A, B \in \text{LTLFor}$ sind, dann sind alle aussagenlogischen Kombinationen von $A$ und $B$ ebenfalls in $\text{LTLFor}$.

3. **Temporale Modaloperatoren**:
   - $\Box A \in \text{LTLFor}$: "Immer A"
   - $\Diamond B \in \text{LTLFor}$: "Irgendwann B"
   - $A \cup B \in \text{LTLFor}$: "A bis B"
   - $X A \in \text{LTLFor}$: "Nächstes A"

#### LTL-Semantik

Die Semantik der LTL-Formeln wird über Omega-Strukturen $R = (N, \leq, \xi)$ definiert. Hier ist eine Definition, wie LTL-Formeln in einer solchen Struktur interpretiert werden:

1. **AL-Atome**:
   - $\xi \models p \Leftrightarrow p \in \xi(0)$
   - Ein Atom $p$ ist in $\xi$ genau dann wahr, wenn $p$ im ersten Zustand von $\xi$ wahr ist.

2. **Aussagenlogische Kombinationen**:
   - $\xi \models \text{op}(A, B)$ für aussagenlogische Kombinationen $\text{op}(A, B)$ von $A$ und $B$ wie üblich.
   - Zum Beispiel: $\xi \models A \land B \Leftrightarrow \xi \models A \text{ und } \xi \models B$.

3. **Temporale Modaloperatoren**:
   - $\xi \models \Box A \Leftrightarrow \text{für alle } n \in N \text{ gilt } \xi_n \models A$
     - "Immer A": $A$ ist in allen zukünftigen Zuständen wahr.
   - $\xi \models \Diamond A \Leftrightarrow \text{es gibt ein } n \in N \text{ mit } \xi_n \models A$
     - "Irgendwann A": $A$ ist in einem zukünftigen Zustand wahr.
   - $\xi \models A \cup B \Leftrightarrow \text{es gibt ein } n \in N \text{ mit } \xi_n \models B \text{ und für alle } m \text{ mit } 0 \leq m < n \text{ gilt } \xi_m \models A$
     - "A bis B": $B$ ist in einem zukünftigen Zustand wahr und $A$ ist in allen vorhergehenden Zuständen wahr.
   - $\xi \models X A \Leftrightarrow \xi_1 \models A$
     - "Nächstes A": $A$ ist im nächsten Zustand wahr.

### Visualisierung der LTL-Semantik

Die Grafiken illustrieren die Semantik der temporalen Modaloperatoren anhand von Szenarien:

1. **Szenario für $\Box A$**:
   - $A$ ist immer wahr (durchgehende rote Linie).

2. **Szenario für $\Diamond A$**:
   - $A$ ist irgendwann wahr (roter Punkt zeigt den Zeitpunkt, an dem $A$ wahr wird).

3. **Szenario für $A \cup B$**:
   - $A$ ist wahr, bis $B$ wahr wird (blaue Linie zeigt, dass $A$ bis zu einem bestimmten Punkt wahr ist, danach wird $B$ wahr).

Diese Operatoren ermöglichen es, komplexe zeitliche Abhängigkeiten in Systemen zu modellieren und zu überprüfen.

### Reduktion auf die Operatoren U und 1

In der temporalen Logik (LTL) können die temporalen Operatoren $\Diamond$ (irgendwann) und $\Box$ (immer) durch die grundlegenden Operatoren $U$ (bis) und 1 (wahr) ausgedrückt werden.

#### Reduktion der temporalen Operatoren

1. **$\Diamond A$ (irgendwann A)**:
   $$
   \Diamond A \leftrightarrow 1 \, U \, A
   $$
   - Bedeutung: $A$ wird irgendwann in der Zukunft wahr. Das kann als "wahr bis $A$" ausgedrückt werden, wobei "wahr" immer wahr ist, bis $A$ irgendwann eintritt.

2. **$\Box A$ (immer A)**:
   $$
   \Box A \leftrightarrow \neg (1 \, U \, \neg A)
   $$
   - Bedeutung: $A$ ist in jedem Moment wahr. Das kann als Negation der Aussage "1 bis nicht $A$" ausgedrückt werden. Das bedeutet, dass es keinen Zeitpunkt gibt, an dem $\neg A$ eintritt.

### Zusätzliche Operatoren

In der LTL gibt es weitere nützliche Operatoren wie $U_w$ und $V$, die zusätzliche zeitliche Abhängigkeiten ausdrücken.

1. **$A \, U_w \, B$ (weak until)**:
   $$
   \xi \models A \, U_w \, B \text{ gdw. für alle } n \in \mathbb{N} \text{ gilt } \xi_n \models (A \land \neg B) \text{ oder es gibt } n \in \mathbb{N} \text{ mit } \xi_n \models B \text{ und für alle } m \text{ mit } 0 \leq m < n \text{ gilt } \xi_m \models A
   $$
   - Bedeutung: $A$ ist wahr, bis $B$ wahr wird, aber $B$ muss nicht unbedingt eintreten. Wenn $B$ nie eintritt, bleibt $A$ wahr.

2. **$A \, V \, B$ (release)**:
   $$
   \xi \models A \, V \, B \text{ gdw. für alle } n \in \mathbb{N} \text{ gilt, falls } \xi_n \models \neg B \text{ dann gibt es ein } m \text{ mit } 0 \leq m < n \text{ und } \xi_m \models A
   $$
   - Bedeutung: $B$ ist wahr, solange $A$ nicht wahr wird. Sobald $A$ wahr wird, kann $B$ falsch sein. Wenn $B$ jemals falsch ist, muss $A$ vorher irgendwann wahr gewesen sein.

### Visualisierung der LTL-Semantik für $A \, V \, B$

Das Diagramm zeigt verschiedene Szenarien für die Formel $A \, V \, B$:

1. **Szenario 1**:
   - $B$ ist immer wahr. In diesem Fall ist $A \, V \, B$ trivialerweise wahr, weil $B$ nie falsch ist.

2. **Szenario 2**:
   - $B$ wird irgendwann falsch, aber vor diesem Zeitpunkt war $A$ wahr. Dies erfüllt ebenfalls $A \, V \, B$, weil $A$ irgendwann vor dem Zeitpunkt, an dem $B$ falsch wird, wahr ist.

In beiden Fällen wird die Bedingung von $A \, V \, B$ erfüllt, entweder weil $B$ nie falsch wird oder weil $A$ vor dem Zeitpunkt, an dem $B$ falsch wird, wahr ist.

### Zusammenfassung

- **Reduktion der temporalen Operatoren**:
  - $\Diamond A$ kann durch $1 \, U \, A$ ausgedrückt werden.
  - $\Box A$ kann durch $\neg (1 \, U \, \neg A)$ ausgedrückt werden.

- **Zusätzliche Operatoren**:
  - $A \, U_w \, B$ (weak until): $A$ ist wahr bis $B$ wahr wird, aber $B$ muss nicht unbedingt eintreten.
  - $A \, V \, B$ (release): $B$ ist wahr, solange $A$ nicht wahr wird. Wenn $B$ falsch ist, muss $A$ vorher wahr gewesen sein.

Diese Operatoren und ihre Semantik sind nützlich, um komplexe zeitliche Abhängigkeiten und Bedingungen in formalen Modellen zu beschreiben.

### LTL (Lineare Temporale Logik) - Lemmas und Beispiele

#### Lemma

1. **$A \, U \, B \leftrightarrow (A \, U_w \, B) \land \Diamond B$**:
   - Bedeutung: $A$ ist wahr bis $B$ wahr wird, und $B$ wird irgendwann wahr.
   - $A \, U \, B$ bedeutet, dass $B$ irgendwann wahr wird und $A$ bis zu diesem Zeitpunkt wahr ist.
   - $A \, U_w \, B$ (weak until) bedeutet, dass $A$ bis $B$ wahr ist, aber $B$ muss nicht unbedingt wahr werden.
   - $\Diamond B$ bedeutet, dass $B$ irgendwann wahr wird.
   - Die Kombination von $A \, U_w \, B$ und $\Diamond B$ entspricht genau der Definition von $A \, U \, B$.

2. **$A \, U_w \, B \leftrightarrow A \, U \, B \lor \Box (A \land \neg B)$**:
   - Bedeutung: $A$ ist wahr bis $B$ wahr wird, oder $A$ bleibt immer wahr und $B$ wird nie wahr.
   - $A \, U_w \, B$ (weak until) bedeutet, dass $A$ bis $B$ wahr ist, aber $B$ muss nicht unbedingt wahr werden.
   - $A \, U \, B$ bedeutet, dass $B$ irgendwann wahr wird und $A$ bis zu diesem Zeitpunkt wahr ist.
   - $\Box (A \land \neg B)$ bedeutet, dass $A$ immer wahr ist und $B$ niemals wahr wird.

3. **$A \, V \, B \leftrightarrow \neg (\neg A \, U \, \neg B)$**:
   - Bedeutung: $B$ ist wahr, solange $A$ nicht wahr wird.
   - $A \, V \, B$ (release) bedeutet, dass $B$ wahr bleibt, solange $A$ nicht wahr wird.
   - $\neg (\neg A \, U \, \neg B)$ bedeutet, dass es keinen Zeitpunkt gibt, an dem $A$ wahr ist, ohne dass $B$ vorher wahr war.

4. **$A \, U \, B \leftrightarrow (B \lor (A \land X (A \, U \, B)))$**:
   - Bedeutung: $A$ ist wahr bis $B$ wahr wird, wobei $B$ irgendwann wahr wird.
   - $B$ kann sofort wahr werden, oder $A$ bleibt wahr und $A \, U \, B$ gilt im nächsten Zustand ($X (A \, U \, B)$).

5. **$A \, V \, B \leftrightarrow (B \land A) \lor (B \land X (A \, V \, B))$**:
   - Bedeutung: $B$ ist wahr, solange $A$ nicht wahr wird.
   - $B$ bleibt immer wahr und entweder $A$ wird irgendwann wahr oder $B$ bleibt weiterhin wahr und $A \, V \, B$ gilt im nächsten Zustand ($X (A \, V \, B)$).

### Beispiel

**Gesucht: Eine LTL-Formel $A2p$, so dass für jedes $\xi$ gilt $\xi \models A2p$ gdw (n ist gerade $\Leftrightarrow$ $p \in \xi(n)$)**

$$
A2p = p \land X \neg p \land \Box (p \leftrightarrow XX p)
$$

- Bedeutung: $p$ ist wahr in den geraden Zuständen und falsch in den ungeraden Zuständen.
- $p \land X \neg p$: $p$ ist wahr und im nächsten Zustand ist $p$ falsch.
- $\Box (p \leftrightarrow XX p)$: $p$ bleibt in den Zuständen wahr, die zwei Schritte auseinander liegen (d.h. in den geraden Zuständen).

**Erstaunlicherweise gibt es keine LTL-Formel $A$, so dass $\xi \models A$ gdw (n ist gerade $\Rightarrow$ $p \in \xi(n)$)**

### Beispiele aus Mustersammlungen

1. **REQUIREMENT**: After OpeningNetworkConnection, an ErrorMessage will pop up in response to a NetworkError

   - **PATTERN**: Response
   - **SCOPE**: After
   - **PARAMETERS**: Propositional
   - **LTL**:
     $$
     \Box (OpenNetworkConnection \rightarrow \Box (NetworkError \rightarrow \Diamond ErrorMessage))
     $$
   - **SOURCE**: Jeff Isom
   - **DOMAIN**: GUI

2. **REQUIREMENT**: Before QueuedMailSent, SMTPServerConnected

   - **PATTERN**: Existence
   - **SCOPE**: Before
   - **ALTERNATE**: Global Precedence
   - **PARAMETERS**: Propositional
   - **LTL**:
     $$
     \Diamond QueuedMailSent \rightarrow (\neg QueuedMailSent \, U \, SMTPServerConnected)
     $$
   - **SOURCE**: Jeff Isom
   - **DOMAIN**: GUI

Diese Beispiele veranschaulichen die Anwendung von LTL zur Spezifikation von Anforderungen in zeitlichen Systemen.

### LTL und Büchi-Automaten

#### Beziehung zwischen LTL und Büchi-Automaten

Büchi-Automaten sind ein Modell zur Erkennung von $\omega$-regulären Sprachen, die durch LTL-Formeln beschrieben werden können. Die Hauptidee ist, dass für jede LTL-Formel ein entsprechender Büchi-Automat existiert, der dieselbe Sprache akzeptiert.

#### Definition eines Büchi-Automaten

Ein Büchi-Automat $B$ ist definiert als ein 5-Tupel:
$$ B = (S, V, s_0, \delta, F) $$
wobei:
- $S$ eine endliche Menge von Zuständen ist,
- $V$ das Eingabealphabet ist,
- $s_0 \in S$ der Anfangszustand ist,
- $\delta : S \times V \rightarrow \mathcal{P}(S)$ die Übergangsfunktion ist,
- $F \subseteq S$ die Menge der Endzustände ist.

#### Omega-Strukturen und unendliche Wörter

Für einen Büchi-Automaten mit $V = 2^\Sigma$ (wobei $\Sigma$ die Menge der aussagenlogischen Atome ist), können wir $\Omega$-Strukturen $\xi$ über $\Sigma$ und unendliche Wörter $w \in V^\omega$ über $V$ identifizieren.

### Notation

Für die folgenden Beispiele verwenden wir die folgende Notation:

- **Aussagenlogische Signatur $\Sigma$**:
  - $\Sigma$ enthält die Atome $p$ und $q$.

- **Eingabealphabet $V$**:
  - $V = 2^\Sigma$, d.h. die Menge aller Teilmengen von $\Sigma$. Das bedeutet, jedes Element von $V$ ist eine Menge von Atomen, die zu einem bestimmten Zeitpunkt wahr sind.

- **Mengen $P$ und $Q$**:
  - $P = \{ b \in V \mid p \in b \}$
    - $P$ enthält alle Elemente von $V$, in denen $p$ wahr ist.
  - $Q = \{ b \in V \mid q \in b \}$
    - $Q$ enthält alle Elemente von $V$, in denen $q$ wahr ist.

### Beispiel für LTL-Formeln und Büchi-Automaten

#### Szenario: LTL-Formeln für $\Box A$, $\Diamond A$, und $A \cup B$

- **$\Box A$**:
  - Bedeutung: $A$ ist immer wahr.
  - Automatische Interpretation: Der Automat muss sicherstellen, dass $A$ in jedem Zustand der unendlichen Folge wahr bleibt.

- **$\Diamond A$**:
  - Bedeutung: $A$ wird irgendwann in der Zukunft wahr.
  - Automatische Interpretation: Der Automat muss sicherstellen, dass es einen Zustand in der unendlichen Folge gibt, in dem $A$ wahr ist.

- **$A \cup B$**:
  - Bedeutung: $A$ ist wahr, bis $B$ wahr wird.
  - Automatische Interpretation: Der Automat muss sicherstellen, dass $A$ in jedem Zustand wahr ist, bis $B$ irgendwann wahr wird.

### Beispiel-Büchi-Automat

Angenommen, wir haben einen Büchi-Automaten $B$ mit den Zuständen $s_0$ und $s_1$ und dem Alphabet $V = 2^\Sigma$. Wir definieren die Übergangsfunktion und die Endzustände wie folgt:

1. **Zustände und Übergänge**:
   - $s_0$:
     - Übergang zu $s_0$ für $b \in P$.
     - Übergang zu $s_1$ für $b \notin P$.
   - $s_1$:
     - Übergang zu $s_1$ für $b \in Q$.
     - Übergang zu $s_0$ für $b \notin Q$.

2. **Endzustände**:
   - Der Endzustand ist $s_1$.

#### Beispiel: LTL-Formel für $\Diamond p$

- **Formel**: $\Diamond p$
- **Automatische Interpretation**:
  - Der Automat startet im Zustand $s_0$.
  - Er bleibt im Zustand $s_0$, solange $p$ nicht wahr ist.
  - Sobald $p$ wahr wird, wechselt der Automat in den Zustand $s_1$ und bleibt dort.

#### Beispiel: LTL-Formel für $p \cup q$

- **Formel**: $p \cup q$
- **Automatische Interpretation**:
  - Der Automat startet im Zustand $s_0$.
  - Er bleibt im Zustand $s_0$, solange $p$ wahr ist und $q$ nicht wahr ist.
  - Sobald $q$ wahr wird, wechselt der Automat in den Zustand $s_1$.

### Zusammenfassung

- **LTL-Formeln**: Beschreiben zeitliche Abhängigkeiten und können durch Büchi-Automaten dargestellt werden.
- **Büchi-Automaten**: Akzeptieren $\omega$-reguläre Sprachen und können verwendet werden, um LTL-Formeln zu interpretieren.
- **Omega-Strukturen**: Ermöglichen die Modellierung von unendlichen Folgen von Zuständen, die durch LTL-Formeln beschrieben werden können.

Diese Konzepte sind nützlich, um zeitabhängige Systeme zu modellieren und zu verifizieren, insbesondere in der formalen Verifikation und Spezifikation von Software und Hardware-Systemen.

### Beispiel Automaten
![[Pasted image 20240626112240.png]]![[Pasted image 20240626112243.png]]![[Pasted image 20240626112252.png]]
### Konjunktionsautomaten für Büchi-Automaten

Das gezeigte Diagramm zeigt einen Büchi-Automaten für die LTL-Formel $\Box \Diamond p \land \Box \Diamond q$. Um zu erklären, wie dieser Automat funktioniert und wie er konstruiert wird, betrachten wir die allgemeine Methode zur Konstruktion eines Konjunktionsautomaten.

#### Lemma

Seien
- $A_1 = (S_1, V, s_{01}, \delta_1, F_1)$
- $A_2 = (S_2, V, s_{02}, \delta_2, F_2)$

zwei Büchi-Automaten, die die LTL-Formeln $C_1$ und $C_2$ akzeptieren.

Dann gibt es einen Büchi-Automaten $C$ mit:
$$ C \models C_1 \land C_2 $$

#### Allgemeine Konstruktion für Konjunktionsautomaten

Gegeben:
- $A_i = (S_i, s_{0i}, \delta_i, F_i)$

Gesucht:
- $C = (S, s_0, \delta, F)$ mit $L_\omega(C) = L_\omega(A_1) \cap L_\omega(A_2)$.

##### Konstruktion

1. **Zustandsmenge**:
   $$ S = S_1 \times S_2 \times \{1, 2\} $$

2. **Startzustand**:
   $$ s_0 = (s_{01}, s_{02}, 1) $$

3. **Endzustandsmenge**:
   $$ F = F_1 \times S_2 \times \{1\} $$
   - $F$ enthält alle Tupel, bei denen der erste Zustand in der Endzustandsmenge von $A_1$ ist und der Index $1$ ist.

4. **Übergangsfunktion**:
   - **Falls $s_1 \in F_1$ und $i = 1$**:
     $$ (t_1, t_2, 2) \in \delta((s_1, s_2, 1), a) \Leftrightarrow t_1 \in \delta_1(s_1, a) \text{ und } t_2 \in \delta_2(s_2, a) $$
   - **Falls $s_2 \in F_2$ und $i = 2$**:
     $$ (t_1, t_2, 1) \in \delta((s_1, s_2, 2), a) \Leftrightarrow t_1 \in \delta_1(s_1, a) \text{ und } t_2 \in \delta_2(s_2, a) $$
   - **Sonst**:
     $$ (t_1, t_2, i) \in \delta((s_1, s_2, i), a) \Leftrightarrow i \in \{1, 2\}, t_1 \in \delta_1(s_1, a) \text{ und } t_2 \in \delta_2(s_2, a) $$

### Beispiel: Automat für $\Box \Diamond p \land \Box \Diamond q$

#### Erklärung des Diagramms

- **Zustände**:
  - Jeder Zustand wird als Tripel $(s1, s2, i)$ dargestellt, wobei $s1$ und $s2$ die Zustände der ursprünglichen Automaten $A_1$ und $A_2$ sind und $i$ den aktuellen Index (1 oder 2) angibt.

- **Startzustand**:
  - Der Startzustand ist $(00-1)$, was bedeutet, dass beide Automaten in ihren Startzuständen sind und der Index 1 ist.

- **Übergänge**:
  - Die Übergänge basieren auf den Zustandsübergängen der beiden ursprünglichen Automaten und dem aktuellen Index.
  - Wenn der Zustand $(s_1, s_2, 1)$ in einen Zustand $(t_1, t_2, 2)$ übergeht, bedeutet das, dass der Zustand des ersten Automaten in einen Endzustand übergeht und der Index auf 2 wechselt.
  - Umgekehrt, wenn der Zustand $(s_1, s_2, 2)$ in einen Zustand $(t_1, t_2, 1)$ übergeht, bedeutet das, dass der Zustand des zweiten Automaten in einen Endzustand übergeht und der Index auf 1 wechselt.

- **Endzustände**:
  - Ein Endzustand ist erreicht, wenn der Zustand des ersten Automaten in der Endzustandsmenge $F_1$ ist und der Index 1 ist.

### Zusammenfassung

- **Konjunktionsautomaten** werden verwendet, um die gemeinsame Erfüllung von zwei LTL-Formeln zu prüfen.
- Die Konstruktion eines Konjunktionsautomaten basiert auf den Zuständen und Übergängen der beiden ursprünglichen Automaten.
- Durch die Verwendung von Tripelzuständen und der Verwaltung eines Index wird sichergestellt, dass beide ursprünglichen Automaten ihre Bedingungen unabhängig voneinander erfüllen können.

Diese Konstruktion ermöglicht es, komplexe LTL-Formeln in automatenbasierte Modelle zu überführen, die effizient überprüft werden können.

### Modellprüfung für LTL (Lineare Temporale Logik)
![[Pasted image 20240626112615.png#invert|400]]
Die Modellprüfung (Model Checking) ist eine Methode zur Verifikation von Systemen, bei der überprüft wird, ob ein Modell eines Systems eine gegebene temporale Logikeigenschaft erfüllt. Im Kontext der LTL erfolgt die Modellprüfung mittels Büchi-Automaten.

#### Theorem

Für jede LTL-Formel $B$ gibt es einen – effektiv konstruierbaren – Büchi-Automaten $A_B$ mit:
$$ L_\omega(A_B) = \{ \xi \in V^\omega \mid \xi \models B \} $$

#### Beweisidee (Details siehe Skriptum)

- Der Beweis beruht darauf, dass man für jede LTL-Formel $B$ einen Büchi-Automaten $A_B$ konstruieren kann, der genau die unendlichen Wörter akzeptiert, die die Formel $B$ erfüllen.
- Die Konstruktion erfolgt durch eine systematische Umwandlung der LTL-Formel in einen entsprechenden Automaten.

### Korollar: Entscheidbarkeit von Erfüllbarkeit und Allgemeingültigkeit

#### Erfüllbarkeit

Eine LTL-Formel $B$ ist erfüllbar, wenn es eine Omega-Struktur $\xi$ gibt, die $B$ erfüllt.

- **Erfüllbarkeitstest**:
  $$ B \text{ ist erfüllbar } \Leftrightarrow L_\omega(A_B) \neq \emptyset $$
  - Konstruktion des Büchi-Automaten $A_B$.
  - Überprüfung, ob $L_\omega(A_B)$ nicht leer ist.

#### Allgemeingültigkeit

Eine LTL-Formel $B$ ist allgemeingültig, wenn sie in jeder Omega-Struktur $\xi$ erfüllt ist.

- **Allgemeingültigkeitstest**:
  $$ B \text{ ist allgemeingültig } \Leftrightarrow L_\omega(A_{\neg B}) = \emptyset $$
  - Konstruktion des Büchi-Automaten $A_{\neg B}$.
  - Überprüfung, ob $L_\omega(A_{\neg B})$ leer ist.

Für jeden Büchi-Automaten $C$ ist die Frage $L_\omega(C) = \emptyset$ entscheidbar, d.h., es gibt Algorithmen, um zu überprüfen, ob die von $C$ akzeptierte Sprache leer ist.

### Modellprüfung mit Büchi-Automaten

Der Prozess der Modellprüfung für LTL-Formeln lässt sich wie folgt beschreiben:

1. **Modell und Eigenschaft**:
   - Ein Modell des Systems wird als Büchi-Automat $A_{\text{mod}}$ dargestellt.
   - Die zu überprüfende Eigenschaft wird als LTL-Formel $\phi$ angegeben.

2. **Konstruktion der Automaten**:
   - Konstruktion des Büchi-Automaten $A_\phi$ für die LTL-Formel $\phi$.
   - Konstruktion des Büchi-Automaten $A_{\neg \phi}$ für die Negation der LTL-Formel $\neg \phi$.

3. **Kombinierter Automat**:
   - Konstruktion des kombinierten Automaten $A_{\text{mod} \land \neg \phi}$, der das Produkt des Modells und der negierten Eigenschaft darstellt.

4. **Leere Sprachüberprüfung**:
   - Überprüfung, ob $L_\omega(A_{\text{mod} \land \neg \phi}) = \emptyset$ ist.
   - Wenn $L_\omega(A_{\text{mod} \land \neg \phi}) = \emptyset$ ist, gilt $\phi$ für $A_{\text{mod}}$ (keine Gegenbeispiele).
   - Wenn $L_\omega(A_{\text{mod} \land \neg \phi}) \neq \emptyset$ ist, gibt es ein Gegenbeispiel.

### Zusammenfassung

- **Theorem**: Für jede LTL-Formel $B$ gibt es einen Büchi-Automaten $A_B$, der die von $B$ akzeptierte Sprache beschreibt.
- **Korollar**: Die Erfüllbarkeit und Allgemeingültigkeit von LTL-Formeln ist entscheidbar.
- **Modellprüfung**: Ein systematischer Prozess zur Verifikation, der durch Konstruktion und Überprüfung von Büchi-Automaten erfolgt. 

Diese Methoden sind grundlegend für die formale Verifikation von Systemen und werden häufig in der Software- und Hardwareverifikation eingesetzt, um die Korrektheit von Systemen sicherzustellen.


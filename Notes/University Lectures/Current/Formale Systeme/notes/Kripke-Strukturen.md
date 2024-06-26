### Definition einer Kripke-Struktur

Eine Kripke-Struktur $K$ über einer Menge aussagenlogischer Variablen $Σ$ besteht aus den folgenden Komponenten:

1. **S (Zustandsmenge):**
   - $S$ ist eine nichtleere Menge, die die möglichen Zustände oder Welten in der Struktur repräsentiert. Im Beispiel ist $S = \{x_1, x_2, x_3, x_4, x_5, x_6\}$.

2. **R (Zugänglichkeitsrelation):**
   - $R \subseteq S \times S$ ist eine Relation, die angibt, welche Zustände von anderen Zuständen aus zugänglich sind. Diese Relation kann als gerichteter Graph dargestellt werden, wobei ein Paar $(s, t) \in R$ bedeutet, dass Zustand $t$ von Zustand $s$ aus zugänglich ist. Im Beispiel sind die Paare in $R$ wie folgt gegeben:
     $$
     R = \{(x_1, x_2), (x_1, x_3), (x_2, x_3), (x_3, x_2), (x_4, x_5), (x_5, x_4), (x_5, x_6), (x_6, x_6)\}
     $$

3. **I (Interpretation):**
   - $I: Σ \times S \to \{W, F\}$ ist eine Funktion, die jeder aussagenlogischen Variablen $P \in Σ$ in jedem Zustand $s \in S$ entweder den Wahrheitswert $W$ (wahr) oder $F$ (falsch) zuweist. Im Beispiel:
     $$
     \begin{align*}
     I(P, x_1) &= 1 \\
     I(P, x_3) &= 1 \\
     I(P, x_6) &= 1 \\
     I(Q, x_2) &= 1 \\
     I(Q, x_3) &= 1 \\
     I(Q, x_4) &= 1 \\
     \end{align*}
     $$
     Für alle anderen Kombinationen von Variablen und Zuständen gilt $I(P, s) = 0$ und $I(Q, s) = 0$.

### Kripke Rahmen

- **(S, R)**: Dies ist der sogenannte Kripke-Rahmen und besteht nur aus der Zustandsmenge $S$ und der Zugänglichkeitsrelation $R$, ohne die Interpretation $I$.

### Beispiel aus dem Bild
![[Pasted image 20240626103852.png#invert|400]]
Im Bild wird eine Kripke-Struktur mit den Zuständen $x_1$ bis $x_6$ und den Beziehungen zwischen diesen Zuständen gezeigt. Die Interpretation $I$ zeigt, welche Variablen $P$ und $Q$ in welchen Zuständen wahr sind.

Zusammengefasst stellt das Bild eine vollständige Kripke-Struktur dar, die alle drei Komponenten $S$, $R$ und $I$ enthält. Die Zustände und ihre Beziehungen werden durch die Pfeile dargestellt, und die Interpretation $I$ gibt an, in welchen Zuständen die logischen Variablen $P$ und $Q$ wahr sind.

### Definition der Auswertung von Formeln

Gegeben sei eine Kripke-Struktur $K = (S, R, I)$. Wir wollen bestimmen, wann eine modale Formel $F$ in einem Zustand $s \in S$ wahr ist.

#### Modale Operatoren

1. **□A (notwendig A):**
   $$
   \text{vals}(□A) =
   \begin{cases}
   W \text{ (wahr)}, & \text{falls für alle } s' \in S \text{ mit } sRs' \text{ gilt } \text{vals}(s', A) = W \\
   F \text{ (falsch)}, & \text{sonst}
   \end{cases}
   $$
   - Das bedeutet, dass $□A$ in $s$ wahr ist, wenn $A$ in allen zugänglichen Zuständen $s'$ von $s$ aus wahr ist.

2. **♢A (möglich A):**
   $$
   \text{vals}(♢A) =
   \begin{cases}
   W \text{ (wahr)}, & \text{falls ein } s' \in S \text{ existiert mit } sRs' \text{ und } \text{vals}(s', A) = W \\
   F \text{ (falsch)}, & \text{sonst}
   \end{cases}
   $$
   - Das bedeutet, dass $♢A$ in $s$ wahr ist, wenn $A$ in mindestens einem zugänglichen Zustand $s'$ von $s$ aus wahr ist.

### Notation

- $K = (S, R, I)$ ist eine Kripke-Struktur.
- $s \in S$ ist ein Zustand.
- $F$ ist eine modale Formel.

#### Wahrheit einer Formel in einem Zustand

- $(K, s) \models F \iff \text{vals}(F) = W$: $F$ ist in der Kripke-Struktur $K$ und im Zustand $s$ wahr.
- Wenn $K$ aus dem Kontext bekannt ist, schreiben wir auch:
  $$
  s \models F \iff \text{vals}(F) = W
  $$

#### Allgemeine Gültigkeit

- $K \models F \iff \text{für alle } s \in S \text{ gilt } (K, s) \models F$: $F$ ist in der gesamten Kripke-Struktur $K$ wahr.
- Gültigkeit in einem Kripke-Rahmen $(S, R)$:
  $$
  (S, R) \models F \iff \text{für alle Interpretationen } I \text{ gilt } (S, R, I) \models F
  $$

Zusammengefasst bedeutet dies:

- Eine modale Formel $□A$ ist in einem Zustand $s$ wahr, wenn $A$ in allen von $s$ aus erreichbaren Zuständen wahr ist.
- Eine modale Formel $♢A$ ist in einem Zustand $s$ wahr, wenn $A$ in mindestens einem von $s$ aus erreichbaren Zustand wahr ist.
- Eine Formel $F$ ist in einer Kripke-Struktur $K$ wahr, wenn sie in jedem Zustand $s \in S$ von $K$ wahr ist.
- Eine Formel $F$ ist in einem Kripke-Rahmen $(S, R)$ wahr, wenn sie für jede mögliche Interpretation $I$ in der Struktur $(S, R, I)$ wahr ist.

### Beispiel
![[Pasted image 20240626104101.png]]

Die Kripke-Struktur $K$ besteht aus vier Zuständen $A, B, C, D$ und den Zugänglichkeitsrelationen zwischen diesen Zuständen. Die Zustände sind durch Pfeile verbunden, die die Relation $R$ darstellen:

- $A$ ist erreichbar von $B$ und $D$.
- $B$ ist erreichbar von $A$.
- $C$ ist erreichbar von $B$.
- $D$ ist erreichbar von $C$.

Die Interpretation der Variablen $P$ in den Zuständen ist wie folgt:

- $(K, A) \models P$ (P ist wahr in $A$).
- $(K, B) \not\models P$ (P ist falsch in $B$).
- $(K, C) \models P$ (P ist wahr in $C$).
- $(K, D) \not\models P$ (P ist falsch in $D$).

#### Auswertung der Formeln

Nun bewerten wir verschiedene modale Formeln in den Zuständen $A, B, C, D$:

1. **$P$**
   - $(K, A) \models P$ : P ist in $A$ wahr.
   - $(K, B) \not\models P$ : P ist in $B$ falsch.
   - $(K, C) \models P$ : P ist in $C$ wahr.
   - $(K, D) \not\models P$ : P ist in $D$ falsch.

2. **$\square P$ (notwendig P)**
   - $(K, A) \models \square P$ : Für alle von $A$ aus erreichbaren Zustände (hier nur $B$) muss $P$ wahr sein. Da $P$ in $B$ falsch ist, gilt $(K, A) \not\models \square P$.
   - $(K, B) \models \square P$ : Für alle von $B$ aus erreichbaren Zustände (hier nur $C$) muss $P$ wahr sein. Da $P$ in $C$ wahr ist, gilt $(K, B) \models \square P$.
   - $(K, C) \not\models \square P$ : Für alle von $C$ aus erreichbaren Zustände (hier nur $D$) muss $P$ wahr sein. Da $P$ in $D$ falsch ist, gilt $(K, C) \not\models \square P$.
   - $(K, D) \not\models \square P$ : $D$ hat keine zugänglichen Zustände, daher triviale Erfüllung, aber da $D$ selbst $P$ nicht erfüllt, gilt $(K, D) \not\models \square P$.

3. **$\square \square P$ (notwendig notwendig P)**
   - $(K, A) \models \square \square P$ : Alle von $A$ aus erreichbaren Zustände müssen $\square P$ erfüllen. Da $A$ nur $B$ erreichen kann und $B \models \square P$, gilt $(K, A) \models \square \square P$.
   - $(K, B) \not\models \square \square P$ : $B$ kann nur $C$ erreichen, aber $C \not\models \square P$, daher $(K, B) \not\models \square \square P$.
   - $(K, C) \not\models \square \square P$ : $C$ kann nur $D$ erreichen, aber $D \not\models \square P$, daher $(K, C) \not\models \square \square P$.
   - $(K, D) \not\models \square \square P$ : Analog zur obigen Bewertung für $\square P$, gilt $(K, D) \not\models \square \square P$.

#### Zusammenfassung der Bewertungen:
- Blau (wahr): $(K, A) \models P$, $(K, C) \models P$, $(K, B) \models \square P$, $(K, A) \models \square \square P$
- Rot (falsch): $(K, B) \not\models P$, $(K, D) \not\models P$, $(K, A) \not\models \square P$, $(K, C) \not\models \square P$, $(K, D) \not\models \square P$, $(K, B) \not\models \square \square P$, $(K, C) \not\models \square \square P$, $(K, D) \not\models \square \square P$

Lassen wir uns die Begriffe und Konzepte Schritt für Schritt erklären, und dann schauen wir uns das Gegenbeispiel genauer an.

### Logische Folgerung in der Modalen Logik

#### Definition der logischen Folgerung

- **Logische Folgerung:** Eine Formel $A$ ist eine logische Folgerung aus einer Menge von Formeln $\Gamma$ (geschrieben als $\Gamma \models A$), wenn für alle Kripke-Strukturen $K$ und jede Welt $s$ von $K$ gilt: 
  $$
  \text{Wenn } (K, s) \models \Gamma \text{, dann auch } (K, s) \models A.
  $$
  Das bedeutet, wenn alle Formeln in $\Gamma$ in der Welt $s$ wahr sind, dann muss auch $A$ in der Welt $s$ wahr sein.

- **Allgemeingültigkeit:** Eine Formel $A$ ist allgemeingültig (geschrieben als $\models A$), wenn $A$ in jeder Kripke-Struktur und in jeder Welt wahr ist. Das kann auch als $\emptyset \models A$ geschrieben werden, was bedeutet, dass $A$ eine logische Folgerung aus der leeren Menge von Formeln ist.

### Beispiele für allgemeingültige Formeln

Hier sind einige Beispiele für allgemeingültige Formeln in der modalen Logik:

1. $\square(P \to Q) \to (\square P \to \square Q)$
2. $(\square P \land \square(P \to Q)) \to \square Q$
3. $(\square P \lor \square Q) \to \square(P \lor Q)$
4. $(\square P \land \square Q) \leftrightarrow \square(P \land Q)$
5. $\square P \leftrightarrow \neg \Diamond \neg P$
6. $\Diamond (P \lor Q) \leftrightarrow (\Diamond P \lor \Diamond Q)$
7. $\Diamond (P \land Q) \to (\Diamond P \land \Diamond Q)$

### Beispiel: Gegenbeispiel zur Allgemeingültigkeit
![[Pasted image 20240626104214.png#invert|400]]
Schauen wir uns das Gegenbeispiel im Bild an. Die Formel, die hier untersucht wird, ist:

$$ \square (P \lor Q) \to (\square P \lor \square Q) $$

Diese Formel soll zeigen, dass sie nicht allgemeingültig ist. 

#### Kripke-Struktur im Bild

Die Kripke-Struktur besteht aus drei Zuständen $s_1, s_2, s_3$:

- **Zustand $s_1$:** $s_1$ hat Zugänglichkeit zu $s_2$ und $s_3$.
- **Zustand $s_2$:** $s_2$ hat $P$ wahr und $Q$ falsch.
- **Zustand $s_3$:** $s_3$ hat $P$ falsch und $Q$ wahr.

#### Bewertung der Formel

1. **$\square (P \lor Q)$ in $s_1$:**
   - $P \lor Q$ ist in beiden erreichbaren Zuständen $s_2$ und $s_3$ wahr.
   - Also ist $\square (P \lor Q)$ in $s_1$ wahr.

2. **$\square P$ und $\square Q$ in $s_1$:**
   - $\square P$ wäre wahr, wenn $P$ in beiden erreichbaren Zuständen $s_2$ und $s_3$ wahr wäre. Da $P$ in $s_3$ falsch ist, ist $\square P$ in $s_1$ falsch.
   - $\square Q$ wäre wahr, wenn $Q$ in beiden erreichbaren Zuständen $s_2$ und $s_3$ wahr wäre. Da $Q$ in $s_2$ falsch ist, ist $\square Q$ in $s_1$ falsch.

Da weder $\square P$ noch $\square Q$ in $s_1$ wahr ist, ist die Aussage $\square P \lor \square Q$ in $s_1$ falsch.

**Fazit:** In $s_1$ ist $\square (P \lor Q)$ wahr, aber $\square P \lor \square Q$ ist falsch. Das zeigt, dass die Implikation $\square (P \lor Q) \to (\square P \lor \square Q)$ nicht allgemeingültig ist, da sie in dieser Kripke-Struktur widerlegt wird.


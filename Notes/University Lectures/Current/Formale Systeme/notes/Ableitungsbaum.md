## Definition: Ableitungsbaum
Ein **Ableitungsbaum** ist ein Baum, dessen Knoten mit Sequenzen markiert sind. Die Struktur und Regeln eines Ableitungsbaums sind wie folgt:
1. **Ein Nachfolgerknoten:**
   - Wenn ein Knoten$n$ genau einen Nachfolgerknoten$n_1$ hat, und die Sequenzen an diesen Knoten$\Gamma \Rightarrow \Delta$ und$\Gamma_1 \Rightarrow \Delta_1$ sind, dann gibt es eine Sequenzenregel, die von$\Gamma_1 \Rightarrow \Delta_1$ zu$\Gamma \Rightarrow \Delta$ führt.
     $$
     \frac{\Gamma_1 \Rightarrow \Delta_1}{\Gamma \Rightarrow \Delta}
     $$
2. **Zwei Nachfolgerknoten:**
   - Wenn ein Knoten$n$ zwei Nachfolgerknoten$n_1$ und$n_2$ hat, und die Sequenzen an diesen Knoten$\Gamma \Rightarrow \Delta$,$\Gamma_1 \Rightarrow \Delta_1$ und$\Gamma_2 \Rightarrow \Delta_2$ sind, dann gibt es eine Sequenzenregel, die von$\Gamma_1 \Rightarrow \Delta_1$ und$\Gamma_2 \Rightarrow \Delta_2$ zu$\Gamma \Rightarrow \Delta$ führt.
     $$
     \frac{\Gamma_1 \Rightarrow \Delta_1 \quad \Gamma_2 \Rightarrow \Delta_2}{\Gamma \Rightarrow \Delta}
     $$
Diese Regeln stellen sicher, dass jeder Knoten in einem Ableitungsbaum in einer bestimmten, logischen Beziehung zu seinen Nachfolgeknoten steht.
### Beispiel eines Ableitungsbaums
Stellen wir uns vor, wir haben eine Sequenz$\Gamma \Rightarrow \Delta$, und wir wenden eine Regel darauf an, um neue Sequenzen zu erzeugen:
1. **Startknoten:**
   $$
   \text{Knoten } n: \Gamma \Rightarrow \Delta
   $$
2. **Ein Nachfolger:**
   $$
   \text{Knoten } n_1: \Gamma_1 \Rightarrow \Delta_1
   $$
   - Hier gibt es eine Regel, die$\Gamma_1 \Rightarrow \Delta_1$ von$\Gamma \Rightarrow \Delta$ ableitet.
3. **Zwei Nachfolger:**
   $$
   \text{Knoten } n_2: \Gamma_2 \Rightarrow \Delta_2 \quad \text{und} \quad \text{Knoten } n_3: \Gamma_3 \Rightarrow \Delta_3
   $$
   - Hier gibt es eine Regel, die$\Gamma_2 \Rightarrow \Delta_2$ und$\Gamma_3 \Rightarrow \Delta_3$ von$\Gamma \Rightarrow \Delta$ ableitet.

## Definition: Geschlossener Ableitungsbaum
Ein **geschlossener** oder **vollständiger** Ableitungsbaum ist ein Beweisbaum, der zusätzlich die folgende Bedingung erfüllt:
3. **Substitution$\sigma$:**
   - Es gibt eine Substitution$\sigma$, so dass für die Markierung$A$ jedes Knotens$n$, der keinen Nachfolgerknoten hat,$\sigma(A)$ ein Axiom ist. Hierzu zählen auch die Gleichheitsaxiome.
### Bedeutung von$\sigma$
- **Substitution$\sigma$:** Eine Substitution$\sigma$ ist eine Zuordnung von Variablen zu Termen, die verwendet wird, um Variablen in einer Formel zu ersetzen. Dadurch können allgemeine Aussagen spezifiziert und zu Axiomen gemacht werden.
- **Unifikator:** Ein Unifikator ist eine Substitution, die zwei Terme$s$ und$t$ gleich macht, d.h.,$\sigma(s) = \sigma(t)$.
### Beispiel für einen geschlossenen Ableitungsbaum
Angenommen, wir haben eine Sequenz$A$ wie$p(s) \Rightarrow p(t)$, die kein Axiom ist. Wenn wir jedoch eine Substitution$\sigma$ finden, die$s$ und$t$ gleich macht, wird$\sigma(A)$ zu einem Axiom.
- **Sequenz vor der Substitution:**
  $$
  A: p(s) \Rightarrow p(t)
  $$
  - Diese Sequenz ist kein Axiom, da$s \neq t$.

- **Substitution$\sigma$:**
  $$
  \sigma = \{s \mapsto t\}
  $$

- **Sequenz nach der Substitution:**
  $$
  \sigma(A): p(t) \Rightarrow p(t)
  $$
  - Diese Sequenz ist ein Axiom, weil$p(t) \rightarrow p(t)$ immer wahr ist.

Ein Ableitungsbaum ist also geschlossen, wenn jede Kante des Baums zu einem Knoten führt, der entweder durch Anwendung einer Regel abgeleitet werden kann oder durch Substitution zu einem Axiom wird.


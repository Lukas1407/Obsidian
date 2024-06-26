Eine **Theorie** in der Logik ist eine Menge von Formeln, die bestimmte Eigenschaften und Regeln erfüllen. Sie dient als Grundlage, um strukturelle und logische Eigenschaften zu untersuchen. Hier sind die grundlegenden Begriffe und Definitionen:
## Definition: Theorie
Gegeben eine Signatur$\Sigma$:
- Eine **Theorie**$T \subseteq \text{Fml}_{\Sigma}$ ist eine Menge geschlossener Formeln (Formeln ohne freie Variablen), die folgende Eigenschaften hat:
  1. **Abgeschlossenheit unter logischer Konsequenz:** Wenn$T \models \varphi$, dann$\varphi \in T$.
     - Das bedeutet, dass jede Formel, die eine logische Konsequenz der Formeln in$T$ ist, ebenfalls in$T$ enthalten sein muss.
  2. **Konsistenz:**$T \not\models \bot$.
     - Das bedeutet, dass$T$ keine Widersprüche enthält und es keine Formel in$T$ gibt, die die leere Aussage$\bot$ (Falsch) impliziert.
### Bemerkung:
- **Konsistenz und Erfüllbarkeit:** Eine Theorie$T$ ist genau dann konsistent, wenn$T$ erfüllbar ist.
  - Das bedeutet, dass es eine Interpretation gibt, in der alle Formeln in$T$ wahr sind.
  - Dies ist gleichbedeutend damit, dass$\bot$ (Falsch) nicht in$T$ enthalten ist, da$T$ unter logischer Konsequenz abgeschlossen ist.
### Begrifflichkeiten zu Theorien
1. **T-Struktur:** Eine Struktur$(D, I)$ heißt eine **T-Struktur**, wenn:
   $$ (D, I) \models \varphi \text{ für alle } \varphi \in T $$
   - Das bedeutet, dass die Struktur$(D, I)$ alle Formeln in der Theorie$T$ erfüllt.

2. **T-Modell:** Eine **T-Struktur**$(D, I)$ ist ein **T-Modell** einer Formel$\psi \in \text{Fml}_{\Sigma}$, wenn:
   $$ (D, I) \models \psi $$
   - Das bedeutet, dass die Struktur$(D, I)$ die Formel$\psi$ erfüllt.

3. **T-Erfüllbarkeit:** Eine geschlossene Formel$\psi \in \text{Fml}_{\Sigma}$ ist **T-erfüllbar**, wenn sie ein T-Modell hat.
   - Das bedeutet, dass es eine T-Struktur$(D, I)$ gibt, in der$\psi$ wahr ist.

4. **T-Allgemeingültigkeit:** Eine geschlossene Formel$\psi \in \text{Fml}_{\Sigma}$ ist **T-allgemeingültig**, wenn jede T-Struktur ein T-Modell von$\psi$ ist.
   $$ \text{T-allgemeingültig:} \quad T \models \psi \text{ und } \psi \in T $$
   - Das bedeutet, dass$\psi$ in jeder möglichen Interpretation, die die Theorie$T$ erfüllt, ebenfalls wahr ist.

5. **Definition von$\models_T$:**
   $$ M \models_T \varphi \quad \text{genau dann wenn} \quad M \cup T \models \varphi $$
   - Eine Formel$\varphi$ ist in einer Theorie$T$ gültig, wenn$\varphi$ in der Vereinigung von$M$ und$T$ wahr ist.

6. **Vollständigkeit einer Theorie:** Eine Theorie$T$ heißt **vollständig**, wenn für alle geschlossenen Formeln$\varphi \in \text{Fml}_{\Sigma}$ entweder$\varphi \in T$ oder$\neg \varphi \in T$.
   $$ \text{Vollständig:} \quad \forall \varphi \in \text{Fml}_{\Sigma}: \quad \varphi \in T \quad \text{oder} \quad \neg \varphi \in T $$
   - Das bedeutet, dass die Theorie$T$ über jede Formel eine Aussage trifft, ob sie wahr oder falsch ist, ohne Widersprüche zu enthalten.
### Beispiele

1. **T-Allgemeingültigkeit:**
   - **Formel:**$\forall x \, (x = x)$
   - Diese Formel ist in jeder Theorie allgemeingültig, weil sie in jeder Struktur wahr ist.

2. **T-Erfüllbarkeit:**
   - **Formel:**$\exists x \, (x > 0)$
   - Diese Formel ist in der Theorie der Arithmetik über den reellen Zahlen$\mathbb{R}$ erfüllbar, aber nicht notwendigerweise in der Theorie über den natürlichen Zahlen$\mathbb{N}$, wenn$>$ als strikte Ordnung interpretiert wird.

3. **Vollständige Theorie:**
   - Die Theorie der natürlichen Zahlen mit der Standardinterpretation (Peano-Axiome) ist ein Beispiel für eine vollständige Theorie, da sie über jede arithmetische Aussage entscheidet.

## Definition von Theorien
In der Logik kann eine Theorie auf zwei verschiedene Weisen definiert werden: durch Axiomatisierung oder durch Festlegung einer Interpretation. Jede Methode hat ihre eigenen Merkmale und Konsequenzen für die Struktur und Eigenschaften der Theorie.
### 1. Durch Axiomatisierung
**Definition:** Eine Theorie$T$ kann durch eine Menge$\text{Ax} \subseteq \text{Fml}_{\Sigma}$ von Axiomen definiert werden. Die Theorie$T$ besteht dann aus allen Formeln, die aus diesen Axiomen logisch folgen:

$$ T = T(\text{Ax}) := \{\varphi \mid \text{Ax} \models \varphi \} $$

- **Axiomatisierbarkeit:** Eine Theorie$T$ ist axiomatisierbar, wenn sie durch eine Menge von Axiomen$\text{Ax}$ definiert werden kann. Das bedeutet, dass alle Formeln der Theorie$T$ logische Konsequenzen dieser Axiome sind.
**Beispiele:**
- **Theorie der Gruppen:** Eine Theorie der Gruppen kann durch Axiome definiert werden, die die Eigenschaften einer Gruppe beschreiben, wie Assoziativität, Existenz eines neutralen Elements und Existenz von Inversen.

### 2. Durch Festlegung einer Interpretation
**Definition:** Eine Theorie$T$ kann durch eine Interpretation$(D, I)$ festgelegt werden. Die Theorie$T$ besteht dann aus allen Formeln, die in dieser Interpretation wahr sind:

$$ T = T(D, I) := \{\varphi \mid (D, I) \models \varphi \} $$

- **Vollständigkeit:** Eine Theorie$T(D, I)$, die durch eine Interpretation$(D, I)$ festgelegt wird, ist immer vollständig. Das bedeutet, dass für jede Formel$\varphi$ entweder$\varphi$ oder$\neg \varphi$ in$T(D, I)$ enthalten ist, weil die Interpretation$(D, I)$ über jede Formel entscheidet, ob sie wahr oder falsch ist.
**Beispiele:**

- **Arithmetik über$\mathbb{N}$:** Die Theorie der Arithmetik über den natürlichen Zahlen$\mathbb{N}$ kann durch die Standardinterpretation der natürlichen Zahlen definiert werden, bei der$(D, I) = (\mathbb{N}, I)$.
### Bemerkungen

1. **Vollständigkeit von$T(D, I)$:**
   -$T(D, I)$ ist stets eine vollständige Theorie, weil eine Interpretation$(D, I)$ jede mögliche Formel entweder als wahr oder falsch klassifiziert.

2. **Rekursive Aufzählbarkeit von$T(\text{Ax})$:**
   - Wenn die Menge der Axiome$\text{Ax}$ rekursiv aufzählbar ist, dann ist auch$T(\text{Ax})$ rekursiv aufzählbar. Das bedeutet, dass man eine Methode (Algorithmus) hat, um alle Formeln der Theorie$T(\text{Ax})$ zu erzeugen.
   - Eine Menge ist rekursiv aufzählbar, wenn man alle ihre Elemente in einer endlichen oder unendlichen Liste aufzählen kann, wobei jeder Listeintrag in endlicher Zeit berechenbar ist.

3. **Entscheidbarkeit von$T(\text{Ax})$:**
   - Selbst wenn die Menge der Axiome$\text{Ax}$ endlich oder entscheidbar ist, ist die Theorie$T(\text{Ax})$ im Allgemeinen nicht entscheidbar. Das bedeutet, dass es im Allgemeinen keinen Algorithmus gibt, der für jede Formel$\varphi$ entscheidet, ob$\varphi$ in$T(\text{Ax})$ enthalten ist oder nicht.

4. **Nicht rekursiv aufzählbare Strukturen:**
   - Es gibt Strukturen$(D, I)$, so dass die Theorie$T(D, I)$ nicht rekursiv aufzählbar ist. Das bedeutet, dass es keine Methode gibt, um alle Formeln, die in$T(D, I)$ wahr sind, aufzuzählen.
   - Solche Theorien sind daher auch nicht durch eine rekursiv aufzählbare Menge von Axiomen$\text{Ax}$ axiomatisierbar.

5. **Einzigartigkeit von$(D, I)$:**
   - Eine gegebene Struktur$(D, I)$ ist im Allgemeinen nicht das einzige Modell für$T(D, I)$. Das bedeutet, dass es andere Strukturen geben kann, die die gleiche Menge von Formeln erfüllen.
   - Diese Strukturen sind im Allgemeinen nicht bis auf Isomorphie eindeutig, was bedeutet, dass sie nicht notwendigerweise strukturell gleich sind, auch wenn sie die gleichen Formeln erfüllen.

## Theorien und freie Variablen
In der formalen Logik und insbesondere in der Theorie der Theorien (im Sinne formaler Systeme) gibt es einige wichtige Konzepte im Umgang mit freien Variablen und der Definition von Erfüllbarkeit und Allgemeingültigkeit. Hier sind die Kernpunkte und die dazugehörigen Definitionen:
### Theorien und geschlossene Formeln
- **Theorien:** Per Definition bestehen Theorien$T$ aus geschlossenen Formeln, das heißt, aus Formeln ohne freie Variablen. Dies stellt sicher, dass alle Aussagen innerhalb der Theorie eindeutig sind und keine Interpretation der freien Variablen benötigen.
### Prüfung von T-Erfüllbarkeit und T-Allgemeingültigkeit
- **T-Erfüllbarkeit und T-Allgemeingültigkeit:** Die Formeln, für die man T-Erfüllbarkeit und T-Allgemeingültigkeit prüft, dürfen jedoch freie Variablen enthalten. Diese freien Variablen müssen in irgendeiner Form behandelt werden, um die Erfüllbarkeit oder Allgemeingültigkeit zu überprüfen.
### Existenzabschluss
- **Existenzabschluss:** Der Existenzabschluss einer Formel$\varphi$ mit freien Variablen ist eine Methode, um diese freien Variablen in geschlossene Formeln umzuwandeln, indem man für jede freie Variable eine Existenzquantifizierung einführt.
  
  **Definition:** Für eine Formel$\varphi$ in der Signatur$\Sigma$ mit der Menge der freien Variablen$\text{Frei}(\varphi) = \{x_1, \ldots, x_n\}$ wird der Existenzabschluss$\text{CL}_\exists(\varphi)$ wie folgt definiert:

  $$ \text{CL}_\exists(\varphi) = \exists x_1 \ldots \exists x_n \, \varphi $$

  - Der Existenzabschluss stellt sicher, dass alle freien Variablen$x_1, \ldots, x_n$ durch Existenzquantoren gebunden werden, wodurch die Formel geschlossen wird.
### Definition von T-Erfüllbarkeit für Formeln mit freien Variablen

- **T-Erfüllbarkeit mit freien Variablen:** Eine Formel$\varphi$ in der Signatur$\Sigma$ heißt T-erfüllbar, wenn ihr Existenzabschluss$\text{CL}_\exists(\varphi)$ T-erfüllbar ist.

  **Definition:** 

  $$ \varphi \text{ ist T-erfüllbar} \text{, wenn } \text{CL}_\exists(\varphi) \text{ T-erfüllbar ist} $$

  - Das bedeutet, dass man die T-Erfüllbarkeit einer Formel mit freien Variablen prüft, indem man die freien Variablen durch Existenzquantoren bindet und dann die Erfüllbarkeit der resultierenden geschlossenen Formel innerhalb der Theorie$T$ prüft.
#### Bemerkung zu freien Variablen in T-SAT-Problemen

- **Freie Variablen als Konstanten:** In T-SAT-Problemen (Satisfiability-Problemen, die eine Theorie$T$ betreffen) verhalten sich freie Variablen wie Konstanten, die jedoch nicht in$T$ vorkommen dürfen. Diese Behandlung ermöglicht es, die Erfüllbarkeit der Formel ohne die Notwendigkeit einer spezifischen Bindung der Variablen zu prüfen.

  - Dies unterscheidet sich von der üblichen Interpretation, in der freie Variablen oft als implizit allquantifiziert betrachtet werden, was bedeutet, dass die Formel für alle möglichen Werte der Variablen gelten muss.

  **Vergleich:** 
  - **Freie Variablen als Konstanten:** Sie werden wie spezifische, aber nicht näher bestimmte Werte behandelt, die nicht in der Theorie$T$ definiert sind.
  - **Implizit allquantifiziert:** Die Formel muss für alle möglichen Werte der freien Variablen wahr sein.
### Beispiel
- **Formel mit freien Variablen:** Angenommen, wir haben die Formel$\varphi(x) = P(x) \land Q(y)$ mit freien Variablen$x$ und$y$.
- **Existenzabschluss:** Der Existenzabschluss ist:

  $$ \text{CL}_\exists(\varphi) = \exists x \exists y \, (P(x) \land Q(y)) $$

- **T-Erfüllbarkeit:** Um zu prüfen, ob diese Formel$T$-erfüllbar ist, überprüfen wir, ob es eine Interpretation gibt, die den Existenzabschluss$\exists x \exists y \, (P(x) \land Q(y))$ innerhalb der Theorie$T$ erfüllt.



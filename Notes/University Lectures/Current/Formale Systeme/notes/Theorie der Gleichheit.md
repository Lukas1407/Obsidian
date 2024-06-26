Die **Theorie der Gleichheit** ist eine zentrale Theorie in der mathematischen Logik, die sich mit den Eigenschaften und Axiomen der Gleichheit (auch Äquivalenzrelation) befasst. Hier sind die wichtigsten Konzepte, Axiome und Theoreme zur Theorie der Gleichheit:

#### Theorem: Gleichheit ist axiomatisierbar

**Aussage:** Gegeben eine Signatur $\Sigma$, gibt es eine Menge $\text{Eq}_{\Sigma} \subseteq \text{Fml}_{\Sigma}$ von Axiomen, die die Gleichheit vollständig beschreiben.

- **Notation:** $\text{S} \models \varphi$ genau dann, wenn $\text{S} \models_{T(\text{Eq}_{\Sigma})} \varphi^{\approx}$
  - $\text{S} \models \varphi$: $\varphi$ ist wahr in der Struktur $\text{S}$.
  - $\text{S} \models_{T(\text{Eq}_{\Sigma})} \varphi^{\approx}$: $\varphi^{\approx}$ ist wahr in der Theorie $T(\text{Eq}_{\Sigma})$, wobei $\text{Eq}_{\Sigma}$ die Axiome der Gleichheit enthält.

- **Ersatz von Symbolen:**
  - $\varphi^{\approx}$: Die Formel $\varphi$, wobei das besonders interpretierte Symbol „.=\“ durch das uninterpretierte Symbol „$\approx$“ ersetzt wird.
  
  Das bedeutet, dass eine Formel $\varphi$, die die Gleichheit verwendet, axiomatisch durch $\text{Eq}_{\Sigma}$ beschrieben werden kann. Eine Struktur $S$ erfüllt $\varphi$, wenn sie auch $\varphi^{\approx}$ in der Theorie $T(\text{Eq}_{\Sigma})$ erfüllt, wo „.=\“ durch „$\approx$“ ersetzt wird.

#### Entscheidbarkeit

- **SAT für UF+Gleichheit:** Das Erfüllbarkeitsproblem für uninterpretierte Funktionssymbole und Gleichheit ist aufzählbar, aber nicht entscheidbar. Das bedeutet, dass es keine allgemeine Methode gibt, die in endlicher Zeit für jede Formel entscheidet, ob sie erfüllbar ist.
- **Nutzung von Axiomen und Kalkülen:** Die Verwendung von Axiomen und vollständigen Kalkülen für die Prädikatenlogik zur Entscheidung von Erfüllbarkeitsproblemen ist möglich, aber ineffizient. Effizientere Verfahren werden später in der Vorlesung behandelt.

#### Axiome für die Theorie der Gleichheit

Die Theorie der Gleichheit $\text{Eq}_{\Sigma}$ basiert auf folgenden grundlegenden Axiomen:

1. **Reflexivität:** Jedes Element ist mit sich selbst gleich.
   $$ \forall x \, (x \approx x) $$

2. **Kongruenz für Funktionssymbole:** Wenn zwei Tupel von Elementen paarweise gleich sind, dann sind auch die Ergebnisse der Anwendung eines Funktionssymbols auf diese Tupel gleich.
   $$ \forall x_1 \ldots \forall x_n \forall x_1' \ldots \forall x_n' \, \left( (x_1 \approx x_1' \land \ldots \land x_n \approx x_n') \rightarrow f(x_1, \ldots, x_n) \approx f(x_1', \ldots, x_n') \right) $$
   - Dies gilt für alle Funktionssymbole $f$ in $\Sigma$ mit der Stelligkeit $n$.

3. **Kongruenz für Prädikaten:**
   $$ \forall x_1 \ldots \forall x_n \forall x_1' \ldots \forall x_n' \, \left( (x_1 \approx x_1' \land \ldots \land x_n \approx x_n') \rightarrow (p(x_1, \ldots, x_n) \leftrightarrow p(x_1', \ldots, x_n')) \right) $$
   - Dies gilt für alle Prädikaten $p$ in $\Sigma$ mit der Stelligkeit $n$.
   - Auch für den Fall $p = \approx$.

- **Symmetrie:** Die Gleichheit ist symmetrisch, d.h., wenn $x \approx y$, dann gilt auch $y \approx x$. Diese Eigenschaft ergibt sich aus den obigen Axiomen.
  
  $$ \forall x \, \forall y \, (x \approx y \rightarrow y \approx x) $$

- **Transitivität:** Die Gleichheit ist transitiv, d.h., wenn $x \approx y$ und $y \approx z$, dann gilt auch $x \approx z$. Diese Eigenschaft ergibt sich ebenfalls aus den obigen Axiomen.

  $$ \forall x \, \forall y \, \forall z \, ((x \approx y \land y \approx z) \rightarrow x \approx z) $$

Diese Axiome bilden die Grundlage für die Behandlung von Gleichheit in logischen Systemen und ermöglichen es, formale Beweise und Analysen durchzuführen, die die Gleichheit berücksichtigen.

### Anwendung und Bedeutung der Theorie der Gleichheit

1. **Formale Verifikation:** In der formalen Verifikation wird die Theorie der Gleichheit verwendet, um sicherzustellen, dass Systeme und Programme ihre spezifizierten Eigenschaften erfüllen. Dies ist besonders wichtig in sicherheitskritischen Anwendungen wie der Verifikation von Software und Hardware.

2. **Mathematische Logik:** Die Theorie der Gleichheit spielt eine zentrale Rolle in der mathematischen Logik und der Modelltheorie, da sie die Grundlage für viele weitere logische Theorien bildet.

3. **Theoretische Informatik:** In der theoretischen Informatik wird die Theorie der Gleichheit verwendet, um Algorithmen und Datenstrukturen zu analysieren und zu beweisen, dass sie korrekt und effizient sind.

### Effizientere Verfahren

Während traditionelle Ansätze zur Prüfung der Erfüllbarkeit auf Axiomen und vollständigen Kalkülen beruhen und oft ineffizient sind, gibt es moderne, effizientere Methoden:

- **SMT-Solver:** Werkzeuge wie Z3 oder CVC4, die spezialisierte Algorithmen zur Erfüllbarkeitsprüfung unter Berücksichtigung von Gleichheit und uninterpretierte Funktionssymbole verwenden.
- **E-Graphen:** Eine Datenstruktur, die speziell zur effizienten Handhabung von Gleichheitsbeziehungen und zur Unterstützung von Entscheidungsproblemen entwickelt wurde.

Falls du spezifische Fragen oder Beispiele zur Theorie der Gleichheit oder den effizienteren Verfahren haben möchtest, stehe ich gerne zur Verfügung!


Entscheidungsprobleme in der Logik beziehen sich auf die Frage, ob bestimmte logische Aussagen in einer gegebenen Theorie erfüllbar oder allgemeingültig sind. Hier sind einige wichtige Definitionen und Konzepte im Zusammenhang mit Entscheidungsproblemen:
## Definitionen
1. **Quantoren-freie Formeln $\text{Fml}_{\Sigma}^{\text{QF}}$:**
   - Die Menge $\text{Fml}_{\Sigma}^{\text{QF}}$ ist die Menge aller quantoren-freien Formeln in der Signatur $\Sigma$.
   - Quantoren-freie Formeln enthalten keine Existenz- ($\exists$) oder Allquantoren ($\forall$). Sie bestehen aus logischen Operationen wie UND, ODER, NICHT und Implikation sowie Variablen und Konstanten.
2. **Allgemeine Formeln $\text{Fml}_{\Sigma}$:**
   - Die Menge $\text{Fml}_{\Sigma}$ umfasst alle Formeln in der Signatur $\Sigma$, einschließlich solcher mit Quantoren.
### Zwei Klassen von Entscheidungsproblemen
Gegeben eine Theorie $T$:
1. **SAT (Satisfiability):**
   - **Problemstellung:** Ist $\varphi$ eine T-erfüllbare Formel für $\varphi \in \text{Fml}_{\Sigma}$?
   - **Beschreibung:** Man prüft, ob es eine Interpretation gibt, die sowohl die Axiome der Theorie $T$ als auch die gegebene Formel $\varphi$ erfüllt. 
     $$ \text{SAT:} \quad T \models \varphi $$
2. **QF-SAT (Quantifier-Free Satisfiability):**
   - **Problemstellung:** Ist $\varphi$ eine T-erfüllbare Formel für $\varphi \in \text{Fml}_{\Sigma}^{\text{QF}}$?
   - **Beschreibung:** Man prüft, ob es eine Interpretation gibt, die die quantoren-freie Formel $\varphi$ in der Theorie $T$ erfüllt.
     $$ \text{QF-SAT:} \quad T \models \varphi \text{ für } \varphi \in \text{Fml}_{\Sigma}^{\text{QF}} $$
### Merke
1. **Erfüllbarkeit und Allgemeingültigkeit:**
   - Eine Formel $\varphi$ ist T-unerfüllbar, genau dann, wenn ihre Negation $\neg \varphi$ T-allgemeingültig ist, auch für Formeln mit freien Variablen.
     $$ \varphi \text{ ist } T\text{-unerfüllbar} \iff \neg \varphi \text{ ist } T\text{-allgemeingültig} $$
   - Das bedeutet, wenn keine Interpretation $\varphi$ erfüllen kann, dann ist $\neg \varphi$ in jeder möglichen Interpretation innerhalb der Theorie $T$ wahr.

2. **Entscheidung von Erfüllbarkeit und Allgemeingültigkeit:**
   - Entscheidungsverfahren für T-Erfüllbarkeit können auch T-Allgemeingültigkeit entscheiden, weil die Prüfung der Erfüllbarkeit einer Formel gleichbedeutend mit der Prüfung der Allgemeingültigkeit ihrer Negation ist.
     - **T-Erfüllbarkeit prüfen:** Ist $\varphi$ in $T$ erfüllbar?
     - **T-Allgemeingültigkeit prüfen:** Ist $\neg \varphi$ in $T$ erfüllbar? Wenn ja, dann ist $\varphi$ T-unerfüllbar.
     - Dies ermöglicht eine effiziente Nutzung der gleichen Verfahren für beide Problemtypen.

## Zusammenhang und Anwendung

- **SAT und QF-SAT:** Beide Entscheidungsprobleme sind zentrale Fragestellungen in der Logik, insbesondere in der formalen Verifikation und der Modellprüfung. Sie ermöglichen es, die Konsistenz und Gültigkeit logischer Systeme und deren Formeln zu überprüfen.

- **SMT-Solver:** Diese Werkzeuge nutzen die Konzepte von SAT und QF-SAT, um komplexe logische Probleme zu lösen. Sie kombinieren allgemeine Erfüllbarkeitsprüfungen mit spezifischen Theorien, um praktische Anwendungen wie Software- und Hardwareverifikation zu ermöglichen.

- **Praxisrelevanz:** Die Fähigkeit, sowohl allgemeine Formeln als auch quantoren-freie Formeln zu überprüfen, ist in vielen Bereichen der Informatik und Mathematik von entscheidender Bedeutung, da sie hilft, komplexe Systeme zu validieren und zu verifizieren.
### Beispiele für Entscheidungsprobleme

1. **Erfüllbarkeit in der Arithmetik:** 
   - **Formel:** $x + y = z$
   - **Theorie:** Arithmetik über den ganzen Zahlen $\mathbb{Z}$
   - **Problem:** Ist die Formel erfüllbar in $\mathbb{Z}$? (Ja, sie ist erfüllbar, da es immer eine Lösung für die Gleichung gibt.)
2. **Erfüllbarkeit in der Logik der Gleichungen:**
   - **Formel:** $P(x) \land \neg P(x)$
   - **Theorie:** Aussagenlogik
   - **Problem:** Ist die Formel erfüllbar? (Nein, sie ist unerfüllbar, da $P(x)$ und $\neg P(x)$ nicht gleichzeitig wahr sein können.)
3. **QF-SAT in der Bitarithmetik:**
   - **Formel:** $(x \& y) | z = 0$
   - **Theorie:** Bitarithmetik
   - **Problem:** Ist die quantoren-freie Formel erfüllbar? (Das hängt von den Werten von $x$, $y$, und $z$ ab und kann durch einen SMT-Solver überprüft werden.)

### Satisfiability Modulo Theories (SMT)

**Satisfiability Modulo Theories (SMT)** ist ein Ansatz zur Überprüfung der Erfüllbarkeit (Satisfiability) von logischen Formeln in Bezug auf gegebene Theorien. Diese Theorien können mathematische Strukturen oder spezifische Domänen beschreiben, wie zum Beispiel Arithmetik, Listen, Arrays, oder bitweise Operationen. Der SMT-Ansatz ist besonders nützlich in der Verifikation, der Modellprüfung und der formalen Methodenentwicklung.

#### Was ist SMT?

- **Erfüllbarkeitsprobleme bezüglich gegebener Theorien:** SMT betrachtet die Frage, ob eine logische Formel$\varphi$ in einer bestimmten Theorie$T$ erfüllbar ist. Eine Formel ist in einer Theorie erfüllbar, wenn es eine Interpretation gibt, die sowohl die Axiome der Theorie als auch die gegebene Formel erfüllt.
  
  $$ \text{SMT}: \quad T \models \varphi $$

- **SMT-Solver:** Systeme zur Entscheidung von SMT-Problemen, die auf verschiedenen Techniken basieren, um die Erfüllbarkeit von Formeln in spezifischen Theorien effizient zu überprüfen.

### SMT-Solver

#### Überblick

- **Erfolgreiche Forschung:** In den letzten Jahren hat die Forschung auf diesem Gebiet große Fortschritte gemacht, und es wurden viele leistungsfähige SMT-Solver entwickelt.
  
- **Fokus auf entscheidbare Theorien:** Die meisten SMT-Solver konzentrieren sich auf Theorien, bei denen Erfüllbarkeitsfragen entscheidbar sind. Das bedeutet, dass es Algorithmen gibt, die in endlicher Zeit feststellen können, ob eine Formel erfüllbar ist oder nicht.

- **Viele verfügbare Systeme:** Es gibt eine Vielzahl von SMT-Solvern, die sich in ihrer Funktionalität und Spezialisierung unterscheiden:

  - **Allround-Solver:** Diese Systeme können Erfüllbarkeitsprobleme für eine breite Palette von Theorien behandeln. Beispiele sind:
    - **Z3:** Ein weit verbreiteter und vielseitiger SMT-Solver von Microsoft.
    - **CVC4:** Ein leistungsstarker und flexibler SMT-Solver, der viele Theorien unterstützt.
    - **Yices:** Ein Solver, der sich durch seine Effizienz und breite Unterstützung für verschiedene Theorien auszeichnet.

  - **Spezial-Solver:** Diese Systeme sind für spezifische Theorien oder Problemtypen optimiert. Beispiele sind:
    - **MathSAT:** Speziell für arithmetische Probleme.
    - **Simplify:** Ein älterer, aber immer noch wichtiger SMT-Solver.

#### Merkmale von SMT-Solvern

- **Sammlungen von Spezialverfahren:** SMT-Solver verwenden in der Regel eine Kombination von spezialisierten Algorithmen und Verfahren, die auf die spezifischen Eigenschaften der behandelten Theorien abgestimmt sind.
  - **Nicht allgemeine Verfahren:** Es gibt kein einziges allgemeines Verfahren, das für alle Theorien und Problemtypen geeignet ist.
  - **Keine Axiome + allgemeiner Kalkül:** Stattdessen werden spezifische Algorithmen für die Behandlung bestimmter Theorien eingesetzt, anstatt allgemeine axiomatische Ansätze zu verwenden.

- **Standardisiertes Eingabeformat:** Das standardisierte Eingabeformat für SMT-Probleme ist **SMT-Lib 2**. Es definiert eine Syntax und Semantik für die Darstellung von SMT-Problemen, die von den meisten SMT-Solvern unterstützt wird.

### Beispielanwendungen

- **Softwareverifikation:** Überprüfung, ob bestimmte Programmteile bestimmte Eigenschaften erfüllen, wie etwa die Einhaltung von Sicherheitsrichtlinien oder die Vermeidung von Laufzeitfehlern.
  
- **Hardwareverifikation:** Verifizierung von Schaltkreisen und Hardwaredesigns, um sicherzustellen, dass sie spezifikationsgemäß funktionieren.
  
- **Planung und Scheduling:** Überprüfung der Machbarkeit von Plänen oder Zeitplänen unter Berücksichtigung von Ressourcen- und Zeitbeschränkungen.

### Funktionsweise von SMT-Solvern

- **Kombination von SAT und Theorie-Solvern:** Die meisten SMT-Solver kombinieren SAT-Solver (für die Erfüllbarkeit von Aussagenlogik) mit speziellen Theorie-Solvern, die die Erfüllbarkeit von Formeln in spezifischen Theorien prüfen. Diese Methode wird als **Lazy SMT** bezeichnet und ermöglicht es, komplexe logische Probleme effizient zu lösen.

- **Eingabeverarbeitung:** Die Eingabe in SMT-Lib 2 wird in einer präzisen und standardisierten Form dargestellt, die die Definition von Variablen, Funktionssymbolen und logischen Beziehungen ermöglicht.

### Beispiele für Theorien in SMT

1. **Arithmetik über Ganzzahlen und reelle Zahlen:**
   - Formeln, die arithmetische Ausdrücke enthalten und deren Erfüllbarkeit geprüft wird.
   - Beispiel:$x + y = z$

2. **Arrays und Listen:**
   - Formeln, die Operationen auf Datenstrukturen wie Arrays oder Listen beschreiben.
   - Beispiel:$A[i] = x$

3. **Bitweise Operationen:**
   - Formeln, die auf bitweisen Operationen wie AND, OR oder NOT basieren.
   - Beispiel:$x \& y = z$


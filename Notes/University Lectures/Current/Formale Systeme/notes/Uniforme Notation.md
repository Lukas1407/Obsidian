Die Vorzeichen 0 und 1 werden verwendet, um die Gültigkeit oder Ungültigkeit von Formeln zu kennzeichnen. Lassen Sie uns die einzelnen Teile genauer betrachten.
### Konjunktive und Disjunktive Formeln
Diese Kategorien unterscheiden, ob die Formeln mit UND (∧) oder ODER (∨) operieren:
- **Konjunktive Formeln (Typ α):**
  - **1(A ∧ B)**: Die Konjunktion $A$ und $B$ ist wahr.
  - **0(A ∨ B)**: Die Disjunktion $A$ oder $B$ ist falsch.
  - **0(A → B)**: Die Implikation $A$ impliziert $B$ ist falsch.
  - **0¬A**: Die Negation von $A$ ist falsch, also ist $A$ wahr.
  - **1¬A**: Die Negation von $A$ ist wahr, also ist $A$ falsch.
- **Disjunktive Formeln (Typ β):**
  - **0(A ∧ B)**: Die Konjunktion $A$ und $B$ ist falsch.
  - **1(A ∨ B)**: Die Disjunktion $A$ oder $B$ ist wahr.
  - **1(A → B)**: Die Implikation $A$ impliziert $B$ ist wahr.
  - **1¬A**: Die Negation von $A$ ist wahr, also ist $A$ falsch.
### Universelle und Existenzielle Formeln
Diese Formeln behandeln Quantoren, die für alle oder einige Elemente einer Menge gelten:
- **Universelle Formeln (Typ γ):**
  - **1∀xA(x)**: Die Aussage $A(x)$ ist wahr für alle $x$.
  - **0∃xA(x)**: Es gibt kein $x$, für das die Aussage $A(x)$ wahr ist.
- **Existenzielle Formeln (Typ δ):**
  - **1∃xA(x)**: Es gibt ein $x$, für das die Aussage $A(x)$ wahr ist.
  - **0∀xA(x)**: Die Aussage $A(x)$ ist nicht für alle $x$ wahr.
### Disjunktive Formeln (Typ β)
1. **0(A ∧ B)** wird in zwei mögliche Fälle aufgeteilt:
   - **β1: 0A** (A ist falsch) oder
   - **β2: 0B** (B ist falsch)
2. **1(A ∨ B)**:
   - **β1: 1A** (A ist wahr) oder
   - **β2: 1B** (B ist wahr)
3. **1(A → B)**:
   - **β1: 0A** (A ist falsch) oder
   - **β2: 1B** (B ist wahr)


> [!abstract] Definition
> Das Davis-Putnam-Logemann-Loveland-Verfahren, kurz DPLL-Verfahren, ist ein zentrales Verfahren zur Lösung des Erfüllbarkeitsproblems ([[SAT Problem]]) für logische Formeln in [[Konjunktive Normalform (KNF)]].

## Grundlegende Konzepte des DPLL-Verfahrens 

1. **Literale und Variablen**:
   - Ein Literal ist eine Variable oder deren Negation (z.B. $A$ oder $\neg A$).
2. **[[Klausel]]**:
   - Eine Klausel ist eine Disjunktion von Literalen (z.B. $A \lor \neg B \lor C$).
3. **Backtracking**:
   - Das Verfahren verwendet eine rekursive Suchstrategie, um die Erfüllbarkeit der Formel zu prüfen. Es untersucht dabei mögliche Zuweisungen der Variablen und nutzt Backtracking, wenn eine Zuweisung zu einem Widerspruch führt.
## Schritte des DPLL-Verfahrens
1. **Löschen von Klauseln mit wahren Literalen**:
   - Wenn eine Klausel ein wahres Literal enthält, wird die Klausel als erfüllt betrachtet und aus der Formel entfernt.
2. **Löschen von negierten Literalen aus Klauseln**:
   - Wenn eine Variable den Wert "wahr" erhält, werden alle Klauseln, in denen das negative Literal dieser Variable vorkommt, um dieses Literal reduziert.
3. **Unit-Propagation (Einheitsklausel-Ausbreitung)**:
   - Eine Einheitsklausel ist eine Klausel mit nur einem Literal. Diese Klausel muss wahr sein, damit die gesamte Formel erfüllt ist. Daher wird die entsprechende Variable so gesetzt, dass die Klausel erfüllt wird, und dies wird auf die Formel angewendet.
4. **Wahl und Zuweisung einer Variable**:
   - Eine noch nicht zugewiesene Variable wird ausgewählt und es werden beide möglichen Werte ("wahr" und "falsch") ausprobiert.
5. **Backtracking**:
   - Wenn eine Zuweisung zu einem Widerspruch führt, wird diese Zuweisung zurückgenommen und eine alternative Zuweisung wird ausprobiert.
### Beispiel
Sehen wir uns eine einfache KNF-Formel an und prüfen wir ihre Erfüllbarkeit mithilfe des DPLL-Verfahrens:
$$C = (A \lor \neg B) \land (\neg A \lor B \lor C) \land (\neg C \lor B) \land (\neg A \lor \neg C)$$
1. **Formel**: $(A \lor \neg B) \land (\neg A \lor B \lor C) \land (\neg C \lor B) \land (\neg A \lor \neg C)$
2. **Einheitsklausel**:
   - Es gibt keine Einheitsklausel.
3. **Wahl einer Variable**:
   - Wählen wir $A$ und setzen es auf "wahr" ($A = wahr$).
4. **Reduzieren der Formel**:
   - Setze $A$ auf "wahr". Die Klauseln, die $A$ enthalten, werden erfüllt:$$\text{Entfernen: } (A \lor \neg B), (\neg A \lor B \lor C), (\neg A \lor \neg C)$$
   - Reduzierte Formel:$$(\neg C \lor B)$$

1. **Einheitsklausel**:
   - Keine Einheitsklausel.
2. **Wahl einer weiteren Variable**:
   - Wählen wir $C$ und setzen es auf "wahr" ($C = wahr$).
3. **Reduzieren der Formel**:
   - Setze $C$ auf "wahr". Die Klausel $(\neg C \lor B)$ wird reduziert:$$\text{Reduziere: } (\neg C \lor B) \rightarrow (B)$$
4. **Einheitsklausel**:
   - Nun gibt es eine Einheitsklausel: $B$.
5. **Setze $B$ auf "wahr"**:
   - Setze $B$ auf "wahr".
6. **Formel wird leer**:
    - Alle Klauseln sind nun erfüllt.

Die Formel ist erfüllbar, da wir eine Zuweisung gefunden haben: $A = wahr$, $B = wahr$, $C = wahr$.

## Python-Implementierung
Hier ist eine Python-Implementierung des DPLL-Verfahrens, die dieses Beispiel veranschaulicht:

```python
def dpll(clauses, assignment):
    # Entfernen von erfüllten Klauseln
    clauses = [clause for clause in clauses if not any(literal in assignment for literal in clause)]
    
    # Entfernen von negierten Literalen
    clauses = [[literal for literal in clause if -literal not in assignment] for clause in clauses]

    # Leere Klausel bedeutet unerfüllbar
    if any(len(clause) == 0 for clause in clauses):
        return False
    
    # Keine Klauseln mehr, bedeutet erfüllbar
    if not clauses:
        return True

    # Einheitsklausel-Ausbreitung
    for clause in clauses:
        if len(clause) == 1:
            literal = clause[0]
            return dpll(clauses, assignment + [literal])

    # Variable wählen und zuweisen
    literal = clauses[0][0]
    return dpll(clauses, assignment + [literal]) or dpll(clauses, assignment + [-literal])

# Definition der Klauseln
clauses = [[1, -2], [-1, 2, 3], [-3, 2], [-1, -3]]

# Start des DPLL-Verfahrens
result = dpll(clauses, [])
print(f"Die Formel ist {'erfüllbar' if result else 'unerfüllbar'}")
```
## Theorem DPLL
1. **Terminierung**:
   - Der DPLL-Algorithmus terminiert für jede Eingabe. Das bedeutet, dass der Algorithmus immer zu einem Ergebnis kommt und nicht in einer Endlosschleife festhängt.
2. **Korrektheit und Vollständigkeit**:
   - Der DPLL-Algorithmus ist korrekt und vollständig.
     - **Korrektheit** bedeutet, dass der Algorithmus nur dann eine Formel als erfüllbar zurückgibt, wenn sie tatsächlich erfüllbar ist.
     - **Vollständigkeit** bedeutet, dass der Algorithmus alle erfüllbaren Formeln als erfüllbar erkennt und keine möglichen Lösungen übersieht.
### Beweisidee
#### 1. Terminierung
- Bei jeder Reduktion fällt mindestens ein Atom (Variable) weg. Da die Anzahl der Variablen in der Eingabeformel endlich ist, kann der Algorithmus nicht unendlich viele Schritte machen. Jeder Schritt des DPLL-Verfahrens reduziert die Anzahl der unbestimmten Literale, entweder durch direktes Setzen eines Werts oder durch Ausschluss von Möglichkeiten. Daher muss der Algorithmus zwangsläufig enden.
#### 2. Korrektheit und Vollständigkeit
- **Erfüllbarkeitsäquivalente Klauselmenge**:
  Jeder Schritt des DPLL-Algorithmus transformiert die Formel in eine äquivalente Klauselmenge hinsichtlich der Erfüllbarkeit. Das bedeutet, dass die Transformationsschritte, wie das Entfernen erfüllter Klauseln oder die Reduktion von Klauseln durch Einheitsklausel-Ausbreitung, die grundlegenden Erfüllbarkeitsbedingungen der Formel nicht verändern.
- **Erfüllbarkeit von ∅**:
  Die leere Klauselmenge (∅) ist per Definition erfüllbar, da keine Einschränkungen vorliegen, die eine Erfüllung verhindern könnten. Wenn der DPLL-Algorithmus eine Formel so weit vereinfacht, dass keine Klauseln mehr übrig sind, bedeutet dies, dass eine Belegung gefunden wurde, die alle ursprünglichen Klauseln erfüllt.
- **Unerfüllbarkeit von {□}**:
  Eine Klauselmenge, die die leere Klausel (□) enthält, ist unerfüllbar. Die leere Klausel repräsentiert einen logischen Widerspruch (immer falsch), sodass keine Zuweisung der Variablen die Formel erfüllbar machen kann.


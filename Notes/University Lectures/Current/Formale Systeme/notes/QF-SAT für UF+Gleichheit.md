### Kongruenzabschluss: Beispiel und Verfahren zur Lösung von QF-SAT für UF+Gleichheit

#### Kongruenzabschluss: Beispiel
![[Pasted image 20240622110723.png#invert|400]]
Das Bild zeigt ein Beispiel für den **Kongruenzabschluss**. Hier wird eine Menge von Formeln in Bezug auf Gleichheit analysiert, um zu überprüfen, ob die Menge erfüllbar ist. Jede Box repräsentiert eine Gleichheitsklasse, und die Pfeile zeigen, wie Terme durch Anwendung von Gleichungen und Gleichheitsrelationen verbunden werden.

**Formelmenge:**
1. $f(g(a)) \doteq g(f(b))$ (A)
2. $\neg a \doteq b$ (B)
3. $g(b) \doteq b$ (C)
4. $g(a) \doteq a$ (D)
5. $f(b) \doteq a$ (E)
6. $\neg f(b) \doteq g(b)$ (F)

Die Formelmenge ist erfüllbar, was durch den Kongruenzabschluss überprüft wird.

### QF-SAT für UF+Gleichheit

Das Problem der quantorenfreien Erfüllbarkeit (QF-SAT) in der Theorie der uninterpretieren Funktionssymbole und Gleichheit (UF+Gleichheit) kann effizient gelöst werden, indem man die Terme und ihre Beziehungen analysiert. Hierbei wird der **Kongruenzabschluss** verwendet, um die Erfüllbarkeit zu bestimmen.

#### Schritte des Verfahrens nach Shostak

1. **Formeldarstellung:**
   - Eine quantorenfreie Formel wird in disjunktive Normalform (DNF) gebracht und die Klauseln werden überprüft.
   - Die Formel hat die Form:
     $$ s_1 \doteq t_1 \land \ldots \land s_n \doteq t_n \land \neg(s'_1 \doteq t'_1) \land \ldots \land \neg(s'_m \doteq t'_m) $$
     - $s_i \doteq t_i$ sind Gleichungen.
     - $\neg(s'_i \doteq t'_i)$ sind Ungleichungen.

2. **Initialisierung:**
   - Jeder vorkommende Term und Unterterm $t$ wird in eine eigene Gleichheitsklasse $K_t$ gesetzt.
   - Zum Beispiel: Die Terme $a, b, g(a), g(b), f(b), f(g(a)), g(f(b))$ bilden zunächst eigene Klassen.

3. **Kongruenzschluss-Schleife:**
   - In jeder Iteration der Schleife werden Klassen vereinigt, basierend auf:
     - **Gleichungen:** Wenn $s_i$ in $K$ und $t_i$ in $K'$ liegen, werden die Klassen $K$ und $K'$ vereinigt.
     - **Transitivität:** Wenn $K \cap K' \neq \emptyset$, werden die Klassen $K$ und $K'$ vereinigt.
     - **Kongruenz:** Wenn für alle $i$ $u_i \in K_i$ und $v_i \in K_i$ gelten und $f(u_1, \ldots, u_n) \in K$ sowie $f(v_1, \ldots, v_n) \in K'$, dann werden $K$ und $K'$ vereinigt.

4. **Terminierung:** 
   - Die Schleife terminiert, wenn keine weiteren Änderungen der Klassen mehr auftreten. Das heißt, es gibt keine weiteren Terme, die zu bestehenden Klassen hinzugefügt werden müssen.

5. **Ergebnis:**
   - **Unerfüllbar:** Wenn es eine Klasse $K$ gibt, die sowohl $s'_i$ als auch $t'_i$ enthält, wird die Formel als unerfüllbar bezeichnet. Dies bedeutet, dass die Gleichheitsbeziehungen in einem Widerspruch enden.
   - **Erfüllbar:** Wenn keine solchen Widersprüche gefunden werden, ist die Formel erfüllbar.

### Beispiel: Anwendung des Verfahrens

Betrachten wir die Formelmenge aus dem Bild:
- **Initialisierung:**
  - Klassen: $K_{f(g(a))}, K_{g(f(b))}, K_{a}, K_{b}, K_{g(a)}, K_{g(b)}, K_{f(b)}$

- **Schleife:**
  - **Gleichungen:**
    - Vereinige $K_{f(g(a))}$ und $K_{g(f(b))}$ aufgrund von (A).
    - Vereinige $K_{g(a)}$ und $K_{a}$ aufgrund von (D).
    - Vereinige $K_{f(b)}$ und $K_{a}$ aufgrund von (E).
    - Vereinige $K_{g(b)}$ und $K_{b}$ aufgrund von (C).
  - **Transitivität:**
    - Vereinige $K_{a}$ und $K_{g(a)}$, wenn $K_{g(a)} \cap K_{a} \neq \emptyset$.
  - **Kongruenz:** Prüfe, ob für Funktionssymbole die Gleichheit durch die Klassenkonsistenz beibehalten wird.

- **Terminierung:** Keine weiteren Änderungen der Klassen möglich.
- **Ergebnis:** 
  - Die Formelmenge ist erfüllbar, da kein Widerspruch vorliegt (keine Klasse enthält beide Terme eines negierten Gleichheitspaares).

### Zusammenfassung

- Der **Kongruenzabschluss** ist ein effizientes Verfahren zur Überprüfung der Erfüllbarkeit von quantorenfreien Formeln in der Theorie der uninterpretieren Funktionssymbole und Gleichheit.
- Das Verfahren basiert auf der Vereinheitlichung von Klassen durch Anwendung von Gleichheits- und Transitivitätsregeln sowie Kongruenzbedingungen.
- **QF-SAT** ist entscheidbar, während **SAT** für allgemeine Formeln co-semidecidierbar ist.

Falls du weitere Fragen zu diesem Thema hast oder detaillierte Erklärungen zu einzelnen Schritten benötigst, lass es mich wissen!


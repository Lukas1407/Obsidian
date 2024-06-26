Das Nelson-Oppen-Verfahren ist ein weit verbreitetes Verfahren zur Überprüfung der Erfüllbarkeit von Formeln, die aus verschiedenen Theorien bestehen. Es basiert auf der Idee, die Formeln in getrennte Teile zu zerlegen, die dann in ihren jeweiligen Theorien analysiert werden. Lass uns die einzelnen Schritte des Verfahrens im Detail betrachten und verstehen, wie sie angewendet werden können.

### Fragestellung

Gegeben ist eine Konjunktion von Literalen aus der Menge $\text{Fml}_{\Sigma_1 \cup \Sigma_2}$. Die Aufgabe ist zu entscheiden, ob diese Konjunktion in der kombinierten Theorie $T_{1,2}$ erfüllbar ist.

### Idee des Nelson-Oppen-Verfahrens

1. **Entmischung der Theorien**: Zerlege die Konjunktion in zwei getrennte Konjunktionen $\varphi_1$ und $\varphi_2$, die jeweils nur Formeln der Theorien $T_1$ und $T_2$ enthalten. Das erfolgt oft durch die Einführung von Abkürzungen und neuen Variablen, um die Theorien zu trennen.

2. **Schleife**:
   - In beiden Theorien werden neue Gleichungen zwischen Variablen abgeleitet.
   - Diese Gleichungen werden ausgetauscht und in beiden Theorien überprüft.

3. **Terminierung**: Das Verfahren endet, wenn einer der folgenden Fälle eintritt:
   - **Unvereinbarkeit**: Ein Teil $\varphi_i \cup \tau$ wird $T_i$-unerfüllbar, was bedeutet, dass die ursprüngliche Formel unerfüllbar ist.
   - **Stabilität**: Es können keine neuen Gleichungen mehr abgeleitet werden, was darauf hinweist, dass die Formel erfüllbar ist.

### Beispiel: Entmischung der Theorien

Nehmen wir die Formel:
$$ f(a) = g(a + 1) \land g(a + b) > f(a) $$

Wir wollen diese Formel in zwei getrennte Theorien entmischen:

#### Schritt 1: Einführung von Abkürzungen

- Ersetze komplexe Terme durch neue Variablen:
  $$ y = a + 1 $$
  $$ u = a + b $$
  $$ z = g(u) $$
  $$ w = f(a) $$

#### Schritt 2: Aufteilung in getrennte Konjunktionen

- Die ursprüngliche Formel kann dann umgeschrieben werden als:
  $$ f(a) = g(y) \land y = a + 1 \land z = g(u) \land u = a + b \land w = f(a) \land z > w $$

- Diese Formel zerfällt in zwei separate Konjunktionen:
  $$ \varphi_1: y = a + 1 \land u = a + b $$
  $$ \varphi_2: f(a) = g(y) \land z = g(u) \land w = f(a) \land z > w $$

#### Schritt 3: Austausch von Gleichungen

- Jetzt müssen wir die Gleichungen zwischen den Variablen $y$, $u$, $z$, und $w$ überprüfen und sicherstellen, dass sie in beiden Theorien konsistent sind.

### Praktische Durchführung

1. **Entmischung**: Zerlege die Formel in $\varphi_1$ und $\varphi_2$.
2. **Initialisierung**: Starte mit den abgeleiteten Gleichungen zwischen Variablen.
3. **Iterative Schleife**:
   - In $T_1$: Analysiere und leite mögliche Gleichungen ab, z.B., ob $y$ gleich $a + 1$ sein kann.
   - In $T_2$: Prüfe die abgeleiteten Gleichungen und integriere sie.
   - Tausche die Ergebnisse aus und füge neue Gleichungen hinzu.

4. **Terminierung**:
   - Wenn eine der Formeln $\varphi_i \cup \tau$ unerfüllbar ist, ist die gesamte Formel unerfüllbar.
   - Wenn keine neuen Gleichungen abgeleitet werden können, ist die Formel erfüllbar.

### Fazit

Das Nelson-Oppen-Verfahren ermöglicht es, die Erfüllbarkeit von Formeln aus kombinierten Theorien durch systematische Entmischung und iterativen Austausch von Gleichungen zu überprüfen. Diese Methodik ist besonders nützlich, wenn die einzelnen Theorien strukturell unabhängig sind und keine gemeinsamen Funktions- oder Prädikatsymbole haben.

Hast du noch Fragen zum Verfahren oder möchtest du mehr über ein spezifisches Beispiel erfahren?

### Nelson-Oppen-Verfahren

Das Nelson-Oppen-Verfahren ist ein bewährtes Verfahren zur Überprüfung der Erfüllbarkeit von quantorenfreien Konjunktionen von Literalen in kombinierten Theorien. Lass uns die Details und die notwendigen Einschränkungen dieses Verfahrens untersuchen.

#### Theorem

Das Nelson-Oppen-Verfahren entscheidet korrekt die quantorenfreie Erfüllbarkeit (QF-SAT) für Konjunktionen von Literalen, wenn folgende Bedingungen erfüllt sind:

1. **Disjunkte Signaturen**: $\Sigma_1 \cap \Sigma_2 = \emptyset$
   - Die Theorien dürfen keine gemeinsamen Funktions- oder Prädikatsymbole haben.
  
2. **Konvexität der Theorien**: Beide Theorien müssen konvex sein. Das bedeutet, dass für eine Theorie $T$ gilt:
   - Wenn $\varphi \rightarrow (x_1 = y_1 \lor \ldots \lor x_n = y_n)$ gilt, dann folgt $\varphi \rightarrow x_i = y_i$ für ein $i \in \{1, \ldots, n\}$.
   - Dies stellt sicher, dass jede erfüllbare Bedingung auf genau eine Lösung verweisbar ist, ohne dass mehrere gleichzeitig gültig sein müssen.

3. **Stabil unendlich**: Beide Theorien müssen stabil unendlich sein. Das bedeutet, dass:
   - Wenn eine Formel in $T_i$ erfüllbar ist, dann existiert ein $T$-Modell mit unendlicher Domäne.
   - Dies verhindert Probleme, die bei endlichen Modellen auftreten könnten, wie etwa Grenzen bei der Anzahl der Werte oder Variablen.

### Beispiel einer Konjunktion von Literalen

Betrachten wir eine Formel:
$$ f(a) = g(a + 1) \land g(a + b) > f(a) $$

Wir entmischen diese Formel in zwei Konjunktionen:
$$ \varphi_1: y = a + 1 \land u = a + b $$
$$ \varphi_2: f(a) = g(y) \land z = g(u) \land w = f(a) \land z > w $$

Das Nelson-Oppen-Verfahren würde diese beiden Theorien getrennt analysieren und die Erfüllbarkeit prüfen, indem neue Gleichungen zwischen Variablen ausgetauscht werden.

### Einschränkungen des Nelson-Oppen-Verfahrens

- **Disjunkte Signaturen**: $\Sigma_1 \cap \Sigma_2 = \emptyset$
  - Die Theorien dürfen keine gemeinsamen Symbole haben, damit sie unabhängig voneinander analysiert werden können.

- **Konvexität**: Theorien müssen so gestaltet sein, dass die Erfüllbarkeit der Disjunktion von Gleichungen auf die Erfüllbarkeit einer einzelnen Gleichung reduziert werden kann.

- **Stabil unendlich**: Die Theorien sollten Modelle mit unendlichen Domänen haben, um die allgemeine Gültigkeit der Schlussfolgerungen sicherzustellen.


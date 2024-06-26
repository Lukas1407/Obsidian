### Gleichungslogik und Kongruenzabschluss

In der Gleichungslogik geht es um das Prüfen, ob eine bestimmte Gleichung $s = t$ aus einer Menge von Gleichungen $E$ ableitbar ist. Wir wollen klären, wie man die Erfüllbarkeit solcher Gleichungen überprüfen kann und welche Werkzeuge dafür zur Verfügung stehen.

#### Fragestellung

Gegeben ist eine Menge $E$ von Gleichungen über einer Signatur $\Sigma$ und zwei Terme $s$ und $t$, die ebenfalls aus der Signatur $\Sigma$ gebildet sind. Die Frage ist:

**Gilt $E \models s = t$?**

Das bedeutet: Ist die Gleichung $s = t$ eine logische Konsequenz der Gleichungen in $E$?

#### Bemerkung

Die Variablen in $E$ sind implizit allquantifiziert. Das heißt, sie gelten für alle möglichen Belegungen der Variablen.

**Formale Bedingung:** $E \models s = t$ gilt genau dann, wenn $\forall E \rightarrow s = t$ in der Theorie $T$ allgemein gültig ist, wobei $T$ die Theorie der Gleichheit und der uninterpretierten Funktionssymbole ist.

#### Theorie der Gleichheit und uninterpretierten Funktionssymbole

Diese Theorie umfasst:
- **Gleichheit:** $=$
- **Uninterpretierte Funktionssymbole:** Funktionssymbole, die keine festgelegte Bedeutung haben und nur aufgrund ihrer Struktur und der Gleichheitsaxiome verwendet werden.

### Kongruenzabschlussverfahren nach Shostak

Das Kongruenzabschlussverfahren ist eine Technik, die verwendet wird, um zu überprüfen, ob eine Gleichung aus einer Menge von Gleichungen in der Theorie der Gleichheit ableitbar ist. Dieses Verfahren erlaubt es, die einfachere Frage zu beantworten:

**Ist $\exists E \rightarrow s = t$ in der Theorie $T$ allgemein gültig?**

Das bedeutet, dass wir überprüfen, ob es eine Belegung der Variablen gibt, die $s = t$ aus den Gleichungen in $E$ erfüllt.

### Kongruenzabschlussverfahren – Schritt für Schritt

1. **Eingabe:** Eine Menge von Gleichungen $E$ und eine Gleichung $s = t$.
2. **Termknoten:** Erzeuge Knoten für jeden Term in $E$, $s$, und $t$.
3. **Initiale Gleichungen:** Verbinde Knoten gemäß den Gleichungen in $E$.
4. **Unifikationen:** Füge weitere Gleichungen hinzu, indem du die Knoten basierend auf der Struktur der Terme gleichsetzt (unifiziere).
5. **Kongruenz:** Verfeinere die Knotenbeziehungen, bis alle notwendigen Gleichungen erfasst sind.
6. **Überprüfung:** Prüfe, ob die Knoten von $s$ und $t$ verbunden sind.

#### Beispiel

Betrachten wir ein einfaches Beispiel:

**Gegeben:**
- $E = \{ f(a) = g(b), b = c \}$
- Frage: Gilt $E \models f(a) = g(c)$?

**Schritt-für-Schritt:**

1. **Eingabe:**
   - Terme: $f(a), g(b), g(c)$
   - Gleichungen: $f(a) = g(b)$ und $b = c$

2. **Initiale Knoten:**
   - Erzeuge Knoten für $f(a), g(b), g(c), a, b, c$

3. **Verbindungen:**
   - Verbinde $f(a)$ mit $g(b)$
   - Verbinde $b$ mit $c$

4. **Unifikation:**
   - Keine neuen Unifikationen notwendig.

5. **Kongruenz:**
   - Da $g(b) = g(c)$ durch $b = c$ impliziert ist, verbinden wir $g(b)$ und $g(c)$.

6. **Überprüfung:**
   - $f(a)$ ist verbunden mit $g(b)$, und $g(b)$ ist verbunden mit $g(c)$.
   - Also, $f(a)$ ist verbunden mit $g(c)$.

**Ergebnis:** $E \models f(a) = g(c)$

### Ersetzung von Gleichem durch Gleiches: Termersetzung

In der formalen Logik und Algebra ist das Ersetzen von gleichen Ausdrücken durch gleiche Ausdrücke ein grundlegender Prozess, der es ermöglicht, komplexe Terme zu vereinfachen oder umzuschreiben. Diese Technik wird häufig in Beweissystemen und automatisierten Theoremprovern eingesetzt.

### Termersetzung: Definition

Gegeben eine Menge $E$ von Gleichungen über einer Signatur $\Sigma$ und Terme $s$, $t$ aus der Menge der Terme $\text{Term}_\Sigma$, dann gilt:

**$s \rightarrow_E t$** genau dann, wenn es eine Gleichung $l = r \in E$ und eine Substitution $\sigma$ gibt, so dass:

- $\sigma(l)$ ist ein Unterterm von $s$.
- $t$ entsteht aus $s$, indem der Unterterm $\sigma(l)$ an genau einer Stelle durch $\sigma(r)$ ersetzt wird.

### Beispiel für Termersetzung

**Gegeben:** $E = \{ (x + y) \cdot z = x \cdot z + y \cdot z \}$

**Terme:**

- $s$: $u \cdot [ ( (a + c) + 2 ) \cdot c ]$
- **Transformation**: $s \rightarrow_E t$

**Ersetzungsschritt:**

- Wir haben die Gleichung $(x + y) \cdot z = x \cdot z + y \cdot z$.
- Substitution $\sigma$:
  - $x \rightarrow (a + c)$
  - $y \rightarrow 2$
  - $z \rightarrow c$

- Unterterm $\sigma((x + y) \cdot z)$ ist $((a + c) + 2) \cdot c$.
- Ersetze $((a + c) + 2) \cdot c$ durch $(a + c) \cdot c + 2 \cdot c$.

- Neuer Term $t$: $u \cdot [ (a + c) \cdot c + 2 \cdot c ]$.

**Schrittweise Ersetzung:**

$$ u \cdot [ ( (a + c) + 2 ) \cdot c ] \rightarrow_E u \cdot [ (a + c) \cdot c + 2 \cdot c ] $$

### Iteration der Ersetzung

- **Einfacher Ersetzungsschritt**: $s \rightarrow_E t$

  Das bedeutet, dass $t$ durch die Anwendung einer Ersetzungsregel aus $E$ auf $s$ erhalten wurde.

- **Mehrfacher Ersetzungsschritt**: $t \rightarrow_E r$

  Dies bedeutet, dass $r$ durch mehrere Anwendungen von $s \rightarrow_E t$ von links nach rechts erhalten wird.

- **Beidseitige Ersetzung**: $t \leftrightarrow_E r$

  Dies bedeutet, dass $r$ durch mehrere Anwendungen von $\rightarrow_E$ in beiden Richtungen erhalten wird, also sowohl von links nach rechts als auch von rechts nach links.

### Detaillierte Beispiele

#### Beispiel 1: Einfache Ersetzung

**Gegeben:** $E = \{ (a + b) \cdot c = a \cdot c + b \cdot c \}$

- **Term:** $(x + y) \cdot z$
- **Substitution:** $x \rightarrow a$, $y \rightarrow b$, $z \rightarrow c$

Ersetzung:

$$ (a + b) \cdot c \rightarrow a \cdot c + b \cdot c $$

#### Beispiel 2: Iteration der Ersetzung

**Gegeben:** $E = \{ (a + b) + c = a + (b + c) \}$

- **Term:** $((x + y) + z) + w$
- **Substitution:** $x \rightarrow a$, $y \rightarrow b$, $z \rightarrow c$, $w \rightarrow d$

Ersetzung:

$$ ((a + b) + c) + d \rightarrow (a + (b + c)) + d \rightarrow a + ((b + c) + d) $$

### Anwendung und Bedeutung

- **Vereinfachung von Ausdrücken**: Durch das systematische Ersetzen von Ausdrücken können Terme vereinfacht und leichter analysiert werden.
- **Automatisierte Beweisführung**: Viele Beweissysteme nutzen diese Technik, um Gleichungen zu transformieren und zu beweisen, dass sie gültig sind.
- **Programmanalyse**: In der Programmverifikation wird Termersetzung verwendet, um Programmzustände und -ausdrücke zu analysieren und zu beweisen, dass sie bestimmte Eigenschaften erfüllen.

### Zusammenfassung

Die Termersetzung durch gleiche Ausdrücke ist ein mächtiges Werkzeug in der formalen Logik, das es ermöglicht, komplexe Terme zu transformieren und zu analysieren. Mit diesen Techniken können wir Gleichungen und logische Ausdrücke systematisch untersuchen und vereinfachen, was in vielen Bereichen der Informatik und Mathematik von großer Bedeutung ist.

Falls du weitere Fragen oder spezifische Beispiele sehen möchtest, lass es mich wissen!


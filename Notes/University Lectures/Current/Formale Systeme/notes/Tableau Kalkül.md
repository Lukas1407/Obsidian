Der Tableaukalkül ist ein Verfahren in der mathematischen Logik, das verwendet wird, um die Erfüllbarkeit oder Unerfüllbarkeit von Formeln zu testen. 
### Wesentliche Eigenschaften
1. **Widerlegungskalkül:**
   - Der Tableaukalkül arbeitet nach dem Prinzip des Widerlegens. Das bedeutet, er überprüft, ob eine gegebene Menge von Aussagen widersprüchlich ist, indem er versucht zu zeigen, dass die Negation der Aussage unerfüllbar ist.
   - Formal: $M \models A$ ist äquivalent zu $M \cup \{ \neg A \} \vdash_T 0$. Das bedeutet, dass ein Modell $M$ die Aussage $A$ erfüllt, wenn und nur wenn $M$ zusammen mit der Negation von $A$ zu einem Widerspruch führt.
2. **Beweis durch Fallunterscheidung:**
   - Der Tableaukalkül nutzt die Fallunterscheidung, um verschiedene Möglichkeiten zu überprüfen und festzustellen, ob eine Formel unerfüllbar ist.
3. **Top-down-Analyse der gegebenen Formeln:**
   - Beim Tableaukalkül beginnt man mit der gesamten Formel und zerlegt sie schrittweise in ihre Bestandteile, um zu sehen, ob ein Widerspruch entsteht.
### Vorteile
1. **Intuitiver als Resolution:**
   - Der Tableaukalkül wird oft als intuitiver empfunden als andere Methoden wie die Resolution, da er visuell durch das Aufspalten von Formeln in kleinere Teile arbeitet.
2. **Formeln müssen nicht in Normalform sein:**
   - Im Gegensatz zu anderen Verfahren, wie der Resolution, müssen die Formeln nicht zuerst in eine bestimmte Normalform (z.B. konjunktive Normalform) umgewandelt werden. Das spart oft Zeit und Aufwand.
3. **Erfüllbare Interpretation im aussagenlogischen Fall:**
   - Wenn eine Formelmenge erfüllbar ist (d.h., der Beweis schlägt fehl), kann der Tableaukalkül eine erfüllende Interpretation (ein Gegenbeispiel) konstruieren. Das bedeutet, dass man eine konkrete Belegung der Variablen erhält, bei der die Formel wahr ist.
### Nachteil
1. **Mehr als eine Regel:**
   - Der Tableaukalkül verwendet mehrere Regeln, um die Formeln zu zerlegen und zu analysieren. Das kann die Handhabung etwas komplizierter machen im Vergleich zu Verfahren, die mit einer einzigen Regel arbeiten.
## Tableauregeln
Die Tableauregeln legen fest, wie Formeln innerhalb eines Tableaus zerlegt und bearbeitet werden. Sie dienen der Ableitung und Prüfung auf Erfüllbarkeit oder Unerfüllbarkeit.
#### α-Regel
Diese Regel wird für konjunktive Formeln verwendet und besagt, dass beide Teilformeln erfüllt sein müssen:
- **α:**
  $$
  \frac{\alpha}{\alpha_1 \quad \alpha_2}
  $$
  Beispiel: $\frac{1(A \land B)}{1A \quad 1B}$
#### β-Regel
Die β-Regel wird für disjunktive Formeln verwendet und legt fest, dass eine der beiden Teilformeln erfüllt sein muss:
- **β:**
  $$
  \frac{\beta}{\beta_1 \mid \beta_2}
  $$
  Beispiel: $\frac{1(A \lor B)}{1A \mid 1B}$
#### γ-Regel
Die γ-Regel behandelt universelle Quantoren und erfordert die Einführung einer neuen Variablen $Y$:
- **γ:**
  $$
  \frac{\gamma}{\gamma_1(Y)}
  $$
  Beispiel: $\frac{1\forall xA(x)}{1A(Y)}$
#### δ-Regel
Die δ-Regel ist für existenzielle Quantoren zuständig und führt eine neue Funktion $f$ ein:
- **δ:**
  $$
  \frac{\delta}{\delta_1(f(X_1, \ldots, X_n))}
  $$
  Beispiel: $\frac{1\exists xA(x)}{1A(f(X_1, \ldots, X_n))}$
Diese Regeln helfen dabei, ein Tableau systematisch zu entwickeln und zu überprüfen, ob eine gegebene logische Formel erfüllbar oder widersprüchlich ist.


 Die Tableaukonstruktion ist eine Methode in der Logik, um die Erfüllbarkeit oder Unerfüllbarkeit von Formeln zu überprüfen. Dabei wird ein sogenanntes Tableau erstellt, das eine systematische Zerlegung von Formeln ermöglicht. Schauen wir uns die Definitionen und Regeln zur Tableaukonstruktion genauer an.

## Definitionen
1. **Tableau:**
   - Ein Tableau ist ein binärer Baum, bei dem jeder Knoten mit einer Vorzeichenformel markiert ist. Die Vorzeichenformel kann entweder $0A$ (A ist falsch) oder $1A$ (A ist wahr) sein.
   - Jeder Knoten repräsentiert eine Aussage über die Wahrheit oder Falschheit einer logischen Formel.
2. **Tableauast:**
   - Ein Tableauast ist ein maximaler Pfad in einem Tableau, der von der Wurzel (dem Anfangsknoten) zu einem Blatt (Endknoten) verläuft. Ein solcher Pfad enthält eine Abfolge von Vorzeichenformeln.
## Schritte zur Tableaukonstruktion
### Initialisierung
- Zu Beginn erstellt man ein Tableau, das nur aus einem einzigen Knoten besteht, welcher die Vorzeichenformel $0A$ trägt. Hierbei steht $A$ für die Formel, die man untersuchen möchte, und $M$ ist die Menge aller gegebenen Formeln.
- Beispiel: Wenn $A$ die zu prüfende Formel ist, startet das Tableau mit dem Knoten $0A$, um die Unerfüllbarkeit von $A$ zu testen.
### Erweiterung
- Um das Tableau zu erweitern, wählt man einen Ast $B$ des aktuellen Tableaus $T$. Auf diesem Ast gibt es eine Formel $F$, die kein Atom ist (also keine einfache Variable, sondern eine komplexe logische Formel).
- Man wendet die für $F$ passende Regel (z.B. $\alpha$- oder $\beta$-Regel) an, um neue Knoten zu erzeugen, die an den Ast $B$ angefügt werden. Das resultierende erweiterte Tableau $T'$ ist dann ein neues Tableau für $A$ über $M$.
- Beispiel: Wenn $F$ die Formel $1(A \land B)$ ist, teilt man diese in $1A$ und $1B$ gemäß der $\alpha$-Regel auf und fügt diese als neue Knoten zum Ast hinzu.
### Voraussetzungsregel
- Diese Regel erlaubt es, ein bestehendes Tableau $T$ um eine Formel $F$ aus der Menge $M$ zu erweitern.
- Man wählt einen beliebigen Ast des Tableaus und fügt dort den Knoten $1F$ hinzu. Das erweiterte Tableau $T'$ ist dann ebenfalls ein gültiges Tableau für $A$ über $M$.
- Beispiel: Wenn $M$ eine Menge von Formeln enthält und $F$ eine Formel aus $M$ ist, kann man jeden Ast des Tableaus durch Hinzufügen von $1F$ erweitern, um weitere Ableitungen vorzunehmen.
### Anwendung der Regeln
#### Beispiel für die Erweiterung eines Tableaus
Nehmen wir an, $A$ ist die Formel $(P \rightarrow Q) \land (R \lor S)$. Wir wollen prüfen, ob $A$ unerfüllbar ist.
1. **Initialisierung:**
   - Starte mit $0A$ (also $0((P \rightarrow Q) \land (R \lor S))$).
2. **Erweiterung durch $\alpha$-Regel:**
   - Wende die $\alpha$-Regel auf $0((P \rightarrow Q) \land (R \lor S))$ an:
     - Teile in $0(P \rightarrow Q)$ und $0(R \lor S)$.
3. **Weiterer Schritt:**
   - Wende die $\alpha$-Regel auf $0(P \rightarrow Q)$ an:
     - Zerlege in $1P$ und $0Q$.
4. **Erweiterung durch $\beta$-Regel:**
   - Wende die $\beta$-Regel auf $0(R \lor S)$ an:
     - Zerlege in zwei Äste: $0R$ oder $0S$.
Am Ende des Prozesses hat man ein Tableau erstellt, das entweder einen Widerspruch enthält oder eine erfüllende Interpretation für die Formel liefert.

Die Begriffe „geschlossener Ast“ und „geschlossenes Tableau“ sind zentrale Konzepte bei der Analyse von logischen Formeln mithilfe des Tableaukalküls. Sie helfen dabei, festzustellen, ob ein Tableau widersprüchlich ist und somit ob eine Formel unerfüllbar ist. Lassen Sie uns diese Begriffe und ihre Bedeutung genauer betrachten.

### Definition: Geschlossener Ast
Ein **Ast** eines Tableaus ist eine Folge von Vorzeichenformeln, die von der Wurzel (Anfangsknoten) bis zu einem Blatt (Endknoten) des Tableaus reicht.
- Ein **Ast $B$** eines Tableaus ist **geschlossen**, wenn sowohl $1F$ als auch $0F$ in $B$ vorkommen.
- Das bedeutet, dass es auf diesem Ast sowohl eine Formel $F$ gibt, die als wahr markiert ist ($1F$), als auch dieselbe Formel $F$, die als falsch markiert ist ($0F$). Dies ist ein offensichtlicher Widerspruch.
#### Beispiel:
- Wenn $B$ die Formeln $1P$ und $0P$ enthält, ist der Ast $B$ geschlossen, da $P$ nicht gleichzeitig wahr und falsch sein kann.
### Definition: Geschlossenes Tableau
Ein **Tableau $T$** ist eine Sammlung von Ästen, die durch die Anwendung von Tableauregeln auf eine Ausgangsformel erstellt wurden.
- Ein Tableau $T$ ist **geschlossen**, wenn es eine kollisionsfreie Substitution $\sigma$ gibt, sodass für jeden Ast $B$ von $T$ der substituierte Ast $\sigma(B)$ geschlossen ist.
- Eine **kollisionsfreie Substitution** $\sigma$ bedeutet, dass Variablen in den Formeln so ersetzt werden, dass keine widersprüchlichen Zuweisungen entstehen.
#### Beispiel:
- Nehmen wir an, ein Tableau enthält zwei Äste:
  - $B_1$ enthält $1P$ und $0P$.
  - $B_2$ enthält $1Q$ und $0Q$.
- In diesem Fall ist das Tableau geschlossen, da jeder Ast einen Widerspruch enthält, nämlich dass dieselbe Formel sowohl wahr als auch falsch ist.
### Anwendung und Bedeutung
- Wenn ein Tableau geschlossen ist, bedeutet das, dass die Ausgangsformel unerfüllbar ist, da in jedem möglichen Ast ein Widerspruch gefunden wurde.
- Ein geschlossenes Tableau zeigt, dass es keine Interpretation gibt, in der die Ausgangsformel wahr sein kann, was bedeutet, dass die Negation der Ausgangsformel immer wahr ist (Widerlegung).
### Schritte zur Überprüfung, ob ein Tableau geschlossen ist:
1. **Erstellen des Tableaus:**
   - Beginne mit der Ausgangsformel und erweitere das Tableau gemäß den $\alpha$-, $\beta$-, $\gamma$- und $\delta$-Regeln.
2. **Überprüfung der Äste:**
   - Prüfe jeden Ast des Tableaus darauf, ob er geschlossen ist. Ein Ast ist geschlossen, wenn er sowohl $1F$ als auch $0F$ für eine Formel $F$ enthält.
3. **Substitution anwenden:**
   - Führe eine Substitution durch, um sicherzustellen, dass keine variablen Kollisionen vorliegen. Ersetze Variablen so, dass der Ast weiterhin geschlossen bleibt.
4. **Schlussfolgerung:**
   - Wenn alle Äste des Tableaus geschlossen sind, ist das gesamte Tableau geschlossen, und die Ausgangsformel ist unerfüllbar.

## Aussagenlogisches Beispiel
Lassen Sie uns zunächst die Schritte für die Verwendung des Tableaukalküls zur Überprüfung der Allgemeingültigkeit einer Aussagenlogik-Formel durchgehen. Nehmen wir als Beispiel die Formel $A \rightarrow (B \lor \neg A)$ und zeigen deren Allgemeingültigkeit mit Hilfe des Tableaukalküls.
### Schritt 1: Negation der Formel
Wir beginnen mit der Negation der Formel, um zu prüfen, ob sie widersprüchlich ist. Die negierte Formel lautet:
$$\neg (A \rightarrow (B \lor \neg A))$$
### Schritt 2: Umformung in Negation
Wir wandeln die Implikation in ihre logische Äquivalenz um:
$$\neg (\neg A \lor (B \lor \neg A))$$
### Schritt 3: Anwendung der Regeln
Wir erweitern das Tableau, um zu zeigen, dass die Formel unerfüllbar ist:
1. **Start:**
   - Beginne mit $0(A \rightarrow (B \lor \neg A))$.
2. **Anwendung der $\alpha$-Regel:**
   - Zerlege $0(A \rightarrow (B \lor \neg A))$:
     $$1A$$
     $$0(B \lor \neg A)$$
3. **Anwendung der $\beta$-Regel auf $0(B \lor \neg A)$:**
   - Zerlege $0(B \lor \neg A)$:
     $$0B$$
     $$1A$$
4. **Überprüfung:**
   - Es entsteht der Widerspruch $1A$ und $0B$.
Da die Formel zu einem Widerspruch führt, ist sie allgemeingültig.
## Prädikatenlogisches Beispiel
Schauen wir uns nun die prädikatenlogische Formel an und zeigen deren Allgemeingültigkeit.
Gegeben sei die Formel:
$$\forall x \, p(x) \rightarrow \exists y \, p(y)$$
#### Schritt 1: Negation der Formel
Negieren wir die Formel, um ihre Unerfüllbarkeit zu prüfen:
$$\neg (\forall x \, p(x) \rightarrow \exists y \, p(y))$$
$$\forall x \, p(x) \land \neg \exists y \, p(y)$$
#### Schritt 2: Umformung in Negation
Die Negation der Existenzquantoren wird umgewandelt:
$$\forall x \, p(x) \land \forall y \, \neg p(y)$$
#### Schritt 3: Anwendung der Regeln
1. **Start:**
   - Beginne mit $0(\forall x \, p(x) \rightarrow \exists y \, p(y))$.
2. **Anwendung der $\alpha$-Regel:**
   - Zerlege $0(\forall x \, p(x) \rightarrow \exists y \, p(y))$:
     $$1\forall x \, p(x)$$
     $$0\exists y \, p(y)$$
3. **Anwendung der $\gamma$-Regel auf $1\forall x \, p(x)$:**
   - Füge für eine neue Variable $X$ hinzu:
     $$1p(X)$$
4. **Anwendung der $\delta$-Regel auf $0\exists y \, p(y)$:**
   - Ersetze durch $\forall$:
     $$0p(Y)$$
#### Abschlussregel
Wir haben jetzt einen Widerspruch auf einem Ast:
1. **Start:**
   - $1\forall x \, p(x)$ (X)
   - $0\exists y \, p(y)$ (Y)
   - $1p(X)$
   - $0p(Y)$
2. **Widerspruch finden:**
   - Substitution von $X$ und $Y$:
     - $1p(X)$ und $0p(X)$

Da $1p(X)$ und $0p(X)$ ein Widerspruch sind, ist das Tableau geschlossen.

## Geschlossenes und offenes Tableau
### Geschlossenes Tableau
Ein **geschlossenes Tableau** ist ein Tableau, bei dem alle Pfade (Äste) durch Widersprüche abgeschlossen sind. Jeder Ast enthält widersprüchliche Aussagen, was bedeutet, dass die Formel unerfüllbar ist.
### Offenes Tableau
Ein **offenes Tableau** ist ein Tableau, das mindestens einen Pfad hat, der keinen Widerspruch enthält. Das bedeutet, dass die Formel nicht widersprüchlich ist und es möglicherweise eine erfüllbare Interpretation gibt.
### Beispiel eines geschlossenen Tableaus
Schauen wir uns ein Beispiel für ein geschlossenes Tableau an:
1. **Formel:** $\neg (\exists y \, \forall x \, p(x, y) \rightarrow \forall x \, \exists y \, p(x, y))$
2. **Negierte Formel:** $\neg \exists y \, \forall x \, p(x, y) \land \neg \forall x \, \exists y \, p(x, y)$
3. **Start des Tableaus:**
   ```
   1. []   0 ∃y∀xp(x, y) → ∀x∃yp(x, y)
   2. [α(1)]  1 ∃y∀xp(x, y)
   3. [α(1)]  0 ∀x∃yp(x, y)
   4. [δ(2)]  1 ∀xp(x, a)
   5. [δ(3)]  0 ∃yp(b, y)
   6. [γ(4)]  1 p(X, a)
   7. [γ(5)]  0 p(b, Y)
   ```
4. **Unifikation:**
   - Der Widerspruch entsteht durch die Zuweisungen $\sigma(X) = b$ und $\sigma(Y) = a$, da $p(b, a)$ nicht gleichzeitig wahr und falsch sein kann.
Dieses Tableau ist geschlossen, da jeder Pfad einen Widerspruch enthält.
### Beispiel eines offenen Tableaus
Schauen wir uns ein Beispiel für ein offenes Tableau an:
1. **Formel:** $\neg (\forall x \, \exists y \, p(x, y) \rightarrow \exists y \, \forall x \, p(x, y))$
2. **Negierte Formel:** $\forall x \, \exists y \, p(x, y) \land \neg \exists y \, \forall x \, p(x, y)$
3. **Start des Tableaus:**
   ```
   1. []   0 ∀x∃yp(x, y) → ∃y∀xp(x, y)
   2. [α(1)]  0 ∃y∀xp(x, y)
   3. [α(1)]  1 ∀x∃yp(x, y)
   4. [γ(2)]  0 ∀xp(x, Y)
   5. [γ(3)]  1 ∃yp(X, y)
   6. [δ(4)]  0 p(f(Y), Y)
   7. [δ(5)]  1 p(X, g(X))
   ```
4. **Fehlende Unifikation:**
   - $p(f(Y), Y)$ und $p(X, g(X))$ können nicht unifiziert werden, da die Zuweisung $\sigma(X) = f(Y)$ und $\sigma(Y) = g(X)$ nicht widerspruchsfrei ist.
Dieses Tableau ist offen, da es mindestens einen Pfad ohne Widerspruch enthält.
### Mehrfache Anwendung der $\gamma$-Regel
Um die Gültigkeit einer logischen Aussage zu beweisen, kann die $\gamma$-Regel mehrfach angewendet werden. Diese Regel ist besonders nützlich für universelle Quantoren und hilft dabei, alle möglichen Fälle zu berücksichtigen.
#### Beweisaufgabe: $p(0) \land \forall x(p(x) \rightarrow p(s(x))) \models p(s(s(0)))$
1. **Negation:** Wir wollen zeigen, dass $\neg (p(0) \land \forall x(p(x) \rightarrow p(s(x))) \rightarrow p(s(s(0)))$ widersprüchlich ist.
2. **Formel:** $p(0) \land \forall x(p(x) \rightarrow p(s(x))) \land \neg p(s(s(0)))$
3. **Start des Tableaus:**
   ```
   1. []   1 p(0)
   2. []   1 ∀x(p(x) → p(s(x)))
   3. []   0 p(s(s(0)))
   ```
4. **Anwendung der $\gamma$-Regel auf $\forall x(p(x) \rightarrow p(s(x)))$:**
   ```
   4. [γ(2)]  1 p(0) → p(s(0))
   5. [γ(2)]  1 p(s(0)) → p(s(s(0)))
   ```
5. **Erweiterung mit den negierten Bedingungen:**
   ```
   6. [β(4)]  0 p(0) oder 1 p(s(0))
   7. [β(5)]  0 p(s(0)) oder 1 p(s(s(0)))
   ```
6. **Überprüfung der Widersprüche:**
   - Pfad mit $1 p(s(0))$:
     ```
     8. []  1 p(s(0))
     9. []  0 p(s(s(0)))
     ```
     Widerspruch, da $p(s(s(0)))$ sowohl wahr als auch falsch sein müsste.
   - Pfad mit $0 p(0)$:
     ```
     10. []  0 p(0)
     11. []  1 p(0)
     ```
     Widerspruch, da $p(0)$ sowohl wahr als auch falsch sein müsste.
Dieses Tableau ist geschlossen, was zeigt, dass die ursprüngliche Formel allgemeingültig ist.

## Theorem: Korrektheit und Vollständigkeit des Tableaukalküls
Das Theorem besagt, dass eine logische Folgerung $A$ aus einer Formelmenge $M$ genau dann möglich ist, wenn es einen Tableaubeweis für $A$ über $M$ gibt.
Formell: $M \models A$ genau dann, wenn $M \vdash_T A$.
- **Korrektheit** bedeutet, dass, wenn es einen Tableaubeweis für $A$ über $M$ gibt ($M \vdash_T A$), dann ist $A$ auch eine logische Folgerung aus $M$ ($M \models A$). Das bedeutet, dass alle durch das Tableau gefundenen Ergebnisse tatsächlich gültig sind.
- **Vollständigkeit** bedeutet, dass, wenn $A$ eine logische Folgerung aus $M$ ist ($M \models A$), es auch einen Tableaubeweis für $A$ über $M$ gibt ($M \vdash_T A$). Das bedeutet, dass das Tableau alle gültigen logischen Folgerungen finden kann.
## Definition: Erfüllbares Tableau
Ein Tableau $T$ für eine Formel $A$ über eine Formelmenge $M$ heißt **$M$-erfüllbar**, wenn es eine Interpretation $D$ über das Alphabet $\Sigma$ gibt, die folgende Bedingungen erfüllt:
1. **Modell von $M$:**
   - Die Interpretation $D$ ist ein Modell von $M$. Das bedeutet, dass alle Formeln in $M$ unter $D$ wahr sind.
2. **Pfad in $T$:**
   - Für jede Variablenbelegung $\beta$ gibt es einen Pfad $\pi$ im Tableau $T$, auf dem alle Variablen $V$ unter der Interpretation $D$ und der Belegung $\beta$ den gewünschten Wert $W$ haben (d.h., $\text{val}_{D, \beta}(V) = W$ für alle Variablen $V$ auf dem Pfad $\pi$).
3. **Erweiterung des Alphabets $\Sigma$:**
   - Das Alphabet $\Sigma$ wird um neue Funktionssymbole $f$ erweitert, die im Tableau $T$ eingeführt werden. Dies stellt sicher, dass die Struktur von $\Sigma$ alle notwendigen Symbole enthält, die im Tableau verwendet werden.
### Beispiel zur Verdeutlichung
Angenommen, wir haben eine Formelmenge $M$ und eine zu beweisende Formel $A$. Um die logische Folgerung zu überprüfen, erstellen wir ein Tableau. 
1. **Formel:** $M = \{ \forall x \, p(x) \rightarrow q(x) \}$, $A = \forall x \, p(x) \rightarrow \forall x \, q(x)$
2. **Negation der zu beweisenden Formel:** $\neg (\forall x \, p(x) \rightarrow \forall x \, q(x))$
3. **Erstellung des Tableaus:** 
   - Beginne mit $0 (\forall x \, p(x) \rightarrow \forall x \, q(x))$.
   - Wende die $\alpha$- und $\gamma$-Regeln an, um die Formeln zu zerlegen und die Pfade zu erweitern.
4. **Überprüfung der Erfüllbarkeit:**
   - Falls alle Pfade geschlossen sind, zeigt das Tableau, dass die Formel unerfüllbar ist, und somit $A$ eine logische Folge von $M$ ist (Korrektheit).
   - Falls es keinen geschlossenen Pfad gibt, zeigt dies, dass es eine Interpretation gibt, die die Formel erfüllt (Vollständigkeit).
### Korrektheitsbeweis für das Tableaukalkül
Der Korrektheitsbeweis für das Tableaukalkül zeigt, dass wenn ein geschlossenes Tableau für eine Formel $A$ über eine Formelmenge $M$ existiert, $A$ eine logische Folge von $M$ ist. Das bedeutet, dass die Methode des Tableaukalküls zuverlässig die logische Gültigkeit von Aussagen überprüfen kann. Hier ist der Beweisplan für die Korrektheit im Detail:
### Theorem
**Theorem:** Sei $A \in \text{For}_\Sigma$ und $M \subseteq \text{For}_\Sigma$, wobei alle Formeln keine freien Variablen enthalten. Wenn es ein geschlossenes Tableau für $A$ über $M$ gibt, dann gilt $M \models A$.
Formell:
$$M \models A \Longleftrightarrow \text{Es gibt ein geschlossenes Tableau für } A \text{ über } M.$$
### Beweisplan
Der Beweisplan besteht aus mehreren Schritten und verwendet verschiedene Lemmata, um die Korrektheit zu zeigen:
1. **Anfangstableau $T_0$:**
   - Beginne mit einem Tableau, das nur den Knoten $0A$ enthält.
   - Zeige, dass dieses Tableau nicht $M$-erfüllbar ist.
2. **Zwischentableaus $T_k$:**
   - Erweiterungen des Tableaus werden systematisch durchgeführt.
   - Zeige, dass jedes Zwischentableau $T_k$ nicht $M$-erfüllbar ist.
3. **Endtableau $T_n$:**
   - Zeige, dass das finale Tableau $T_n$ geschlossen ist und somit nicht $M$-erfüllbar ist.
   - Ein geschlossenes Tableau bedeutet, dass jeder Pfad widersprüchlich ist.
### Lemmata
Um den Beweis zu strukturieren, verwenden wir drei wesentliche Lemmata:
#### Lemma 1: Endtableau
**Lemma:** Jedes geschlossene Tableau für $A$ über $M$ ist unerfüllbar. 
- **Beweisidee:** Jeder Ast eines geschlossenen Tableaus enthält einen Widerspruch (z.B. $1A$ und $0A$), was bedeutet, dass der Ast nicht erfüllbar ist. Da jeder Ast eines geschlossenen Tableaus widersprüchlich ist, kann das gesamte Tableau nicht erfüllt werden.
#### Lemma 2: Anfangstableau
**Lemma:** Ist das Anfangstableau für $A$ über $M$ nicht $M$-erfüllbar, dann gilt $M \models A$.
- **Beweisidee:** Wenn das Anfangstableau $0A$ nicht erfüllbar ist, bedeutet dies, dass $M \cup \{0A\}$ widersprüchlich ist. Daraus folgt, dass $A$ wahr sein muss, wenn $M$ wahr ist, d.h. $M \models A$.
#### Lemma 3: Korrektheitslemma
**Lemma:** Sei $M$ eine Formelmenge ohne freie Variablen. Wenn das Tableau $T_1$ über $M$ durch Anwendung einer Tableauregel aus $T$ über $M$ hervorgeht und $T$ $M$-erfüllbar ist, dann ist auch $T_1$ $M$-erfüllbar.
- **Beweisidee:** Die Anwendung einer Tableauregel bewahrt die Erfüllbarkeit des Tableaus. Das bedeutet, wenn $T$ erfüllbar ist, dann bleibt es auch nach der Regelanwendung erfüllbar.
### Schritte des Korrektheitsbeweises
1. **Initialisierung:** 
   - Beginne mit der Formel $A$ und der Formelmenge $M$. Erstelle das Anfangstableau $T_0$ mit $0A$.
2. **Erweiterungsschritte:**
   - Wende systematisch Tableauregeln an, um das Tableau zu erweitern und schrittweise Zwischentableaus $T_1, T_2, \ldots, T_k$ zu erstellen.
3. **Anwendung der Lemmata:**
   - Verwende Lemma 1, um zu zeigen, dass jedes geschlossene Tableau nicht erfüllbar ist.
   - Verwende Lemma 2, um zu zeigen, dass das Anfangstableau nicht $M$-erfüllbar ist, was bedeutet, dass $M \models A$.
   - Verwende Lemma 3, um die Erfüllbarkeit von Zwischentableaus durch Regelanwendungen sicherzustellen.
4. **Schlussfolgerung:**
   - Zeige, dass das Endtableau $T_n$ geschlossen ist. Da jedes geschlossene Tableau nicht erfüllbar ist, folgt, dass $A$ eine logische Folge von $M$ ist.
### $\alpha$-Fall
- **Beschreibung:** Der $\alpha$-Fall behandelt konjunktive Formeln, die in zwei Teilformeln zerlegt werden, die beide erfüllt sein müssen.
- **Beispiel:** Für eine Formel $1(A \land B)$ wird diese in $1A$ und $1B$ zerlegt.
- **Korrektheit:** Wenn $T$ ein Tableau ist, das eine Formel $V$ enthält, dann ist die erweiterte Version $T_1$ durch $V_1$ und $V_2$ ebenfalls erfüllbar, da beide Teilformeln unabhängig erfüllt werden können.
![[Pasted image 20240622091137.png#invert|300]]
### $\beta$-Fall
- **Beschreibung:** Der $\beta$-Fall behandelt disjunktive Formeln, die in zwei alternative Teilformeln zerlegt werden, von denen mindestens eine erfüllt sein muss.
- **Beispiel:** Für eine Formel $0(A \lor B)$ wird diese in $0A$ und $0B$ zerlegt.
- **Korrektheit:** Wenn $T$ ein Tableau ist, das eine disjunktive Formel $V$ enthält, wird die Erweiterung in $T_1$ durch $V_1$ oder $V_2$ die Erfüllbarkeit sicherstellen, da mindestens eine der beiden Teilformeln erfüllt werden muss.
![[Pasted image 20240622091151.png#invert|300]]
### $\gamma$-Fall
- **Beschreibung:** Der $\gamma$-Fall behandelt universelle Quantoren, bei denen für jede mögliche Interpretation der Quantor erfüllt sein muss.
- **Beispiel:** Für eine Formel $1\forall x F(x)$ wird diese zu $F(y)$ erweitert, wobei $y$ ein spezifisches Element ist.
- **Korrektheit:** Wenn $T$ eine universelle Formel $\forall x F(x)$ enthält, dann wird die Erweiterung $T_1$ durch $F(y)$ für eine beliebige Wahl von $y$ sicherstellen, dass $T_1$ ebenfalls erfüllbar ist.
![[Pasted image 20240622091206.png#invert|300]]
### $\delta$-Fall
- **Beschreibung:** Der $\delta$-Fall behandelt existenzielle Quantoren, bei denen es mindestens ein spezifisches Element gibt, das die Quantorformel erfüllt.
- **Beispiel:** Für eine Formel $0\exists x F(x)$ wird diese zu $F(f(x_1, \ldots, x_n))$ erweitert, wobei $f$ ein neues Funktionssymbol ist.
- **Korrektheit:** Wenn $T$ eine existentielle Formel $\exists x F(x)$ enthält, dann wird die Erweiterung $T_1$ durch $F(f(x_1, \ldots, x_n))$ für eine spezielle Wahl von $x$ sicherstellen, dass $T_1$ ebenfalls erfüllbar ist.
![[Pasted image 20240622091220.png#invert|300]]
#### Beweis des Korrektheitslemmas im $\delta$-Fall
Der $\delta$-Fall im Korrektheitsbeweis des Tableaukalküls behandelt existenzielle Quantoren. Der Beweis zeigt, dass die Anwendung der $\delta$-Regel die Erfüllbarkeit eines Tableaus bewahrt. Dies ist entscheidend, um sicherzustellen, dass ein Tableau, das durch Hinzufügen von Existenzquantoren erweitert wird, weiterhin die ursprüngliche logische Gültigkeit beibehält.
#### Grundidee des Beweises
Die $\delta$-Regel besagt, dass für jede Formel der Form $\exists x \, F(x)$ ein neuer Wert für $x$ eingeführt wird, der die Formel $F(x)$ erfüllt. Der Beweis zeigt, dass die Erweiterung des Tableaus durch diese Regel die Erfüllbarkeit nicht zerstört.
#### Voraussetzungen
- $D$ ist eine Interpretation, die ein Modell des ursprünglichen Tableaus $T$ über die Formelmenge $M$ ist.
- Wir konstruieren eine neue Interpretation $D' = (D, I')$, die von $D$ abgeleitet ist, indem wir ein neues Funktionssymbol $f$ einführen, das für die Existenzquantoren verwendet wird.
#### Konstruktion der neuen Interpretation $D'$
1. **Einführung des Funktionssymbols $f$:**
   - Das Funktionssymbol $f$ wird eingeführt, um Werte für die existenzielle Variable $x$ zu definieren. Die Interpretation von $f$, $I'(f)$, hängt von den Parametern $d_1, \ldots, d_n$ ab, die $f$ als Argumente übergeben werden.
2. **Definition von $I'(f)$:**
   - $I'(f)(d_1, \ldots, d_n) = \begin{cases} d & \text{wenn } (D, \beta) \models \exists x \, F(x) \text{ und } (D, \beta_d^x) \models F(x) \\ \text{ein beliebiger Wert} & \text{sonst} \end{cases}$
   - Hierbei ist $\beta$ eine Belegung der Variablen, die $x_i$ auf $d_i$ abbildet. Wenn die Formel $\exists x \, F(x)$ unter der Interpretation $D$ und der Belegung $\beta$ wahr ist, gibt es ein $d \in D$, das $F(x)$ erfüllt.
3. **Konstruktion von $D'$:**
   - Die neue Interpretation $D'$ unterscheidet sich von $D$ nur durch die Einführung und Zuordnung des Funktionssymbols $f$.
#### Nachweis der Erfüllbarkeit von $T_1$
1. **Überprüfung der Belegung $\beta$ für $D'$:**
   - Jede Belegung $\beta$ für $D'$ ist auch eine Belegung für $D$, da sich der Grundbereich $D$ nicht geändert hat.
2. **Pfad $\pi$ in $T$:**
   - Es gibt einen Pfad $\pi_0$ in $T$, sodass $(D, \beta) \models \pi_0$. Dies gilt für alle Belegungen.
3. **Spezifischer Pfad $\pi$:**
   - Der interessante Fall ist der Pfad $\pi$, der die Formel $\exists x \, F(x)$ enthält. Aus der Tatsache, dass $(D, \beta) \models \exists x \, F(x)$, folgt durch die Konstruktion von $D'$ auch, dass $(D', \beta) \models F(f(x_1, \ldots, x_n))$.
4. **Erfüllbarkeit der restlichen Formeln:**
   - Da das Funktionssymbol $f$ in den restlichen Formeln des Pfades $\pi$ und in $M$ nicht vorkommt, bleibt $(D', \beta) \models \pi \cup \{ F(f(x_1, \ldots, x_n)) \}$ und $(D', \beta) \models M$ gültig.
### Voraussetzungsregel
![[Pasted image 20240622092056.png#invert|300]]
#### Gegebene Voraussetzungen
- **$T$** ist ein Tableau, das die Erfüllbarkeit einer Formelmenge $M$ überprüft.
- **$T_1$** ist das erweiterte Tableau, das durch Hinzufügen einer Formel $V_M$ aus $M$ entsteht.
- **$π$** ist ein Pfad im Tableau $T$, der durch die Hinzufügung von $V_M$ zu einem Pfad im erweiterten Tableau $T_1$ wird.
#### Ziele des Beweises
- Zeigen, dass das Hinzufügen von $V_M$ zu einem Pfad $π$ in $T$ die Erfüllbarkeit von $T$ nicht verletzt und somit die Erfüllbarkeit von $T_1$ sichergestellt wird.
#### Beweisschritte
1. **Annahme der $M$-Erfüllbarkeit:**
   - Wir nehmen an, dass die Formelmenge $M$ erfüllbar ist. Das bedeutet, dass es eine Interpretation gibt, unter der alle Formeln in $M$ wahr sind.
2. **Hinzufügen von $V_M$ zum Pfad $π$:**
   - Der Pfad $π$ in $T$ enthält bereits Formeln, die $M$ erfüllen. Durch das Hinzufügen von $V_M$, das ein Element von $M$ ist, bleibt die Erfüllbarkeit erhalten, da $V_M$ offensichtlich ebenfalls erfüllt wird.
3. **Konsequenz der Erfüllbarkeit:**
   - Da $V_M \in M$ und $M$ erfüllt ist, wird $V_M$ automatisch erfüllt, wenn $M$ erfüllt ist.
   - Dadurch bleibt auch der Pfad $π$, der jetzt $V_M$ enthält, erfüllt.
4. **Erfüllbarkeit des erweiterten Tableaus $T_1$:**
   - Da $T_1$ lediglich durch das Hinzufügen einer erfüllbaren Formel $V_M$ zu $T$ erweitert wird, bleibt die Erfüllbarkeit erhalten.
   - Das Tableau $T_1$ ist also weiterhin erfüllbar, wenn $T$ erfüllbar ist.

Um die Idee des vollständigen Tableau-Kalküls in der Prädikatenlogik zu verstehen, lass uns die Konzepte Schritt für Schritt durchgehen:

### 1. Erschöpfte Tableaux in der Aussagenlogik
- Ein Tableau$T$ ist "erschöpft", wenn alle nicht-atomaren Formeln darin durch Regelanwendungen vollständig zerlegt wurden.
- Aus einem solchen erschöpften Tableau kann für jeden offenen Pfad$A$ ein Modell für die Wurzel$W$ abgeleitet werden.
- Dazu werden die Atome in$A$ betrachtet, um ein Modell zu konstruieren.
- Deswegen ist der Tableau-Kalkül in der Aussagenlogik vollständig: Jede erfüllbare Formel kann mit einem Modell verifiziert werden, indem man ein erschöpftes Tableau analysiert.

### 2. Übertragung auf die Prädikatenlogik
- In der Prädikatenlogik sind "erschöpfte" Tableaux im Allgemeinen unendlich, weil sie aufgrund der unendlichen Möglichkeiten von Grundtermen und Variablen nicht abgeschlossen werden können.
- Um damit umzugehen, wird eine Aufzählung aller möglichen Grundterme$t_1, t_2, \ldots$ vorgenommen.

### 3. Konstruktion von Tableaux und Grundsubstitutionen
- Parallel zur Erstellung einer Folge von Tableaux$T_i$ wird eine Folge von Grundsubstitutionen$\sigma_i$ gebildet.
- Wenn$T_{i+1}$ aus$T_i$ durch Anwendung einer$\gamma$-Regel auf die Formel$F$ auf einem Pfad$\pi$ entsteht, dann gilt:
  $$ \sigma_{i+1} = \{X/t_n\} \circ \sigma_i $$
  -$X$ ist die neu eingeführte Variable.
  -$t_n$ ist der n-te Grundterm, der für$X$ eingeführt wird.
  - Dies wird bei jeder Anwendung der$\gamma$-Regel auf$F$ auf dem Pfad$\pi$ so gehandhabt.
- Wenn keine$\gamma$-Regel angewendet wird, bleibt die Substitution gleich:
  $$ \sigma_{i+1} = \sigma_i $$
### 4. Abgeschlossene Pfade
- Ein Pfad$\pi$ im Tableau$T_i$ wird nicht erweitert, wenn die Anwendung der Substitution$\sigma_i(\pi)$ den Pfad abschließt, also zu einem Widerspruch führt oder vollständig bearbeitet ist.

## Vollständigkeit des Tableaukalküls
Um die Vollständigkeit des Tableaukalküls zu verstehen, betrachten wir das Theorem und den Beweisansatz Schritt für Schritt:
### Theorem
- **Formeln ohne freie Variablen:** Sei$A$ eine Formel und$M$ eine Menge von Formeln, die keine freien Variablen enthalten.
- **Semantische Gültigkeit:** Wenn$M \models A$ gilt, das heißt, wenn jede Interpretation, die$M$ erfüllt, auch$A$ erfüllt, dann führt jedes faire Verfahren, das:
  - mit der Formel$0A$ und der Identitätssubstitution$\sigma_0 = \text{id}$ beginnt,
  - und die Konstruktionsvorschrift einhält,
  nach endlich vielen Schritten zu einem geschlossenen Tableau.
- **Fairness:** Ein Verfahren ist fair, wenn auf jedem Pfad jede mögliche Regelanwendung schließlich stattfindet, insbesondere:
  - Jede$\gamma$-Formel wird unbeschränkt oft verwendet.
  - Jede Formel aus$M$ wird berücksichtigt.
### Beweisansatz
- **Annahme der Nichtterminierung:** Angenommen, die Folge $(T_0, \sigma_0), (T_1, \sigma_1), \ldots$ terminiert nicht, das heißt, das Tableau wird nie vollständig geschlossen.
- **Unendlicher Baum:** Setzen wir:
  $$ T = \bigcup_{i \geq 0} T_i \quad \text{und} \quad \sigma = \bigcup_{i \geq 0} \sigma_i $$
  $\sigma(T)$ ist dann ein unendlicher, aber endlich verzweigender Baum.
- **Königs Lemma:** Nach Königs Lemma existiert in einem solchen unendlichen Baum immer ein unendlicher Pfad$\pi$.
- **Offener Pfad:** Nach Konstruktion muss$\pi$ ein offener Pfad sein, da das Verfahren nicht terminiert und somit keine Widersprüche erzeugt werden.
- **Herbrand-Modell:** Aus diesem offenen Pfad$\pi$ kann man ein Herbrand-Modell$D$ ableiten, in dem:
  $$ D \models M \quad \text{und} \quad D \models \neg A $$
- **Widerspruch:** Dies steht im Widerspruch zu der Annahme$M \models A$, da es ein Modell$D$ gibt, das$M$ erfüllt, aber$A$ nicht erfüllt.
### Schrittweise Erläuterung
1. **Formel und Formelmenge:** Du hast eine Formel$A$ und eine Menge von Formeln$M$, die keine freien Variablen enthalten.
2. **Semantische Bedingung:** Wenn$M \models A$ gilt, bedeutet das, dass jede Interpretation, die alle Formeln in$M$ wahr macht, auch$A$ wahr macht.
3. **Faires Verfahren:** Ein faires Verfahren stellt sicher, dass jede Regel auf jedem Pfad irgendwann angewendet wird, sodass alle Formeln berücksichtigt werden.
4. **Annahme der Nichtterminierung:** Angenommen, der Prozess hört nie auf, d.h., das Tableau wird nie vollständig abgeschlossen.
5. **Unendlicher Pfad:** Nach Königs Lemma existiert in einem unendlichen, aber endlich verzweigenden Baum immer ein unendlicher Pfad. Dieser Pfad kann nicht abgeschlossen sein, weil sonst der Prozess terminiert hätte.
6. **Herbrand-Modell:** Ein unendlicher offener Pfad ermöglicht es, ein Modell$D$ zu konstruieren, das alle Formeln in$M$ erfüllt, aber$A$ nicht erfüllt. Dies widerspricht der Annahme$M \models A$.
7. **Widerspruch:** Da wir angenommen haben, dass$M \models A$ gilt, führt der offene Pfad und das daraus abgeleitete Modell zu einem Widerspruch, was zeigt, dass das Verfahren terminieren muss.

## König's Lemma und Hintikka-Mengen
### König's Lemma
**Aussage:** In jedem unendlichen, aber endlich verzweigenden Baum existiert ein unendlicher Pfad.
- **Unendlicher Baum:** Ein Baum, der unendlich viele Knoten besitzt.
- **Endliche Verzweigung:** Jeder Knoten hat nur eine endliche Anzahl von Nachfolgeknoten.
**Warum ist das wichtig?**
- Es stellt sicher, dass in einem unendlichen Tableau (mit endlichen Regelanwendungen) immer ein unendlicher Pfad existiert, wenn das Tableau nicht terminiert.
### Hintikka-Mengen
**Definition:** Eine Menge$H$ geschlossener Vorzeichenformeln über einer Signatur$\Sigma$ heißt Hintikka-Menge, wenn die folgenden Bedingungen erfüllt sind:
- **(H 1) α-Formeln:**
  - Wenn eine α-Formel$F$ in$H$ ist, dann müssen auch$F_1$ und$F_2$ in$H$ sein.
  - **Beispiel:** Für$F$ wie$F_1 \land F_2$, wenn$F \in H$, dann müssen auch$F_1$ und$F_2$ in$H$ sein.
- **(H 2) β-Formeln:**
  - Wenn eine β-Formel$F$ in$H$ ist, dann muss entweder$F_1$ oder$F_2$ in$H$ sein.
  - **Beispiel:** Für$F$ wie$F_1 \lor F_2$, wenn$F \in H$, dann muss entweder$F_1$ oder$F_2$ in$H$ sein.
- **(H 3) δ-Formeln:**
  - Wenn eine δ-Formel$F$ in$H$ ist, dann gibt es einen Grundterm$t$, sodass$F_1(t) \in H$ ist.
  - **Beispiel:** Für$F$ wie$\exists x \, F_1(x)$, wenn$F \in H$, dann gibt es einen Term$t$, sodass$F_1(t) \in H$ ist.
- **(H 4) γ-Formeln:**
  - Wenn eine γ-Formel$F$ in$H$ ist, dann gilt$F_1(t)$ für jeden Grundterm$t$.
  - **Beispiel:** Für$F$ wie$\forall x \, F_1(x)$, wenn$F \in H$, dann muss$F_1(t)$ für jeden Grundterm$t$ in$H$ sein.
- **(H 5) Widerspruchsfreiheit:**
  - Für keine Formel$A$ kommen$1A$ und$0A$ beide in$H$ vor.
  - **Beispiel:** Wenn$A$ in$H$ wahr ist, dann darf$\neg A$ nicht in$H$ sein.
### Modellexistenz
- **Jede Hintikka-Menge besitzt ein Herbrand-Modell.**
  - Ein Herbrand-Modell ist ein Modell, das durch die Menge der Grundterme und Funktionssymbole bestimmt wird.
  - Es zeigt, dass für jede Hintikka-Menge ein Modell existiert, das diese Menge erfüllt.
- **Jeder offene Ast in einem fairen, nicht abgeschlossenen Tableau ist eine Hintikka-Menge.**
  - Ein fairer, nicht abgeschlossener Pfad in einem Tableau enthält keine Widersprüche und erfüllt die Bedingungen für eine Hintikka-Menge.
  - Dadurch kann man aus einem solchen Pfad ein Modell ableiten.

## Unentscheidbarkeit der Prädikatenlogik
**Theorem:** Die folgenden Probleme sind unentscheidbar:
1. Ist eine prädikatenlogische Formel$F \in \text{For}_{\Sigma}$ erfüllbar?
2. Ist eine prädikatenlogische Formel$F \in \text{For}_{\Sigma}$ allgemeingültig?
### Erklärung der Begriffe
- **Erfüllbar:** Eine Formel$F$ ist erfüllbar, wenn es eine Interpretation gibt, in der$F$ wahr ist.
- **Allgemeingültig:** Eine Formel$F$ ist allgemeingültig, wenn$F$ in jeder Interpretation wahr ist.
### Beweisansatz (Church, 1936)
- **Kodierung eines unentscheidbaren Problems:** Um die Unentscheidbarkeit der Prädikatenlogik zu zeigen, kodieren wir ein bekanntes unentscheidbares Problem, wie das Halteproblem, in prädikatenlogische Formeln.
  - **Das Halteproblem:** Das Halteproblem fragt, ob ein gegebenes Computerprogramm bei einer bestimmten Eingabe jemals hält (endet) oder unendlich läuft.
  - **Kodierung:** Man kann eine prädikatenlogische Formel konstruieren, die genau dann erfüllbar ist, wenn das entsprechende Programm hält. Da das Halteproblem unentscheidbar ist, muss auch das Problem der Erfüllbarkeit in der Prädikatenlogik unentscheidbar sein.
### Korollar
- **Minimale Anzahl von$\gamma$-Regel-Anwendungen:** Die minimale Anzahl der Anwendungen von$\gamma$-Regeln in einem Tableaubeweis für eine prädikatenlogische Formel$F \in \text{For}_{\Sigma}$ ist nicht berechenbar.
  - **Grund:** Die Anzahl der notwendigen Anwendungen von$\gamma$-Regeln hängt von der Komplexität der prädikatenlogischen Formel ab. Da die Entscheidungsprobleme selbst unentscheidbar sind, ist es auch nicht möglich, die minimale Anzahl der notwendigen Regelanwendungen zu berechnen.
## Tableauregel für den logischen Äquivalenzoperator$\leftrightarrow$
**Regel:** Der Äquivalenzoperator$\leftrightarrow$ (logische Äquivalenz) kann in einem Tableau durch die folgenden Regeln dargestellt werden:
- Für$W \leftrightarrow F$:
  - Falls$W \leftrightarrow F$ wahr ist (W), dann sind$W$ und$F$ entweder beide wahr oder beide falsch.
  - Falls$W \leftrightarrow F$ falsch ist (F), dann sind$W$ und$F$ unterschiedlich.
**Tableauregeln:**
- **W$\leftrightarrow$ W:** Wenn$W \leftrightarrow F$ wahr ist, dann:
  $$
  W \quad F
  $$

- **F$\leftrightarrow$ W:** Wenn$W \leftrightarrow F$ falsch ist, dann:
  $$
  W \quad \neg F
  $$

- **W$\leftrightarrow$ F:** Wenn$\neg (W \leftrightarrow F)$ wahr ist, dann:
  $$
  \neg W \quad F
  $$

- **F$\leftrightarrow$ F:** Wenn$\neg (W \leftrightarrow F)$ falsch ist, dann:
  $$
  \neg W \quad \neg F
  $$
### Korrekte und vollständige Tableauregeln
- **Korrekt:** Eine Regel ist korrekt, wenn sie keine Widersprüche erzeugt und nur wahre Aussagen aus wahren Annahmen ableitet.
- **Vollständig:** Eine Regel ist vollständig, wenn sie in der Lage ist, jede gültige Formel oder jeden gültigen Schluss vollständig abzubilden.

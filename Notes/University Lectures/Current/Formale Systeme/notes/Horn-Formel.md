
Eine Horn-Formel ist eine Formel in der Aussagenlogik, die folgende Eigenschaften hat:
1. **[[Konjunktive Normalform (KNF)]]**:
   Die Formel muss in konjunktiver Normalform vorliegen. Das bedeutet, sie ist eine Konjunktion von Klauseln, wobei jede Klausel eine Disjunktion von Literalen ist.
2. **Beschränkung auf positive Literale**:
  <mark style="background: #FFB86CA6;"> Jede Klausel in der Horn-Formel enthält höchstens ein positives Literal</mark>. Ein positives Literal ist eine Variable <mark style="background: #FFB86CA6;">ohne Negation</mark> (zum Beispiel $A$), während ein negatives Literal eine negierte Variable ist (zum Beispiel $\neg B$).
### Alternative Schreibweise
- **Disjunktive Form**: $\neg B_1 \lor \neg B_2 \lor \ldots \lor \neg B_m \lor A$
  - Das bedeutet, dass zumindest eines der negativen Literale nicht wahr sein muss oder das positive Literal wahr ist.
- **Implikative Form**: $B_1 \land B_2 \land \ldots \land B_m \rightarrow A$
  - Dies kann als eine Implikation interpretiert werden, wobei die Konjunktion der negierten Literale den Rumpf und das positive Literal den Kopf der Implikation bildet.
### Besondere Formen
- **Fakt**: $A$
  - Ein Fakt ist eine Klausel ohne negative Literale. Es bedeutet einfach, dass $A$ wahr ist.
- **Leere Disjunktion**: $\Box$
  - Eine leere Disjunktion ist immer falsch.
### Bezeichnungen
- **Rumpf**: $B_1 \land \ldots \land B_m$
  - Der Rumpf ist die Konjunktion der negativen Literale oder die Bedingung, die erfüllt sein muss.
- **Kopf**: $A$
  - Der Kopf ist das positive Literal, das wahr wird, wenn der Rumpf wahr ist.
### Beispiel
Sehen wir uns ein Beispiel an, um das besser zu verstehen:
**Formel**: $\neg P \lor \neg Q \lor R$
- **In KNF**: Die Formel ist eine Disjunktion, also entspricht sie der KNF.
- **Höchstens ein positives Literal**: Es gibt genau ein positives Literal, nämlich $R$.
Alternative Schreibweise:
- **Implikative Form**: $P \land Q \rightarrow R$
  - Diese Form zeigt, dass $R$ wahr ist, wenn sowohl $P$ als auch $Q$ wahr sind.
### Wichtige Eigenschaften von Horn-Formeln
- **Erfüllbarkeitsproblem**: Das <mark style="background: #FFB86CA6;">Erfüllbarkeitsproblem für Horn-Formeln ist effizient lösbar</mark> (in quadratischer Zeit). Das bedeutet, dass es für Horn-Formeln effizient entschieden werden kann, ob es eine Belegung der Variablen gibt, die die Formel wahr macht.
- **Nicht NP-vollständig**: Im Gegensatz zu allgemeinen logischen Formeln sind Horn-Formeln nicht NP-vollständig, sondern können in polynomialer Zeit gelöst werden.

## Erfüllbarkeitstest für Horn-Formeln
Der Erfüllbarkeitstest für Horn-Formeln kann verwendet werden, um zu bestimmen, ob eine Horn-Formel erfüllbar ist, d.h., ob es eine Zuweisung der Variablen gibt, die die Formel wahr macht. 
### Schritt-für-Schritt-Anleitung zum Erfüllbarkeitstest
1. **Initialisierung**:
   - Geben Sie die Horn-Formel $C$ in [[Konjunktive Normalform (KNF)]] an: $C = D_1 \land D_2 \land \ldots \land D_m$.
   - Markieren Sie alle Fakten in $C$. Ein Fakt ist eine Klausel ohne negative Literale, also ein positives Literal wie $A$.
2. **Überprüfung auf leere Klauseln**:
   - Wenn $\Box$ (leere Klausel) in $C$ vorkommt, ist die Formel unerfüllbar. Geben Sie „unerfüllbar“ aus und stoppen Sie.
3. **Überprüfung auf Fakten**:
   - Wenn keine Fakten vorhanden sind (d.h., keine positiven Literale ohne Bedingung), geben Sie „erfüllbar“ aus und stoppen Sie.
4. **Schritt 1: Rumpf-Überprüfung**:
   - Suchen Sie nach einer Klausel der Form $B_1 \land B_2 \land \ldots \land B_m \rightarrow K$, bei der alle Atome $B_i$ im Rumpf (den $B_i$-Teil) bereits markiert sind.
     - Wenn keine solche Klausel existiert, geben Sie „erfüllbar“ aus und stoppen Sie.
     - Wenn es eine solche Klausel gibt, in der der Kopf $K$ 0 ist (d.h., $B_1 \land B_2 \land \ldots \land B_m \rightarrow 0$), ist die Formel unerfüllbar. Geben Sie „unerfüllbar“ aus und stoppen Sie.
     - Wenn alle $B_i$ markiert sind, aber der Kopf $A$ nicht, markieren Sie $A$ und wiederholen Sie Schritt 1.
5. **Ende**:
   - Falls keine weiteren Änderungen mehr gemacht werden können, und Sie keinen der unerfüllbaren Bedingungen gefunden haben, geben Sie „erfüllbar“ aus und stoppen Sie.
### Detaillierte Erklärung der Schritte
1. **Markieren der Fakten**:
   - Dies bedeutet, dass alle positiven Literale, die ohne Bedingungen wahr sind (Fakten), als wahr markiert werden. Zum Beispiel wird $A$ in $A \land (B \rightarrow C)$ als Fakt markiert.
2. **Überprüfung auf leere Klauseln**:
   - Eine leere Klausel $\Box$ ist immer falsch. Wenn eine solche Klausel existiert, kann die Formel nicht erfüllt werden, da keine Zuweisung diese Klausel wahr machen kann.
3. **Überprüfung auf das Vorhandensein von Fakten**:
   - Wenn es keine Fakten gibt, bedeutet das, dass die Formel keine unveränderlichen Wahrheitswerte hat, was bedeutet, dass sie erfüllbar ist.
4. **Überprüfung von Klauseln mit markierten Rumpfatomen**:
   - Wir suchen Klauseln, deren Bedingung (der Rumpf) vollständig erfüllt ist (alle $B_i$ sind markiert).
     - Wenn der Kopf dieser Klausel 0 ist, führt dies zu einem Widerspruch, und die Formel ist unerfüllbar.
     - Wenn der Kopf ein positives Literal ist, wird dieses markiert, da die Bedingung bereits erfüllt ist.
### Beispiel
$$
C = (\neg A \lor B) \land (\neg B \lor C) \land (\neg C) \land A
$$
Das entspricht den Klauseln:
- $D_1: \neg A \lor B$
- $D_2: \neg B \lor C$
- $D_3: \neg C$
- $D_4: A$
#### 1. **Initialisierung:**
   - Die Horn-Formel $C$ ist bereits in konjunktiver Normalform gegeben:
     $$
     C = (\neg A \lor B) \land (\neg B \lor C) \land (\neg C) \land A
     $$
   - Markieren der **Fakten**: Ein Fakt ist eine Klausel, die kein negatives Literal enthält. Die Klausel $A$ ist ein Fakt, weil sie nur ein positives Literal $A$ enthält.
   - Markiere $A$.
#### 2. **Überprüfung auf leere Klauseln:**
   - Es gibt keine leere Klausel (den Ausdruck $\Box$), daher fahren wir mit dem nächsten Schritt fort.
#### 3. **Überprüfung auf Fakten:**
   - Wir haben bereits den Fakt $A$ markiert. Das bedeutet, wir müssen weiter prüfen.
#### 4. **Schritt 1: Rumpf-Überprüfung (Propagieren der Markierungen):**
   - Wir suchen nach einer Klausel, deren **Rumpf** (d.h., der Teil ohne das positive Literal) vollständig markiert ist und deren Kopf noch nicht markiert ist.
   - Betrachten wir die Klauseln nacheinander:
     - **Klausel $D_1$:** $\neg A \lor B$
       - $A$ ist markiert, und $\neg A$ bedeutet, dass $B$ wahr sein muss. Daher markieren wir $B$.
     - **Klausel $D_2$:** $\neg B \lor C$
       - $B$ ist jetzt markiert, daher wird $C$ ebenfalls markiert.
     - **Klausel $D_3$:** $\neg C$
       - $C$ ist markiert, aber $\neg C$ erfordert, dass $C$ falsch ist. Dies führt zu einem Widerspruch, da $C$ markiert ist.
#### 5. **Ende:**
   - Da es einen Widerspruch gibt (die Klausel $\neg C$ verlangt, dass $C$ falsch ist, obwohl $C$ bereits markiert wurde), ist die Formel **unerfüllbar**.
Die Horn-Formel $C = (\neg A \lor B) \land (\neg B \lor C) \land (\neg C) \land A$ ist **unerfüllbar**, da wir einen Widerspruch erhalten haben, als wir $C$ markiert haben und gleichzeitig $\neg C$ erfüllen mussten.
### Beispiel
![[erfllbarkeits-test-ezgif.com-speed.gif#invert|600]]
```python
# Definition der Klauseln
formel = [
    ([-1], 2),  # ¬A ∨ B
    ([-2], 3),  # ¬B ∨ C
    ([-3], -4),  # ¬C ∨ ¬D
    ([], 1)  # A (Fakt)
]

# Markierungen initialisieren
markierungen = set()

# Funktion zum Überprüfen der Erfüllbarkeit
def ist_erfüllbar(formel):
    while True:
        # Überprüfen, ob eine leere Klausel vorhanden ist
        if any(not rumpf and kopf == 0 for rumpf, kopf in formel):
            return "unerfüllbar"

        # Markieren von Fakten
        neue_markierungen = {kopf for rumpf, kopf in formel if not rumpf and kopf not in markierungen}
        if not neue_markierungen:
            break
        markierungen.update(neue_markierungen)

        # Klauseln überprüfen
        neue_markierungen = set()
        for rumpf, kopf in formel:
            if all(atom in markierungen for atom in rumpf):
                if kopf == 0:
                    return "unerfüllbar"
                neue_markierungen.add(abs(kopf))
        if not neue_markierungen:
            break
        markierungen.update(neue_markierungen)
    return "erfüllbar"

# Testen der Formel
ergebnis = ist_erfüllbar(formel)
print(f"Die Horn-Formel ist: {ergebnis}")

```

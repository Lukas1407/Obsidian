### Java Modeling Language (JML)

JML ist eine Spezifikationssprache, die zur Formulierung von Verhaltensspezifikationen für Java-Klassen und -Methoden verwendet wird. Die Hauptidee von JML ist es, das Konzept "Design by Contract" zu unterstützen, indem Vorbedingungen, Nachbedingungen und Invarianten definiert werden.

### Historie

- **Initiator:** Gary Leavens
- **Erste Publikation:** 1999
- **Community:** Seitdem wurde eine weltweite Community aufgebaut.
- **Standardisierung:** Seit 2016 gibt es Bemühungen zur Standardisierung.

### Grundidee

**Design by Contract:** Dieser Ansatz stammt ursprünglich von Bertrand Meyer und der Programmiersprache Eiffel. Die Idee ist, dass Softwaremodule (Klassen und Methoden) Verträge abschließen, die durch Vorbedingungen, Nachbedingungen und Invarianten spezifiziert werden.

### JML-Annotationen

JML-Annotationen sind spezielle Kommentare im Quelltext, die zur Spezifikation der folgenden Aspekte verwendet werden:

1. **Vorbedingung (requires):** Bedingung, die vor der Ausführung einer Methode erfüllt sein muss.
2. **Nachbedingung (ensures):** Bedingung, die nach der Ausführung einer Methode erfüllt sein muss.
3. **Invariante (invariant):** Bedingung, die in allen sichtbaren Zuständen der Klasse erfüllt sein muss.
4. **Normale Terminierung (normal_behavior):** Gibt an, dass die Methode normal terminiert, ohne eine Exception zu werfen.

### Beispiel einer JML-Spezifikation

Das folgende Beispiel zeigt eine JML-Spezifikation für eine Java-Klasse:

```java
public class PostInc {
    PostInc rec;
    int x, y;

    /*@ public invariant x >= 0 && y >= 0 &&
      @ rec.x >= 0 && rec.y >= 0;
      @*/

    /*@ public normal_behavior
      @ requires true;
      @ ensures rec.x == \old(rec.y) &&
      @ rec.y == \old(rec.y) + 1;
      @*/
    public void postinc() { 
        rec.x = rec.y++; 
    }
}
```

### Erklärung der JML-Spezifikation

1. **Invariante:**

```java
/*@ public invariant x >= 0 && y >= 0 &&
  @ rec.x >= 0 && rec.y >= 0;
  @*/
```

- Diese Invariante stellt sicher, dass die Felder `x` und `y` der Klasse `PostInc` sowie `rec.x` und `rec.y` immer größer oder gleich 0 sind.

2. **Normales Verhalten:**

```java
/*@ public normal_behavior
  @ requires true;
  @ ensures rec.x == \old(rec.y) &&
  @ rec.y == \old(rec.y) + 1;
  @*/
```

- **requires true:** Es gibt keine spezifische Vorbedingung für die Methode `postinc()`, d.h. sie kann immer aufgerufen werden.
- **ensures:** Die Nachbedingungen stellen sicher, dass nach der Ausführung der Methode:
  - `rec.x` den Wert von `rec.y` vor der Ausführung der Methode hat.
  - `rec.y` um 1 erhöht wurde.

3. **Methode `postinc()`:**

```java
public void postinc() { 
    rec.x = rec.y++; 
}
```

- Diese Methode weist `rec.x` den aktuellen Wert von `rec.y` zu und erhöht dann `rec.y` um 1.

### Syntax und JML-Ausdrücke

- JML verwendet die Syntax der Java-Sprache für Datentypen und Ausdrücke.
- JML-Ausdrücke können auf Deklarationen und Felder der umgebenden Java-Klasse zugreifen.
- **Syntactic Sugar:** JML erlaubt syntaktische Vereinfachungen ähnlich wie in Java:
  - `x >= 0` steht für `this.x >= 0`.


### Alternative zum \old Operator in JML

Der \old Operator in JML wird verwendet, um den Wert einer Variablen vor der Ausführung einer Methode zu speichern und zu vergleichen. Es gibt jedoch auch Alternativen, wie zum Beispiel die explizite Speicherung von Werten in temporären Variablen vor dem Methodenaufruf.

#### Beispiel:

```java
PostInc oldrec;

/*@
  @ requires oldrecy == rec.y;
  @ ensures rec.x == oldrecy && rec.y == oldrecy + 1;
  @*/
```

- **Erklärung:** In diesem Beispiel wird `oldrecy` vor der Ausführung der Methode gespeichert und als Vergleichswert in den Nachbedingungen verwendet. Dies ist eine Alternative zum \old Operator.

### Nicht-Null-Default in JML

#### Problem bei Null-Referenzen:

Was passiert, wenn die Methode in einem Zustand aufgerufen wird, in dem `this.rec == null` gilt?

- Wenn `rec` null ist und die Methode aufgerufen wird, wird eine `NullPointerException` ausgelöst. Dies bedeutet, dass die Methode ihren Vertrag nicht erfüllt, da normale Terminierung verlangt wird (ohne Ausnahme).

#### JML Standardannahme:

- JML nimmt standardmäßig an, dass alle vorkommenden Attribute und Parameter mit einem Objekttyp vom Nullobjekt verschieden sind (Nicht-Null-Default). Dies bedeutet, dass ohne explizite Angabe davon ausgegangen wird, dass alle Objekte nicht null sind.

### Überschreiben des Nicht-Null-Defaults

Es gibt Situationen, in denen es notwendig ist, den Nicht-Null-Default von JML zu überschreiben, um explizit anzugeben, dass ein Attribut oder Parameter null sein kann.

#### Beispiel:

```java
public class PostInc {
    public /*@ nullable @*/ PostInc rec;
    public int x, y;

    /*@ public invariant x >= 0 && y >= 0 &&
      @ (rec != null ==> rec.x >= 0 && rec.y >= 0);
      @*/

    /*@ public normal_behavior
      @ requires rec != null;
      @ ensures rec.x == \old(rec.y) &&
      @ rec.y == \old(rec.y) + 1;
      @*/
    public void postinc() {
        rec.x = rec.y++;
    }
}
```

- **Nullable Annotation:** Die Annotation `/*@ nullable @*/` wird verwendet, um anzugeben, dass `rec` null sein kann.
- **Verstärkte Vorbedingung:** Die Vorbedingung `requires rec != null;` stellt sicher, dass `rec` nicht null ist, bevor die Methode `postinc` aufgerufen wird.
- **Änderung der Invarianten:** Die Invariante wird angepasst, um die Möglichkeit zu berücksichtigen, dass `rec` null sein kann.

### Zusammenfassung

1. **Alternative zum \old Operator:** Statt den \old Operator zu verwenden, können Werte explizit in temporären Variablen gespeichert werden, um sie vor und nach der Methodenaufruf zu vergleichen.
2. **Nicht-Null-Default:** JML nimmt standardmäßig an, dass alle Objekte nicht null sind, es sei denn, es wird ausdrücklich anders angegeben.
3. **Überschreiben des Nicht-Null-Defaults:** Mit der Annotation `/*@ nullable @*/` kann angegeben werden, dass ein Attribut oder Parameter null sein kann. Dies erfordert verstärkte Vorbedingungen und Anpassungen der Invarianten, um sicherzustellen, dass der Vertrag der Methode erfüllt wird.

Diese Mechanismen helfen, die Robustheit und Zuverlässigkeit von Java-Programmen durch präzise Spezifikationen und Bedingungen zu erhöhen.

### Richtige Nachbedingung für die Methode `postinc`

#### Klasse `PostIncxx`

Betrachten wir die Klasse `PostIncxx` und die Methode `postinc`, die darin definiert ist:

```java
public class PostIncxx {
    public PostInc rec;
    public int x;

    /*@ public invariant x >= 0 && rec.x >= 0;
      @*/

    /*@ public normal_behavior
      @ requires true;
      @ ensures ???;
      @*/
    public void postinc() {
        rec.x = rec.x++; // statt y++
    }
}
```

#### Nachbedingung der Methode `postinc`

In der Methode `postinc` wird der Wert von `rec.x` inkrementiert. Die Methode sollte nach ihrer Ausführung sicherstellen, dass die erwarteten Veränderungen in den Attributen korrekt beschrieben sind.

1. **Invariante:** `x >= 0 && rec.x >= 0` stellt sicher, dass sowohl `x` als auch `rec.x` immer nicht-negativ sind.

2. **Nachbedingung:** Für die Methode `postinc` können wir folgende Nachbedingung spezifizieren:
   - `rec.x` sollte um 1 erhöht werden.

Die richtige Nachbedingung lautet daher:

```java
/*@ public normal_behavior
  @ requires true;
  @ ensures rec.x == \old(rec.x) + 1;
  @*/
```

Dies bedeutet, dass nach der Ausführung der Methode `rec.x` genau um 1 größer sein soll als der ursprüngliche Wert von `rec.x`.

### Beispiel: Methode `commonEntry`

Betrachten wir die Methode `commonEntry` in der Klasse `SITA`:

```java
class SITA {
    int[] a1, a2;

    /*@ public normal_behaviour
      @ requires 0 <= l && l < r &&
      @ r <= a1.length && r <= a2.length;
      @ ensures (l <= \result && \result < r &&
      @ a1[\result] == a2[\result])
      @ || \result == r;
      @ ensures (\forall int j; l <= j && j < \result;
      @ a1[j] != a2[j]);
      @ assignable \nothing;
      @*/
    public int commonEntry(int l, int r) {
        // Method implementation goes here
    }
}
```

#### Erklärung der Nachbedingungen

Die Methode `commonEntry` soll einen Index im Bereich `[l, r)` finden, an dem die Arrays `a1` und `a2` denselben Wert haben. Falls kein solcher Index existiert, soll die Methode `r` zurückgeben.

1. **Vorbedingungen:**
   - `0 <= l && l < r && r <= a1.length && r <= a2.length`: Diese Bedingungen stellen sicher, dass die Indizes `l` und `r` gültig sind und sich im Bereich der beiden Arrays befinden.

2. **Nachbedingungen:**
   - `(l <= \result && \result < r && a1[\result] == a2[\result]) || \result == r`: Diese Bedingung besagt, dass entweder ein Index gefunden wurde, bei dem die Werte in `a1` und `a2` übereinstimmen, oder dass `\result` gleich `r` ist, wenn kein solcher Index existiert.
   - `(\forall int j; l <= j && j < \result; a1[j] != a2[j])`: Diese Bedingung stellt sicher, dass für alle Indizes `j` im Bereich `[l, \result)` die Werte in `a1` und `a2` unterschiedlich sind.
   - `assignable \nothing`: Dies bedeutet, dass die Methode keine Felder oder Variablen verändert.

### Zusammenfassung

- **Methode `postinc`:** Die Nachbedingung `ensures rec.x == \old(rec.x) + 1;` stellt sicher, dass `rec.x` nach der Methode um 1 erhöht ist.
- **Methode `commonEntry`:** Die Nachbedingungen beschreiben, dass entweder ein Index gefunden wurde, bei dem `a1` und `a2` denselben Wert haben, oder dass `\result` gleich `r` ist, wenn kein solcher Index existiert. Zudem wird sichergestellt, dass alle Indizes davor unterschiedliche Werte in `a1` und `a2` haben.

Diese Spezifikationen helfen dabei, das erwartete Verhalten der Methoden formal zu beschreiben und sicherzustellen, dass die Implementierung diese Erwartungen erfüllt.

Natürlich, lass uns das zusammen durchgehen.

### Quantoren in JML (Java Modeling Language)

In JML werden Quantoren verwendet, um Aussagen über Mengen von Objekten zu machen. Es gibt zwei Haupttypen von Quantoren: $\forall$ (für alle) und $\exists$ (es existiert).

### Syntax

Die allgemeine Syntax für Quantoren in JML ist:
- $(\forall C x; B; R)$
- $(\exists C x; B; R)$

Hier steht:
- $C$ für die Klasse oder den Typ des Objekts $x$.
- $B$ für die Bereichseinschränkung.
- $R$ für den Rumpf, also die Aussage, die gelten soll.

### Bedeutung in prädikatenlogischer Notation

Diese Quantoren haben in der Prädikatenlogik die folgende Bedeutung:
- $(\forall C x; B; R)$ wird zu $\forall x \in C (B \rightarrow R)$.
- $(\exists C x; B; R)$ wird zu $\exists x \in C (B \land R)$.

Das bedeutet:
- $\forall x \in C (B \rightarrow R)$ heißt: Für alle $x$ aus der Menge $C$ gilt, dass wenn $B$ wahr ist, dann ist auch $R$ wahr.
- $\exists x \in C (B \land R)$ heißt: Es gibt mindestens ein $x$ aus der Menge $C$, für das sowohl $B$ als auch $R$ wahr sind.

### Beispiel

- $(\forall C x; B; R)$ und $(\forall C x; \text{true}; (B \rightarrow R))$ sind äquivalent.

Das bedeutet, dass man bei der Bereichseinschränkung $B$ auf $\text{true}$ setzen kann, und dann $B \rightarrow R$ als Rumpf nimmt.

### Vergleich der Notationen

Hier ist eine Übersicht über die Notation in JML und die entsprechende prädikatenlogische Notation:

- $==$ entspricht $.$ (Gleichheit)
- $.$ entspricht $=$
- $\&\&$ entspricht $\land$ (logisches UND)
- $\|\|$ entspricht $\lor$ (logisches ODER)
- $!$ entspricht $\neg$ (Negation)
- $==>$ entspricht $\rightarrow$ (Implikation)
- $<==>$ entspricht $\leftrightarrow$ (Äquivalenz)

### Quantoren in JML und ihre prädikatenlogische Entsprechung

- $(\forall C x; e1; e2)$ entspricht $\forall x (\neg(x = \text{null}) \land [e1] \rightarrow [e2])$.
- $(\exists C x; e1; e2)$ entspricht $\exists x (\neg(x = \text{null}) \land [e1] \land [e2])$.

Dabei steht $[e_i]$ für die prädikatenlogische Notation des JML-Ausdrucks $e_i$.

Falls du zu einem bestimmten Punkt Fragen hast oder weitere Beispiele benötigst, lass es mich wissen!

Natürlich, lass uns das zusammen durchgehen.

### Methode `commonEntry`

#### Vorbedingungen und Nachbedingungen

Die Methode `commonEntry` ist eine Methode in der Klasse `SITA`, die zwei Arrays `a1` und `a2` besitzt. Die Methode sucht nach einem gemeinsamen Eintrag in beiden Arrays innerhalb eines bestimmten Bereichs.

#### Vorbedingungen
Die Vorbedingungen legen fest, welche Bedingungen erfüllt sein müssen, bevor die Methode aufgerufen werden kann:
- `0 <= l && l < r && r <= a1.length && r <= a2.length`: Dies bedeutet, dass der Startindex `l` und der Endindex `r` innerhalb der Grenzen der Arrays `a1` und `a2` liegen müssen, wobei `l` kleiner als `r` sein muss.

#### Nachbedingungen
Die Nachbedingungen legen fest, was nach der Ausführung der Methode gelten muss:
- `(l <= \result && \result < r && a1[\result] == a2[\result]) || \result == r`: Das Ergebnis `\result` muss entweder ein Index innerhalb des Bereichs `[l, r)` sein, an dem `a1[\result] == a2[\result]` gilt, oder `\result` ist gleich `r`.
- `(\forall int j; l <= j && j < \result; a1[j] != a2[j])`: Für alle Indizes `j` im Bereich `[l, \result)` gilt, dass `a1[j] != a2[j]`.

#### `assignable`
Die `assignable \nothing;` Klausel bedeutet, dass die Methode keine Änderungen am Zustand des Heaps vornimmt.

### Implementierung der Methode

```java
public int commonEntry(int l, int r) {
    int k = l;
    /*@ loop_invariant
    @ l <= k && k <= r &&
    @ (\forall int i; l<=i && i<k; a1[i] != a2[i]);
    @ assignable \nothing;
    @ decreases a1.length - k;
    @*/
    while (k < r && a1[k] != a2[k]) {
        k++;
    }
    return k;
}
```

#### Schleifeninvarianten

Eine Schleifeninvariante ist eine Bedingung, die zu Beginn und nach jeder Iteration der Schleife wahr sein muss.

1. **Anfangsfall:**
   ```java
   int k = l;
   /*@ loop_invariant l <= k && k <= r &&
   @ (\forall int i; l<=i && i<k; a1[i] != a2[i]);
   @*/
   while (k < r && a1[k] != a2[k]) {
       k++;
   }
   ```
   - Bei der Initialisierung ist `k = l`. Die Invariante muss direkt vor dem Eintritt in die Schleife gelten: `l <= l && l <= r && (\forall int i; l<=i && i<l; a1[i] != a2[i])`. 
   - Dies ist trivialerweise wahr, weil der quantifizierte Bereich leer ist (es gibt keine `i` für die Bedingung `l <= i < l`).

2. **Iterationsschritt:**
   - Vor dem Schleifendurchlauf: `l <= k && k <= r && (\forall int i; l<=i && i<k; a1[i] != a2[i])`.
   - Schleifenbedingung: `k < r && a1[k] != a2[k]`.
   - Nach dem Schleifendurchlauf: `k ⇝ k + 1`, d.h., `l <= k+1 && k+1 <= r && (\forall int i; l<=i && i<k+1; a1[i] != a2[i])`.

#### Terminierung

Die `decreases` Klausel (`a1.length - k`) gibt eine Maßzahl an, die sich bei jeder Iteration verringert und somit die Terminierung der Schleife sicherstellt.

### Nachprüfung bei Terminierung

1. **Fall 1: `k >= r`**
   - Wenn die Schleife beendet wird, weil `k >= r` gilt, dann ist `\result == r` und die Nachbedingung `((l <= \result && \result < r && a1[\result] == a2[\result]) || \result == r)` ist erfüllt.
   
2. **Fall 2: `k < r && a1[k] == a2[k]`**
   - Wenn die Schleife beendet wird, weil `a1[k] == a2[k]` gilt, dann ist `\result == k` und die Nachbedingung `((l <= \result && \result < r && a1[\result] == a2[\result]) || \result == r)` ist ebenfalls erfüllt.

### Fazit
Die Methode `commonEntry` sucht nach dem ersten Index, an dem die beiden Arrays `a1` und `a2` denselben Wert haben, und stellt sicher, dass die Bedingungen, die in den Vor- und Nachbedingungen definiert sind, erfüllt sind. Die Verwendung von Schleifeninvarianten hilft, die Korrektheit der Schleife zu beweisen.Natürlich, lass uns den Beweis der Terminierung durch die Variante genauer betrachten.

### Beweis der Terminierung: Variante

Die Terminierung einer Schleife kann durch die Angabe einer Variante nachgewiesen werden. Eine Variante ist eine Funktion oder eine Maßzahl, die bei jedem Schleifendurchlauf kleiner wird und stets nicht negativ ist. Dies stellt sicher, dass die Schleife nach endlich vielen Durchläufen terminiert.

#### Variante in `commonEntry`

```java
public int commonEntry(int l, int r) {
    int k = l;
    /*@ decreases a1.length - k; */
    while (k < r && a1[k] != a2[k]) {
        k++;
    }
    return k;
}
```

#### Erklärung der Variante

- **Variante: `a1.length - k`**
  - Diese Variante misst die Differenz zwischen der Länge des Arrays `a1` und der aktuellen Position `k` in der Schleife.
  - Diese Differenz ist zu Beginn der Schleife positiv oder null, da `k` zu Beginn `l` ist und `l` kleiner als `a1.length` ist.

#### Eigenschaften der Variante

1. **Nicht-Negativität:**
   - Zu jedem Zeitpunkt der Schleife gilt `a1.length - k >= 0`.
   - Dies liegt daran, dass `k` zu Beginn `l` ist, und `l` ist kleiner oder gleich `a1.length`. Während `k` innerhalb der Schleife inkrementiert wird, bleibt es immer kleiner oder gleich `r`, und `r` ist kleiner oder gleich `a1.length`.

2. **Strikte Abnahme:**
   - Bei jedem Durchlauf der Schleife wird `k` um 1 erhöht (`k++`).
   - Dadurch wird `a1.length - k` um 1 verringert.
   - Die Variante `a1.length - k` wird also bei jedem Schleifendurchlauf echt kleiner.

#### Folgerung: Terminierung der Schleife

Da die Variante `a1.length - k`:

- zu Beginn der Schleife eine nicht-negative Zahl ist,
- bei jedem Schleifendurchlauf um 1 verringert wird,
- und da es keine unendliche Anzahl positiver ganzer Zahlen gibt,

muss die Schleife nach einer endlichen Anzahl von Durchläufen terminieren.

### Zusammenfassung

Die Terminierung der Schleife in der Methode `commonEntry` wird durch die Angabe der Variante `a1.length - k` sichergestellt. Diese Variante erfüllt zwei wesentliche Bedingungen:

- Sie ist stets nicht negativ.
- Sie wird bei jedem Schleifendurchlauf strikt kleiner.

Diese beiden Eigenschaften zusammen garantieren, dass die Schleife nach einer endlichen Anzahl von Iterationen endet.




Natürlich, lass uns die verallgemeinerten Quantoren in JML und ihre Beispiele genauer betrachten.

### Verallgemeinerte Quantoren in JML

In JML gibt es neben den bekannten Quantoren $\forall$ und $\exists$ auch verallgemeinerte Quantoren, die spezielle Operationen über eine Menge von Werten durchführen. Diese sind besonders nützlich, um aggregierte Eigenschaften von Arrays oder anderen Datenstrukturen zu spezifizieren. Die wichtigsten verallgemeinerten Quantoren sind:

1. **Summation ($\sum$)**
2. **Produkt ($\product$)**
3. **Maximum ($\max$)**
4. **Minimum ($\min$)**

#### Allgemeine Syntax

- $(\sum T x; R; t)$
- $(\product T x; R; t)$
- $(\max T x; R; t)$
- $(\min T x; R; t)$

Hier steht:
- $T$ für den Typ der Variablen $x$.
- $R$ für die Bereichseinschränkung, die festlegt, für welche Werte von $x\ die Operation ausgeführt wird.
- $t$ für den Ausdruck, der für jedes $x$ ausgewertet wird.

#### Beispiele

1. **Summation ($\sum$)**

   $$
   (\sum \text{int } i; 0 \leq i \ \&\&\ i < 5; i)
   $$
   Dieses Beispiel summiert alle $i$ von 0 bis 4:
   $$
   0 + 1 + 2 + 3 + 4 = 10
   $$

2. **Produkt ($\product$)**

   $$
   (\product \text{int } i; 0 < i \ \&\&\ i < 5; i)
   $$
   Dieses Beispiel multipliziert alle $i$ von 1 bis 4:
   $$
   1 \times 2 \times 3 \times 4 = 24
   $$

3. **Maximum ($\max$)**

   $$
   (\max \text{int } i; 0 \leq i \ \&\&\ i < 5; i)
   $$
   Dieses Beispiel findet das Maximum aller $i$ von 0 bis 4:
   $$
   \max(0, 1, 2, 3, 4) = 4
   $$

4. **Minimum ($\min$)**

   $$
   (\min \text{int } i; 0 \leq i \ \&\&\ i < 5; i - 1)
   $$
   Dieses Beispiel findet das Minimum aller $i - 1$ für $i$ von 0 bis 4:
   $$
   \min(-1, 0, 1, 2, 3) = -1
   $$

#### Erklärung der Beispiele

- **Summation**: Der Summationsquantor $\sum$ addiert alle Werte des Ausdrucks $t$ für alle $x$, die die Bereichseinschränkung $R$ erfüllen.
  
  Beispiel: $(\sum \text{int } i; 0 \leq i \ \&\&\ i < 5; i)$
  - Bereichseinschränkung: $0 \leq i \ \&\&\ i < 5$
  - Ausdruck: $i$
  - Summe: $0 + 1 + 2 + 3 + 4$

- **Produkt**: Der Produktquantor $\prod$ multipliziert alle Werte des Ausdrucks $t$ für alle $x$, die die Bereichseinschränkung $R$ erfüllen.

  Beispiel: $(\prod \text{int } i; 0 < i \ \&\&\ i < 5; i)$
  - Bereichseinschränkung: $0 < i \ \&\&\ i < 5$
  - Ausdruck: $i$
  - Produkt: $1 \times 2 \times 3 \times 4$

- **Maximum**: Der Maximumquantor $\max$ findet den größten Wert des Ausdrucks $t$ für alle $x$, die die Bereichseinschränkung $R$ erfüllen.

  Beispiel: $(\max \text{int } i; 0 \leq i \ \&\&\ i < 5; i)$
  - Bereichseinschränkung: $0 \leq i \ \&\&\ i < 5$
  - Ausdruck: $i$
  - Maximum: $\max(0, 1, 2, 3, 4)$

- **Minimum**: Der Minimumquantor $\min$ findet den kleinsten Wert des Ausdrucks $t$ für alle $x$, die die Bereichseinschränkung $R$ erfüllen.

  Beispiel: $(\min \text{int } i; 0 \leq i \ \&\&\ i < 5; i - 1)$
  - Bereichseinschränkung: $0 \leq i \ \&\&\ i < 5$
  - Ausdruck: $i - 1$
  - Minimum: $\min(-1, 0, 1, 2, 3)$

Diese verallgemeinerten Quantoren ermöglichen es, komplexe Aggregationen und Berechnungen in JML-Spezifikationen auszudrücken.


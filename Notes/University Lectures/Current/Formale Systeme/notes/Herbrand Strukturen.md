> [!abstract] Definition
> Diese Strukturen spielen eine zentrale Rolle in der Logik und insbesondere bei der automatisierten Theorembeweisung. Sie ermöglichen eine spezielle Art der Interpretation logischer Formeln, die auf der Menge der Grundterme basiert. 
## Definition von Herbrand-Strukturen
### Herbrand-Interpretation oder Herbrand-Struktur
Eine Interpretation $(D, I)$ wird als Herbrand-Interpretation oder Herbrand-Struktur bezeichnet, wenn sie die folgenden Bedingungen erfüllt:
1. **Domäne $D$ ist die Menge der Grundterme $\text{Term}_\Sigma^0$**:
   - Die Domäne $D$ besteht aus der Menge aller Grundterme, die mit den Symbolen in $\Sigma$ gebildet werden können.
   - Ein **Grundterm** ist ein term, der keine Variablen enthält, z.B. $a$, $f(a)$, $g(f(a), b)$, wobei $a$, $b$ Konstanten und $f$, $g$ Funktionssymbole sind.
2. **Interpretation der Funktionssymbole**:
   - Jedes Funktionssymbol $f$ in $\Sigma$ wird auf sich selbst abgebildet. Das bedeutet:
     $$I(f)(t_1, \ldots, t_n) = f(t_1, \ldots, t_n)$$
     - Hierbei sind $t_1, \ldots, t_n$ Grundterme.
     - In einer Herbrand-Struktur bedeutet dies, dass die Anwendung eines Funktionssymbols auf Grundterme ebenfalls wieder ein Grundterm ist.
### Selbstinterpretation der Grundterme
- In einer Herbrand-Struktur wird jeder Grundterm $t$ als er selbst interpretiert:
 $ $\text{val}_{D, I}(t) = t$$
Das bedeutet, die Bewertung eines Grundterms $t$ in der Domäne $D$ ist einfach der Term $t$ selbst.
### Spielraum bei der Interpretation der Prädikatsymbole
- Die Flexibilität in einer Herbrand-Struktur besteht nur bei der Interpretation der Prädikatsymbole. Prädikate können auf verschiedene Weisen interpretiert werden, um je nach Interpretation unterschiedliche Wahrheitswerte für die Grundterme zu liefern.
### Beispiel zur Veranschaulichung
#### Signatur und Grundterme
Angenommen, wir haben die Signatur $\Sigma$ mit:
- Konstanten: $a, b$
- Funktionssymbolen: $f$ (einstellig)
Dann ist die Menge der Grundterme:
$$\text{Term}_\Sigma^0 = \{ a, b, f(a), f(b), f(f(a)), f(f(b)), \ldots \}$$
#### Herbrand-Interpretation
1. **Domäne $D$**:
   - Die Domäne $D$ ist die Menge aller Grundterme:
     $D = \text{Term}_\Sigma^0$
2. **Interpretation der Funktionssymbole**:
   - Die Funktion $f$ wird wie folgt interpretiert:
     $I(f)(t) = f(t)$
Das bedeutet, wenn wir $f$ auf einen Grundterm $t$ anwenden, ist das Ergebnis einfach der Term $f(t)$.
#### Beispiel für eine Herbrand-Struktur
Wenn $\Sigma$ die Konstanten $a$ und $b$ sowie das Funktionssymbol $f$ enthält, könnte eine Herbrand-Struktur wie folgt aussehen:
- **Grundterme**:
$$\{a, b, f(a), f(b), f(f(a)), \ldots\}$$
- **Interpretation der Funktionssymbole**:
  $$I(f)(a) = f(a)$$
  $$I(f)(b) = f(b)$$
Hierbei ist $f(a)$ einfach der Term $f$ angewendet auf $a$, und $f(b)$ ist der Term $f$ angewendet auf $b$.

## Satz von Herbrand
Der Satz besagt, dass die folgenden vier Aussagen äquivalent sind:
1. **$M$ hat ein Modell**: Es gibt eine Interpretation, in der alle Formeln in $M$ wahr sind.
2. **$M$ hat ein Herbrand-Modell**: Es gibt eine Herbrand-Interpretation, in der alle Formeln in $M$ wahr sind.
3. **Grundinstanzen($M$) hat ein Modell**: Es gibt eine Interpretation, in der alle Grundinstanzen der Formeln in $M$ wahr sind.
4. **Grundinstanzen($M$) hat ein Herbrand-Modell**: Es gibt eine Herbrand-Interpretation, in der alle Grundinstanzen der Formeln in $M$ wahr sind.
### Überblick über den Beweis
Die Beweisschritte zeigen, wie die äquivalenten Aussagen ineinandergreifen:
1. **Implikationen 4 ⇒ 3 und 2 ⇒ 1 sind trivial**:
   - Wenn $Grundinstanzen(M)$ ein Herbrand-Modell hat (4), dann hat es natürlich auch ein allgemeines Modell (3), da ein Herbrand-Modell ein spezielles Modell ist.
   - Wenn $M$ ein Herbrand-Modell hat (2), dann hat es auch ein allgemeines Modell (1), da ein Herbrand-Modell auch ein Modell ist.
2. **Implikationen 1 ⇒ 3 und 2 ⇒ 4**:
   - **1 ⇒ 3**: Wenn $M$ ein Modell hat, sind alle universell quantifizierten Formeln in $M$ wahr, was bedeutet, dass auch ihre Grundinstanzen wahr sein müssen.
   - **2 ⇒ 4**: Wenn $M$ ein Herbrand-Modell hat, dann sind auch alle Grundinstanzen in einer Herbrand-Interpretation wahr.
3. **Beweis, dass 3 ⇒ 2**:
   - Wenn $Grundinstanzen(M)$ ein Modell $D$ hat, können wir eine Herbrand-Interpretation $H$ definieren, die ebenfalls alle Formeln in $M$ wahr macht. Der Beweis dieses Schrittes wird detailliert dargestellt.
### Detaillierter Beweis für 3 ⇒ 2
Angenommen, $D$ ist ein Modell für die Grundinstanzen von $M$. Wir konstruieren eine Herbrand-Interpretation $H = (Term^0_\Sigma, J)$, die $M$ erfüllt:
1. **Definition der Herbrand-Interpretation $H$**:
   - **Domäne $D$**: Die Domäne ist die Menge der Grundterme $Term^0_\Sigma$, also alle Terme, die ohne Variablen gebildet werden können.
   - **Interpretation $J$ der Prädikatsymbole**:
     $$J(p) := \{(t_1, \ldots, t_n) \mid t_i \in Term^0_\Sigma \; \text{und} \; val_D(p(t_1, \ldots, t_n)) = \text{wahr} \}$$
     - Diese Definition bedeutet, dass $J$ alle $n$-Tupel von Grundtermen enthält, für die das Prädikat $p$ in der Interpretation $D$ wahr ist.
2. **Beweis der Wahrheitswerte für geschlossene, quantorenfreie Formeln**:
   - **Geschlossene Atome**: Ein Atom ist eine quantorenfreie, einfache Aussage. Für jedes geschlossene Atom $A$ gilt:
     $$val_H(A) = val_D(A)$$
     - Das bedeutet, die Bewertung von $A$ in der Herbrand-Interpretation $H$ ist gleich der Bewertung in $D$.
   - **Induktion für quantorenfreie Formeln**: Durch Induktion kann man zeigen, dass diese Beziehung auch für komplexere, quantorenfreie Formeln $A$ gilt, die aus Atomen bestehen.
3. **Beweis für universell quantifizierte Formeln**:
   - Für jede Formel $\forall x_1 \ldots \forall x_n \; B$ in $M$ gilt:
     $$val_D(\{ x_1 / t_1, \ldots, x_n / t_n \} B) = \text{wahr}$$
     für alle Grundinstanzen $t_1, \ldots, t_n$.
   - Da in $H$ die Bewertungen von geschlossenen Formeln gleich sind:
     $$val_H(\{ x_1 / t_1, \ldots, x_n / t_n \} B) = \text{wahr}$$
   - Dadurch ergibt sich:
     $$val_H(\forall x_1 \ldots \forall x_n \; B) = \text{wahr}$$
   - Der letzte Schritt verwendet das Substitutionstheorem, welches besagt, dass eine universell quantifizierte Formel in einer Interpretation wahr ist, wenn ihre Grundinstanzen wahr sind.
## Satz von Herbrand: 2. Form
### Voraussetzungen
- **$\varphi$**: Eine quantorenfreie Formel ohne Gleichheit. Diese Formel hat nur eine freie Variable $ x $.
- **Grundterm**: Ein term ohne Variablen, also ein spezifischer, konkreter Wert oder eine Konstante.
### Aussagen des Satzes
1. **Existenzquantor**:
  $$\exists x \; \varphi \text{ ist allgemeingültig, genau dann wenn es ein } n \in \mathbb{N} \text{ und Grundterme } t_1, \ldots, t_n \text{ gibt, so dass } \varphi(t_1) \lor \ldots \lor \varphi(t_n) \text{ allgemeingültig ist.}$$
2. **Allquantor**:
   $\forall x \; \varphi \text{ ist unerfüllbar, genau dann wenn es ein } n \in \mathbb{N} \text{ und Grundterme } t_1, \ldots, t_n \text{ gibt, so dass } \varphi(t_1) \land \ldots \land \varphi(t_n) \text{ unerfüllbar ist.}$

### Beweis der zweiten Form des Satzes von Herbrand
Die Beweise nutzen grundlegende logische Beziehungen und den Endlichkeitssatz der Aussagenlogik. Hier ist die Schritt-für-Schritt-Erklärung:
#### Beweis für den Existenzquantor
1. **Formulierung**:
   $$\exists x \; \varphi \text{ ist allgemeingültig}$$
2. **Umformung in die Negation**:
   $$\exists x \; \varphi \text{ ist allgemeingültig} \Leftrightarrow \neg \exists x \; \varphi \text{ ist unerfüllbar}$$
   - Das bedeutet, dass die Negation der Aussage $\exists x \; \varphi$ keine Interpretation hat, in der sie wahr ist.
3. **Verwendung der Negationsregel**:
   $$\neg \exists x \; \varphi \Leftrightarrow \forall x \; \neg \varphi$$
   - Wir verwenden die logische Regel, dass die Negation eines Existenzquantors in einen Allquantor umgewandelt wird und die Negation des inneren Teils übernimmt.
4. **Umwandlung in Grundinstanzen**:
   $$\forall x \; \neg \varphi \text{ ist unerfüllbar}$$
   $$\Leftrightarrow \{\neg \varphi(t) \mid t \text{ ist ein Grundterm}\} \text{ ist unerfüllbar}$$
   - Das bedeutet, dass für jede mögliche Grundinstanz die Negation $\neg \varphi(t)$ keine Interpretation hat, in der sie wahr ist.
5. **Verwendung des Endlichkeitssatzes**:
   $$\Leftrightarrow \text{es gibt ein } n \text{ und Grundterme } t_1, \ldots, t_n \text{, so dass } \{\neg \varphi(t_1), \ldots, \neg \varphi(t_n)\} \text{ unerfüllbar ist}$$
   - Der Endlichkeitssatz besagt, dass, wenn eine unendliche Menge von Aussagen unerfüllbar ist, dann auch eine endliche Teilmenge davon unerfüllbar ist.
6. **Umkehrung der Negation**:
   $$\Leftrightarrow \neg (\neg \varphi(t_1) \land \ldots \land \neg \varphi(t_n)) \text{ ist unerfüllbar}$$
   $$\Leftrightarrow \varphi(t_1) \lor \ldots \lor \varphi(t_n) \text{ ist allgemeingültig}$$
   - Wir verwenden die Regel, dass die Negation einer Konjunktion in eine Disjunktion der negierten Teile umgewandelt wird.
Das zeigt, dass $\exists x \; \varphi$ allgemeingültig ist, genau dann, wenn es eine endliche Anzahl von Grundtermen gibt, deren Disjunktion allgemeingültig ist.
#### Beweis für den Allquantor
1. **Formulierung**:
   $$\forall x \; \varphi \text{ ist unerfüllbar}$$
2. **Umformung in die Negation**:
   $$\forall x \; \varphi \text{ ist unerfüllbar} \Leftrightarrow \neg \forall x \; \varphi \text{ ist allgemeingültig}$$
   - Das bedeutet, dass die Negation der Aussage $\forall x \; \varphi$ keine Interpretation hat, in der sie wahr ist.
3. **Verwendung der Negationsregel**:
   $$\neg \forall x \; \varphi \Leftrightarrow \exists x \; \neg \varphi$$
   - Wir verwenden die logische Regel, dass die Negation eines Allquantors in einen Existenzquantor umgewandelt wird und die Negation des inneren Teils übernimmt.
4. **Umwandlung in Grundinstanzen**:
   $$\exists x \; \neg \varphi \text{ ist allgemeingültig}$$
   $$\Leftrightarrow \{\neg \varphi(t) \mid t \text{ ist ein Grundterm}\} \text{ ist allgemeingültig}$$
   - Das bedeutet, dass für jede mögliche Grundinstanz die Negation $\neg \varphi(t)$ keine Interpretation hat, in der sie wahr ist.
5. **Verwendung des Endlichkeitssatzes**:
   $$\Leftrightarrow \text{es gibt ein } n \text{ und Grundterme } t_1, \ldots, t_n \text{, so dass } \{\neg \varphi(t_1), \ldots, \neg \varphi(t_n)\} \text{ allgemeingültig ist}$$
6. **Umkehrung der Negation**:
   $$\Leftrightarrow \neg (\neg \varphi(t_1) \lor \ldots \lor \neg \varphi(t_n)) \text{ allgemeingültig}$$
   $$\Leftrightarrow \varphi(t_1) \land \ldots \land \varphi(t_n) \text{ ist unerfüllbar}$$
   - Wir verwenden die Regel, dass die Negation einer Disjunktion in eine Konjunktion der negierten Teile umgewandelt wird.
Das zeigt, dass $\forall x \; \varphi$ unerfüllbar ist, genau dann, wenn es eine endliche Anzahl von Grundtermen gibt, deren Konjunktion unerfüllbar ist.

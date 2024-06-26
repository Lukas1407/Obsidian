> [!abstract] 
> Eine Formel $A$ ist in Negationsnormalform (NNF), wenn:
> 1. **Jedes Negationszeichen direkt vor einer atomaren Teilformel steht**: Das bedeutet, dass die Negation nur auf atomare Formeln (wie $p$, $q$, etc.) angewendet wird und nicht auf komplexere Ausdrücke oder Kombinationen von Formeln.
> 	- **Atomare Formel**: Eine Formel ohne logische Operatoren (wie $p$ oder $q$).
> 	- **Keine Teilformel der Form $\neg \neg B$**: Es dürfen keine doppelten Negationen vorkommen.
> 2.  **Keine Implikation in der Formel vorkommt**: Die Formel darf keine Implikationen $(\rightarrow)$ enthalten. Implikationen müssen durch äquivalente Ausdrücke umgeschrieben werden.
## Bereinigte Formel
Eine Formel $A$ ist bereinigt, wenn:
1. **Freie und gebundene Variablen überschneiden sich nicht**: Es dürfen keine Variablen geben, die sowohl frei als auch gebunden sind. Das bedeutet, dass $\text{Frei}(A) \cap \text{Bd}(A) = \emptyset$ gilt.
   - **Freie Variablen**: Variablen, die nicht durch einen Quantor $\forall$ oder $\exists$ gebunden sind.
   - **Gebundene Variablen**: Variablen, die durch einen Quantor gebunden sind.
2. **Die hinter Quantoren stehenden Variablen sind paarweise verschieden**: Jede gebundene Variable muss eindeutig sein und darf nicht mehrfach in verschiedenen Quantoren verwendet werden.
## Theorem zur Existenz von äquivalenten Formeln
1. **Negationsnormalform**: Zu jeder Formel $A$ gibt es eine logisch äquivalente Formel $B$ in Negationsnormalform. Das bedeutet, dass für jede logische Formel eine Darstellung existiert, in der die genannten Bedingungen für die NNF erfüllt sind.
2. **Bereinigte Formel**: Zu jeder Formel $A$ gibt es eine logisch äquivalente, bereinigte Formel $B$. Das bedeutet, dass es möglich ist, die Formel so umzuformen, dass die freien und gebundenen Variablen sich nicht überschneiden und die Variablen hinter den Quantoren eindeutig sind.
## Umwandlung in Negationsnormalform
Die Umwandlung einer beliebigen logischen Formel in NNF erfolgt durch systematisches Umschreiben. Hier sind die wesentlichen Schritte:
1. **Ersetzen von Implikationen und Äquivalenzen**:
   - Implikationen $A \rightarrow B$ werden durch $\neg A \lor B$ ersetzt.
   - Äquivalenzen $A \leftrightarrow B$ werden durch $(A \land B) \lor (\neg A \land \neg B)$ ersetzt.
2. **Verschieben von Negationen nach innen**:
   - Verwende die De Morgan'schen Gesetze, um Negationen nach innen zu verschieben:
     - $\neg (A \land B)$ wird zu $\neg A \lor \neg B$
     - $\neg (A \lor B)$ wird zu $\neg A \land \neg B$
   - Negationen vor Quantoren verschieben:
     - $\neg \forall x A$ wird zu $\exists x \neg A$
     - $\neg \exists x A$ wird zu $\forall x \neg A$
3. **Eliminierung von doppelten Negationen**:
   - $\neg \neg A$ wird zu $A$
4. **Bereinigen von freien und gebundenen Variablen**:
   - Stelle sicher, dass freie und gebundene Variablen sich nicht überschneiden und dass die gebundenen Variablen eindeutig sind.
### Beispiel für die Umwandlung in NNF
Angenommen, wir haben die Formel:$$\forall x (\neg (p(x) \rightarrow q(x)) \land \exists y (r(y) \rightarrow \neg p(y)))$$
1. **Ersetze Implikationen**:$$p(x) \rightarrow q(x) \text{ wird zu } \neg p(x) \lor q(x)$$$$r(y) \rightarrow \neg p(y) \text{ wird zu } \neg r(y) \lor \neg p(y)$$
   Die Formel wird dann zu:$$\forall x (\neg (\neg p(x) \lor q(x)) \land \exists y (\neg r(y) \lor \neg p(y)))$$

2. **Verschiebe Negationen nach innen**:$$\neg (\neg p(x) \lor q(x)) \text{ wird zu } p(x) \land \neg q(x) \text{ (De Morgan)}$$

   Die Formel wird dann zu:   $$\forall x ((p(x) \land \neg q(x)) \land \exists y (\neg r(y) \lor \neg p(y)))$$

3. **Bereinige die Variablen**:
   - Stellen wir sicher, dass $y$ in $\exists y$ nicht im selben Bereich wie $x$ verwendet wird. Da dies hier nicht notwendig ist, bleibt die Formel wie sie ist.
Die resultierende Formel ist in Negationsnormalform:$$\forall x ((p(x) \land \neg q(x)) \land \exists y (\neg r(y) \lor \neg p(y)))$$

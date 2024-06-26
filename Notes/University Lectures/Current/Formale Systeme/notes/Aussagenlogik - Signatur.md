> [!abstract] Definition
> Eine Signatur in der Aussagenlogik ist eine abzählbare Menge von Symbolen $\Sigma$, die verwendet werden, um Aussagen zu bilden. 

## Was bedeutet "abzählbare Menge"?
- Eine abzählbare Menge ist eine Menge, deren Elemente in einer Sequenz oder Liste angeordnet werden können. Das heißt, wir können die Elemente der Menge nummerieren, wie $P_{1},P_{2},\dots$
- Dies bedeutet, dass es entweder eine endliche Anzahl von Symbolen gibt, oder wenn die Menge unendlich ist, können wir sie in einer unendlichen Liste aufzählen.
## Elemente der Signatur
Die Elemente einer Signatur heißen:
- **Atomare Aussagen**: Sie sind die grundlegendsten Aussagen, die in der Logik betrachtet werden, und können wahr oder falsch sein.
- **Atome**: Ein anderer Begriff für atomare Aussagen.
- **Aussagevariablen**: Diese Variablen stehen für beliebige Aussagen, deren Wahrheitswert wir später festlegen.
## Verwendung der Signatur
### Warum ist eine Signatur wichtig?
- Eine Signatur legt den Rahmen fest, innerhalb dessen wir unsere logischen Aussagen formulieren.
- Sie definiert die Basisbausteine, aus denen wir komplexere logische Ausdrücke bilden können.
### Beispielhafte Nutzung
- Mit einer Signatur $\Sigma = \{P_0, P_1, P_2\}$ können wir Aussagen wie $P_0 \wedge \neg P_1$ oder $P_2 \vee P_0$​ bilden.
- Diese Aussagen können dann mit logischen Operationen kombiniert werden, um komplexere Aussagen zu erstellen.
## Definition der Menge der Formeln $\text{For}^0_\Sigma$
- $\text{For}^0_\Sigma$ ist die Menge and Formeln die über $\Sigma$ definiert werden kann
### Grundlegende Elemente
- $1\in \text{For}^0_\Sigma$
- $0\in \text{For}^0_\Sigma$
- $\Sigma\subseteq \text{For}^0_\Sigma$​: Alle atomaren Aussagen oder Variablen aus der Signatur $\Sigma$ sind Formeln
### Induktive Erweiterung
- Wenn $A$ und $B$ Formeln in $\text{For}^0_\Sigma$​ sind, dann sind auch die folgenden zusammengesetzten Formeln Elemente von $\text{For}^0_\Sigma$:
	- $\lnot A$
	- $(A\land B)$
	- $(A\lor B)$
	- $(A\rightarrow B)$
	- $(A\leftrightarrow B)$
## Modell einer Formel $A\in \text{For}^0_\Sigma$
- Eine [[Aussagenlogik - Interpretation]] $I$ über der Signatur $\Sigma$ wird als Modell einer Formel $A$ bezeichnet, wenn die Auswertung von $A$ unter dieser [[Aussagenlogik - Interpretation|Interpretation]] wahr ist.
- Formal: $I$ ist ein Modell von $A$, wenn $$\text{val}_{I}(A)=W$$
## Modell einer Formelmenge $M\subseteq\text{For}^0_\Sigma$
- Eine [[Aussagenlogik - Interpretation|Interpretation]] $I$ ist ein Modell einer Menge von Formeln $M$, wenn sie ein Modell für jede einzelne Formel in M ist.
	- -> Sie ist für jede Formel in $M$ wahr
## Allgemeingültigkeit
- Eine Formel ist allgemeingültig, wenn sie unter jeder möglichen [[Aussagenlogik - Interpretation|Interpretationen]] über der Signatur $\Sigma$ wahr ist
- Formal: $A$ ist allgemeingültig, wenn für jede Interpretation $I$ über $\Sigma$ gilt:$$ \text{val}_I(A) = W$$
### Beispiele
#### Selbstimplikation ($A\rightarrow A$)
- **Beschreibung:** Eine Aussage AAA impliziert sich selbst.
#### Tertium non datur $(¬A ∨ A$)
- **Beschreibung:** Auch bekannt als das Gesetz des ausgeschlossenen Dritten.
- **Bedeutung:** Eine Aussage ist entweder wahr oder falsch; es gibt keine dritte Möglichkeit.
- **Beispiel:** Entweder es regnet (AAA) oder es regnet nicht (¬A\neg A¬A).
#### Abschwächung ($A → (B → A)$)
- **Beschreibung:** Eine Aussage $A$ bleibt wahr, unabhängig von der Implikation $B \rightarrow A$
- **Bedeutung:** Wenn $A$ wahr ist, dann ist die Implikation $B \rightarrow A$ immer wahr, unabhängig von $A$.
- **Beispiel:** Wenn $A$ „Es regnet“ ist, dann bedeutet $A \rightarrow (B \rightarrow A)$ „Wenn es regnet, dann folgt aus jeder Aussage $B$, dass es regnet“.
#### Ex falso quodlibet ($0 → A$)
- **Beschreibung:** Aus einem Widerspruch folgt jede Aussage.
- **Bedeutung:** Wenn $0$ (falsch) wahr ist, dann kann jede Aussage $A$ daraus gefolgert werden, weil ein Widerspruch alle Aussagen wahr macht.
- **Beispiel:** Wenn „es regnet und es regnet nicht“ wahr wäre, könnte man daraus jede beliebige Aussage ableiten.
#### Modus Ponens ($(A ∧ (A → B)) → B$)
- **Beschreibung:** Wenn $A$ wahr ist und $A \rightarrow B$ wahr ist, dann ist $B$ wahr.
- **Bedeutung:** Aus der Wahrheit von $A$ und der Wahrheit der Implikation $A \rightarrow B$ folgt die Wahrheit von $B$
- **Beispiel:** Wenn „es regnet“ ($A$) und „wenn es regnet, dann wird die Straße nass“ ($A \rightarrow B$) wahr ist, dann ist „die Straße wird nass“ ($B$) wahr.
#### Idempotenz ($A ∧ A ↔ A$)
- **Beschreibung:** Eine Aussage und sich selbst ergibt wieder die gleiche Aussage.
- **Bedeutung:** Das logische Und von $A$ mit sich selbst ist äquivalent zu $A$.
- **Beispiel:** „Es regnet und es regnet“ ist äquivalent zu „es regnet“.
#### Doppelnegation ($(¬¬A) ↔ A$)
- **Beschreibung:** Eine doppelte Negation ist äquivalent zu der ursprünglichen Aussage.
#### Absorption ($A ∧ (A ∨ B) ↔ A$)
- **Beschreibung:** Eine Aussage absorbiert sich selbst aus einer Disjunktion.
- **Bedeutung:** Das logische Und von $A$ und $A \lor B$ ist äquivalent zu $A$.
- **Beispiel:** „Es regnet und es regnet oder es schneit“ ist äquivalent zu „es regnet“.
#### Äquivalenz/Implikation ($(A ↔ B) ↔ ((A → B) ∧ (B → A))$)
- **Beschreibung:** Eine Äquivalenz kann als zwei wechselseitige Implikationen dargestellt werden.
- **Bedeutung:** Zwei Aussagen $A$ und $B$ sind äquivalent, wenn $A$ $B$ impliziert und $B$ $A$ impliziert.
- **Beispiel:** „Es regnet genau dann, wenn die Straße nass ist“ ist äquivalent zu „wenn es regnet, dann ist die Straße nass“ und „wenn die Straße nass ist, dann regnet es“.
#### Distributivität von $∧$ über $∨$ ($A ∧ (B ∨ C) ↔ (A ∧ B) ∨ (A ∧ C)$)
- **Beschreibung:** Das logische Und verteilt sich über das logische Oder.
- **Bedeutung:** Eine Aussage $A$ und $B$ oder $C$ ist äquivalent zu $A$ und $B$ oder $A$ und $B$.
- **Beispiel:** „Es regnet und (es ist kalt oder es schneit)“ ist äquivalent zu „es regnet und es ist kalt oder es regnet und es schneit“.
#### Distributivität von $∨$ über $∧$ ($A ∨ (B ∧ C) ↔ (A ∨ B) ∧ (A ∨ C)$)
- **Beschreibung:** Das logische Oder verteilt sich über das logische Und.
- **Bedeutung:** Eine Aussage $A$ oder BBB $B$ $C$ ist äquivalent zu $A$ oder $B$ und $A$ oder $C$.
- **Beispiel:** „Es regnet oder (es ist kalt und es schneit)“ ist äquivalent zu „es regnet oder es ist kalt und es regnet oder es schneit“.
### Kontraposition ($(A→B)↔(¬B→¬A)$)
- **Beschreibung:** Die Implikation $A \rightarrow B$ ist äquivalent zur Implikation $\neg B \rightarrow \neg A$
- **Bedeutung:** Wenn $A$ die Bedingung für $B$ ist, dann bedeutet das auch, dass wenn $B$ nicht wahr ist, $A$ ebenfalls nicht wahr sein kann.
- **Beispiel:** „Wenn es regnet ($A$), wird die Straße nass ($B$)“ ist äquivalent zu „Wenn die Straße nicht nass ist ($\neg B$), hat es nicht geregnet ($\neg A$)“.
#### Verteilen ($(A→(B→C))↔((A→B)→(A→C))$)
- **Beschreibung:** Eine verschachtelte Implikation kann umgeformt werden, sodass die äußere Implikation zuerst betrachtet wird.
- **Bedeutung:** Wenn $A$ impliziert, dass $B$ $C$ impliziert, dann ist das äquivalent dazu, dass wenn $A$ $B$ impliziert, dann $A$ auch $B$ impliziert.
- **Beispiel:** „Wenn es regnet ($A$), dann wenn die Straße nass ist ($B$), dann wird es rutschig ($C$)“ ist äquivalent zu „Wenn es regnet, dann wenn die Straße nass ist, dann wird es rutschig“.
#### De Morgan'sche Gesetze für Disjunktion ($¬(A∨B)↔(¬A∧¬B)$)
- **Beschreibung:** Die Negation einer Disjunktion ist äquivalent zur Konjunktion der Negationen.
- **Bedeutung:** „Es ist nicht der Fall, dass entweder $A$ oder $B$ wahr ist“ ist äquivalent zu „$A$ ist nicht wahr und $B$ ist nicht wahr“.
- **Beispiel:** „Es ist nicht der Fall, dass es regnet oder schneit“ ist äquivalent zu „Es regnet nicht und es schneit nicht“.
#### De Morgan'sche Gesetze für Konjunktion ($¬(A∧B)↔(¬A∨¬B)$)
- **Beschreibung:** Die Negation einer Konjunktion ist äquivalent zur Disjunktion der Negationen.
- **Bedeutung:** „Es ist nicht der Fall, dass sowohl $A$ als auch $B$ wahr sind“ ist äquivalent zu „$A$ ist nicht wahr oder $B$ ist nicht wahr“.
- **Beispiel:** „Es ist nicht der Fall, dass es regnet und die Straße nass ist“ ist äquivalent zu „Entweder regnet es nicht oder die Straße ist nicht nass“.
## Erfüllbarkeit einer Formel $A\in \text{For}^0_\Sigma$
- Eine Formel ist erfüllbar, wenn es mindestens eine [[Aussagenlogik - Interpretation|Interpretation]] gibt, unter der die Formel wahr ist.
## Logische Folgerung ($M ⊨ A$)
- Gelesen: „Aus $M$ folgt $A$“ oder „$M$ impliziert $A$“.
- $M$ ist eine Menge von Formeln (Aussagen) über einer Signatur $\Sigma$
- $A$ ist eine einzelne Formel aus $\text{For}^0_\Sigma$
- $M⊨A$ bedeutet, dass jedes Modell von $M$ auch ein Modell von $A$ ist. Das heißt, wenn alle Formeln in $M$ wahr sind, dann ist auch $A$ wahr
### Beispiel
- Wenn $M = \{P \rightarrow Q, P\}$ ist und $A=Q$, dann bedeutet $M\models A$, dass aus den Formeln in $M$ folgt, dass $Q$ wahr ist, wenn $P$ wahr ist.
## Logische Äquivalenz ($A≡B$)
- Gelesen: „AAA und BBB sind logisch äquivalent“.
- $A$ und $B$ sind zwei Formeln über der gleichen Signatur $\Sigma$
- $A$ und $B$ sind logisch äquivalent, wenn aus $A$ $B$ folgt und aus $B$ $A$ folgt. Das bedeutet, dass beide Formeln unter allen möglichen Interpretationen denselben Wahrheitswert haben.
### Formal
- $A$ und $B$ sind logisch äquivalent, wenn sowohl $\{A\} \models B$ als auch $\{B\} \models A$ gilt
### Beispiel
- Die Formeln $A = P \land Q$ und $B = Q \land P$ sind logisch äquivalent, weil sie unter jeder Interpretation denselben Wahrheitswert haben. Formal heißt das: $\{P \land Q\} \models Q \land P$ und $\{Q \land P\} \models P \land Q$

> [!summary] 
> Berechnung eines scharfen Wertes (präzisen numerischen Wert umgewandelt) aus der Fuzzy-Menge der Ausgangsgröße (Ergebnis der [[Inferenz bei Fuzzy-Systemen#Prämissenauswertung (Synonym Aggregation)|Akkumulation]]) 

## Varianten
### Maximum-Methode
- Bei dieser Methode wird der Ausgangswert gewählt, der die höchste Zugehörigkeit in der Fuzzy-Menge hat.
-  Wenn mehrere Werte die gleiche höchste Zugehörigkeit haben, wird eine Strategie benötigt, um einen Wert auszuwählen:
	- **LM (linkes Maximum)**: Wählt den am weitesten links liegenden Wert.
	- **RM (rechtes Maximum)**: Wählt den am weitesten rechts liegenden Wert.
	- **MoM (Mean of Maxima)**: Berechnet den mittleren Wert aller Maxima.
- Diese Methode kann Sprünge im Ein-Ausgangsverhalten erzeugen und ist besonders geeignet, wenn die Ausgangsterme wertediskret sein müssen, wie z.B. Gänge in einem Schaltgetriebe oder Fehlertypen.
- besonders geeignet, wenn die Ausgangsterme wertediskret sein müssen (Beispiel Gänge im Schaltgetriebe, Fehlertypen usw.)
### Schwerpunkt-Methode für Singleton (Center of Gravity for Singletons (COGS))
- Bei dieser Methode werden Singletons als Ausgangs-Zugehörigkeitsfunktionen (ZGF) verwendet. Sie kann direkt auf die Regelkonklusionen $C_{r}$ oder auf die Ergebnisse der Akkumulation angewendet werden.
### Center of Gravity for Takagi-Sugeno (Center of Gravity for Takagi-Sugeno-Systeme (COGTS))
- Jede Regel hat eine Funktion als Konklusion. Diese Variante erzeugt normalerweise ein stetiges Ein-/Ausgangsverhalten und ist besonders geeignet für Regelungen oder die Fusion von Modellen.

## Beispiel
- Gegeben sind die linguistischen Terme und ihre Zugehörigkeiten
![[Pasted image 20240413080712.png#invert|500]]
### COGS-Methode
- berechnet den Ausgangswert $y$ durch Gewichtung der Singletons mit ihren Zugehörigkeiten und Division durch die Summe der Zugehörigkeiten:$$
y=\frac{0∙ −1 +0∙ −0.5 +0.35∙0+0.5 ∙0.5+0.15 ∙1}{0+0+0.35+0.5+0.15}=0.4$$
### Maximum-Methode
- würde der Wert $y=0.5$ sein, da “AUF” die höchste Zugehörigkeit hat und der Singleton für “AUF” bei 0.5 positioniert ist.
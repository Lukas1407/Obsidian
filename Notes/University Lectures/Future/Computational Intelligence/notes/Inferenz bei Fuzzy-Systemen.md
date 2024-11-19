> [!summary] Definition
>  Der Inferenzmechanismus wendet die Regeln auf die fuzzifizierten Eingaben an, um eine unscharfe Ausgabe zu erzeugen.

## Regelbasis Varianten
am Beispiel Temperaturregelung mit $T$: Temperatur, $DT$: Temperaturänderung, $DU$: Öffnung Heizventil (Richtung der Änderung)
### Liste
1. WENN $T$=ZU WARM UND $DT$=POS DANN $DU$=WEIT ZU 
2. WENN $T$=ANGENEHM UND $DT$=POS DANN $DU$=ZU 
3. WENN $T$=ZU KALT UND $DT$=POS DANN $DU$=GLEICH 
4. WENN $T$=ZU WARM UND $DT$=NULL DANN $DU$=ZU 
5. WENN $T$=ANGENEHM UND $DT$=NULL DANN $DU$=GLEICH 
6. WENN $T$=ZU KALT UND $DT$=NULL DANN $DU$=AUF 
7. WENN $T$=ZU WARM UND $DT$=NEG DANN $DU$=GLEICH 
8. WENN $T$=ANGENEHM UND $DT$=NEG DANN $DU$=AUF 
9. WENN $T$=ZU KALT UND $DT$=NEG DANN $DU$=WEIT AUF
### Tabelle
![[Pasted image 20240413073247.png#invert|500]]
## Konklusionen
- In Regelbasen liegen Regeln meist so vor:$$R_{r}:\text{WENN}\ \underbrace{ \underbrace{x_{1}=A_{1,R_{r}}}_{\text{Teilprämisse\ $V_{r1}$}} \ \text{UND ... UND } \underbrace{x_{s}=A_{s,R_{r}}}_{\text{Teilprämisse\ $V_{rs}$}}}_{\text{Prämisse $V_{r}$}} \ \text{DANN } y=C_{r}$$
### Mamdani-Systeme
- In Mamdani-Systemen ist die Konklusion $y = C_r = B_c$ ein linguistischer Term, der durch eine Fuzzy-Menge repräsentiert wird
- Die Ausgabe jeder Regel ist eine Fuzzy-Menge, und die endgültige Ausgabe wird durch die Kombination dieser Fuzzy-Mengen und anschließende Defuzzifizierung erhalten.
-  Dieser Ansatz ist intuitiv und gut geeignet für menschliche Eingaben und Expertensysteme, da die Regeln aus menschlichem Expertenwissen erstellt werden können.
- Z.B.: ... DANN $DU$=WEIT ZU
### Takagi-Sugeno-Systeme (TS)
- Takagi-Sugeno-Systeme verwenden Funktionen der Eingangswerte als Konklusionen. 
- Die Ausgabe $y = C_r = f_r(x)$ kann eine konstante oder eine lineare Funktion der Eingangswerte sein. Im Gegensatz zu Mamdani-Systemen, bei denen die Ausgabe eine Fuzzy-Menge ist, ist die Ausgabe bei TS-Systemen eine reelle Zahl, die durch gewichtete Durchschnitte oder Summen berechnet wird, was sie rechnerisch effizient macht.
- Z.B.: ... DANN $y = DU = a (Tsoll-T) + b (DT)^ 2$ mit Konstanten $a, b$ und zusätzlichem Eingang $T_{soll}$
### Singleton-Systeme
- Bei Singleton-Systemen ist die Ausgabe $y = y_r$ ein reeller Wert, der als Singleton bezeichnet wird.
-  Ein Singleton ist eine Fuzzy-Menge, die nur an einem einzigen Punkt den Wert 1 hat und sonst überall 0 ist. 
- Diese Systeme sind nützlich, wenn die Ausgabe als eine bestimmte Aktion oder ein spezifischer Wert ohne Unsicherheit dargestellt werden soll
- Z.B.: ... DANN $y = DU = -1 min^{-1}$

## Prämissenauswertung (Synonym: Aggregation)
- Hierbei wird der Zugehörigkeitsgrad der gesamten Prämisse einer linguistischen Regel bestimmt. Dies geschieht durch die Verknüpfung der Zugehörigkeitsgrade aller linguistischen Teilprämissen mittels [[Fuzzy-Logik#Fuzzy-Logik|Fuzzy-Operatoren]]
	- Operation: UND - Verknüpfung der Zugehörigkeitsgrade aller $s$ Teilprämissen einer Regel $R_r$ (evtl. unterlagertes ODER innerhalb der Teilprämissen)
	- Ausgangsgröße: ein Zugehörigkeitsgrad der Prämisse pro Regel
- Die **Aktivierung** oder Komposition folgt der Prämissenauswertung und bestimmt den Zugehörigkeitsgrad der Konklusion einer linguistischen Regel basierend auf dem Zugehörigkeitsgrad der Prämisse. Dieser Schritt bestimmt, wie stark die Konklusion der Regel aktiviert wird.
- Die **Akkumulation** ist der Prozess, bei dem die Zugehörigkeitsgrade der Terme der Ausgangsgröße aus allen aktivierten Regeln zusammengefasst werden. Dies führt zu einer Gesamt-Fuzzy-Menge für die Ausgangsgröße, die dann [[Defuzzyfizierung]] werden muss, um einen scharfen Ausgabewert zu erhalten.
### Beispiel
1. Prämissenauswertung: 
	- Die Regel lautet “WENN $T$ = ANGENEHM UND $DT$ = NULL DANN …”
	- Die Zugehörigkeitsgrade für die linguistischen Terme “ANGENEHM” und “NULL” sind gegeben als:
		-  $\mu_{\text{ANGENEHM}}(x_1) = 0.5$ und $\mu_{\text{NULL}}(x_2) = 0.7$
	- Diese werden multipliziert (siehe [[Fuzzy-Logik#Fuzzy-Logik|Multiplikation in der Fuzzy.Logik]], um den Zugehörigkeitsgrad der gesamten Prämisse zu erhalten: $\mu_{V5} = 0.5 \times 0.7 = 0.35$
2. Aktivierung: 
	- Die Aktivierung bestimmt den Zugehörigkeitsgrad der Konklusion basierend auf dem Zugehörigkeitsgrad der Prämisse. In diesem Fall wird der Wert 0.35 als Aktivierungsgrad für die Konklusion der Regel verwendet.
![[Pasted image 20240413075342.png#invert|500]]
3. Akkumulation: 
	- Die Akkumulation fasst alle Werte aus der Aktivierung für denselben linguistischen Term zusammen. 
	- Wenn mehrere Regeln denselben Ausgangsterm haben, werden ihre Zugehörigkeitsgrade mit einem ODER-Operator (hier als beschränkte Summe verwendet, um sicherzustellen, dass die resultierende Zugehörigkeit nicht größer als 1 wird) kombiniert. 
	- Dies führt zu einem Gesamtzugehörigkeitsgrad für jeden linguistischen Term der Ausgangsgröße.
![[Pasted image 20240413075944.png#invert|500]]

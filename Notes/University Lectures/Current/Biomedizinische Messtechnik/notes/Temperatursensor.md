## Thermistor (Temperature-sensitive resistor) (Wärmeleitung)
- Ändern ihren Widerstand abhängig von der Temperatur
![[1592479227-243-worutpwan.jpg.1280x0.webp#invert|600]]
### PTC (Positive temperature coefficient) resistor
- Elektrischer Widerstand steigt mit der Temperatur 
- z.B. Reine Metalle 
- Insbesondere Platin (PT100), über einen weiten Temperaturbereich hinreichend lineare Kennlinie 
### NTC (Negative temperature coefficient) resistor 
- Elektrischer Widerstand sinkt mit der Temperatur 
- z.B. Halbleiter (Bandlücke !) 
- Hohe Sensitivität -> Ideal für kleinen Temperaturbereich 
- Aber: stark nichtlineare Kennlinie!
- Besser für Körpertemperatur, da sie sensibler (größere Steigung) im Bereich von 30-45 Grad sind
- Die NTC-Kennlinie kann in guter Näherung durch die B-Formel modelliert werden (Messprinzip):$$R=R_{0}\cdot e^{B(\frac{1}{T}- \frac{1}{T_{0}})}$$, mit $R_0$ = Widerstand bei der Referenztemperatur $T_0$
- Ein NTC-Widerstand (Negativer Temperaturkoeffizient) wird möglichst hochomig gewählt, um eine hohe Empfindlichkeit gegenüber Temperaturänderungen zu gewährleisten. Bei einem hohen Widerstandswert ändert sich der Widerstand des NTCs deutlich mit der Temperatur, was zu einer größeren Änderung des durchfließenden Stroms führt. Dies erleichtert die Messung und Steuerung von Temperaturänderungen.
- Außerdem wird ein hochomiger NTC in der Anfangsphase, wenn das Gerät kalt ist, den Einschaltstrom begrenzen. Der hohe Anfangswiderstand reduziert den anfänglichen Stromstoß, der durch das Gerät fließt, wenn es eingeschaltet wird. Nach dem Einschalten erwärmt sich der NTC durch den Stromfluss und sein Widerstand nimmt ab, was zu einem normalen Betriebsstrom führt. Diese Eigenschaft macht hochomige NTCs nützlich als Einschaltstrombegrenzer, um elektronische Geräte vor Schäden durch hohe Einschaltströme zu schützen
![[Pasted image 20240429083330.png#invert|400]]
#### Analogschaltung
![[Pasted image 20240429083552.png#invert|300]]
Die Analogschaltung, die im Bild dargestellt ist, zeigt einen Spannungsteiler, der aus zwei Widerständen besteht. In einer Temperaturmessanwendung könnte einer dieser Widerstände ein Thermistor sein, dessen Widerstand sich mit der Temperatur ändert. Hier ist eine allgemeine Erklärung, wie ein Spannungsteiler in einer Temperaturmessschaltung funktionieren könnte:
- **Spannungsteiler**: Ein Spannungsteiler besteht aus zwei in Reihe geschalteten Widerständen. Die Ausgangsspannung $U_{out}$ wird zwischen dem zweiten Widerstand und dem gemeinsamen Erdungspunkt gemessen. Die Formel für den Spannungsteiler lautet:$$\frac{U_{out}}{U_{in}}= \frac{R_{2}}{R_{1}+R_{2}}$$
    wobei $U_{in}$ die Eingangsspannung ist und $R_{1}$ und $R_{2}$ die Widerstandswerte der beiden Widerstände.
- **Variable Widerstände**: In einer Temperaturmessschaltung kann ein variabler Widerstand wie ein Thermistor oder ein Potentiometer verwendet werden. Wenn sich die Temperatur ändert, ändert sich auch der Widerstand des Thermistors, was zu einer Änderung der Ausgangsspannung $U_{out}$ führt.
- **Messung**: Die Änderung der Ausgangsspannung kann dann gemessen und in eine Temperatur umgerechnet werden, indem man die Charakteristik des verwendeten Thermistors kennt.
In der Praxis würde man das Potentiometer so einstellen, dass es bei einer bekannten Temperatur eine bestimmte Ausgangsspannung liefert. Ändert sich die Temperatur, ändert sich der Widerstand des Thermistors, was zu einer Änderung der Ausgangsspannung führt. Diese Änderung kann dann gemessen und zur Bestimmung der Temperatur verwendet werden.
#### Mit Kondensator
**Unterschiede zur vorherigen Schaltung:**
- Die alternative Schaltung könnte einen zusätzlichen Kondensator enthalten, der parallel zum NTC-Thermistor geschaltet ist, um das Rauschen weiter zu reduzieren.
- Die Anordnung der Komponenten könnte sich unterscheiden, was die Empfindlichkeit und Reaktionszeit der Temperaturmessung beeinflussen kann.

## Thermopile-Sensoren für Bolometer (Wärmestrahlung Methode 1) 
- Thermopile-Sensoren sind eine Art von Thermoelementen, die zur Messung von Wärmestrahlung verwendet werden. Sie bestehen aus mehreren Thermoelementen, die in Reihe geschaltet sind, um eine höhere Spannung zu erzeugen, die proportional zur Temperaturdifferenz über den Elementen ist.
- Die an einem Thermoelement (Thermocouple) auftretende Spannung $U_{tc}$ ist abhängig von der Temperaturdifferenz über dem Element und dem Seebeck-Koeffizienten $\alpha$: $$U_{tc}=\alpha\Delta T_{tc} \tag{3.1.16}$$, hierbei ist $U_{tc}$ die Spannung, die am Thermoelement auftritt,$\alpha$ ist der Seebeck-Koeffizient, der materialabhängig ist, und $\Delta T_{tc}$ ist die Temperaturdifferenz über dem Thermoelement.
- Wenn wir die Wärmeleitung ( G ) über das Element berücksichtigen, ergibt sich aus den Gl. 3.1.3 und 3.1.16 für eine Reihenschaltung von $n$ Elementen die Beziehung:$$U_{tc}=\frac{n\alpha}{G_{tc}}\dot Q_{tc} \tag{3.1.17}$$, hier ist $\dot Q_{tc}$ die Wärmemenge, die pro Zeit durch das Thermoelement fließt.
- Aus der Bedingung $\dot Q_{tc}=\dot Q_{rad}$ folgt $$U(T_{Ob}) = \frac{\epsilon_{A}A_{A}\sigma n\alpha}{G_{tc}}(T_{Ob}^{4}-T_{Det}^{4}) \tag{3.1.18}$$
### Übungsaufgaben
#### Leite Gl. 3.1.7 her
- Diese Gleichung leitet sich aus der Grundgleichung für ein Thermoelement ab:$$U_{tc}=\alpha\Delta T_{tc} \tag{3.1.16}$$wobei $\Delta T_{tc}$ die Temperaturdifferenz über dem Thermoelement ist.
- Die Wärmemenge $\dot Q_{tc}$, die durch das Thermoelement fließt, ist proportional zur Temperaturdifferenz:$$\dot Q_{​tc}​=G_{tc}​ΔT_{tc}$$wobei $G_{tc}$ die Wärmeleitfähigkeit des Thermoelements ist.
- Setzen wir $\Delta T_{tc}$ in die Gleichung für $U_{tc}$ ein, erhalten wir:$$U_{tc}​=\frac{α\dot Q_{​tc}}{G_{tc}}​$$
- Für eine Reihenschaltung von $n$ Thermoelementen multiplizieren wir beide Seiten der Gleichung mit $n$:$$U_{tc}=\frac{n\alpha}{G_{tc}}\dot Q_{tc} \tag{3.1.17}$$
#### Leite Gl. 3.1.18 her
- Diese Gleichung basiert auf dem Stefan-Boltzmann-Gesetz, das besagt, dass die von einem schwarzen Körper emittierte Strahlungsleistung ( P ) proportional zur vierten Potenz seiner absoluten Temperatur ( T ) ist:$$P=\sigma\cdot\epsilon\cdot A \cdot T^{4} \tag{3.1.10}$$
- Für ein Bolometer, das Strahlung misst, wird die Gleichung angepasst, um die Differenz zwischen der Temperatur des Objekts ( T_{Ob} ) und der Temperatur des Detektors ( T_{Det} ) zu berücksichtigen: $$\dot Q_{rad} = \epsilon_{A} A_{A} \sigma (T_{Ob}^{4} - T_{Det}^{4})$$
- Da ( \dot Q_{rad} = \dot Q_{tc} ), können wir die Gleichung für die Thermopile-Spannung ( U(T_{Ob}) ) aufstellen:$$U(T_{Ob}) = \frac{n\alpha}{G_{tc}} \dot Q_{rad}$$
- Setzen wir ( \dot Q_{rad} ) ein, erhalten wir:$$U(T_{Ob}) = \frac{n\alpha}{G_{tc}} \epsilon_{A} A_{A} \sigma (T_{Ob}^{4} - T_{Det}^{4}) \tag{3.1.18}$$
#### Was passiert, wenn das Messobjekt kälter als der Detektor ist?
- Wenn das Messobjekt kälter als der Detektor ist, kehrt sich die Situation im Vergleich zu dem Fall um, bei dem das Messobjekt wärmer als der Detektor ist. Die Gleichungen, die wir zuvor diskutiert haben, bleiben gültig, aber die Temperaturdifferenz ( $T_{Ob} - T_{Det}$ ) wird negativ, da ( $T_{Ob}$ ) kleiner als ( $T_{Det}$ ) ist.
- In der Gleichung$$U(T_{Ob}) = \frac{\epsilon_{A}A_{A}\sigma n\alpha}{G_{tc}}(T_{Ob}^{4}-T_{Det}^{4})$$wird der Ausdruck $T_{Ob}^{4}-T_{Det}^{4}$ negativ, was bedeutet, dass auch die Spannung $U(T_{Ob})$ negativ wird. Dies zeigt an, dass die Richtung des Wärmeflusses umgekehrt ist – Wärme fließt vom Detektor zum Messobjekt, anstatt vom Messobjekt zum Detektor.
In der Praxis bedeutet dies, dass das Bolometer oder der Thermopile-Sensor eine negative Spannung erzeugt, wenn das Messobjekt kälter als der Detektor ist. Diese Information kann dann genutzt werden, um die Temperatur des Messobjekts zu bestimmen, selbst wenn es kälter als der Detektor ist.
## Thermoresistive (NTC) Sensoren für Wärmekameras (Wärmestrahlung Methode 2)
**Grundlagen:**
- Thermoresistive Sensoren, auch als NTC-Sensoren (Negative Temperature Coefficient) bezeichnet, sind elektrische Widerstände, deren Widerstand mit der Temperaturänderung variiert.
- Die Temperaturabhängigkeit des Widerstands wird durch den Temperaturkoeffizienten ( \alpha ) beschrieben.
- In erster Näherung folgt der Widerstand ( R(T) ) einem linearen Zusammenhang mit der Temperaturdifferenz ( T - T_0 ):$$R(T)=R_{0}(1+\alpha(T-T_{0})) \tag{3.1.19}$$
	 - ( R(T) ): Widerstand bei der aktuellen Temperatur ( T )
	 - ( R_0 ): Widerstand bei der Referenztemperatur ( T_0 )
	 - ( \alpha ): Temperaturkoeffizient
	 - ( T ): Aktuelle Temperatur
	 - ( T_0 ): Referenztemperatur
 **Halbleiter und NTC-Eigenschaften:**
-  Bei Halbleitern wie Silizium ist der Temperaturkoeffizient ( \alpha ) negativ. Das bedeutet, dass der Widerstand mit steigender Temperatur abnimmt.
- NTC-Sensoren sind stark nichtlinear, da der Widerstand exponentiell mit der Temperatur variiert.
- Diese Nichtlinearität macht NTC-Sensoren besonders empfindlich für kleine Temperaturänderungen.
**Anwendung in Wärmekameras (Methode 2):**
- Nehmen wir an, der Widerstand eines NTC-Sensors ist gleichzeitig ein thermisch isolierter Strahlungsabsorber.
- Wenn der Sensor auf ein Objekt gerichtet ist, das Wärmestrahlung emittiert (z. B. ein Körper bei einer bestimmten Temperatur), wird der Sensor erwärmt.
- Der Spannungsabfall ( U(T) ) über dem Widerstand kann durch den Strom ( I ) und den Widerstand ( R_0 ) ausgedrückt werden:$$U(T)=I \cdot R_{0}(1+\alpha(T_{obj}-T_{0}) \tag{3.1.20})$$
### Übungsaufgaben
#### Warum eignet sich Methode 2 insbesondere für Wärmekameras?
1. **Empfindlichkeit gegenüber Temperaturänderungen:**
    - NTC-Sensoren haben einen stark nichtlinearen Widerstandsverlauf in Abhängigkeit von der Temperatur.
    - Kleine Temperaturänderungen führen zu signifikanten Widerstandsänderungen, was die Empfindlichkeit der Messung erhöht.
    - In Wärmekameras ist es wichtig, auch geringe Temperaturunterschiede genau zu erfassen, um Wärmequellen zu identifizieren.
2. **Thermisch isolierter Strahlungsabsorber:**
    - Der NTC-Sensor dient gleichzeitig als Strahlungsabsorber.
    - Wenn der Sensor auf ein Objekt gerichtet ist, das Wärmestrahlung emittiert, wird der Sensor erwärmt.
    - Die Änderung des Widerstands des NTC-Sensors spiegelt die aufgenommene Wärmestrahlung wider.
3. **Kalibrierung und Kompensation:**
    - Die Nichtlinearität der NTC-Sensoren erfordert eine sorgfältige Kalibrierung und Kompensation.
    - Moderne Wärmekameras verwenden ausgeklügelte Algorithmen, um die Nichtlinearität zu berücksichtigen und genaue Temperaturmessungen zu ermöglichen.
#### Warum wird bei Spot-Messungen (Bolometer, Fieberthermometer) die Methode 1 eingesetzt?
1. **Einfache Handhabung:**
    - Methode 1 erfordert nur einen einzigen Sensor und eine Referenztemperatur (Reservoir).
    - Dies macht die Implementierung in tragbaren Geräten wie Fieberthermometern praktisch und einfach.
2. **Geringer Temperaturbereich:**
    - Spot-Messungen zielen auf einen begrenzten Temperaturbereich ab (z. B. Körpertemperatur).
    - Die differenzielle Methode ist ausreichend genau für solche begrenzten Bereiche.
3. **Kompensation der Umgebungstemperatur:**
    - Bei Fieberthermometern wird die Körpertemperatur relativ zur Umgebungstemperatur gemessen.
    - Die differenzielle Methode kompensiert automatisch die Umgebungstemperatur, da sie die Temperaturdifferenz misst.
#### Beschreiben Sie den Verlauf von $U_{out}(t)$
![[Pasted image 20240429110320.png#invert|300]]
1. **K offen (Schalter geöffnet):**
    - Wenn der Schalter K offen ist, wird der Kondensator C über den Widerstand ( R(T) ) aufgeladen.
    - Die Ausgangsspannung ( U_{out}(t) ) steigt exponentiell an, bis sie einen stabilen Wert erreicht, der durch die Eingangsspannungen ( U_1 ) und ( U_2 ), den Widerstand ( R(T) ) und die Kapazität C bestimmt wird.
    - Die Zeitkonstante für den Ladevorgang ist ( R(T) \cdot C ).
2. **K geschlossen (Schalter geschlossen):**
    - Wenn der Schalter K geschlossen ist, wird der Kondensator C kurzgeschlossen und entlädt sich sofort.
    - Die Ausgangsspannung ( U_{out}(t) ) fällt auf Null ab, da der Kondensator keine Ladung speichern kann.
    - Die Entladung erfolgt sehr schnell, nahezu augenblicklich, im Vergleich zum Ladevorgang.
Zusammenfassend variiert ( U_{out}(t) ) je nach Zustand des Schalters K. Bei geöffnetem Schalter lädt sich der Kondensator auf und ( U_{out}(t) ) steigt, während bei geschlossenem Schalter der Kondensator sich entlädt und ( U_{out}(t) ) auf Null fällt. Die genaue Form des Verlaufs hängt von den spezifischen Werten der Schaltungskomponenten ab.
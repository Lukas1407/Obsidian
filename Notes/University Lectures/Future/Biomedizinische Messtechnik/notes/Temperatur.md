- Die Temperatur ist eine Zustandsgröße von einem System
	- Wie z.B. das Volumen 
- Sie ist definiert über das thermodynamische Gleichgewicht: 
	1. **Nullter Hauptsatz der Thermodynamik**:
	    - Dieser besagt, dass zwei Körper (oder Systeme) dieselbe Temperatur haben, wenn zwischen ihnen in direktem Kontakt **kein Wärmeaustausch** stattfindet. Das bedeutet, dass sie im **thermischen Gleichgewicht** sind.
- **Wärmeübertragung**:
	    - Wenn zwei Körper unterschiedliche Temperaturen haben, fließt zwischen ihnen in direktem Kontakt so lange Wärme, bis das **Temperaturgleichgewicht** hergestellt ist.
	    - Die Mechanismen der Wärmeübertragung sind: **[[Wärmeleitung]]**, **[[Konvektion]]** und **[[Wärmestrahlung]]**.
	    - Die resultierende Gleichgewichtstemperatur ist abhängig von den Wärmekapazitäten C[J/K]

## Gleichungen
![[Pasted image 20240429075508.png#invert|200]]
- Mit Q: Wärme
### Wärmestrom
$$\Phi=\dot Q=\frac{dQ}{dt} \tag{3.1.1}$$
- Der Wärmestrom $\Phi$ beschreibt die Menge an Wärme, die pro Zeiteinheit von einem Körper zum anderen fließt.
- $Q$ die übertragene Wärme ist und $\frac{dt}{dQ}​$ die Änderung der Wärme pro Zeiteinheit darstellt.
- Wenn die Temperaturen der beiden Körper unterschiedlich sind ($T_{1​}\ne T_{2}$​), ist der Wärmestrom nicht null.
### Wärmekapazität
$$C=\frac{dQ}{dT}=c\cdot m \tag{3.1.2}$$
- Die Wärmekapazität $C$ eines Körpers gibt an, wie viel Wärmeenergie benötigt wird, um seine Temperatur um eine bestimmte Einheit zu erhöhen.
- $c$: Spezifische Wärmekapazität
- $m$: Masse
### Gleichgewichtstemperatur
$$T_{eq}=\frac{C_{1}T_{1}+C_{2}T_{2}}{C_{1}+C_{2}} \tag{3.1.3}$$
- Wenn zwei Körper Wärme austauschen, erreichen sie nach einer gewissen Zeit eine gemeinsame Temperatur, die als Gleichgewichtstemperatur $T_{eq}$​ bezeichnet wird.
- Die Gleichgewichtstemperatur ergibt sich aus dem gewichteten Mittelwert der Temperaturen der beiden Körper, basierend auf ihren Wärmekapazitäten
### Beispiel
- Welche Gleichgewichtstemperatur stellt sich ein für:
- $m_1 = 1g$
- $c_1 = 0,45 kJ/kgK$ (Metall-Sensor) 
- $m_2 = 70kg$
- $c_2 = 4,18 KJ/kgK$ (Wasserkörper)?

## Temperaturmessung über Wärmeleitung (Kontaktthermometer)
![[Pasted image 20240429080207.png#invert|300]]
- Mit einem Wärmeübergangswiederstand $R$, da der Kontakt i.d.R. nicht perfekt ist
### Messverfahren: Kontaktthermometer
- Der Körper 2 ist das Thermometer 
- Der Körper 1 ist das Messobjekt
- Körper 1 wärmt das Thermometer bis zur Gleichgewichtstemperatur auf!
- -> Körper 1 wird kälter! Nicht signifikant, da die Wärmekapazität von Körper 1 wesentlich größer ist
	- Für $C_{1}>>C_{2}$ folgt $dT_{1}<<dT_{2}$
### Wärmestrom
$$\Phi =\frac{T_{1}-T_{2}}{R} \tag{3.1.3}$$
- Für den Wärmestrom aus 1 und nach 2 ergibt sich nach der Gl. 3.1.1 und 3.1.2: $$\Phi=C_{2} \frac{dT_{2}}{dt}=C_{1}- \frac{dT_{1}}{dt} \tag{3.1.4}$$
- mit Gl. 3.1.3 und 3.1.4 folgt die Differentialgleichung:$$C_{2}R \frac{dT_{2}}{dt}=T_{1}-T_{2} \tag{3.1.5}$$
- mit der Lösung:$$T_{2}(t)=T_{1}-(T_{1}-T_{2}(t_{0}))e^{\frac{1}{RC_{2}}t} \tag{3.1.6}$$
- Zeitkonstante$$\tau =RC_{2} \tag{3.1.7}$$
### Temperaturmessung
- Aus der Umformung von Gl. 3.1.5 folgt $$T_{2}=T_{1}-\tau \frac{dT_{2}}{dt} \tag{3.1.8}$$ ergeben sich 2 [[Messung im Sinne der Messtechnik#Messmethode|Messmethoden]]:
#### Thermisches Gleichgewicht
- Nach ausreichend langen warten ($t>>\tau)$ ergibt sich: $$T_{2}\rightarrow T_1$$
- Vorteile:
	- Genauigkeit: Die Messung ist sehr genau, da das Thermometer die tatsächliche Temperatur des Messobjekts annimmt.
	- Einfachheit: Es sind keine komplexen Berechnungen oder Kalibrierungen erforderlich; man wartet einfach, bis sich die Temperatur nicht mehr ändert.
- Nachteile:
	- Zeitaufwand: Es kann lange dauern, bis das thermische Gleichgewicht erreicht ist, besonders bei Thermometern mit großer Wärmekapazität oder bei schlechter Wärmeleitung zwischen den Körpern.
	- Mögliche Beeinflussung des Messobjekts: Wenn das Messobjekt klein ist oder eine geringe Wärmekapazität hat, könnte es durch das Thermometer abgekühlt werden.
#### Extrapolation
- Mit Kenntnis der Zeitkonstanten $\tau$ (Kalibrierung) und Messung der Thermometertemperatur $T_{2}$ und der Temperaturänderungsgeschwindigkeit zum Zeitpunkt t = 0 $\frac{T_{2}}{dt_{0}}$ kann auf $T_1$ extrapoliert werden.
- Vorteile:
	- Schnelligkeit: Die Temperatur des Messobjekts kann schneller bestimmt werden, ohne auf das thermische Gleichgewicht warten zu müssen.
	- Minimale Beeinflussung: Das Messobjekt wird weniger beeinflusst, da die Messung schneller erfolgt.
- Nachteile
	- Komplexität: Die Methode erfordert eine genaue Kenntnis der Zeitkonstante und eine Kalibrierung des Thermometers.
	- Potenzielle Fehlerquellen: Fehler bei der Messung der Temperaturänderungsrate oder bei der Kalibrierung können zu ungenauen Ergebnissen führen.
## Temperaturmessung über Wärmestrahlung
- Jeder Körper, dessen Temperatur über dem absoluten Nullpunkt liegt, gibt thermische Strahlung ab. Diese Strahlung ist eine Form von elektromagnetischer Energie.
- Ein idealer Strahler, auch schwarzer Körper genannt, ist im Gleichgewicht mit seinem Strahlungsfeld, wenn er die meiste Strahlung, die er aussendet, auch wieder absorbiert. Dies ermöglicht eine genaue Messung der abgegebenen Strahlung.
- Thermische Strahlung ist nicht über die Wellenlänge, sondern über die spektrale Verteilung definiert. Die Spektrale Verteilung wird durch das plancksche Strahlungsgesetz beschrieben Dieses Gesetz zeigt, dass die Strahlung ein Maximum bei einer bestimmten Wellenlänge hat, die von der Temperatur des Körpers abhängt:$$\lambda_{max}= \frac{2897.8[\mu mK]}{T[K]} \tag{3.1.9}$$
- Die Lage des Maximums ist abhängig von der Temperatur des Strahlers (Wiensches Verschiebungsgesetz)
- Die Leistung $P$ der abgestrahlten Energie ist stark von der Temperatur abhängig und wird durch das Stefan-Boltzmann-Gesetz beschrieben:$$P=\sigma\cdot\epsilon\cdot A \cdot T^{4} \tag{3.1.10}$$ mit $\sigma=5.67\cdot 10^{-8} \frac{W}{m^{2}K^{4}}$ der Stephan-Bolzmann -Konstante und $\epsilon$ dem Emissionsgrad
- Gleichzeitig nimmt jeder Körper Strahlungsleistung aus seiner Umgebung auf. Stehen zwei Körper unterschiedlicher Temperatur im Strahlungsaustausch miteinander, dann ergibt sich ein resultierender Wärmestrom $$\dot Q_{rad}=\sigma\frac{T_{1}^{4}-T_{2}^{4}}{ \frac{1-\epsilon_{1}}{A_{1}\epsilon_{1}} +\frac{1}{A_{1}F_{1\rightarrow 2}} -\frac{1-\epsilon_{2}}{A_{2}\epsilon_{2}}} \tag{3.1.11}$$
	- Der Sichtfaktor $F_{1\rightarrow 2}$ gibt an, wie viel von$A_1$ diffus abgestrahlten Leistung auf $A_2$ trifft
### Temperaturmessung
Die Messung der Oberflächentemperatur aus größerer Entfernung mittels Infrarot-Thermometrie kann durch verschiedene Faktoren beeinflusst werden, die das Messsignal verfälschen:
- **Reflexion der Umgebungsstrahlung**: Die Oberfläche des Körpers reflektiert einen Teil der Strahlung aus der Umgebung, was das Messsignal erhöhen kann.
- **Absorption der Umgebungsstrahlung**: Die Oberfläche absorbiert ebenfalls Strahlung aus der Umgebung, was zu einer Erhöhung der gemessenen Temperatur führen kann.
- **Erfassung der Umgebungsstrahlung vom Thermometer**: Das Thermometer kann zusätzlich zur Strahlung des Körpers auch Strahlung aus der Umgebung erfassen.
- **Atmosphärische Absorption/Emission**: Die Atmosphäre zwischen dem Thermometer und dem Körper kann einen Teil der Strahlung absorbieren oder selbst Strahlung emittieren, was das Messsignal beeinflusst.
Die effektive abgestrahlte Leistung $P_{eff}$ eines Körpers unter Berücksichtigung der Umgebungsstrahlung wird durch die Differenz der emittierten Leistung $P_{em}$ und der absorbierten Leistung $P_{abs}$ bestimmt:$$P_{eff}=P_{em}-P_{abs}=A\sigma\epsilon(T_{\text{Körperoberfläche}}^{4} −T_{\text{Umgebung}}^{4}) \tag{3.1.13}$$
#### Übungsaufgabe: Wärmestrahlungleistung eines nackten Menschen
- Hauttemperatur: ( 33°C + 273.15 = 306.15K )
- Umgebungstemperatur: ( 20°C + 273.15 = 293.15K )
- Körperoberfläche 1,7$m^2$
- Emissivität = 0,90
- Nun setzen wir die Werte in die Formel ein:$$P=5.67⋅10^{−8} \frac{W}{m^{2}K^{4}}​⋅0.90⋅1.7m^{2}⋅(306.15K^{4}−293.15K^{4})\approx 76W$$
#### Messung der Trommelfelltemperatur
![[Pasted image 20240429092729.png#invert|400]]
- kein Einfluss von Umgebungsstrahlung
- der thermisch abgeschlossene Gehörgang wirkt wie ein Holraumstrahler 
- Störeinflüsse über das Thermometer möglich, weil es wärmer oder kälter sein kann
	- Aber: Gehörgang mit Thermometer ist so gut isoliert, dass schnell ein Wärmegleichgewicht hergestellt ist 
#### Messmethoden
- Nutzen Bolometer
	- besitzen einen Absorber, der Strahlungsleistung vollständig (Emissionsgrad nahe 1) absorbiert und liefern ein elektrisches Signal S, welches die Absorbertemperatur repräsentiert.
![[Pasted image 20240429093253.png#invert|200]]
**Methode 1: Differenziell** 
- Bei der differenziellen Methode wird das elektrische Ausgangssignal $S$ durch die Temperaturdifferenz zwischen dem Absorber und einem Referenzreservoir bestimmt. $$S=f(T_{A}-T_{R}) \tag{3.1.14}$$
- Der Absorber nimmt die Strahlungsleistung auf und erwärmt sich. 
- Diese Temperaturerhöhung wird im Vergleich zur Temperatur des Reservoirs gemessen.
- Ein definierter Wärmeleiter verbindet den Absorber mit dem thermostatischen Reservoir, das eine konstante Temperatur hat. 
- Die Temperaturdifferenz erzeugt ein elektrisches Signal, das proportional zur aufgenommenen Strahlungsleistung ist.
**Methode 2: Absolut** 
- Die absolute Methode misst die Temperatur des Absorbers direkt. 
- Hierbei ist der Absorber, abgesehen von seiner Messfläche, thermisch von seiner Umgebung isoliert. 
- Das bedeutet, dass keine Wärme zwischen dem Absorber und seiner Umgebung ausgetauscht wird, außer über die Messfläche. 
- Das elektrische Signal $S$ repräsentiert dann die absolute Temperatur des Absorbers, die durch die absorbierte Strahlungsleistung erhöht wird$$S=f(T_{A}) \tag{3.1.15}$$
In beiden Fällen wird die Temperaturänderung des Absorbers, die durch die Absorption von Strahlungsenergie entsteht, in ein elektrisches Signal umgewandelt, das dann gemessen werden kann. Bolometer sind sehr empfindlich und können für die Messung von Strahlung über ein breites Spektrum von
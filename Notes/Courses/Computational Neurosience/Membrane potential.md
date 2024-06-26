> [!summary] Definition
> Das **Membranpotential** (auch als **Transmembranspannung** bezeichnet) ist eine spezielle elektrische Spannung, die sich zwischen zwei Flüssigkeitsräumen bildet, in denen geladene Teilchen (Ionen) unterschiedlicher Konzentration vorliegen. Dieses Potential entsteht, wenn die Flüssigkeitsräume durch eine Membran getrennt sind, die mindestens eine dieser Ionenarten durchlässt, aber nicht alle Ionen gleich gut passieren kann (Semipermeabilität).
- Das Zellinnere ist gegenüber dem extrazellulären Medium immer negativ geladen.
- Neuronen und Muskelzellen können ihr TP zeitlich und örtlich verändern. 
- Übersteigt diese Veränderung einen Schwellwert, können [[Action Potential|Aktionspotentiale]] ausgebildet und weitergeleitet werden. 
- Dadurch können Signale generiert und weitergeleitet werden

## Entstehung des Membranpotentials
- Die Membran trennt zwei Flüssigkeitsräume mit unterschiedlichen Ionenkonzentrationen voneinander ab.
- Die Membran ist semipermeabel, was bedeutet, dass sie für bestimmte Ionenarten durchlässig ist.
- Die Ionen, die eine Rolle spielen, sind im Zellinneren große, negativ geladene Anionen (A-) und positiv geladene Kaliumionen (K+), während außerhalb der Zelle vor allem positive Natriumionen (Na+) und negative Chloridionen (Cl-) vorhanden sind.
- Aufgrund der Ionenkanäle ist die Membran am durchlässigsten für Kaliumionen, weniger durchlässig für Chloridionen und am wenigsten durchlässig für Natriumionen.
- Die Ionen bewegen sich aufgrund ihres Konzentrationsgefälles über die Membran (Diffusion), was zu einer Ladungstrennung führt.
- Das Äußere der Zelle wird durch die K±Ionen positiver geladen, während das Innere negativer wird.

## Nernst-Gleichung
- Die Nernst-Gleichung beschreibt das Gleichgewichtspotential für eine bestimmte Ionenart.
- Für Kaliumionen (K+) lautet die Nernst-Gleichung:$$ E_K = \frac{RT}{z_{e}F} \ln \frac{c_{\text{aussen}}}{c_{\text{innen}}}=\frac{61.51mV}{z_{e}}\lg\frac{c_{\text{aussen}}}{c_{\text{innen}}} $$wobei:
    - ($E_K$) das Gleichgewichtspotential für Kalium ist,
    - ($R$) die Gaskonstante,
    - ($T$) die Temperatur in Kelvin,
    - ($z$) die Ionenladung (für Kalium +1),
    - ($F$) die Faraday-Konstante,
    - ($c_{\text{außen}}$) die Kaliumkonzentration außerhalb der Zelle und
    - ($c_{\text{innen}}$) die Kaliumkonzentration im Zellinneren ist.
### Vernachlässigter Diffusionsprozess
- Bei dieser Vereinfachung wird der aktive Transport vernachlässigt.
- Aktiver Transport bezieht sich auf den energieaufwändigen Transport von Ionen gegen ihren Konzentrationsgradienten mithilfe von speziellen Ionenpumpen (z. B. die Natrium-Kalium-Pumpe).
- In der gegebenen Übungsaufgabe konzentrieren wir uns jedoch auf den passiven Transport (Diffusion), der durch die Nernst-Gleichung beschrieben wird.
## Verlauf der elektrischen Größen über die Membran
![[Pasted image 20240430101846.png#invert|250]]
- Die getrennten Ladungen (Ionen) lagern sich an der Membranoberfläche an 
- In vereinfachter Näherung ergibt sich das Bild eines Kondensators 
	- Über die Membran entsteht ein konstantes Feld
	-  Über die Membran entsteht ein linearer Potentialverlauf 
- Die Raumladungsdichte fällt exponentiell mit dem Abstand zur Membran ab (Debye-Länge ca. 1nm)
-  Außerhalb der Raumladungszone ist das el. Potenzial konstant 
	- die Membranspannung kann in beliebigem Abstand gemessen werden
### Übungsaufgabe
#### Berechnung der elektrischen Feldstärke an einer Lipiddoppelmembran
- Wir haben eine **Lipiddoppelmembran** mit einer Dicke (d) des lipophilen Kerns von **5 nm**.
- Die elektrische Feldstärke (E) entsteht über die Membran.
- Wir verwenden die Formel für die elektrische Feldstärke (E) zwischen zwei Platten: $$E = \frac{{U}}{{d}}$$ wobei:
    - (E) die elektrische Feldstärke ist,
    - (U) die Spannung (Potentialdifferenz) zwischen den Platten ist und
    - (d) die Dicke der Membran ist.
- Nehmen wir an, dass die Spannung (U) über die Membran **konstant** ist (was in diesem vereinfachten Modell der Fall ist).
- Setzen wir die bekannten Werte ein: $$E = \frac{{U}}{{d}} = \frac{{230 , \text{V}}}{{5 \times 10^{-9} , \text{m}}} = 4.6 \times 10^{10} , \text{V/m}$$
#### Vergleich mit der Durchschlagsfestigkeit von Luft
- Die **Durchschlagsfestigkeit** ist die maximale elektrische Feldstärke, die ein Isolator (wie Luft) ohne Durchschlag aushalten kann.
- Die Durchschlagsfestigkeit von **trockener Luft** liegt bei etwa ($3 \times 10^6 , \text{V/m}$) bis ($3 \times 10^7 , \text{V/m}$).
- Vergleichen wir die berechnete Feldstärke mit der Durchschlagsfestigkeit von Luft:
    - ($4.6 \times 10^{10} , \text{V/m}$) (Membran) vs. ($3 \times 10^6 , \text{V/m}$) bis ($3 \times 10^7 , \text{V/m}$) (Luft)
    - Die Feldstärke an der Lipiddoppelmembran ist **deutlich höher** als die Durchschlagsfestigkeit von Luft.
- **Fazit**:
    - Die Lipiddoppelmembran kann ein viel stärkeres elektrisches Feld aushalten als Luft, da sie eine geringere Durchschlagsfestigkeit hat.


## Membranpotenial unter Beteiligung mehrerer Ionensorten
- Die Membran weist für die unterschiedlichen Ionen unterschiedliche Permeabilitäten auf. 
- Zusätzlich werden Konzentrationsgradienten durch Ionenpumpen aktiv aufrechterhalten.
-  Es stellt sich kein Gleichgewichtszustand ein 
	- Die [[Nernst Potential|Nernst-Gleichung]] ist nicht anwendbar! 
- Es stellt sich ein stationärer Zustand ein
	- Zeitlich konstantes Membranpotential E, wenn $I_{in} = I_{out}$ 
	- Das Membranpotential im stationären Zustand wird durch die [[Goldman-Gleichung]] beschrieben
### Näherungen
1. **Ionenströme verändern die Ionenkonzentrationen nicht merklich**:
    - Diese Annahme besagt, dass die Bewegung der Ionen über die Membran die Gesamtkonzentrationen der Ionen auf beiden Seiten der Membran nicht signifikant ändert.
    - Das bedeutet, dass trotz der Diffusion und aktiven Transportprozesse die Konzentrationen relativ stabil bleiben, was eine kontinuierliche Berechnung des Membranpotentials ermöglicht.
2. **Unabhängigkeit der Ionen untereinander**:
    - Hier wird angenommen, dass die Bewegung eines Iontyps über die Membran nicht von der Bewegung anderer Ionenarten beeinflusst wird.
    - In der Realität gibt es Wechselwirkungen zwischen verschiedenen Ionen, aber diese Näherung erlaubt es, die Gleichung zu vereinfachen, indem man die Ionenströme als unabhängig betrachtet.
3. **Konstante Feldstärke über die Membran**:
    - Diese Annahme impliziert, dass das elektrische Feld über die Dicke der Membran hinweg gleich bleibt.
    - Es wird davon ausgegangen, dass keine großen lokalen Unterschiede in der Ladungsdichte existieren, die das Feld beeinflussen könnten.
4. **Homogener Transport über die Membran**:
    - Es wird angenommen, dass die Membran überall gleich durchlässig für Ionen ist und dass es keine bevorzugten Pfade für den Ionenfluss gibt.
    - Das bedeutet, dass die Ionen gleichmäßig über die gesamte Membran verteilt transportiert werden.
### Übungsaufgabe: Erkläre, wie es zu der ungleichen Verteilung von K+ und Na+ im Intra- und Extrazellularraum kommt.
1. **Aktive Ionenpumpen**:
    - **Natrium-Kalium-Pumpe (Na⁺/K⁺-Pumpe)**: Diese Pumpen sind in der Zellmembran eingebettet und transportieren aktiv Natriumionen aus der Zelle heraus und Kaliumionen in die Zelle hinein.
    - Die Na⁺/K⁺-Pumpe trägt dazu bei, die Konzentrationen von Na⁺ und K⁺ auf beiden Seiten der Membran aufrechtzuerhalten.
    - Sie verbraucht Energie (ATP), um gegen die Konzentrationsgradienten zu arbeiten.
2. **Selektive Permeabilität der Membran**:
    - Die Zellmembran ist selektiv permeabel, was bedeutet, dass sie für verschiedene Ionen unterschiedlich durchlässig ist.
    - Kaliumionen (K⁺) können leichter durch die Membran diffundieren als Natriumionen (Na⁺).
    - Dies führt dazu, dass mehr K⁺-Ionen in der Zelle verbleiben und weniger Na⁺-Ionen eindringen.
3. **Diffusion und elektrische Ladung**:
    - Aufgrund der unterschiedlichen Permeabilität und der Aktivität der Na⁺/K⁺-Pumpe entsteht ein Konzentrationsgradient für K⁺ und Na⁺.
    - K⁺ diffundiert aus der Zelle heraus, da es in der Zelle eine höhere Konzentration hat.
    - Na⁺ diffundiert in die Zelle hinein, da es außerhalb der Zelle reichlich vorhanden ist.
    - Diese Bewegung der Ionen erzeugt ein elektrisches Potential über die Membran.
4. **Ruhepotential und Gleichgewichtspotential**:
    - Im Ruhezustand (ohne Reize) erreicht das Membranpotential einen stabilen Wert, das sogenannte **Ruhepotential**.
    - Das Ruhepotential wird durch die Kombination von Konzentrationsgradienten und elektrischem Gradienten bestimmt.
    - Das **Gleichgewichtspotential** für K⁺ und Na⁺ ist das Membranpotential, bei dem der Netto-Ionenfluss null ist (d.h., es gibt keinen Unterschied zwischen Diffusion und aktiver Transport).

## Messtechnik
### Intrazelluläre Ableitung mit einer Mikroelektrode
- Verdünnung einer Glaskapillare: Eine Glaskapillare wird bis auf einen sub-mikrometer Durchmesser ausgezogen, um eine Mikroelektrode zu bilden.
- Permeation der Zellmembran: Die Spitze der Mikroelektrode wird durch die Zellmembran gestochen, um direkten Zugang zum Zellinneren zu erhalten.
- Elektrolytischer Kontakt über KCl-Lösung: Die Mikroelektrode wird mit einer Kaliumchlorid-Lösung gefüllt, die den elektrischen Kontakt zwischen dem Inneren der Zelle und dem Messgerät herstellt.
- Ag/AgCl-Elektroden: Silber/Silberchlorid-Elektroden werden verwendet, da sie nicht-polarisierend sind und ein stabiles Elektrodenpotential bieten.
- Signalverstärkung: Ein Instrumentenverstärker verstärkt das Signal, typischerweise um das 100-fache, und ermöglicht eine stromlose Messung durch sehr hohe Eingangsimpedanz.
### Patch-Clamp-Technik
- Kontaktierung einzelner Ionenkanäle: Eine Mikropipette wird verwendet, um einzelne Ionenkanäle zu kontaktieren.
- Isolation durch Ansaugtechnik: Durch leichten Unterdruck wird ein kleiner Bereich der Zellmembran an die Spitze der Mikropipette angesaugt, wodurch ein dichter Verschluss entsteht.
- Mögliche Messungen:
	- Membranpotentiale (Current-Clamp): Messung des Membranpotentials bei variierendem Strom.
	- Transmembranströme (Voltage-Clamp): Messung der Ionenströme durch die Membran bei konstant gehaltener Spannung.
- Elektrische Stimulation: Es ist möglich, elektrische Reize zu setzen, um die Aktivität der Ionenkanäle zu beeinflussen.
- Aktivierung spannungsgesteuerter Kanäle: Die Technik ermöglicht es, die Reaktion von spannungsgesteuerten Ionenkanälen auf Veränderungen des Membranpotentials zu untersuchen567.

## Modulation durch gesteuerte Ionenkanäle
Gesteuerte Ionenkanäle spielen eine zentrale Rolle bei der Modulation des Membranpotentials. Sie beeinflussen die Permeabilität der Zellmembran für bestimmte Ionen und können somit das Membranpotential verändern. Hier sind die verschiedenen Arten von gesteuerten Ionenkanälen und ihre Auswirkungen auf das Membranpotential:
1. **Spannungsgesteuerte Ionenkanäle**:
    - Diese Kanäle reagieren auf Veränderungen des Membranpotentials.
    - Bei einer bestimmten Spannung ändern sie ihre Konformation und werden durchlässig für Ionen.
    - Sie sind entscheidend für die Generierung und Weiterleitung von Aktionspotentialen in Neuronen und Muskelzellen
2. **Ligandensensitive Ionenkanäle**:
    - Öffnen oder schließen sich in Reaktion auf die Bindung eines spezifischen chemischen Signals (Liganden) wie [[Neurotransmitter]] oder Hormone.
    - Diese Kanäle sind wichtig für die Signalübertragung an Synapsen und für die Reaktion auf zelluläre Signale
3. **Mechanosensitive Ionenkanäle**:
    - Werden durch mechanische Reize wie Druck oder Zug aktiviert.
    - Sie sind wichtig für die Wahrnehmung von Berührung und Druck.
4. **Ionensensitive Ionenkanäle**:
    - Reagieren auf die Konzentration bestimmter Ionen in der Zellumgebung.
    - Beispielsweise können Kalzium-sensitive Kanäle das Membranpotential in Reaktion auf Veränderungen der Kalziumkonzentration modulieren.
**Auswirkungen auf das Ruhepotential**:
- **Hyperpolarisierend**: Wenn die Permeabilität für K⁺ oder Cl⁻ erhöht wird, kann das Ruhepotential weiter negativiert werden, was zu einer Hyperpolarisation führt.
- **Depolarisierend**: Eine erhöhte Permeabilität für Na⁺ kann das Ruhepotential weniger negativ machen, was zu einer Depolarisation führt.
**Elektrotonische Potentiale**:
- Unterschwellige Potentiale, die nicht stark genug sind, um ein Aktionspotential auszulösen, werden als elektrotonische Potentiale bezeichnet.
- Sie können sich lokal ausbreiten und die Erregbarkeit der Zelle modulieren.
**Rezeptorpotentiale und postsynaptische Potentiale**:
- In Sinneszellen führen reizkorrelierte Veränderungen der Ionenpermeabilität zu Rezeptorpotentialen, die sensorische Informationen kodieren.
- An Synapsen entstehen durch die Freisetzung von Neurotransmittern postsynaptische Potentiale, die die Signalübertragung zwischen Neuronen ermöglichen

For the exterior of the cell, typical values of membrane potential, normally given in units of mili volts and denoted as mV, range from –80 mV to –40 mV.



This produces concentration gradients, which allows ions to move through [[Ion Channels]] down this concentration gradient.

## Levels of membrane potential
Each excitable patch of membrane has two important levels of membrane potential: the [[Resting Potential]] and a higher value called the [[Threshold Potential]].
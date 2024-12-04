> [!abstract] Definition
>  Diffusion ist ein grundlegender physikalischer Prozess, der die Bewegung von Teilchen von einem Bereich höherer Konzentration zu einem Bereich niedrigerer Konzentration beschreibt, bis ein Gleichgewicht erreicht ist.

## Erklärung
### Was ist Diffusion?
- Diffusion ist die selbstständige Durchmischung von Teilchen verschiedener Stoffe, meist Gase oder Flüssigkeiten. Die Teilchen bewegen sich aufgrund ihrer Eigenbewegung (Brownsche Molekularbewegung) und verteilen sich gleichmäßig im verfügbaren Raum.
### Wie funktioniert Diffusion?
- Konzentrationsgradient: Die treibende Kraft hinter der Diffusion ist der Konzentrationsunterschied zwischen zwei Bereichen. Teilchen bewegen sich spontan vom Ort höherer Konzentration zum Ort niedrigerer Konzentration.
- Erreichung des Gleichgewichts: Der Prozess setzt sich fort, bis die Konzentration der Teilchen überall gleich ist, was als Gleichgewichtszustand bezeichnet wird. Im Gleichgewicht bewegen sich die Teilchen weiterhin, aber es gibt keine Nettoänderung in der Konzentration.
## Kollektive Diffusion
- beschreibt die Bewegung mehrerer Teilchen entlang eines Konzentrationsgradienten
- **Ursache**: Die **thermische Zufallsbewegung** der Atome und Moleküle (auch bekannt als **Brown’sche Molekularbewegung**) ist die treibende Kraft hinter der Diffusion. Diese Bewegung führt dazu, dass sich Teilchen statistisch gesehen eher aus einem Bereich hoher Teilchendichte in einen Bereich niedriger Teilchendichte bewegen.
- **Nettotransport**: Wenn ein Konzentrationsgradient vorhanden ist, bewegen sich mehr Teilchen in Richtung des niedrigeren Konzentrationsbereichs als umgekehrt. Dies führt zu einem **Nettotransport** der Teilchen entlang des Gradienten.
- **Gleichgewichtszustand**: Wenn die Teilchen gleichmäßig verteilt sind und keine Nettoveränderung mehr stattfindet, ist ein **Gleichgewichtszustand** erreicht. In diesem Zustand ist die Konzentration überall gleich.
- **Thermodynamik und Entropie**: Der Gleichgewichtszustand ist thermodynamisch durch eine **Erhöhung der Entropie** (dem Maß für die Unordnung) gekennzeichnet. Das zweite Gesetz der Thermodynamik besagt, dass die Entropie im Universum immer zunimmt.
## Diffusion in Flüssigkeiten
Die Diffusion in Flüssigkeiten und das 1. Fick’sche Gesetz beschreiben den Zusammenhang zwischen der Teilchenstromdichte $J$ und dem Konzentrationsgradienten. Hier ist eine detaillierte Erklärung:
- **Teilchenstromdichte $J$**: Sie gibt an, wie viele Teilchen pro Flächeneinheit und Zeit durch eine Fläche strömen. Sie ist **proportional und entgegengerichtet** zum Konzentrationsgradienten. Die Formel lautet:$$J=-D\frac{\partial c}{\partial x}$$
- **Diffusionskoeffizient $D$**: Dies ist die Proportionalitätskonstante in der obigen Gleichung und gibt an, wie schnell die Diffusion erfolgt. Ein hoher Wert von $D$ bedeutet eine schnelle Diffusion. $D$ hat die Einheit ( $m^2/s$ ) und kann experimentell bestimmt werden durch:$$D=\frac{\langle x^{2} \rangle}{2t}$$
- **Stokes-Einstein-Gleichung**: Für die Diffusion von Teilchen in Flüssigkeiten kann der Diffusionskoeffizient $D$ auch durch die Stokes-Einstein-Gleichung beschrieben werden:$$D=\frac{k_B T}{6\pi\eta R_{0}}$$
    wobei:
    - ( $k_B$ ) die Boltzmann-Konstante ist,
    - ( $T$ ) die absolute Temperatur,
    - ( $\eta$ ) die dynamische Viskosität des Lösungsmittels und
    - ( $R_0$ ) der hydrodynamische Radius der diffundierenden Teilchen ist.

Diese Gleichungen sind grundlegend für das Verständnis der Diffusion in Flüssigkeiten und finden Anwendung in vielen wissenschaftlichen und technischen Bereichen. Sie helfen dabei, die Bewegung von Teilchen auf mikroskopischer Ebene zu beschreiben und zu quantifizieren.

## Einfache Diffusion
- Moleküle passieren die Lipiddoppelschicht der Membran ohne Hilfe von Proteinen
![[Pasted image 20240416080716.png#invert|400]]
- **Teilchenstromdichte ( J )**: Sie beschreibt, wie viele Teilchen pro Zeiteinheit durch eine bestimmte Fläche der Zellmembran fließen. ( J ) ist abhängig vom Verteilungskoeffizienten $\gamma$ der Teilchen zwischen Wasser und Membran sowie vom Diffusionskoeffizienten ( D ) in der Membran.
	- **Verteilungskoeffizient $\gamma$** : Dieser gibt an, wie gut sich die Teilchen zwischen der wässrigen Phase und der Membran verteilen. Für lipidlösliche Substanzen ist dieser Wert groß, was bedeutet, dass sie leicht durch die Membran diffundieren können:$$\gamma=\frac{c_{m}}{c_{w}}>>1$$
	- **Diffusionskoeffizient ( D )**: Er misst, wie schnell die Teilchen innerhalb der Membran diffundieren.
- **Permeabilitätskoeffizient ( P )**: Dieser beschreibt die Durchlässigkeit der Membran für bestimmte Teilchen und ist definiert als:$$P=\frac{\gamma D}{d}$$
    wobei ( $\gamma$ ) der Verteilungskoeffizient, ( $D$ ) der Diffusionskoeffizient und ( $d$ ) die Dicke der Membran ist.
- **Teilchenflussdichte ( J )**: Im stationären Zustand, wenn sich ein linearer Konzentrationsgradient eingestellt hat, kann ( J ) berechnet werden als:$$
    J=−PΔc$$
    wobei ( $\Delta c$ ) der Unterschied in der Konzentration der Teilchen auf beiden Seiten der Membran ist.
### Homogene Membran
- Eine homogene Membran ist gleichmäßig in ihrer Zusammensetzung und Struktur. Unter dieser Annahme und einer schnellen Einstellung des Verteilungsgleichgewichts über die Membrangrenze hinweg, kann man davon ausgehen, dass sich im stationären Zustand ein linearer Konzentrationsgradient einstellt.
- Im stationären Zustand, wenn sich der Konzentrationsgradient nicht mehr ändert, ist ( J ) gegeben durch:$$J=-D\frac{dc_m}{dx}=-\frac{c''_m-c'_m}{d}$$Hierbei ist ( D ) der Diffusionskoeffizient, ( $c’_m$ ) und ( $c’'_m$ ) sind die Konzentrationen der Teilchen auf den beiden Seiten der Membran, und ( d ) ist die Dicke der Membran.
- Unter Berücksichtigung des Verteilungskoeffizienten ergibt sich:$$\gamma=\frac{c'_m}{c'_w} = \frac{c''_m}{c''_w}$$Im Beispiel ist ( \gamma = 2 ), was bedeutet, dass die Konzentration des Stoffes in der Membran doppelt so hoch ist wie im Wasser.
- Unter Berücksichtigung des Verteilungskoeffizienten ergibt sich für die Diffusion durch die Membran mit einem Konzentrationsunterschied ( $\Delta c = c’_w - c’'_w$ ) die Teilchenflussdichte ( J ) als:$$J=-\gamma D \frac{c''_w-c'_w}{d}=\gamma D \frac{\Delta c}{d}$$Dies zeigt, dass die Teilchenflussdichte direkt proportional zum Produkt aus dem Verteilungskoeffizienten ( \gamma ), dem Diffusionskoeffizienten ( D ) und dem Konzentrationsunterschied ( \Delta c ) ist und umgekehrt proportional zur Dicke der Membran ( d ).
![[Pasted image 20240416080751.png#invert|400]]
Das Schaubild stellt die Konzentration von Stoffen auf beiden Seiten einer Membran dar und zeigt, wie die Diffusion diese Konzentrationen ausgleicht:
- **Links von der Membran**: Die Konzentration in der Membran (C^m) ist höher als die Konzentration im Wasser (C^w), was darauf hindeutet, dass die Teilchen bevorzugt in der Membran gelöst sind.
- **Rechts von der Membran**: Die Konzentrationen (C^m) und (C^w) gleichen sich aus, was bedeutet, dass ein Gleichgewicht erreicht wurde und keine Netto-Diffusion mehr stattfindet.
Die x-Achse des Diagramms zeigt den Abstand von der Membran, und die y-Achse zeigt die Konzentration an. Die Membran trennt zwei Bereiche, und das Diagramm veranschaulicht, wie die Diffusion dazu führt, dass sich die Konzentrationen auf beiden Seiten der Membran angleichen, bis kein Konzentrationsgradient mehr besteht. Dies ist ein typisches Verhalten bei der Diffusion durch Membranen, insbesondere wenn lipidlösliche Substanzen beteiligt sind, die leicht durch die Membran diffundieren können.
### Faktoren die die Diffusionsgeschwindigkeit beeinflussen
- **Unpolarität (Fettlöslichkeit)**: Unpolare, also fettlösliche Moleküle können leichter durch die lipophile (fettliebende) Schicht der Zellmembran diffundieren. Die Membran besteht hauptsächlich aus Lipiden, und unpolare Substanzen lösen sich gut in dieser Schicht auf, was ihren Durchtritt erleichtert.
- **Größe des Moleküls**: Kleinere Moleküle können einfacher zwischen den Lipidmolekülen der Membran hindurchtreten. Große Moleküle oder solche mit komplexen Strukturen haben es schwerer, die Membran zu durchdringen.
- **Ionen**: Geladene Teilchen, wie Ionen, können nicht einfach durch die hydrophobe Lipidschicht der Zellmembran permeieren, da sie wasserlöslich sind und die Membran eine Barriere für geladene Teilchen darstellt.
- **Gase**: Viele Gase sind klein und unpolar, was ihnen erlaubt, fast ungehindert durch die Zellmembran zu diffundieren.
- **Wassermoleküle**: Obwohl Wasser polar ist, kann es aufgrund seiner kleinen Größe und der Präsenz von Aquaporinen – spezialisierten Kanälen in der Zellmembran – durch die Membran diffundieren.
- **Fettlösliche Zellgifte**: Substanzen wie Methanol, Ethanol, Benzol und Ammoniak sind fettlöslich und können daher die Zellmembran durchdringen. Dies macht sie potenziell gefährlich, da sie in Zellen eindringen und dort Schaden anrichten können.
- **Volatile Anästhetika**: Die Wirksamkeit von volatilen Anästhetika, die bei Inhalationsnarkosen verwendet werden, korreliert mit ihrer Fettlöslichkeit. Dies wird durch die Meyer-Overton-Korrelation beschrieben, die besagt, dass die Potenz eines Anästhetikums mit seiner Löslichkeit in Lipiden zusammenhängt. Je höher die Fettlöslichkeit, desto wirksamer ist das Anästhetikum, da es leichter in die Nervenzellen eindringen und dort die Signalübertragung beeinflussen kann.
## Erleichterte Diffusion
- Die erleichterte Diffusion ist ein Prozess, bei dem Moleküle oder Ionen durch spezialisierte Proteine in der Zellmembran transportiert werden, ohne dass dafür Energie in Form von [[Adenosintriphosphat (ATP)|ATP]] benötigt wird. 
- **[[Kanalproteine]]**: Diese bilden wassergefüllte Poren in der Membran, durch die bestimmte Ionen oder Moleküle passieren können. Sie sind oft hochselektiv und ermöglichen nur bestimmten Substanzen den Durchtritt.
- **[[Carrier-Proteine]]**: Diese binden spezifische Moleküle, wie Glucose, auf einer Seite der Membran, ändern ihre Konformation und setzen das Molekül auf der anderen Seite frei.
- **Elektrisches Membranpotential**: Selektive Ionenkanäle können zur Trennung von Ladungsträgern führen, was ein elektrisches Potential über die Membran erzeugt.
### Durchtrittsgeschwindigkeit
- **Substratabhängig**: Ähnlich der Enzymkinetik, folgt sie der Michaelis-Menten-Gleichung, was bedeutet, dass die Geschwindigkeit bis zu einem gewissen Punkt mit der Substratkonzentration steigt und dann ein Maximum erreicht.
- **Begrenzt**: Es gibt eine maximale Rate, bei der die Carrier gesättigt sind und nicht schneller transportieren können.
- **Hemmungsabhängig**: Die Transportrate kann durch kompetitive (Konkurrenz um den Bindungsplatz) oder nichtkompetitive Hemmung (Bindung an anderer Stelle, die die Funktion beeinträchtigt) beeinflusst werden.
- **Temperaturabhängig**: Die Transportrate ist stärker temperaturabhängig als bei einfacher Diffusion, da die Konformationsänderung der Proteine temperatursensitiv ist.
## Osmose
Osmose ist ein spezieller Fall der Diffusion, bei dem Wasser (das Lösungsmittel) durch eine semipermeable Membran von einem Ort niedrigerer Konzentration gelöster Teilchen zu einem Ort höherer Konzentration gelöster Teilchen fließt. Dies geschieht, um den Konzentrationsgradienten auszugleichen.
- **Osmotischer Druck ( \Delta\pi )**: Dies ist der Druck, der notwendig wäre, um die Osmose zu verhindern und ist direkt proportional zur Temperatur ( T ), der osmotischen Konzentrationsdifferenz ( $\Delta c_{osm}$ ) und dem Reflexionskoeffizienten ( $\sigma$ ), der angibt, wie gut die Membran für die gelösten Teilchen undurchlässig ist. Die Formel lautet:$$
    Δπ=σ⋅R⋅T⋅Δc_{osm}​$$
- **Osmolarität**: Sie misst die Gesamtzahl der osmotisch aktiven Teilchen pro Liter Lösung und wird in Osmol pro Liter (osmol/L) angegeben. 
	- Zum Beispiel dissoziiert 1 mol NaCl -> 1 mol Na⁺ + 1 -> mol Cl⁻ -> 2 osmol 
- **Tonizität**: Sie beschreibt die Auswirkung einer Lösung auf das Volumen einer Zelle, die in diese Lösung gelegt wird:
    - **Isoton**: Kein Nettotransport von Wasser, das Volumen der Zelle bleibt gleich.
    - **Hyperton**: Wasser verlässt die Zelle, was zu einer Schrumpfung führt.
    - **Hypoton**: Wasser tritt in die Zelle ein, was zu einer Schwellung führt.
> [!abstract] Definition
>  Die Messkette beschreibt die Schritte, die ein Signal von der Erfassung bis zur Verarbeitung durchläuft. 

## Analoge Messkette
- Signalwandlung: Umwandlung einer physikalischen Größe in ein analoges elektrisches Signal.
- Störunterdrückung: Reduzierung von Rauschen und anderen Störungen im Signal.
- Anpassung an die AD-Wandlung: Vorbereitung des Signals für die Umwandlung in ein digitales Format.
- Vorteile: Echtzeitverarbeitung und hohe Messgenauigkeit.
- Nachteile: Anfälligkeit für Störungen und aufwändige Signalverarbeitung.
### Generische Analoge Messkette
![[Pasted image 20240427125130.png#invert|600]]
- **Physikalische Wandlung**: Hier wird eine physikalische Größe wie Temperatur, Druck oder eine andere messbare Eigenschaft in ein elektrisches Signal umgewandelt.
- **Elektrische Wandlung**: Das elektrische Signal, das die physikalische Größe repräsentiert, wird weiter verarbeitet, um es für die nächsten Schritte vorzubereiten.
	- z.B. Impedanzwandlung, Transimpedanzwandlung
- **Trennung Nutzanteil - Störanteil**: In diesem Schritt wird das nützliche Signal vom Störsignal getrennt, oft durch Filterung, um die Signalqualität zu verbessern.
	- z.B. Offsetkorrektur, Differenzbildung, Filterung
	- Nutzanteil / Störanteil >> 1!: Dieses Verhältnis gibt an, dass der Nutzanteil des Signals deutlich größer sein sollte als der Störanteil, um eine klare und verwertbare Messung zu erhalten.
- **Verstärkung**: Das gefilterte Signal wird verstärkt, um es für die Messung nutzbar zu machen.
	- Anpassung an die AD-Wandlung; Eingangsspannungsbereich, Abtastrate
- **TP-Filterung**: TP steht vermutlich für “Tiefpass”, was bedeutet, dass das Signal durch einen Tiefpassfilter geleitet wird, um hochfrequente Störungen zu entfernen.
- **AD-Wandlung**: Schließlich wird das analoge Signal in ein digitales Signal umgewandelt, damit es von digitalen Geräten verarbeitet werden kann.
## Digitale Messkette:
- Extraktion von Information aus Rohdaten: Herausfiltern relevanter Informationen aus den gesammelten Daten.
- Übertragung zum Informationsempfänger: Senden der Informationen an den Empfänger.
- Darstellung von Information: Visualisierung der Informationen für den Nutzer.
- Speicherung von Information (und Rohdaten): Langfristige Aufbewahrung der Informationen und der ursprünglichen Daten.
- Vorteile: Geringe Störanfälligkeit und Möglichkeit für komplexe Signalverarbeitung.
- Nachteile: Potenzieller Informationsverlust durch Analog-Digital-Wandlung und Abhängigkeit von Rechenleistung.
## KI (z.B. wissensbasierte Systeme):
- Verknüpfung mehrerer Informationsquellen: Integration verschiedener Datenquellen für eine umfassende Analyse.
- Mustererkennung: Identifizierung von Mustern und Trends in den Daten.
- Entscheidungsunterstützung: Bereitstellung von Empfehlungen oder Vorhersagen basierend auf den analysierten Daten.

## Einflussgrößen / Störgrößen
Die Einflussgrößen und Störgrößen von Messketten sind externe Faktoren, die die Genauigkeit und Zuverlässigkeit von Messungen beeinflussen können. Basierend auf dem Bild, das Sie bereitgestellt haben, lassen sich diese Größen wie folgt erklären:
- **Einflussgrößen**: Dies sind Variablen, die das Messergebnis beeinflussen, aber nicht unbedingt als Fehler betrachtet werden. Sie können aus der Umgebung stammen, wie Temperatur, Luftdruck oder Feuchtigkeit, und müssen bei der Messung berücksichtigt werden.
- **Störgrößen**: Diese sind unerwünschte Einflüsse, die das Messsignal verfälschen können. Dazu gehören elektromagnetische Störungen, Vibrationen oder Rauschen, die das Signal überlagern und die Messung stören.
![[Pasted image 20240427125829.png#invert|600]]
- Die Messkette beginnt mit dem **Messobjekt**, von dem die **Messgröße** erfasst wird. Das **Messsignal** wird dann durch den **Messaufnehmer** und den **Fühler + Detektor** aufgenommen, wobei **Hilfsenergie** benötigt wird. Nach der **Signalverarbeitung**, die **Messverstärkung** und Rechnerverarbeitung umfassen kann, wird der **Messwert** ausgegeben und kann angezeigt, registriert oder gespeichert werden.

- Der **Signal-Störabstand (SNR)** ist ein Maß dafür, wie gut das Signal im Vergleich zum Störabstand ist. Ein höheres SNR bedeutet, dass das Signal deutlicher im Vergleich zum Rauschen ist, was zu einer genaueren Messung führt.
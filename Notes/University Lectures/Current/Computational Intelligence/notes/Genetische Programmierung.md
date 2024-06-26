- Individuen sind „Programme“, 
	- die durch Syntaxbäume dargestellt werden -> [[Genetische Programmierung#Kartesische genetische Programmierung|Kartesische genetische Programmierung]]
	- oder als Sequenz von Befehlen -> [[Genetische Programmierung#Lineare genetische Programmierung|Lineare genetische Programmierung]]

- Beispiele für Anwendungen 
	- Kurvenanpassung 
	- Datenmodellierung 
	- Symbolische Regression (sucht nach Funktion zur optimalen Beschreibung von Daten) 
	- Merkmalsauswahl (engl. feature selection) im Maschinellen Lernen (nur eine Teilmenge der möglichen Merkmale wird verwendet)

> [!success] Vorteile
> - Gut parallelisierbar 
## Kartesische genetische Programmierung
![[Pasted image 20240412081304.png#invert|400]]
### Mutation
![[Pasted image 20240412081325.png#invert|350]]
### Crossover
![[Pasted image 20240412081348.png#invert|350]]
## Lineare genetische Programmierung
![[Pasted image 20240412081443.png#invert|600]]

## Selektion
- Verletzt ein Individuum [[Optimierungsprobleme#Restriktionen|Restriktionen]] (z.B. Kollision mit Hindernis), „stirbt“ es oder wird stark abgewertet bezüglich Fittness
- Beschränkungen: 
	- Begrenzungen der zulässigen Parameterwerte 
	- verbotene Bereiche bei den 
- Bewertungskriterien Bewertungskriterien könnten hier sein: 
	- Zielabweichung 
	- Bewegungsdauer 
	- Energieverbrauch

## Unterschied zu [[Evolutionary Algorithms (EA)|EA]]
![[Pasted image 20240412081835.png#invert|400]]
- Andere Kodierung der Startpopulation 
- Veränderliche Größe der Individuen/Nachkommen
- Analogie zu natürlichen Schwärmen 
- Gruppendynamik, Folgen der Gruppenbewegung
- Idee: Beschleunigen der Partikel hin zu der besten bisher gefundenen Lösung
![[ParticleSwarmArrowsAnimation.gif#invert|400]]
> [!done] Vorteile
>  - Gut parallelisierbar
>  -Kann auch aus lokalem Optimum wieder raus

- Beispiele für Anwendungen: 
	- Auslegungsverfahren von Batteriesystemen 
	- Verkehrsplanung/-optimierun
## Algorithmus
- Eine Population ist gegeben durch ein Parametersatz $\textbf{p}$ mit $n$ Partikeln $p_{i}$
- Jeder Partikel startet mit einer zufälligen 
	- Startposition $p_{i,0}$
	- und Geschwindigkeit $v_{i,0}$
- Weitere Parameter jedes Partikels:
	- Trägheit der Bewegung $\omega$ 
	- Gespeicherter individueller Bestwert $p_{i,best}$ 
	- Gespeicherter globaler Bestwert ($g_{best}$) bzw. Bestwert der umgebenden Partikel ("[[Partikelschwarmoptimierung#Lokale Variante|lokale Variante]]") ($l_{best}$)
	- “kognitiver” Gewichtungsfaktor $c_{k}$ (bezieht sich auf den individuellen Bestwert) 
	- sozialer Gewichtungsfaktor $c_{s}$ (bezieht sich auf den globalen Bestwert)

- Bestwerte $p_{i,best}$ und $g_{best}$ werden nach jedem Schritt aktualisiert
- In jedem Schritt $n$ wird für jeden Partikel $i$
	- Neue Geschwindigkeit berechnet: $$v_{i,n+1}=\underbrace{\omega *v_{i,n}}_{\text{Trägheit}} +\underbrace{c_{k}*r_{1}*(p_{i,best}-p_{i,n})}_{\text{Verbleiben im kognitiven Optimum}}+\underbrace{c_{s}*r_{2}*(g_{best}+p_{i,n})}_{\text{Anpassung an soziales Optimum}}$$
	- Neue Position berechnet:$$p_{i,n+1}=p_{i,n}+v_{i,n+1}$$
	- Mit $r_{1},r_{2}$ zufällige Parameter
		- Balance zwischen „Exploration“ (global) und „Exploitation“ (lokal)

- Abbruchbedingung: z.B. wenn sich $g_{best}$ in bestimmter Anzahl von Schritten nicht mehr ändert
### Lokale Variante
- Bestwerte $p_{i,best}, l_{best}$ werden nach jedem Schritt aktualisiert
- -> Partikel kommunizieren nur mit ihren nächsten Nachbarn -> für das finden lokaler Optima geeignet
### Korrekturmöglichkeiten bei [[Optimierungsprobleme#Restriktionen|Restriktionen]]
- Durch das Update der Position können prinzipiell alle Werte angekommen werden unabhängig von Restriktionen
- -> Müssen korrigiert werden:
	- Entfernung des Lösungskandidaten 
	- Projektion auf Parametergrenzen (a) 
	- Reflektion an Parametergrenzen (b) 
	- Periodische Randbedingung (c)
![[Pasted image 20240412083749.png#invert|400]]

## Unterschied zu [[Evolutionary Algorithms (EA)|EA]]
![[Pasted image 20240412083807.png#invert|400]]
- Wie bei EA: Initialisierung zufälliger Startpopulation 
- Zusätzlich: initiale zufällige Geschwindigkeiten 
- Kein Crossover/Mutation 
- „Nachkommen“ sind Individuen selbst 
- Hat im Gegensatz zu EA ein „Gedächtnis“
- Jedes Individuum wird akzeptiert

## Partikelschwarmoptimierung vs Gradientenverfahren
- Partikelschwarmoptimierung:
	- Richtung resultiert aus Lage der anderen Partikel
- Gradientenverfahren
	- Richtung resultiert aus Zielfunktion
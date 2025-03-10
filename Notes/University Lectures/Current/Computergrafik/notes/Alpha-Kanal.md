- Bilder sind oft mit 32 Bits pro Pixel kodiert, was eine detaillierte Farb- und <mark style="background: #FFB86CA6;">Transparenzdarstellung</mark> ermöglicht.
- **RGBA-Format**: Das RGBA-Format enthält 24 Bits für Farbe (8 Bits pro Kanal für Rot, Grün und Blau) und zusätzlich 8 Bits für den Alpha-Kanal. <mark style="background: #FFB86CA6;">Der Alpha-Kanal repräsentiert die Transparenz (Opazität) des Pixels</mark> – also, wie durchscheinend das Bild an bestimmten Stellen ist.
- **Anwendungen**: Der Alpha-Kanal wird im Frame Buffer der Grafikkarte, in <mark style="background: #FFB86CA6;">PNG-Bildern</mark> und Texturen verwendet, um Transparenzeffekte darzustellen.
- **Bedeutung**: Transparenz ist in der Bildbearbeitung, in der Computergrafik (z.B. für Texturen) und bei Blue-Screen-Effekten essenziell.
![[Pasted image 20241028083645.png|400]]
### Alpha-Kanal bei der Texturierung
- **3D-Modelle mit Texturen**: Hier wird der Alpha-Kanal verwendet, um detaillierte Texturen auf ein 3D-Modell anzuwenden, wie bei den Palmenblättern.
- **Texturdateien**: Die 24-Bit-Farbinformation enthält die Farbe der Blätter, während der 8-Bit-Alpha-Kanal eine Maske bietet, die nur die Blattformen sichtbar macht.
- **Ergebnis**: Die Alpha-Maske hilft, die Illusion von Tiefe und Form in 3D-Szenen zu erzeugen, ohne dass die Geometrie des Modells angepasst werden muss.
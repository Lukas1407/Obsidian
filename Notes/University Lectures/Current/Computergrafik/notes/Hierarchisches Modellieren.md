![[Pasted image 20241127130753.png#invert|300]]
- **Grundidee**: 
  - Beim Modellieren komplexer Szenen wird eine hierarchische Struktur genutzt.
  - Objekte bestehen oft aus mehreren Unterobjekten (z. B. ein Auto besteht aus Karosserie, Reifen, Felgen, Schrauben).
  - **Ziel**: Effiziente Verwaltung der Geometrien, um Speicherplatz zu sparen und einfache Änderungen zu ermöglichen.
  
- **Beispiel**:
  - Ein Auto hat 4 Reifen, jeder Reifen hat 5 Schrauben.
  - Statt jede Schraube oder jedes Rad 20-mal zu modellieren, speichert man sie nur einmal und erstellt **Instanzen**.
  - Dadurch kann man dieselbe Geometrie an verschiedenen Positionen wiederverwenden.
### **Szenengraphen und Transformationen**
![[Pasted image 20241127130825.png#invert|400]]
- **Szenengraph**:
  - Ein **gerichteter azyklischer Graph** wird verwendet, um die Transformationen in einer Szene hierarchisch zu organisieren.
  - Jede Transformation in der Szene hängt von vorherigen Transformationen ab.
  
- **Beispiel**: Platzierung von Stühlen und Tischen in einem Raum:
  1. Stuhl und Tisch werden lokal modelliert.
  2. Stühle und Tische werden in Kombinationen (z. B. „4er Combo“) gruppiert.
  3. Diese Gruppen werden mit weiteren Transformationen im Raum platziert.

- **Transformationen im Szenengraphen**:
  - Die Transformation eines Stuhls im Weltkoordinatensystem wird berechnet, indem die Transformationen in der Hierarchie multipliziert werden:
    - Beispiel: $T_{C1} R_{180} T_{S2}$.
### **Matrix Stacks**

- **Matrix Stacks**:
  - Ein Werkzeug zur effizienten Verwaltung von Transformationen.
  - Transformationen werden auf einem **Stack** gespeichert.
  - Jede Transformation wird durch Matrizenoperationen (Multiplikation, Push, Pop) definiert.
  
- **Beispiel**:
  - Zuerst wird die Identitätsmatrix geladen.
  - Transformationen wie $T_{C1}$, $R_{180}$, und $T_{S1}$ werden nacheinander angewendet.
  - Nach Abschluss wird der ursprüngliche Zustand durch „Pop“ wiederhergestellt.

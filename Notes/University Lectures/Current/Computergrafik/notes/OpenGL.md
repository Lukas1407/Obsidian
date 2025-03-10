## Blending

Blending basiert auf der Gleichung:

$$
C_{\text{out}} = C_{\text{src}} \cdot F_{\text{src}} + C_{\text{dst}} \cdot F_{\text{dst}}
$$

- $C_{\text{out}}$: Die resultierende Farbe, die im Framebuffer gespeichert wird.
- $C_{\text{src}}$: Die Farbe des Fragments (Source).
- $C_{\text{dst}}$: Die Farbe, die bereits im Framebuffer gespeichert ist (Destination).
- $F_{\text{src}}$: Der Blending-Faktor für die Source-Farbe.
- $F_{\text{dst}}$: Der Blending-Faktor für die Destination-Farbe.


- Kombination der Farben des Fragments mit den im Frame Buffer gespeicherten Farben
- Blending muss aktiviert werden: 
```
glEnable(GL_BLEND);
```
### **Blending-Funktionen in OpenGL**
1. **`glBlendFunc`: Bestimmt die Blending-Faktoren**
```
void glBlendFunc(GLenum sfactor, GLenum dfactor);
```
- **Parameter:**
    - `sfactor`: Bestimmt FsrcF_{\text{src}}Fsrc​, den Faktor der Quellfarbe.
    - `dfactor`: Bestimmt FdstF_{\text{dst}}Fdst​, den Faktor der Ziel-/Framebuffer-Farbe.
    - **Häufige Werte für `sfactor` und `dfactor`:**
        - `GL_ZERO`: Faktor = 0 (ignoriert die Quelle oder das Ziel).
        - `GL_ONE`: Faktor = 1 (nutzt die Farbe unverändert).
        - `GL_SRC_ALPHA`: Faktor = Alpha-Wert der Quelle.
        - `GL_ONE_MINUS_SRC_ALPHA`: Faktor = $1- \text{Alpha der Quelle}$
        - `GL_DST_ALPHA`: Faktor = Alpha-Wert der Ziel-Farbe.
        - `GL_ONE_MINUS_DST_ALPHA`: Faktor = $1 - \text{Alpha der Ziel-Farbe}$.
        - `GL_SRC_COLOR`: Faktor = Farbwerte der Quelle.
        - `GL_ONE_MINUS_SRC_COLOR`: Faktor = $1 - \text{Farbwerte der Quelle}$
    **Beispiel für typische Transparenz:**
```
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
```
### **Blending-Gleichung**
`glBlendEquation`: Bestimmt die Blending Gleichung
```
void glBlendEquation(GLenum mode);
```

- **Parameter:**
    - `mode`: Gibt die Gleichung für die Kombination der Farben an.
        - `GL_FUNC_ADD`: Standard. Addiert $\text{src} \cdot F_{\text{src}} + \text{dst} \cdot F_{\text{dst}}$​.
        - `GL_FUNC_SUBTRACT`: Subtrahiert $\text{src} \cdot F_{\text{src}} - \text{dst} \cdot F_{\text{dst}}$.
        - `GL_FUNC_REVERSE_SUBTRACT`: Subtrahiert $\text{dst} \cdot F_{\text{dst}} - \text{src} \cdot F_{\text{src}}$​.
        - `GL_MIN`: Nimmt den kleineren Wert aus Quelle und Ziel.
        - `GL_MAX`: Nimmt den größeren Wert aus Quelle und Ziel.
**Beispiel:**
```
glBlendEquation(GL_FUNC_ADD)`
```

## Alpha Blending
**Alpha Blending** ist ein Verfahren in der Computergrafik, mit dem die Transparenz von Objekten simuliert wird, indem Farben eines Fragments (das gezeichnet werden soll) mit den Farben eines bereits im Framebuffer gespeicherten Pixels kombiniert werden.
### **Grundidee:**
- Jedes Fragment hat einen **Alpha-Wert** ($\alpha$), der die Transparenz oder Deckkraft des Fragments angibt:
  - $\alpha = 1.0$: Vollständig undurchsichtig (opak).
  - $\alpha = 0.0$: Vollständig transparent.
  - $0.0 < \alpha < 1.0$: Teilweise transparent.

Die resultierende Farbe ($C_{\text{out}}$) ergibt sich aus einer Mischung von:
- Der Quellfarbe ($C_{\text{src}}$): Farbe des Fragments.
- Der Ziel-/Framebuffer-Farbe ($C_{\text{dst}}$): Bereits gespeicherte Farbe.
### **Blending-Gleichung:**
$$
C_{\text{out}} = C_{\text{src}} \cdot \alpha + C_{\text{dst}} \cdot (1 - \alpha)
$$
### Sortierung beim Alpha-Blending
2. **Problem mit Transparenz:**
   - <mark style="background: #FFB86CA6;">Blending ist nicht kommutativ, d. h. die Reihenfolge der Objekte beeinflusst das Ergebnis</mark>:
     $$
     C_{\text{out}} \neq C_{\text{out}}' \quad \text{wenn die Reihenfolge der Objekte unterschiedlich ist.}
     $$
   - <mark style="background: #FFB86CA6;">Wenn ein weiter entferntes Fragment zuerst gezeichnet wird und anschließend ein näher liegendes Fragment, wird das Blending fehlerhaft, da die korrekten Farbanteile des Hintergrunds nicht berücksichtigt</mark> werden.

3. **Z-Test und Transparenz-Konflikt:**
   - Der Z-Test **überschreibt** standardmäßig die Tiefe eines Pixels, selbst bei semitransparenten Objekten.
   - <mark style="background: #FFB86CA6;">Ohne Sortierung könnte ein Fragment im Hintergrund durch den Z-Test blockiert werden, bevor es zum Blending kommt</mark>.
### **Wie werden Objekte sortiert?**
Um Transparenz korrekt darzustellen, müssen semitransparente Objekte <mark style="background: #FFB86CA6;">nach ihrer Entfernung zur Kamera sortiert werden</mark>, bevor sie gezeichnet werden. Die Sortierung erfolgt in zwei Schritten:
1. <mark style="background: #FFB86CA6;">Opake Objekte zuerst (Front-to-Back):</mark>
	   - **Opake Objekte** werden zuerst gezeichnet.
	   - <mark style="background: #FFB86CA6;">Dabei wird der Tiefentest verwendet</mark>, um sicherzustellen, dass nur die nächstliegenden opaken Objekte sichtbar sind.
	   - Diese füllen den Tiefenpuffer.
1. <mark style="background: #FFB86CA6;">Semitransparente Objekte danach</mark> (Back-to-Front):
	   - **Semitransparente Objekte** werden in der Reihenfolge ihrer Entfernung zur Kamera gezeichnet (von hinten nach vorne).
	   - <mark style="background: #FFB86CA6;">Diese Sortierung stellt sicher, dass beim Blending die Farben der Objekte im Hintergrund zuerst gezeichnet werden, bevor sie mit den näheren Objekten gemischt werden</mark>.

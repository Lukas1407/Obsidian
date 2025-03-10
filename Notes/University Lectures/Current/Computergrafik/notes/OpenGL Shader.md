mat.xyz
vec4(v3 , 1.0)
vec3(0.0);
normalize(v)
length(v3)

## Ausgabe des Shaders
- Definiert bei `out`
- Definiert also den Return Type, weil die main function immer void ist
```cpp
// Shader-Ausgaben (gehen an den Fragment Shader) 
out vec3 FragPos; // Weltkoordinaten der Fragmente 
out vec3 Normal; // Transformierte Normale in Weltkoordinaten 
out vec2 TexCoords; // Texturkoordinaten
```

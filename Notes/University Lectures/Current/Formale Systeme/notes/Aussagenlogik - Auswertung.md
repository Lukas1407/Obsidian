> [!abstract] Definition
> Die Auswertung in der Aussagenlogik gibt an, wie der Wahrheitswert einer Formel basierend auf einer gegebenen [[Aussagenlogik - Interpretation|Interpretation]] bestimmt wird. 

## Formale Definition
$$\text{val}_I :\text{For}_{\Sigma}^{0}\rightarrow \{W,F\}$$
- Jeder Formel, die aus der [[Aussagenlogik - Signatur|Signatur]] erstellt werden kann wird ein Wahrheitswert zugeordnet
### Auswertung
$$\text{val}_I(1):W$$
$$\text{val}_I(0):F$$
$$\text{val}_I(P_{i}):I(P_{i})\ \ \forall P_{i}\in\Sigma$$
$$$$
![[Pasted image 20240611162928.png|400]]

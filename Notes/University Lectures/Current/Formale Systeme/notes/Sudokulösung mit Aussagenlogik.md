![[Pasted image 20240611155611.png#invert|400]]
- Wir führen für jede Zellenposition $(i,j)$ und jede Zahl $k$ (zwischen 1 und 9) eine Boolesche Variable ein: $$D_{i,j}^{k}$$sodass $D_{i,j}^{k}$ wahr ist, wenn in der Zelle $(i,j)$ die Zahl $k$ steht
- $D_{9,1}^{7}$ bedeutet dann, dass in der links unteren Ecke die Zahl 7 steht
- $$D_{1,9}^{1}\lor D_{2,9}^{1}\lor D_{3,9}^{1}\lor D_{4,9}^{1}\lor D_{5,9}^{1}\lor D_{6,9}^{1}\lor D_{7,9}^{1}\lor D_{8,9}^{1}\lor D_{9,9}^{1}$$ bedeutet, dass die Zahl 1 mindestens ein mal in der ersten Zeile vorkommen muss
- $$D_{1,1}^{1}\lor D_{1,2}^{1}\lor D_{1,3}^{1}\lor D_{1,4}^{1}\lor D_{1,5}^{1}\lor D_{1,6}^{1}\lor D_{1,7}^{1}\lor D_{1,8}^{1}\lor D_{1,9}^{1}$$ bedeutet, dass die Zahl 1 mindestens 1 mal in der ersten Reihe vorkommen muss
- $$D_{1,1}^{1}\lor D_{1,2}^{1}\lor D_{1,3}^{1}\lor D_{2,1}^{1}\lor D_{2,2}^{1}\lor D_{2,3}^{1}\lor D_{3,1}^{1}\lor D_{3,2}^{1}\lor D_{3,3}^{1}$$ bedeutet, dass die Zahl 1 mindestens 1 mal in der ersten Region vorkommen muss
- -> Ergibt soweit: (9 + 9 + 9) ∗ 9 = 243 Formeln.
- Man muss noch sagen, dass auf jeder Zelle höchstens eine Zahl stehen kann:$$\lnot(D_{1,1}^{1}\land D_{1,1}^{2}),\lnot(D_{1,1}^{1}\land D_{1,1}^{3}),\dots$$
	- Allgemein: $$\lnot(D_{i,j}^{n}\land D_{k,l}^{m})$$ für alle $i,j,n,m\in [1,9]$ und $n<m$
- Ergibt insgesamt: 243 + 81 ∗ 36 = 3159 Formeln
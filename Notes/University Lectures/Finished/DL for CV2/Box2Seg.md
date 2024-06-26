- No hand crafted components except for [[GrabCut]]

## Architecture
- 1 encoder 3 decoder architecture![[Pasted image 20240226085654.png|400]]
- $\beta$-Decoder: For pixel embeddings, such that pixels with similar semantics are represented similarly
- $y$-Decoder: To produce the final segmentation mask
- $\alpha$-Decoder: Uses an attention mask to help focus on the foreground objects

## Training
1. Box2Seg uses coarse masks obtained from [[GrabCut]], which are trained via cross-entropy:$$L_{GC}=-\frac{1}{m}\sum_{c=0}^{L}\sum_{i=1}^{m}M(i,c)log(y(i,c))$$
2. [[GrabCut]] masks are coarse and noisy -> use attention mechanism to help focus on the foreground objects, by providing an attention value $\alpha(i,c)\in[0,1]$ for each pixel, with loss:$$L_{fg}=-\frac{1}{\sum_{i=1}^{m}B(i,c)}\sum_{c=1}^{L}\sum_{i=1}^{m}\alpha(i,c)B(i,c)log(y(i,c))$$
	- Problem: Model can just learn $\alpha(i,c)=0$ for all $i,c$
	- Solution: Add regularization term to prevent this:$$L_{\alpha}^{fr}=max(0,\gamma\eta_{c}-\eta'_{c}),$$where $\gamma\in[0,1]$, $\eta_{c}=\frac{\sum_{i=1}^{m}M(i,c)}{\sum_{i=1}^{m}B(i,c)}$ (the filling rate of the [[GrabCut]] mask), and $\eta'_{c}=\frac{\sum_{i=1}^{m}\alpha(i,c)B(i,c)}{\sum_{i=1}^{m}B(i,c)}$ (the filling rate of the attention map)
	- This penalizes if the attention map fails to focus on foreground pixels 
3. The attention map only considers foreground classes, so for the background class, we need an additional loss:$$L_{bg}=-\frac{1}{\sum_{i=1}^{m}B(i,0)}\sum_{i=1}^{m}B(i,0)log(y(i,0))$$
4. Loss for the pixel embedding, which enforces similarities between pixels that share the same semantics and dissimilarities between pixels that don‘t:$$L_{\mathop{\mathbb{A}}}=\sum_{i,j}(\mathop{\mathbb{A}}(i,j)-y_j^Ty_{i})^2,$$ with $\mathop{\mathbb{A}}$ being the affinity matrix, which measures the similarity between pixels based on the normalized dot-product.
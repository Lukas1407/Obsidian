![[Entropy]]

- “Regularize” Reinforcement Learning Objective with Entropy$$\pi^{*}=\arg\max_{\pi}\mathbb{E}_{\pi}\left[\sum_{t=0}^{\infty}\gamma^{t}(r_{t}+\alpha\mathcal{H}(\pi|s_{t})) \right]$$, with $\alpha$ the trade-off between maximizing reward and keeping entropy (i.e. exploration)
- V-Function:$$\begin{split} V^{\pi}(s)=\mathbb{E}_{\pi}\left. \left[\sum_{t=0}^{\infty}\gamma^{t}(r_{t}+\alpha\mathcal{H}(\pi|s_{t}))\right |s_{0}=s \right] \\ V^{\pi}(s)=\mathbb{E}_{\pi}\left. \left[Q^{\pi}(s,a)+\alpha\mathcal{H}(\pi|s_{t}))\right |s_{0}=s \right] \end{split}$$
- Q-Function:$$Q^{\pi}(s,a)=\mathbb{E}_{\pi}\left. \left[\sum_{t=0}^{\infty}\gamma^{t}(r_{t}+ \underbrace{\sum_{\textcolor{red}{t=1}}^{\infty}}_\text{Q-Function does not include entropy of current time step – simplifies policy optimization later}  \alpha\mathcal{H}(\pi|s_{t}))\right |s_{0}=s,\ a_{0}=a \right]$$
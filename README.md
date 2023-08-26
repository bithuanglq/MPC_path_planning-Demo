# MPC_path_planning-Demo
A Quick Start Demo of MPC Path Planning Based on YALMIP in MATLAB
\\
Before using this code, we want you to have installed [Yalmip](https://yalmip.github.io/download/) and IPOPT solver.\\

vehicle's parameters:\\

\\
kinematic model:\\
$ \begin{equation}
\left\{\begin{matrix}\dot{x}=v\cdot\cos\left(\theta\right)\\\dot{y}=v\cdot\sin\left(\theta\right)\\\dot{v}=a\\\dot{\theta}=v\cdot\tan\left(\varphi\right)/b\_width\\\dot{\varphi}=\eta\end{matrix}\right.
\end{equation}$
\\
[Chinese blog](https://zhuanlan.zhihu.com/p/652511722)

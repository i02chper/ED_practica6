# El tipo abstracto Grafo Ponderado

Un grafo ponderado es un grafo (dirigido o no) donde cada lado $e_{uv}$ tiene asociado un peso $w_{uv} \in \mathcal{R}$.

Un vértice $u$ se dirá que es adyacente al vértice $v$ si el lado $e_{uv}$ tiene un peso $w_{uv}<\infty$.

Para diseñar la implementación del tipo abstracto usaremos una matrix de adyacencia que almacenará los pesos para todas las $N^2$ posibles conexiones entre pares de nodos.

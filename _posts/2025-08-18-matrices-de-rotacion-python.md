---
layout: post
title: "Matrices de rotación"
date: 2025-08-18 10:00:00 +0000
categories: [robotics, python]
tags: [robotics,kinematics,python]
author: Pedro Jorge De Los Santos
---

Las matrices de rotación son un tipo de matrices especiales que nos sirven para representar rotaciones de un sólido en el plano o en el espacio. Son muy utilizadas en robótica y en la generación de gráficos por computadora. Aquí nos centraremos en cómo estas matrices nos sirven en robótica para representar la orientación de los eslabones de un manipulador con respecto a un sistema de referencia.

## Una primera aproximación: rotando vectores

Para irnos familiarizando con las matrices de rotación comenzaremos viendo algunos ejemplos de cómo podemos utilizarlas para rotar vectores. La matriz mostrada a continuación es una matriz que representa una rotación de un ángulo $\theta$ en el plano $xy$ (o alrededor del eje $z$, para ser más precisos).

$$
R = \begin{bmatrix}
\cos\theta  & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

Por ahora no te preocupes con respecto a de dónde *sale* esta matriz, más adelante nos encargaremos de esto. Si hacemos que $\theta$ tome un valor específico, por ejemplo $90°$, entonces la matriz $R$ estaría dada por:

$$
R = \begin{bmatrix}
0 & -1 \\
1 & 0 \\
\end{bmatrix}
$$

Si tomamos un vector en el plano y lo multiplicamos por esta matriz, notaremos que el vector que resulta está rotado $90°$ en sentido antihorario. Por ejemplo, sea $\vec{u}$ un vector dado por:

$$  
\vec{u} = \begin{bmatrix} 5 \\ 0  \end{bmatrix}
$$

Si lo multiplicamos por $R$ entonces resulta:

$$
\begin{bmatrix}
0 & -1 \\
1 & 0 \\
\end{bmatrix} \begin{bmatrix} 5 \\ 0  \end{bmatrix} 
= \begin{bmatrix} 0 \\ 5  \end{bmatrix}
$$

Observa que el vector resultante tiene componente distinta de cero únicamente en la dirección de $y$. Pero ¿qué pasaría si en lugar de utilizar $\theta=90°$ usamos $\theta=-90°$? Veamos, ahora $R$ sería:

$$
R = \begin{bmatrix}
0 & 1 \\
-1 & 0 \\
\end{bmatrix}
$$

Y entonces:

$$
\begin{bmatrix}
0 & 1 \\
-1 & 0 \\
\end{bmatrix} \begin{bmatrix} 5 \\ 0  \end{bmatrix} 
= \begin{bmatrix} 0 \\ -5  \end{bmatrix}
$$

Notarás que en este caso la componente $y$ es negativa, es decir, ahora el vector ha sido rotado en sentido horario.


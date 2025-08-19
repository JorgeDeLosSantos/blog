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
---
layout: post
title: "Calculando la matriz jacobiana de un manipulador utilizando Python"
date: 2025-08-18 10:00:00 +0000
categories: [robotics, python]
tags: [robotics,kinematics,python]
author: Pedro Jorge De Los Santos
---

Cuando trabajamos con robots manipuladores (brazos robóticos), uno de los conceptos más importantes en cinemática diferencial es la matriz jacobiana. Esta matriz es fundamental porque conecta el espacio articular (las variables de las juntas) con el espacio cartesiano (posición y orientación del efector final).

En este post veremos:

- Qué es la matriz jacobiana
- Para qué sirve
- Un algoritmo general para calcularla
- Cómo calcularla simbólicamente con SymPy
- Cómo calcularla numéricamente con NumPy

## ¿Qué es la matriz jacobiana?

Consideremos un manipulador serial de **n grados de libertad**, descrito por un vector de variables articulares:
$$ \mathbf{q} = [q_1, q_2, ..., q_n]^T $$

El efector final del robot tiene una posición y orientación que podemos agrupar en una variable cartesiana:

$$ \mathbf{x} = f(q) $$

donde $f$ representa la cinemática directa.

La **matriz jacobiana** es una matriz que relaciona los cambios diferenciales en las articulaciones con los cambios diferenciales del efector final:

$$ \dot{\mathbf{x}} = J(\mathbf{q}) \dot{\mathbf{q}}  $$
donde:

- $\dot{\mathbf{q}}$ es el vector de velocidades articulares
- $\dot{\mathbf{x}}$ es la velocidad cartesiana del efector final
- $J(\mathbf{q})$ es la matriz jacobiana

En robótica, normalmente se usa la **jacobiana geométrica**, que relaciona velocidades articulares con la velocidad lineal y angular del efector final:

$$
\mathbf{V} = \begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix} = J(\mathbf{q}) \dot{\mathbf{q}}
$$

donde:

- $\mathbf{v}$ es la velocidad lineal del efector final
- $\boldsymbol{\omega}$ es la velocidad angular del efector final

Por lo tanto, para un manipulador serial:

$$ J(\mathbf{q}) \in \mathbb{R}^{6 \times n} $$
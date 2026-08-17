---
layout: post
title: "Matrices de rotación"
date: 2025-08-18 10:00:00 +0000
categories: [robotics, python]
tags: [robotics,kinematics,python]
author: Pedro Jorge De Los Santos
---

Las matrices de rotación son un tipo de matrices especiales que nos sirven para representar rotaciones de un sólido en el plano o en el espacio. Son muy utilizadas en robótica y en la generación de gráficos por computadora. Aquí nos centraremos en mostrar cómo estas matrices nos sirven en robótica para representar la orientación de los eslabones de un manipulador con respecto a un sistema de referencia y cómo utilizar Python para realizar operaciones con este tipo de matrices.

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

Observa que el vector resultante tiene componente distinta de cero únicamente en la dirección de $y$. Pero ¿qué pasaría si en lugar de utilizar $\theta=90°$, usamos $\theta=-90°$? Veamos, ahora $R$ sería:

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

## ¿Cómo obtener las matrices de rotación?

En la sección anterior utilizamos directamente una matriz de rotación sin preocuparnos demasiado por su origen. Veamos ahora de dónde proviene.

Consideremos nuevamente un vector unitario sobre el eje (x):

$$
\vec{e}_x =
\begin{bmatrix}
1 \
0
\end{bmatrix}
$$

Si rotamos este vector un ángulo (\theta) en sentido antihorario, sus nuevas componentes pueden obtenerse directamente utilizando trigonometría:

$$
\vec{e}_x' =
\begin{bmatrix}
\cos\theta \
\sin\theta
\end{bmatrix}
$$

Ahora hagamos lo mismo con el vector unitario correspondiente al eje (y):

$$
\vec{e}_y =
\begin{bmatrix}
0 \
1
\end{bmatrix}
$$

Después de rotarlo el mismo ángulo (\theta), obtenemos:

$$
\vec{e}_y' =
\begin{bmatrix}
-\sin\theta \
\cos\theta
\end{bmatrix}
$$

Si colocamos estos dos vectores rotados como columnas de una matriz obtenemos:

$$
R(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \
\sin\theta & \cos\theta
\end{bmatrix}
$$

que es precisamente la matriz que utilizamos anteriormente.

Una manera útil de interpretar una matriz de rotación es, por tanto, considerar que **sus columnas representan las direcciones que toman los vectores de la base después de realizar la rotación**.

### Rotaciones en el espacio

En tres dimensiones podemos definir matrices equivalentes para representar rotaciones alrededor de cada uno de los ejes coordenados.

Adoptaremos la convención habitual de considerar positivas las rotaciones que siguen la **regla de la mano derecha**.

Una rotación de un ángulo (\theta) alrededor del eje (x) está dada por:

$$
R_x(\theta) =
\begin{bmatrix}
1 & 0 & 0 \
0 & \cos\theta & -\sin\theta \
0 & \sin\theta & \cos\theta
\end{bmatrix}
$$

Observa que la componente asociada al eje (x) permanece sin cambios. La rotación ocurre únicamente en el plano (yz).

De forma similar, una rotación alrededor del eje (y) se representa mediante:

$$
R_y(\theta) =
\begin{bmatrix}
\cos\theta & 0 & \sin\theta \
0 & 1 & 0 \
-\sin\theta & 0 & \cos\theta
\end{bmatrix}
$$

y una rotación alrededor del eje (z) mediante:

$$
R_z(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \
\sin\theta & \cos\theta & 0 \
0 & 0 & 1
\end{bmatrix}
$$

Esta última es simplemente la versión tridimensional de la matriz de rotación que utilizamos al comienzo del post.

Por ejemplo, si queremos rotar el vector

$$
\vec{u} =
\begin{bmatrix}
1 \
0 \
0
\end{bmatrix}
$$

un ángulo de (90^\circ) alrededor del eje (z), utilizamos:

$$
R_z(90^\circ) =
\begin{bmatrix}
0 & -1 & 0 \
1 & 0 & 0 \
0 & 0 & 1
\end{bmatrix}
$$

y obtenemos:

$$
R_z(90^\circ)\vec{u}
====================

\begin{bmatrix}
0 \
1 \
0
\end{bmatrix}
$$

Como era de esperarse, el vector inicialmente alineado con el eje (x) queda ahora alineado con el eje (y).

En robótica estas matrices no solamente nos permiten rotar vectores. También pueden utilizarse para describir **la orientación de un sistema de referencia con respecto a otro**, algo que necesitaremos constantemente al estudiar la cinemática de manipuladores.

## Propiedades de las matrices de rotación

Las matrices de rotación poseen algunas propiedades que las distinguen de una matriz cualquiera.

En tres dimensiones, una matriz de rotación pertenece al conjunto denominado (SO(3)), el grupo especial ortogonal de dimensión 3. Esto significa que debe satisfacer dos condiciones fundamentales.

La primera es:

$$
R^T R = R R^T = I
$$

es decir, la matriz es **ortogonal**.

De esta propiedad se desprende inmediatamente que:

$$
R^{-1} = R^T
$$

Esta igualdad resulta especialmente útil porque calcular la inversa de una matriz en general puede ser una operación relativamente costosa, mientras que obtener su transpuesta es trivial.

Geométricamente, la inversa representa simplemente realizar la rotación en sentido contrario.

Por ejemplo:

$$
R_z(\theta)^{-1}
================

# R_z(-\theta)

R_z(\theta)^T
$$

La segunda condición que debe satisfacer una matriz de rotación es:

$$
\det(R)=1
$$

Una matriz ortogonal cuyo determinante es (-1) no representa una rotación pura, sino una transformación que incluye una reflexión.

Además, debido a que las columnas de una matriz de rotación representan vectores unitarios mutuamente perpendiculares, éstas forman una base ortonormal.

Si escribimos:

$$
R =
\begin{bmatrix}
\vert & \vert & \vert \
\vec{r}_1 & \vec{r}_2 & \vec{r}_3 \
\vert & \vert & \vert
\end{bmatrix}
$$

entonces:

$$
\vec{r}_i^T\vec{r}_i = 1
$$

y para (i\neq j):

$$
\vec{r}_i^T\vec{r}_j = 0
$$

Las matrices de rotación también preservan la longitud de los vectores. Si:

$$
\vec{v}' = R\vec{v}
$$

entonces:

$$
|\vec{v}'| = |\vec{v}|
$$

Esto tiene sentido desde el punto de vista geométrico: al rotar un vector únicamente cambiamos su dirección, no su magnitud.

## Composición de rotaciones

Una de las ventajas de representar rotaciones mediante matrices es que podemos combinar varias rotaciones utilizando simplemente multiplicación matricial.

Supongamos, por ejemplo, que realizamos una rotación (R_1) y posteriormente una rotación (R_2). Si inicialmente tenemos un vector (\vec{v}), después de la primera rotación obtenemos:

$$
\vec{v}' = R_1\vec{v}
$$

y después de la segunda:

$$
\vec{v}'' = R_2\vec{v}'
$$

Sustituyendo la primera expresión:

$$
\vec{v}'' = R_2R_1\vec{v}
$$

Por lo tanto, las dos rotaciones pueden representarse mediante una única matriz:

$$
R = R_2R_1
$$

Aquí aparece una cuestión muy importante: **el orden de las rotaciones importa**.

En general:

$$
R_2R_1 \neq R_1R_2
$$

La multiplicación de matrices no es conmutativa y, por lo tanto, aplicar primero una rotación alrededor de (x) y después una alrededor de (z) no produce necesariamente el mismo resultado que hacerlo en el orden contrario.

Esta propiedad será fundamental cuando estudiemos representaciones de orientación como los ángulos de Euler y, posteriormente, la cinemática de robots manipuladores.

## Matrices de rotación utilizando Python

Como ocurre con muchos otros objetos matemáticos, podemos trabajar con matrices de rotación en Python tanto de manera simbólica como numérica.

El enfoque simbólico resulta conveniente cuando queremos desarrollar expresiones algebraicas en función de uno o varios ángulos. Para ello podemos utilizar **SymPy**.

Cuando conocemos los valores concretos de los ángulos y queremos realizar cálculos numéricos, podemos utilizar **NumPy**.

### La forma simbólica

Comencemos definiendo simbólicamente una matriz de rotación alrededor del eje (z):

```python
import sympy as sp

theta = sp.symbols("theta")

Rz = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta), 0],
    [sp.sin(theta),  sp.cos(theta), 0],
    [0,              0,             1]
])

Rz
```

Debido a que `theta` es una variable simbólica, SymPy conserva las funciones trigonométricas dentro de la matriz.

Podemos utilizar esta matriz para rotar un vector:

```python
u = sp.Matrix([1, 0, 0])

v = Rz * u
v
```

El resultado es:

$$
\begin{bmatrix}
\cos\theta \
\sin\theta \
0
\end{bmatrix}
$$

También podemos utilizar SymPy para verificar algunas de las propiedades que vimos anteriormente. Por ejemplo:

```python
sp.simplify(Rz.T * Rz)
```

produce la matriz identidad:

$$
I =
\begin{bmatrix}
1 & 0 & 0 \
0 & 1 & 0 \
0 & 0 & 1
\end{bmatrix}
$$

También podemos comprobar su determinante:

```python
sp.simplify(Rz.det())
```

obteniendo:

```text
1
```

Otra ventaja del cálculo simbólico es que podemos componer varias rotaciones sin asignar todavía valores específicos a los ángulos.

Por ejemplo:

```python
alpha, beta = sp.symbols("alpha beta")

Rx = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(alpha), -sp.sin(alpha)],
    [0, sp.sin(alpha),  sp.cos(alpha)]
])

Rz = sp.Matrix([
    [sp.cos(beta), -sp.sin(beta), 0],
    [sp.sin(beta),  sp.cos(beta), 0],
    [0,             0,            1]
])

R = Rz * Rx
```

La matriz resultante representa la composición de ambas rotaciones.

### La forma numérica

Cuando los ángulos de rotación son conocidos podemos realizar los cálculos utilizando NumPy.

Por ejemplo, definamos una rotación de (90^\circ) alrededor del eje (z):

```python
import numpy as np

theta = np.deg2rad(90)

Rz = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]
])
```

Es importante recordar que las funciones trigonométricas de NumPy trabajan con **radianes**, por lo que utilizamos `np.deg2rad` para convertir los grados.

Ahora podemos rotar un vector:

```python
u = np.array([1, 0, 0])

v = Rz @ u

print(v)
```

El operador `@` realiza la multiplicación matricial.

El resultado será aproximadamente:

```text
[6.123234e-17 1.000000e+00 0.000000e+00]
```

Puede resultar extraño que la primera componente no sea exactamente cero. Esto se debe a los errores asociados a la representación numérica en punto flotante. El valor

```text
6.123234e-17
```

es tan pequeño que, para efectos prácticos, puede considerarse igual a cero.

Podemos comprobarlo utilizando:

```python
np.allclose(v, [0, 1, 0])
```

que devuelve:

```text
True
```

También podemos verificar numéricamente las propiedades de la matriz:

```python
np.allclose(Rz.T @ Rz, np.eye(3))
```

y:

```python
np.linalg.det(Rz)
```

obteniendo una matriz identidad en el primer caso y un determinante aproximadamente igual a (1) en el segundo.

Las matrices de rotación constituyen una de las herramientas fundamentales para describir orientación y movimiento en robótica. Más adelante veremos que también forman parte de objetos más generales, como las **matrices de transformación homogénea**, que nos permitirán representar simultáneamente la posición y orientación de los sistemas de referencia asociados a un robot.

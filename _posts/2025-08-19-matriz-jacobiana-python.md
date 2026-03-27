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

$$ \mathbf{x} = f(\mathbf{q}) $$

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

En general, para un manipulador serial de $n$ grados de libertad, la matriz jacobiana posee siempre 6 filas y tantas columnas como grados de libertad, es decir:

$$ J(\mathbf{q}) \in \mathbb{R}^{6 \times n} $$

La matriz jacobiana es dependiente de la configuración del manipulador serial. Por cuestiones de simplicidad, en lo subsiguiente esto se obviará.

## Para qué sirve la matriz jacobiana


### Calcular velocidades del elemento terminal

Si conocemos las velocidades articulares $\dot{\mathbf{q}}$, podemos determinar la velocidad lineal y angular del elemento terminal:

$$ \begin{bmatrix} \mathbf{v} \\ \mathbf{\omega} \end{bmatrix} = J \dot{\mathbf{q}} $$

También es posible calcular las velocidades articulares requeridas $\dot{\mathbf{q}}$ para una determinada velocidad lineal y angular deseada, para esto se usa la inversa de la matriz jacobiana:

$$ \dot{\mathbf{q}} = J^{-1} \begin{bmatrix} \mathbf{v} \\ \mathbf{\omega} \end{bmatrix}  $$

### Análisis de singularidades

La matriz jacobiana también nos permite identificar configuraciones donde el robot pierde grados de libertad (esos momentos donde el robot *se bloquea*). Una configuración de este tipo (denominada configuración singular) ocurre cuando el jacobiano pierde rango, es decir:

$$ \det(J) = 0 $$

### Relacionar fuerzas

La matriz jacobiana también permite *mapear* fuerzas ($\mathbf{F}$) y torques ($\boldsymbol{\mu}$) aplicadas en el extremo del manipulador a las fuerzas/torques que experimentan las articulaciones:

$$  
\boldsymbol{\tau} = J^T \begin{bmatrix} \mathbf{F} \\ \boldsymbol{\mu} \end{bmatrix}
$$

Esto es de mucha utilidad sobre en todo en dinámica y control de robots.

## Algoritmo para calcularla

**Algoritmo: compute_jacobian(dh_params, joint_types)**

**Entrada:**
- $\text{dh\_params = }[(a_1, \alpha_1, d_1, \theta_1), (a_2, \alpha_2, d_2, \theta_2), \cdots, (a_n, \alpha_n, d_n, \theta_n)]$
- $\text{joint\_types = } [j_1, j_2, \cdots, j_n]$

1. Inicializar:
   - $T_s \leftarrow [I_4]$

2. Para $i = 0$ hasta $n-1$:
   - $T_i \leftarrow T_s[i] \cdot \text{dh\_matrix}(a_i, \alpha_i, d_i, \theta_i)$
   - $T_s \leftarrow [T_s, T_i] $

3. $\mathbf{o}_n \leftarrow T_s[n]_{0:3,3}$

4. $J \leftarrow 0^{6 \times n}$

5. Para $i = 0$ hasta $n-1$:
   - $\mathbf{o}_{i-1} \leftarrow T_s[i]_{0:3,3}$
   - $\mathbf{z}_{i-1} \leftarrow T_s[i]_{0:3,2}$
   - Si $j_i$ es revoluta:
     - $J_{v_i} \leftarrow \mathbf{z}_{i-1} \times (\mathbf{o}_n - \mathbf{o}_{i-1})$
     - $J_{\omega_i} \leftarrow \mathbf{z}_{i-1}$
   - Si $j_i$ es prismática:
     - $J_{v_i} \leftarrow \mathbf{z}_{i-1}$
     - $J_{\omega_i} \leftarrow \mathbf{0}$
   - $J[:,i] \leftarrow \begin{bmatrix} J_{v_i} \\ J_{\omega_i} \end{bmatrix}$

**Salida**
- Jacobiano: $J \in \mathbb{R}^{6 \times n}$


## Cómo calcularla simbólicamente con SymPy

Ahora veremos cómo calcular la matriz jacobiana utilizando SymPy. Esta librería nos permite utilizar variables simbólicas en los cálculos, lo cual la hace muy útil para cuestiones de enseñanza/aprendizaje. 

Lo primero que haremos es importar la librería:

```python
import sympy as sp
```

Y ahora procedemos a definir la función `dh_matrix` que calcula la matriz de Denavit-Hartenberg $A_i$, dados como argumentos los cuatro parámetros ($a_i$, $\alpha_i$, $d_i$, $\theta_i$):

```python
def dh_matrix(a,alpha,d,theta):
    ct = sp.cos(theta)
    st = sp.sin(theta)
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)
    A = sp.Matrix([[ct, -st*ca,  st*sa, a*ct],
                   [st,  ct*ca, -ct*sa, a*st],
                   [0,     sa,     ca,    d ],
                   [0,      0,      0,     1 ]])
    return A
```

Definimos la función `compute_jacobian` que calcula la matriz jacobiana:

```python
def compute_jacobian(dh_params,joint_types):
    n = len(joint_types)
    Ts = [sp.eye(4)]
    for params in dh_params:
        Ts.append( Ts[-1] * dh_matrix(*params) )
    
    on = Ts[n][:3,3]
    J = sp.zeros(6,n)
    for i in range(n):
        z_im1 = Ts[i][:3,2]
        o_im1 = Ts[i][:3,3]

        if joint_types[i] == "R":
            Jv = z_im1.cross(on - o_im1)
            Jw = z_im1
        elif joint_types[i] == "P":
            Jv = z_im1
            Jw = sp.zeros(3,1)
        J[:3,i] = Jv
        J[3:,i] = Jw
    
    return sp.simplify(J)
```

Para probar nuestra función implementada vamos a utilizar el clásico manipulador RR planar, cuyos parámetros de Denavit-Hartenberg están dados en la siguiente tabla:

|  $i$  | $a_i$  | $ \alpha_i $ | $ d_i $ | $ \theta_i $ |
|:---:|:-----:|:---:|:---:|:----------:|
| $1$ | $l_1$ | $0$ | $0$ | $\theta_1$ |
| $2$ | $l_2$ | $0$ | $0$ | $\theta_2$ |

Creamos las variables simbólicas a utilizar:

```python
l1,l2 = sp.symbols("l_1, l_2")
theta1,theta2 = sp.symbols("theta_1, theta_2")
```

Definimos las variables `dh_params` y `joint_types`, y se las pasamos como argumentos a la función `compute_jacobian`:

```python
dh_params = [(l1,0,0,theta1), (l2,0,0,theta2)]
joint_types = ["R","R"]

J = compute_jacobian(dh_params,joint_types)
```

Ahora en `J` tendremos almacenada la matriz jacobiana calculada. Si estás usando Jupyter o Google Colab, podrás ver la matriz *renderizada* de una manera muy conveniente. A continuación, se muestra la matriz que resulta del cálculo.

$$
\left[\begin{matrix}- l_{1} \sin{\left(\theta_{1} \right)} - l_{2} \sin{\left(\theta_{1} + \theta_{2} \right)} & - l_{2} \sin{\left(\theta_{1} + \theta_{2} \right)}\\l_{1} \cos{\left(\theta_{1} \right)} + l_{2} \cos{\left(\theta_{1} + \theta_{2} \right)} & l_{2} \cos{\left(\theta_{1} + \theta_{2} \right)}\\0 & 0\\0 & 0\\0 & 0\\1 & 1\end{matrix}\right]
$$

Puedes probar la función con otros manipuladores y verificar que funciona correctamente.

## Cómo calcularla numéricamente con NumPy
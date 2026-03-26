# Maze Solver — Proyecto 2

Comparación de algoritmos de búsqueda (BFS, DFS, Greedy, A*) para resolver laberintos representados como matrices en archivos `.txt`.

---

## Estructura del proyecto

```
Proyecto2/
├── main.py       # Punto de entrada: carga laberintos y ejecuta todos los algoritmos
├── maze.py       # Utilidades: carga de laberinto, clase Node, vecinos, heurísticas
├── bfs.py        # Breadth First Search
├── dfs.py        # Depth First Search
├── greedy.py     # Greedy Best First Search
├── astar.py      # A* Search
└── data/         # Archivos de laberinto (.txt)
```

---

## Formato de los laberintos

Cada archivo `.txt` es una matriz de caracteres donde:

| Valor | Significado |
|-------|-------------|
| `0`   | Celda libre (pasillo) |
| `1`   | Pared (bloqueada) |
| `2`   | Punto de inicio |
| `3`   | Meta (goal) |

El movimiento es en 4 direcciones en orden: **Arriba → Derecha → Abajo → Izquierda**.

Se soportan dos formatos de archivo:
- **Concatenado**: cada fila es una cadena continua de dígitos (`0110210...`)
- **CSV**: valores separados por coma (`0,1,1,0,2,...`)

El formato se detecta automáticamente al leer el archivo.

---

## Cómo ejecutar

```bash
# Correr todos los laberintos en data/
python3 main.py

# Correr uno o más laberintos específicos
python3 main.py data/Prueba_1.txt
python3 main.py data/Laberinto1-1.txt data/test_maze.txt
```

### Ejemplo de salida

```
======================================================================
Maze: test_maze.txt
Size: 64x64  Start: (1, 1)  Goal: (43, 47)
----------------------------------------------------------------------
  BFS                       path=128     nodes=665      time=0.000992s  b*=1.0213
  DFS                       path=184     nodes=514      time=0.000517s  b*=1.0097
  Greedy (Manhattan)        path=128     nodes=240      time=0.000329s  b*=1.0089
  Greedy (Euclidean)        path=132     nodes=402      time=0.000621s  b*=1.0146
  A* (Manhattan)            path=128     nodes=547      time=0.000789s  b*=1.0191
  A* (Euclidean)            path=128     nodes=600      time=0.000876s  b*=1.0201
```

Cuando no existe camino se muestra `NO PATH` junto con el total de nodos explorados.

---

## Descripción de cada módulo

### `maze.py` — Núcleo compartido

Contiene las estructuras y funciones usadas por todos los algoritmos.

**Clase `Node`**

Representa un nodo del árbol de búsqueda:

```python
Node(position, parent, g, h)
```

| Atributo | Descripción |
|----------|-------------|
| `position` | Tupla `(fila, columna)` de la celda |
| `parent`   | Nodo del que proviene (para reconstruir el camino) |
| `g`        | Costo acumulado desde el inicio |
| `h`        | Estimación heurística al goal |
| `f`        | Propiedad calculada: `g + h` (usado por A*) |

`Node` implementa `__eq__`, `__lt__` y `__hash__` para poder usarse en conjuntos y colas de prioridad.

---

**`load_maze(filepath)`**

Lee el archivo `.txt` y devuelve `(grid, start, goal)`:
- `grid`: lista de listas de enteros
- `start`: tupla `(fila, columna)` de la celda con valor `2`
- `goal`: tupla `(fila, columna)` de la celda con valor `3`

---

**`get_neighbors(grid, position)`**

Dado un `(fila, columna)`, devuelve las celdas adyacentes válidas (no paredes, dentro de los límites) en el orden: Arriba, Derecha, Abajo, Izquierda.

---

**`reconstruct_path(node)`**

Traza el camino desde el nodo goal hasta el inicio siguiendo los punteros `parent`. Devuelve la lista de posiciones en orden inicio → meta.

---

**Heurísticas**

```python
manhattan(pos, goal)   # |Δrow| + |Δcol|
euclidean(pos, goal)   # sqrt(Δrow² + Δcol²)
```

---

### `bfs.py` — Breadth First Search

**Garantía**: encuentra siempre el camino de menor cantidad de pasos (óptimo en grafos no ponderados).

**Estructura de datos**: cola FIFO (`deque`).

**Funcionamiento**:
1. Agrega el nodo inicial a la cola y lo marca como visitado.
2. En cada iteración extrae el nodo del frente.
3. Si es el goal, reconstruye y devuelve el camino.
4. Agrega a la cola todos los vecinos no visitados y los marca visitados inmediatamente al encolar (no al expandir).

El marcado al encolar es clave para evitar duplicados y garantizar que el primer camino encontrado sea el más corto.

**Retorna**: `(path, nodes_explored)` — `path=None` si no existe camino.

---

### `dfs.py` — Depth First Search

**Garantía**: encuentra un camino si existe, pero no es necesariamente el más corto.

**Estructura de datos**: pila LIFO (`list` con `append`/`pop`).

**Funcionamiento**:
1. Agrega el nodo inicial a la pila.
2. Extrae el tope de la pila; si ya fue visitado, lo descarta.
3. Marca como visitado y expande sus vecinos, apilándolos si no fueron visitados.
4. Al encontrar el goal, reconstruye el camino.

El marcado al desapilar (en lugar de al apilar) permite que distintos caminos lleguen al mismo nodo, lo que hace a DFS explorar menos nodos totales pero puede generar caminos más largos.

**Retorna**: `(path, nodes_explored)` — `path=None` si no existe camino.

---

### `greedy.py` — Greedy Best First Search

**Garantía**: no garantiza optimalidad. Prioriza nodos que parecen más cercanos al goal según la heurística.

**Estructura de datos**: cola de prioridad (`heapq`), ordenada por `h(n)`.

**Funcionamiento**:
1. Inserta el nodo inicial con prioridad `h(start)`.
2. Extrae siempre el nodo con menor `h` (más "prometedor").
3. Si ya fue visitado, lo descarta (lazy deletion).
4. Expande vecinos no visitados, calcula su `h` y los inserta en el heap.

Usa un contador `tie_breaker` para desempatar nodos con igual `h` de forma estable.

Acepta el parámetro `heuristic='manhattan'` o `'euclidean'`.

**Retorna**: `(path, nodes_explored)` — `path=None` si no existe camino.

---

### `astar.py` — A* Search

**Garantía**: encuentra el camino óptimo siempre que la heurística sea admisible (nunca sobreestima el costo real). Tanto Manhattan como Euclidean son admisibles en este laberinto.

**Estructura de datos**: cola de prioridad (`heapq`), ordenada por `f(n) = g(n) + h(n)`.

**Funcionamiento**:
1. Mantiene un diccionario `best_g` con el menor costo `g` conocido para cada posición.
2. Inserta el nodo inicial con `f = 0 + h(start)`.
3. Al extraer un nodo, si su `g` es mayor que el `best_g` registrado, lo descarta (fue superado por un camino mejor).
4. Expande vecinos: si `new_g < best_g[vecino]`, actualiza `best_g` e inserta al heap.

A diferencia de BFS, A* no usa un conjunto `visited` binario sino `best_g` para permitir re-expansiones cuando se encuentran mejores caminos.

Acepta el parámetro `heuristic='manhattan'` o `'euclidean'`.

**Retorna**: `(path, nodes_explored)` — `path=None` si no existe camino.

---

### `main.py` — Punto de entrada

Orquesta la carga de laberintos y la ejecución de los 6 algoritmos.

**`run_algorithm(name, func, grid, start, goal)`**

Ejecuta un algoritmo, mide el tiempo con `time.perf_counter()`, calcula el branching factor efectivo y muestra los resultados formateados.

**`effective_branching_factor(nodes_explored, path_length)`**

Estima el factor de ramificación efectivo `b*` tal que:

```
1 + b* + b*² + ... + b*^d ≈ nodes_explored
```

donde `d = path_length`. Se resuelve numéricamente con búsqueda binaria en 64 iteraciones. Un `b*` cercano a 1.0 indica que el algoritmo exploró muy pocos nodos extra además del camino.

**`solve_maze(filepath)`**

Carga un laberinto e invoca `run_algorithm` para los 6 algoritmos:
- BFS
- DFS
- Greedy (Manhattan)
- Greedy (Euclidean)
- A* (Manhattan)
- A* (Euclidean)

**`main()`**

- Si se pasan argumentos por línea de comandos, procesa esos archivos.
- Si no, busca todos los `.txt` en el directorio `data/` y los procesa en orden alfabético.

---

## Métricas reportadas

| Métrica | Descripción |
|---------|-------------|
| `path`  | Longitud del camino (número de pasos, sin contar el inicio) |
| `nodes` | Total de nodos expandidos durante la búsqueda |
| `time`  | Tiempo de ejecución en segundos (`perf_counter`) |
| `b*`    | Factor de ramificación efectivo (qué tan selectivo fue el algoritmo) |

---

## Comparación de algoritmos

| Algoritmo | Óptimo | Completo | Memoria | Velocidad típica |
|-----------|--------|----------|---------|-----------------|
| BFS | Sí | Sí | O(b^d) | Moderada |
| DFS | No | Sí* | O(d) | Rápida (pocos nodos) |
| Greedy | No | Sí* | O(b^d) | Muy rápida |
| A* | Sí | Sí | O(b^d) | Moderada–rápida |

*Completo en grafos finitos con manejo de visitados.

- **BFS** es la referencia para el camino óptimo.
- **DFS** usa menos memoria y es rápido, pero el camino puede ser muy largo.
- **Greedy** es el más rápido en explorar nodos, pero no garantiza optimalidad.
- **A*** combina lo mejor: óptimo y más eficiente que BFS gracias a la heurística.

La heurística **Manhattan** suele explorar menos nodos que **Euclidean** en laberintos de cuadrícula porque se ajusta mejor al movimiento en 4 direcciones.

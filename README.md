# Blog

Blog personal construido con Jekyll y publicado en GitHub Pages.

## Publicar desde un notebook de Jupyter

Los artículos tradicionales en `_posts/` siguen funcionando como siempre. Para nuevos artículos con código, ecuaciones y figuras:

1. Instala las dependencias con `python -m pip install -r requirements-notebooks.txt`.
2. Crea un archivo `notebooks/YYYY-MM-DD-slug.ipynb` y escribe el artículo en celdas Markdown y Python.
3. En los metadatos del notebook agrega `post.title` y opcionalmente `post.categories`, `post.tags` (listas de textos) y `post.author`. La fecha se toma del nombre del archivo.
4. Ejecuta las celdas necesarias **y guarda el notebook con sus resultados**.
5. Ejecuta `python scripts/build_posts.py` desde la raíz del repositorio (o pasa una ruta para convertir solamente ese notebook).
6. Revisa y confirma en Git los cambios en `notebooks/`, `_posts/` y `assets/notebooks/`. GitHub Pages publica el Markdown con Jekyll, sin ejecutar Python.

El script convierte usando `nbconvert`, sin ejecutar el notebook, y extrae imágenes de las salidas. Los archivos generados no deben editarse manualmente: modifica el notebook y vuelve a convertir. Para una primera prueba, consulta `notebooks/2026-11-08-newton-raphson.ipynb`.

**Limitación inicial:** los gráficos se publican como imágenes estáticas (PNG o SVG), no como widgets interactivos.

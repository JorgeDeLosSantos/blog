"""Convierte notebooks de Jupyter en posts Markdown para Jekyll.

Uso: python scripts/build_posts.py [notebooks/archivo.ipynb ...]
"""

import argparse
import json
import re
from pathlib import Path

import nbformat
from nbconvert import MarkdownExporter

ROOT = Path(__file__).resolve().parents[1]
NOTEBOOKS = ROOT / "notebooks"
POSTS = ROOT / "_posts"
ASSETS = ROOT / "assets" / "notebooks"
NAME = re.compile(r"^\d{4}-\d{2}-\d{2}-[a-z0-9]+(?:-[a-z0-9]+)*$")


def convert(path):
    path = path.resolve()
    if not path.is_relative_to(NOTEBOOKS.resolve()):
        raise ValueError(f"El archivo debe estar en notebooks/: {path}")
    stem = path.stem
    if not NAME.fullmatch(stem):
        raise ValueError(f"Nombre esperado: YYYY-MM-DD-slug.ipynb: {path.name}")

    notebook = nbformat.read(path, as_version=4)
    post = notebook.metadata.get("post", {})
    title = post.get("title")
    if not isinstance(title, str) or not title.strip():
        raise ValueError(f"Falta metadata.post.title en {path.name}")

    fields = ["---", "layout: post",
              f"title: {json.dumps(title, ensure_ascii=False)}",
              f"date: {json.dumps(stem[:10])}"]
    for name in ("categories", "tags"):
        values = post.get(name, [])
        if not isinstance(values, list) or not all(isinstance(v, str) for v in values):
            raise ValueError(f"metadata.post.{name} debe ser una lista de textos")
        fields.append(f"{name}: {json.dumps(values, ensure_ascii=False)}")
    if post.get("author"):
        fields.append(f"author: {json.dumps(str(post['author']), ensure_ascii=False)}")
    fields.extend(["---", ""])

    relative_dir = f"assets/notebooks/{stem}"
    markdown, resources = MarkdownExporter().from_notebook_node(
        notebook,
        resources={"unique_key": stem, "output_files_dir": relative_dir},
    )

    # Solo usa los outputs guardados; NO ejecuta celdas durante la conversión.
    target_assets = ASSETS / stem
    # No borrar la carpeta: Windows puede impedirlo si tiene archivos abiertos.
    # Solo sobrescribimos los recursos generados; los obsoletos se limpian manualmente.
    outputs = resources.get("outputs", {})
    if outputs:
        target_assets.mkdir(parents=True, exist_ok=True)
    for output_path, data in outputs.items():
        filename = Path(output_path).name
        (target_assets / filename).write_bytes(
            data.encode("utf-8") if isinstance(data, str) else data
        )
        liquid_url = "{{ '/assets/notebooks/" + stem + "/" + filename + "' | relative_url }}"
        markdown = markdown.replace(f"({output_path})", f"({liquid_url})")

    POSTS.mkdir(parents=True, exist_ok=True)
    target = POSTS / f"{stem}.md"
    target.write_text("\n".join(fields) + markdown.rstrip() + "\n", encoding="utf-8")
    print(f"Generado: {target.relative_to(ROOT)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("notebooks", nargs="*", type=Path)
    args = parser.parse_args()
    paths = args.notebooks or sorted(NOTEBOOKS.glob("*.ipynb"))
    if not paths:
        parser.error("No hay notebooks en notebooks/")
    for path in paths:
        convert(path)


if __name__ == "__main__":
    main()

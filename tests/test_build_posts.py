"""Pruebas de la exportación de resultados LaTeX a Jekyll."""

import unittest

import nbformat
from nbconvert import MarkdownExporter

from scripts.build_posts import normalize_latex_outputs


class DisplayMathTests(unittest.TestCase):
    def test_latex_output_uses_display_math(self):
        matrix = r"$\displaystyle \left[\begin{matrix}1 & 2\\3 & 4\end{matrix}\right]$"
        cell = nbformat.v4.new_code_cell(
            "M",
            outputs=[nbformat.v4.new_output(
                output_type="execute_result",
                execution_count=1,
                data={"text/latex": matrix, "text/plain": "Matrix([[1, 2], [3, 4]])"},
            )],
            execution_count=1,
        )
        notebook = nbformat.v4.new_notebook(cells=[
            nbformat.v4.new_markdown_cell("Ecuación inline $x^2$ y bloque $$y^2$$."),
            cell,
        ])
        normalize_latex_outputs(notebook)
        normalize_latex_outputs(notebook)  # Debe ser idempotente.

        latex = notebook.cells[1].outputs[0].data["text/latex"]
        expected = "$$\n" + matrix[1:-1] + "\n$$"
        self.assertEqual(latex, expected)

        markdown, _ = MarkdownExporter().from_notebook_node(notebook)
        self.assertIn(expected, markdown)
        self.assertIn("inline $x^2$", markdown)
        self.assertIn("bloque $$y^2$$", markdown)

    def test_existing_display_math_is_preserved(self):
        source = "$$\nE = mc^2\n$$"
        output = nbformat.v4.new_output(
            output_type="display_data", data={"text/latex": source}
        )
        notebook = nbformat.v4.new_notebook(cells=[
            nbformat.v4.new_code_cell("display(E)", outputs=[output])
        ])
        normalize_latex_outputs(notebook)
        self.assertEqual(notebook.cells[0].outputs[0].data["text/latex"], source)


if __name__ == "__main__":
    unittest.main()

import unittest
from pathlib import Path


CANVAS_JS = Path(__file__).resolve().parents[1] / "web/js/canvas.js"


class CanvasSchemaFormTest(unittest.TestCase):
    def test_execute_uses_schema_types_and_ignores_hidden_action_fields(self):
        source = CANVAS_JS.read_text(encoding="utf-8")
        execute = source[
            source.index("async function _executeCard") :
            source.index("function _showResult")
        ]

        self.assertLess(execute.index("const _schemaProps"), execute.index("const args"))
        self.assertIn("field?.style.display === 'none'", execute)
        self.assertIn("fieldSchema.type === 'number'", execute)
        self.assertIn("fieldSchema.type === 'integer'", execute)


if __name__ == "__main__":
    unittest.main()

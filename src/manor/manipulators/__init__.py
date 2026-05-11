"""
Importing this package triggers registration of every supported manipulator family with the variant + model
registries defined in manipulator_variant.py. Each line below pulls in one family's model module, which
transitively imports the matching variant module via its own internal imports, so the two registry decorators
fire as a side effect of `import manor.manipulators`. See README.md for the rationale and the steps to add
a new family.
"""

from manor.manipulators.lite6 import model as _lite6_model  # noqa: F401

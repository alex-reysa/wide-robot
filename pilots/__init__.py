"""Research pilots that CONSUME the frozen csg verifier from outside the package.

Code here imports ``csg`` as-is (like any third-party user) and is intentionally
NOT part of the distributed ``csg`` package (see ``[tool.setuptools] packages`` in
pyproject.toml). Keeping pilots out of ``csg`` preserves the project's core claim:
a frozen, dependency-free, leakage-audited verifier that was not adapted to fit any
particular external trace source.
"""

"""Compatibility import; implementation lives in the independent spatial-capture package.

Install the vision extra for the pinned dependency. Keeping the actual module
object preserves class identity and existing monkeypatch/import behavior.
"""
import importlib as _importlib
import sys as _sys

_implementation = _importlib.import_module('spatial_capture.reconstruction')
_sys.modules[__name__] = _implementation

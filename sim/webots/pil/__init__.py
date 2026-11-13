"""Host-only Webots PIL models and transport. Hardware qualification is separate.

Extend the package path for explicit, parallel SW4 integration via PYTHONPATH;
canonical S3 source must not be copied into SW5's commits.
"""

from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)

"""Exact URDF asset allowlist for the derived, mock-only WebXR server.

No client path is joined to a filesystem root. The URDF and mesh directory are
operator-owned configuration, never values supplied by the HTTP client.
Requires immutable/read-only registered mesh storage during serving.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET


class AssetPathError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def request_key(value):
    if not isinstance(value, str) or not value or len(value) > 4096:
        raise AssetPathError(403, "Invalid asset path")
    # ASGI already decodes a normal URL once. Reject leftover escapes to prevent
    # double-decoding ambiguity; decoded dot segments are rejected below.
    if any(ord(c) < 32 or ord(c) == 127 for c in value) or any(c in value for c in "\\%?#"):
        raise AssetPathError(403, "Invalid asset path")
    path = value
    if value.startswith("package://"):
        path = value[len("package://"):]
        package = path.split("/", 1)[0]
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", package):
            raise AssetPathError(403, "Invalid package name")
    elif ":" in value:
        raise AssetPathError(403, "Absolute/scheme paths are not served")
    if path.startswith("/") or ":" in path or any(part in ("", ".", "..") for part in path.split("/")):
        raise AssetPathError(403, "Invalid asset path components")
    return value


def within(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


class AssetPolicy:
    """Serve one generated URDF and exactly its referenced mesh/texture files."""
    def __init__(self, urdf_path, mesh_path=None, package_resolver=None):
        self.urdf = Path(urdf_path).resolve(strict=True)
        if not self.urdf.is_file() or self.urdf.stat().st_size > 8 * 1024 * 1024:
            raise ValueError("Expected a bounded generated URDF file")
        self.root = Path(mesh_path).resolve(strict=True) if mesh_path else self.urdf.parent
        if not self.root.is_dir() or self.root == Path(self.root.anchor):
            raise ValueError("A specific immutable asset directory is required")
        if self.root.as_posix() in ("/opt", "/usr", "/home", "/etc", "/var", "/tmp"):
            raise ValueError("Broad filesystem roots cannot be registered")
        self.assets = {}
        self.urdf_document = ET.parse(self.urdf)
        self.uris = []
        for element in self.urdf_document.getroot().iter():
            if element.tag.rsplit("}", 1)[-1] not in ("mesh", "texture"):
                continue
            original = element.get("filename")
            if not original:
                continue
            target = self._configured_asset(original, package_resolver)
            key = target.relative_to(self.root).as_posix()
            request_key(key)
            self._register(key, target)
            # Keep old package URL access only if this exact URI occurred in the
            # generated URDF. Never resolve arbitrary client package names.
            if original.startswith("package://"):
                self._register(request_key(original), target)
            element.set("filename", key)
            self.uris.append({"original": original, "served": key})
        self.content = ET.tostring(self.urdf_document.getroot(), encoding="unicode")

    def _configured_asset(self, original, package_resolver):
        if original.startswith("package://"):
            key = request_key(original)
            package, separator, relative = key[len("package://"):].partition("/")
            if not separator or package_resolver is None:
                raise ValueError("Unresolved package resource in generated URDF")
            candidates = [Path(package_resolver(package)) / relative]
        else:
            candidate = Path(original)
            candidates = [candidate] if candidate.is_absolute() else [self.urdf.parent / candidate, self.root / candidate]
        for candidate in candidates:
            try:
                target = candidate.resolve(strict=True)
            except OSError:
                continue
            if within(target, self.root) and target.is_file():
                return target
        raise ValueError("Generated URDF references a missing/outside registered asset")

    def _register(self, key, target):
        previous = self.assets.get(key)
        if previous is not None and previous != target:
            raise ValueError("Conflicting asset aliases")
        self.assets[key] = target

    def resolve(self, supplied):
        key = request_key(supplied)
        if key == "robot.urdf":
            return None
        target = self.assets.get(key)
        if target is None:
            raise AssetPathError(404, "Asset not registered")
        try:
            current = target.resolve(strict=True)
        except OSError as exc:
            raise AssetPathError(404, "Asset unavailable") from exc
        # A changed symlink/parent cannot extend the frozen exact-file manifest.
        if current != target or not within(current, self.root) or not current.is_file():
            raise AssetPathError(403, "Registered asset boundary changed")
        return current

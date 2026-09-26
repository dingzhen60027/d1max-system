"""Expose only explicitly completed MOLA PCD artifacts, never arbitrary run files."""
import json
from pathlib import Path
import re


def completed_artifacts(root):
    root = Path(root).resolve()
    for manifest_path in root.glob('*/manifest.json'):
        directory = manifest_path.parent
        if directory.is_symlink() or manifest_path.is_symlink() or not re.fullmatch('[a-f0-9]{32}', directory.name):
            continue
        try:
            manifest = json.loads(manifest_path.read_text())
            for entry in manifest.get('artifacts', []):
                filename, role = entry.get('file'), entry.get('role')
                if (filename, role) not in {('raw.pcd', 'mola_frontend'), ('optimized.pcd', 'mola_optimized')}:
                    continue
                path = directory / filename
                if entry.get('status') != 'complete' or path.is_symlink() or not path.is_file():
                    continue
                yield path, role, manifest
        except (OSError, ValueError, AttributeError, TypeError):
            continue

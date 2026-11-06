"""Reuse reviewed removal evidence on original PCD records, without a Z warp.

The conditioned PCD is deliberately never read. A source/evidence-bound index
mask is transferred instead. This is an offline static planning selection, not
semantic person detection or permission to ignore live obstacles.
"""
import json
from pathlib import Path

import numpy as np

from .runner import sha256


def reviewed_source_keep(source, count, evidence, floor_id, settings, support_ids):
    """Validate cached lineage and return a same-length original-record mask."""
    if not settings:
        return np.ones(count, dtype=bool), {"enabled": False, "removed_points": 0}
    if settings.get("schema") != "d1max.reviewed_source_record_cleanup/v1":
        raise ValueError("reviewed_source_record_cleanup_schema_invalid")
    manifest_path = Path(settings["manifest_path"]).resolve(strict=True)
    evidence = Path(evidence).resolve(strict=True)
    if sha256(manifest_path) != settings["manifest_sha256"]:
        raise ValueError("reviewed_cleanup_manifest_changed")
    if sha256(evidence) != settings["evidence_sha256"]:
        raise ValueError("reviewed_cleanup_evidence_changed")
    manifest = json.loads(manifest_path.read_text())
    if (manifest.get("status") != "complete"
            or manifest.get("processing_schema") != "d1max.multifloor_planning/v1"
            or Path(manifest["source_path"]).resolve() != Path(source).resolve()
            or manifest["source_sha256"] != sha256(source)
            or manifest["input_points"] != count
            or evidence != manifest_path.parent / "audit/floor_evidence.npz"):
        raise ValueError("reviewed_cleanup_source_lineage_mismatch")
    with np.load(evidence, allow_pickle=False) as archive:
        removed = archive["removed_indices"]
        floor_ids = archive[floor_id + "_source_indices"]
        protected = archive["protected_stairs"]
        for ids in (removed, floor_ids, np.asarray(support_ids)):
            if (ids.ndim != 1 or ids.dtype.kind not in "iu"
                    or np.any(ids < 0) or np.any(ids >= count)
                    or len(np.unique(ids)) != len(ids)):
                raise ValueError("reviewed_cleanup_source_indices_invalid")
        if (len(removed) != manifest["removed_points"]
                or protected.shape != (count,) or protected.dtype.kind != "b"
                or np.any(protected[removed])):
            raise ValueError("reviewed_cleanup_protected_lineage_invalid")
    # Do not transfer classifications from another floor. Existing measured
    # support remains present; it is not silently re-estimated or flattened.
    selected = np.intersect1d(removed, floor_ids, assume_unique=True)
    retained_support = np.intersect1d(selected, support_ids, assume_unique=True)
    selected = np.setdiff1d(selected, support_ids, assume_unique=True)
    maximum_fraction = float(settings["maximum_removed_fraction"])
    if not 0 < maximum_fraction < .10 or len(selected) / count > maximum_fraction:
        raise ValueError("reviewed_cleanup_removal_budget_exceeded")
    keep = np.ones(count, dtype=bool)
    keep[selected] = False
    return keep, {"enabled": True, "manifest_path": str(manifest_path),
        "manifest_sha256": settings["manifest_sha256"],
        "evidence_sha256": settings["evidence_sha256"],
        "reviewed_all_floor_removed_points": len(removed),
        "protected_observed_support_points": len(retained_support),
        "removed_points": len(selected), "floor_id": floor_id,
        "source_xyz_modified": False, "synthetic_points": 0,
        "trajectory_tube_clearing": False,
        "semantics": "Reviewed unsupported low geometry selection; not semantic people or live safety"}

"""Offline, non-destructive point-cloud preprocessing utilities."""


def load_config(path):
    """YAML config with ${D1MAX_*} roots expanded by the shared path layer."""
    from pathlib import Path
    import yaml
    from d1max_pct_planner.paths import expand_tree
    return expand_tree(yaml.safe_load(Path(path).read_text()))

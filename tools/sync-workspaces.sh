#!/usr/bin/env bash
# Copy reviewed source into this publication repo; never deploy or delete files.
set -euo pipefail

usage() {
  printf '%s\n' \
    'Usage: bash tools/sync-workspaces.sh --app-root PATH --nav-root PATH [--apply]' \
    'Default: checksum dry run. --apply backs up overwritten files under /tmp.' \
    'No deletion, service operation, Git staging, commit or push is performed.'
}

app_root=''
nav_root=''
apply=false
while (($#)); do
  case "$1" in
    --app-root|--nav-root)
      (($# >= 2)) || { usage >&2; exit 2; }
      if [[ "$1" == --app-root ]]; then app_root="$2"; else nav_root="$2"; fi
      shift 2
      ;;
    --apply) apply=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ -n "$app_root" && -n "$nav_root" ]] || { usage >&2; exit 2; }
command -v rsync >/dev/null
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(git -C "$script_dir" rev-parse --show-toplevel)"
app_root="$(realpath -e -- "$app_root")"
nav_root="$(realpath -e -- "$nav_root")"
[[ "$script_dir" == "$repo_root/tools" && -f "$repo_root/SOURCE_SNAPSHOT.md" ]] || {
  printf '%s\n' 'Refusing an unexpected publication destination.' >&2
  exit 2
}
for source_root in "$app_root" "$nav_root"; do
  if [[ "$source_root" == / || "$source_root" == "$repo_root" ||
        "$source_root" == "$repo_root/"* || "$repo_root" == "$source_root/"* ]]; then
    printf '%s\n' 'Source and publication trees must be separate, non-root directories.' >&2
    exit 2
  fi
done
[[ -d "$app_root/d1max_ros2/map_manager/backend" &&
   -d "$app_root/d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge" &&
   -d "$nav_root/src/d1max_localization" && -d "$nav_root/src/faster_lio" ]] || {
  printf '%s\n' 'Source workspace markers are missing.' >&2
  exit 2
}
for entry in start_d1max_communication.sh start_d1max_map_manager.sh; do
  [[ -f "$app_root/$entry" && ! -L "$app_root/$entry" ]] || {
    printf 'Missing or symbolic-link entry: %s\n' "$entry" >&2
    exit 2
  }
done
for destination in "$repo_root/d1max_ros2" "$repo_root/d1max_nav_ws"; do
  [[ ! -L "$destination" ]] || {
    printf 'Refusing a symbolic-link destination: %s\n' "$destination" >&2
    exit 2
  }
done

args=(-rlpt --checksum --itemize-changes --omit-dir-times --safe-links
      --prune-empty-dirs --filter="merge $script_dir/snapshot.rsync-filter")
if "$apply"; then
  backup_root="$(mktemp -d /tmp/d1max-source-backup-XXXXXXXX)"
  printf 'Backup of overwritten publication files: %s\n' "$backup_root"
  rsync "${args[@]}" --backup --backup-dir="$backup_root/d1max_ros2" \
    "$app_root/d1max_ros2/" "$repo_root/d1max_ros2/"
  rsync "${args[@]}" --backup --backup-dir="$backup_root/d1max_nav_ws" \
    "$nav_root/" "$repo_root/d1max_nav_ws/"
  rsync "${args[@]}" --backup --backup-dir="$backup_root/entrypoints" \
    "$app_root/start_d1max_communication.sh" "$app_root/start_d1max_map_manager.sh" "$repo_root/"
else
  rsync "${args[@]}" --dry-run "$app_root/d1max_ros2/" "$repo_root/d1max_ros2/"
  rsync "${args[@]}" --dry-run "$nav_root/" "$repo_root/d1max_nav_ws/"
  rsync "${args[@]}" --dry-run \
    "$app_root/start_d1max_communication.sh" "$app_root/start_d1max_map_manager.sh" "$repo_root/"
fi
printf '%s\n' 'Source-only sync finished. Destination-only files were retained; review them separately.'

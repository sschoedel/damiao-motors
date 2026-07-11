#!/usr/bin/env bash
# Continuously mirror this git repo to an SSH target.
#
# The local working tree is ground truth: files deleted locally are deleted
# on the target, and remote edits are overwritten on the next sync. Files
# ignored by .gitignore (and the .git directory itself) are not synced.
#
# The repo lands in ~/<repo_dir_name>_<local_user> on the target, e.g.
# running from ~/git/damiao-motors as user sam syncs to ~/damiao-motors_sam.
#
# Usage:
#   ./sync_to_robot <ssh-target> [interval-seconds]
#   ./sync_to_robot pineapplebot@pineapplebot.local
#   ./sync_to_robot pineapplebot@10.0.0.197 2

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $(basename "$0") <ssh-target> [interval-seconds]" >&2
    exit 1
fi

TARGET="$1"
INTERVAL="${2:-1}"

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || {
    echo "error: not inside a git repository" >&2
    exit 1
}

REMOTE_DIR="$(basename "$REPO_ROOT")_$(whoami)"

echo "syncing $REPO_ROOT -> $TARGET:~/$REMOTE_DIR every ${INTERVAL}s"
echo "local tree is ground truth (remote changes will be overwritten)"
echo "press ctrl-c to stop"

while true; do
    # --itemize-changes prints one line per changed file; empty output = no-op
    if CHANGES="$(rsync -az --delete --itemize-changes \
        --exclude ".git" \
        --filter ":- .gitignore" \
        "$REPO_ROOT/" "$TARGET:$REMOTE_DIR/")"; then
        if [[ -n "$CHANGES" ]]; then
            printf "synced %s\n%s\n" "$(date +%H:%M:%S)" "$CHANGES"
        fi
    else
        echo "rsync failed — retrying in ${INTERVAL}s" >&2
    fi
    sleep "$INTERVAL"
done

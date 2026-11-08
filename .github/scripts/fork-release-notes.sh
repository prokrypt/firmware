#!/usr/bin/env bash
# prokrypt fork only: prints Markdown release notes for <ref> (default HEAD): fork changes and
# upstream commits since the previous published fork release that <ref> contains.
# Needs full history, upstream develop at <upstream>, and GH_TOKEN for `gh release list`.
set -euo pipefail
ref=${1:-HEAD}
upstream=${2:-upstream/develop}
# Take the nearest such release, not the first listed: fork tags all read 2.8.2.<hash>, so list order isn't age order.
prev=""
prev_sha=""
for tag in $(gh release list --exclude-drafts --limit 100 --json tagName -q '.[].tagName'); do
  sha=$(git rev-parse -q --verify "refs/tags/$tag^{commit}") || continue
  [ "$sha" != "$(git rev-parse "$ref^{commit}")" ] || continue
  git merge-base --is-ancestor "$sha" "$ref" || continue
  if [ -z "$prev_sha" ] || git merge-base --is-ancestor "$prev_sha" "$sha"; then
    prev=$tag
    prev_sha=$sha
  fi
done
range=${prev:+$prev..}$ref
base=$(git merge-base "$ref" "$upstream")

echo "Fork build of no-auto-writes at $(git rev-parse --short=7 "$ref")${prev:+, changes since $prev}."
echo
echo "## Fork changes"
fork=$(git log --no-merges --format='- %s (%h)' "$range" --not "$upstream")
echo "${fork:-- None}"
echo
echo "## Upstream changes"
if [ -z "$prev" ]; then
  echo "- Based on upstream develop at $(git rev-parse --short=7 "$base")"
  exit 0
fi
up=$(git log --no-merges --format='- %s (%h)' "$prev..$base" \
  | sed -E 's/\(#([0-9]+)\)/(meshtastic\/firmware#\1)/g')
echo "${up:-- None}"

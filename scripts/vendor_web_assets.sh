#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="$ROOT_DIR/assets/vendor"
CM_VER="5.65.20"
PY_VER="0.25.1"

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

need_cmd curl
need_cmd tar

mkdir -p "$ASSET_DIR/codemirror/$CM_VER/lib"
mkdir -p "$ASSET_DIR/codemirror/$CM_VER/theme"
mkdir -p "$ASSET_DIR/codemirror/$CM_VER/mode/python"
mkdir -p "$ASSET_DIR/codemirror/$CM_VER/addon/edit"
mkdir -p "$ASSET_DIR/codemirror/$CM_VER/addon/selection"
mkdir -p "$ASSET_DIR/pyodide/v$PY_VER/full"

fetch() {
  local url="$1"
  local out="$2"
  echo "Downloading $url"
  curl -fL --retry 3 --connect-timeout 15 "$url" -o "$out"
}

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

# CodeMirror assets used by index.html
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/lib/codemirror.min.css" \
  "$ASSET_DIR/codemirror/$CM_VER/lib/codemirror.min.css"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/theme/material-darker.min.css" \
  "$ASSET_DIR/codemirror/$CM_VER/theme/material-darker.min.css"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/lib/codemirror.min.js" \
  "$ASSET_DIR/codemirror/$CM_VER/lib/codemirror.min.js"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/mode/python/python.min.js" \
  "$ASSET_DIR/codemirror/$CM_VER/mode/python/python.min.js"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/addon/edit/matchbrackets.min.js" \
  "$ASSET_DIR/codemirror/$CM_VER/addon/edit/matchbrackets.min.js"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/addon/edit/closebrackets.min.js" \
  "$ASSET_DIR/codemirror/$CM_VER/addon/edit/closebrackets.min.js"
fetch "https://cdn.jsdelivr.net/npm/codemirror@$CM_VER/addon/selection/active-line.min.js" \
  "$ASSET_DIR/codemirror/$CM_VER/addon/selection/active-line.min.js"

# Pyodide full runtime
PY_ARCHIVE="$TMP_DIR/pyodide-$PY_VER.tar.bz2"
fetch "https://github.com/pyodide/pyodide/releases/download/$PY_VER/pyodide-$PY_VER.tar.bz2" "$PY_ARCHIVE"

EXTRACT_DIR="$TMP_DIR/pyodide-extract"
mkdir -p "$EXTRACT_DIR"
tar -xjf "$PY_ARCHIVE" -C "$EXTRACT_DIR"

PY_SRC_DIR="$EXTRACT_DIR"
if [ -d "$EXTRACT_DIR/pyodide" ]; then
  PY_SRC_DIR="$EXTRACT_DIR/pyodide"
fi

rm -rf "$ASSET_DIR/pyodide/v$PY_VER/full"
mkdir -p "$ASSET_DIR/pyodide/v$PY_VER/full"
cp -a "$PY_SRC_DIR/." "$ASSET_DIR/pyodide/v$PY_VER/full/"

echo "Vendored assets ready in $ASSET_DIR"
echo "You can now serve index.html without CDN dependencies."

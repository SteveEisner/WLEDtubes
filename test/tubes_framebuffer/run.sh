#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$test_dir/../.." && pwd)"
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/wled-tubes-framebuffer-tests.XXXXXX")"
trap 'rm -rf "$build_dir"' EXIT

"${CXX:-c++}" \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -pedantic \
  -I"$repo_dir/wled00" \
  "$test_dir/framebuffer_count_test.cpp" \
  -o "$build_dir/framebuffer_count_test"

"$build_dir/framebuffer_count_test"

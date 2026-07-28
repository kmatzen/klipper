#!/bin/bash
# Empirical byte-identical proof for a tick-mode test (TICK_PROTOCOL_DESIGN.md
# §5.2). Runs the named test N times under the deterministic tick path with
# KLIPPY_TICK_TRACE enabled, then verifies every R-trace (klippy) and B-trace
# (bridge) is byte-identical across runs. Determinism for that test is proven
# iff every R-trace hashes the same and every B-trace hashes the same.
#
#   ./test/emulator/proof.sh <test-basename> [N]
#
# Example: ./test/emulator/proof.sh temperature 3
#
# Designed to run inside the scripts/Dockerfile.emulator-test image (which has
# the simavr bridge + dicts + ELFs). The runner is pinned to PYTHONHASHSEED=0
# so set / dict iteration is bit-stable across processes.
#
# Exit code:
#   0  - PASS: all R-traces byte-identical AND all B-traces byte-identical
#   1  - FAIL: at least one trace diverged (first divergent seq is logged)
#   2  - usage / setup error

set -u

usage() {
    cat >&2 <<EOF
usage: $0 <test-basename> [N]
    <test-basename>  e.g. 'temperature', 'load_cell' (without .test suffix)
    [N]              run count, default 3 (minimum 2)
EOF
    exit 2
}

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    usage
fi

TEST_BASE=$1
N=${2:-3}
if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -lt 2 ]; then
    echo "error: N must be an integer >= 2" >&2
    exit 2
fi

TEST_PATH="test/klippy/${TEST_BASE}.test"
if [ ! -f "$TEST_PATH" ]; then
    echo "error: $TEST_PATH not found" >&2
    exit 2
fi

# Resolve a python interpreter and the dict directory the runner expects.
PYTHON=${PYTHON:-}
if [ -z "$PYTHON" ]; then
    for cand in /venv/bin/python python3 python; do
        if command -v "$cand" >/dev/null 2>&1; then
            PYTHON=$cand
            break
        fi
    done
fi
if [ -z "$PYTHON" ]; then
    echo "error: no python interpreter found (set \$PYTHON)" >&2
    exit 2
fi
DICTDIR=${DICTDIR:-ci_build/dict}
if [ ! -d "$DICTDIR" ]; then
    echo "error: dict directory '$DICTDIR' not found (set \$DICTDIR)" >&2
    exit 2
fi

TRACE_DIR=$(mktemp -d)
trap 'rm -rf "$TRACE_DIR"' EXIT

echo "==> proof: $TEST_BASE (N=$N runs)"
for i in $(seq 1 "$N"); do
    run_trace="$TRACE_DIR/run${i}"
    echo "    run $i/$N -> $run_trace.*"
    PYTHONHASHSEED=0 KLIPPY_TICK_TRACE="$run_trace" \
        "$PYTHON" scripts/test_klippy.py -d "$DICTDIR" \
        --force-emulator "$TEST_PATH" \
        > "$TRACE_DIR/run${i}.runlog" 2>&1
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "FAIL: $TEST_BASE run $i exited rc=$rc" >&2
        tail -20 "$TRACE_DIR/run${i}.runlog" >&2
        exit 1
    fi
done

# Each tick socket produces one bridge trace file named
# "${KLIPPY_TICK_TRACE}.<socket-basename>" plus the klippy-side ".klippy". The
# socket basename is stable across runs (the runner names sockets after the
# test fixture), so the file *set* per run is the same; we compare set-equal
# and hash-equal.
all_pass=1
reference_files=""
declare -A reference_hash=()
for i in $(seq 1 "$N"); do
    run_trace="$TRACE_DIR/run${i}"
    files=$(cd "$TRACE_DIR" && ls "run${i}".* 2>/dev/null \
            | grep -v "\.runlog$" \
            | sed -e "s|^run${i}\.||" | sort)
    if [ "$i" -eq 1 ]; then
        reference_files=$files
        if [ -z "$reference_files" ]; then
            echo "FAIL: run 1 produced no trace files (is tick mode active?)" >&2
            exit 1
        fi
        for suffix in $reference_files; do
            reference_hash[$suffix]=$(sha256sum "$run_trace.$suffix" \
                                      | awk '{print $1}')
            echo "    reference $suffix -> ${reference_hash[$suffix]:0:16}..."
        done
        continue
    fi
    if [ "$files" != "$reference_files" ]; then
        echo "FAIL: run $i trace file set differs from run 1" >&2
        echo "  run 1 : $reference_files" >&2
        echo "  run $i : $files" >&2
        all_pass=0
        continue
    fi
    for suffix in $files; do
        h=$(sha256sum "$run_trace.$suffix" | awk '{print $1}')
        if [ "$h" != "${reference_hash[$suffix]}" ]; then
            echo "FAIL: run $i $suffix diverged from run 1" >&2
            first_diff=$(diff "$TRACE_DIR/run1.$suffix" "$run_trace.$suffix" \
                         | head -10)
            echo "  first diff:" >&2
            echo "$first_diff" | sed 's/^/    /' >&2
            all_pass=0
        fi
    done
done

if [ "$all_pass" -eq 1 ]; then
    echo "PASS: $TEST_BASE byte-identical across $N runs"
    exit 0
fi
exit 1

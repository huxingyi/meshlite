#!/bin/sh

set -ex

# Profile an example target
#
# Dtrace doesn't appear to work reliably, at leasts not on my machine. It will
# silently fail to output anything for some seemingly random builds or periods
# of time. I will stop using macOS for profiling from now on, feel free to
# delete this script.
# -------------------------
EXAMPLE='benchmark_triangulate'
DTRACE_FILE='out.stacks'
sudo rm -f ${DTRACE_FILE}
cargo build --release --example ${EXAMPLE}
sudo dtrace \
    -c "./target/release/examples/${EXAMPLE}" \
    -o ${DTRACE_FILE} \
    -n 'profile-997 /execname == "'${EXAMPLE}'" / { @[ustack(100)] = count(); }'
sudo chown ${USER} ${DTRACE_FILE}
sudo chown -R ${USER} target
../FlameGraph/stackcollapse.pl ${DTRACE_FILE} | ../FlameGraph/flamegraph.pl > flame.svg

# A different example
# --------------------------
# EXE_NAME='subdivide_3_high-a0eae922b4b74767'
# DTRACE_FILE='out.stacks'
# sudo rm -f ${DTRACE_FILE}
# target/release/$EXE_NAME --measure-only &
# BENCH_PID=$!
# sudo dtrace \
# 	-p ${BENCH_PID} \
# 	-o ${DTRACE_FILE} \
# 	-n "profile-997 /pid == ${BENCH_PID} / { @[ustack(100)] = count(); }"
# sudo chown $USER out.stacks
# sudo chown -R $USER target
# ../FlameGraph/stackcollapse.pl out.stacks | ../FlameGraph/flamegraph.pl > flame.svg

# Based on
# --------
# Based on http://carol-nichols.com/2017/04/20/rust-profiling-with-dtrace-on-osx/
# Make sure to uncomment the debug flag under [profile.release] in Cargo.toml.
# Needs https://github.com/brendangregg/FlameGraph at ../FlameGraph/


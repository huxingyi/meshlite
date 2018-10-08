#!/bin/bash

# Based on https://gist.github.com/KodrAus/97c92c07a90b1fdd6853654357fd557a
# Make sure to uncomment the debug flag under [profile.release] in Cargo.toml.
# Needs https://github.com/brendangregg/FlameGraph at ../FlameGraph/
cargo build --release --example subdiv_benchmark
sudo perf record --call-graph dwarf -F 10000 target/release/examples/subdiv_benchmark
sudo chown $USER perf.data
perf script | ../FlameGraph/stackcollapse-perf.pl | ../FlameGraph/flamegraph.pl > flame.svg


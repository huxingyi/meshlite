#!/bin/bash

# Based on https://gist.github.com/KodrAus/97c92c07a90b1fdd6853654357fd557a
sudo perf record --call-graph dwarf target/release/examples/subdiv_benchmark
sudo chown $USER perf.data
perf script | ../FlameGraph/stackcollapse-perf.pl | ../FlameGraph/flamegraph.pl > flame.svg


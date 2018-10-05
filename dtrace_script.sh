#!/bin/sh

# Based on http://carol-nichols.com/2017/04/20/rust-profiling-with-dtrace-on-osx/
# Make sure to uncomment the debug flag under [profile.release] in Cargo.toml.
# Needs https://github.com/brendangregg/FlameGraph at ../FlameGraph/
sudo dtrace -c './target/release/examples/subdiv_benchmark' -o out.stacks -n 'profile-997 /execname == "subdiv_benchmark"/ { @[ustack(100)] = count(); }'
../FlameGraph/stackcollapse.pl out.stacks | ../FlameGraph/flamegraph.pl > pretty-graph.svg


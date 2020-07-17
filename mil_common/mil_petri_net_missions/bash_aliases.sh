#!/bin/bash

PETRI_NET_INTERPRETER_DIR="$(realpath $(dirname $BASH_SOURCE)/petri_net_interpreter)"

alias render_graph="$PETRI_NET_INTERPRETER_DIR/render_graph"

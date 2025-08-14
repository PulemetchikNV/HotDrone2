#!/usr/bin/env bash
nix-shell drone/chess/shell.nix --run "python3 test_distributed.py --mode distributed"

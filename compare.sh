#!/bin/bash

compareLength=${1:-16}

# Build and Generate Log File:
make
bin/nesblue | head -n $compareLength > blue.log

# Diff:
diff -p -w <(head -n${compareLength} blue.log) <(head -n${compareLength} nestest.log)
#diff -p -w <(head -n${compareLength} blue.log | grep -v NOP) <(head -n${compareLength} nestest.log | grep -v NOP)

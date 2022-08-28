#!/bin/bash

rm example.dot example.dot.png
textx generate example.pblg --grammar dsl_grammar.tx --target dot
dot -Tpng -O example.dot
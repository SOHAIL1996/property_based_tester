#!/bin/bash

rm test_definitions.dot test_definitions.dot.png
textx generate test_definitions.pblg --grammar dsl_grammar.tx --target dot
dot -Tpng -O test_definitions.dot
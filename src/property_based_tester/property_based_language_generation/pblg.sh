#!/bin/bash

rm test_definitions.dot test_definitions.dot.png dsl_grammar.dot
textx generate test_definitions.pblg --grammar dsl_grammar.tx --target dot
textx generate dsl_grammar.tx --target dot
dot -Tpng -O test_definitions.dot
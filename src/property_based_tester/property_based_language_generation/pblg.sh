#!/bin/bash

rm standard_test_definitions.dot standard_test_definitions.dot.png
textx generate standard_test_definitions.pblg --grammar rules.tx --target dot
dot -Tpng -O standard_test_definitions.dot
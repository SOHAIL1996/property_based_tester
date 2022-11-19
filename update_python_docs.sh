#!/bin/bash

sphinx-apidoc -o sphinx/ src

cd sphinx
make clean 
make html
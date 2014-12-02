#!/bin/bash
mkdir -p include
libtoolize -i
aclocal -I m4
autoreconf -fi

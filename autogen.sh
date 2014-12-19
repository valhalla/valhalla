#!/bin/bash
libtoolize -i
aclocal -I m4
autoreconf -fi --warning=no-portability

#!/usr/bin/env bash
# Script for shared dependencies

set -x -o errexit -o pipefail -o nounset


# Lets start with setting us up for installing the non-trusted ppa of primeserver
apt-get update --assume-yes
apt-get install --assume-yes \
    gnupg2

# We're manually adding the sources since the ppa has no 20.04 package yet
cat << EOF > /etc/apt/sources.list.d/foo.list
deb http://ppa.launchpad.net/valhalla-core/valhalla/ubuntu bionic main
deb-src http://ppa.launchpad.net/valhalla-core/valhalla/ubuntu bionic main
EOF

# Also adding the signing key to apt keyring
cat << EOF | apt-key add -
-----BEGIN PGP PUBLIC KEY BLOCK-----

xsFNBFkQzlUBEADEump7EBiDI8a9M9tpEUL+QNP1BNVWR3g8P4WsfjngaPS9jn1Q
KKoR8AOOwRDoZK7iUkV9SzNw8GqQ4alHbssCxEGd74hnWUwc7c/7m0Nk14Lm0XCS
OXKTxXKXOfkMdrRxyxRDpzrZFeMDGqVrU0qo/1wr1dCh5BJwjx0mv7tVp9/08/qe
ki2OOtycZi6OrAJl+EmcEo+HdvRYGuX/OmIWmkOTj1/lqoHVmH/0crGboKbdWqnB
ug4bCg+U5sra/y+EHHP/LWbeYPpLV4dSEHnc683JNhOhF/sNntdXZSt0Ka//+axV
zRBR/OSeg+bjPrTmxiP4Ht8lQ1eLBjRfvvAabobWyHb1dYz2NDAvN8zoI4O6r3De
4mOeeTozIP3AvGIVy2oH0Gos1wcp/k2t2pSnEgSyY8PZt9HhDJ8ZCgtGozeXSqc1
uJvfYH2kgAGCkJh9edKa/uvwHrg4ugZbif1zc3ZWzCwVoL+urvsswxcE1KzQ7/el
w4SEfzAQN/SH3gYi5KKCrVtW7OAN4RC8yV4KfbQa9GUsThrPFHLhMbhXDUHbAlby
tVw3v0KKru+mUxEykDTx+JpWeiQVk82caiQml2UCfPOchhvdjQToG7niykv4yWnA
LVGZBVlxX+1cGpMesUZ+/Q6J50kuHk3j07amUYrzB90mojNVjyk31BAupwARAQAB
zRpMYXVuY2hwYWQgUFBBIGZvciB2YWxoYWxsYcLBeAQTAQIAIgUCWRDOVQIbAwYL
CQgHAwIGFQgCCQoLBBYCAwECHgECF4AACgkQqgeTI759x6m6hg//bWZ+6U7TGpBI
3SGnqQLz21rns7eQIDwy7WPI+l+4r1Mjn/7w+fpH+lyEMiFMvqykPJgSvmK5Daxq
QAeNHtHM/ouwa1TBhmhbWebA4n/kpPckuDG/0Bmqrso4CGjIAQi+WqhhD5cEhAYn
xfNugGD/iA81JUvlU5wYZew9SrDrRP9MNKgJBLhQhrCIJ8QUVGHVtOG72EEks4Ic
lfK6nW/iF61vO13tahxD/TCPxgTHIXl7lNBLSHzed/20lt1BCvlOm+jTezJ3DBWr
zcL2r1EyJi1aAzstLpBqL+LHoaeVUsAF6gDqKq+RARBjwRuIPK61sSpU/FTNloAm
mPI8xPv7Hrnaf0/a1eP9g/a25+/lnkEIU7/Nt2SBevNZepeABy1kWjYPNQQtC70H
WFRGNxrUwAM5Eq23IkLs++ev9X0WdrcQyykJ7zwZwXLmwjVPOHiciBmK8tWJO977
S2JCaMq3zLplmexROikVJ7PrVf8mU+uGj/p6jNGSnIaMrR303pmPyfT1+iol785/
y84P4wxRJsWgp+PGD1si462XYIiuFJZ/VZpNA0QOEBL9Wn1/yhpm/g4ItnPt192Z
Uf9D8rmVMoZyp1CtISGDIS7MOZFZPLF4qIRFv/gvbIIzkbe5Z80GLGHhoCC5aCq9
g8xu9wygPm48Sa6bb46JzW+FkUltd98=
=ym+M
-----END PGP PUBLIC KEY BLOCK-----
EOF

# Now, go through and install the build dependencies
apt-get update --assume-yes
env DEBIAN_FRONTEND=noninteractive apt-get install --yes --quiet \
    autoconf \
    automake \
    ccache \
    clang \
    clang-tidy \
    coreutils \
    curl \
    cmake \
    g++ \
    gcc \
    git \
    jq \
    lcov \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libgeos++-dev \
    libgeos-dev \
    libluajit-5.1-dev \
    liblz4-dev \
    libprime-server0.6.5-dev \
    libprotobuf-dev \
    libspatialite-dev \
    libsqlite3-dev \
    libsqlite3-mod-spatialite \
    libtool \
    locales \
    luajit \
    make \
    osmium-tool \
    parallel \
    pkg-config \
    prime-server0.6.5-bin \
    protobuf-compiler \
    python-all-dev \
    python3-all-dev \
    python3-minimal \
    spatialite-bin \
    unzip \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*

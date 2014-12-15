# Valhalla Library #

## Motivation ##

Valhalla is a collection of libraries and binaries (named after Norse mythological figures) created with the intetion of providing a flexible opeen street map based routing infrastructure. We endeavor to do much more than this describes but we'll flesh out the docs to explain each piece as the project matures. 

## Building ##

Valhalla does have several dependencies that you must have before configure will properly produce a Makefile. It is true that many of these dependencies are not actually needed to build the valhalla code however we left the dependencies in because we forsee having to use them at some point or another of the course of the projects development.

## Testing ##

Tests can be found in the `tests/` directory, and are run by autotools when you run `make check`. If you are preparing a pull request or patch, please make sure these pass before submitting it.

There is a test coverage tool built into the build system, but it's fairly annoying to use. Please see
[the coverage docs](test_coverage.md) for more information and details on running it.

## Components ##

### Mjolnir ###

A command line tool for cutting route tiles directly from [OSM Pbf](http://wiki.openstreetmap.org/wiki/PBF_Format) or from an [osm2pgsql] (http://wiki.openstreetmap.org/wiki/Osm2pgsql) import (which is useful for changeset/diff application).

#### osm2pgsql ####

To import a small dataset such as that of Liechtenstein you'll first need to install osm2pgsql like so:

    #get deps, compile, test, and install osm2pgsql. you only need to do this once
    sudo apt-get install postgresql-9.3-postgis-2.1 pgadmin3 postgresql-contrib libbz2-dev libgeos++-dev libproj-dev libpq-dev liblua5.2 liblua5.2-dev libprotobuf-c0-dev protobuf-c-compiler python-pip
    sudo pip install psycopg2
    git clone https://github.com/openstreetmap/osm2pgsql.git
    cd osm2pgsql
    ./autogen.sh
    ./configure
    sudo -u postgres createuser -s $USER 
    sudo mkdir -p /tmp/psql-tablespace
    sudo chown postgres.postgres /tmp/psql-tablespace
    psql -c "CREATE TABLESPACE tablespacetest LOCATION '/tmp/psql-tablespace'" postgres
    make test
    sudo make install

After you've gotten osm2pgsql installed and ready to use you can do an import of whatever dataset by doing something like this:

    #create sample routing database from the mjolnir top level dir
    createdb gis
    psql -d gis -c 'CREATE EXTENSION postgis; CREATE EXTENSION hstore;'
    cd import/osm2pgsql
    osm2pgsql --create --latlong --slim --style route.json --database gis --output multi ../../test/data/liechtenstein-latest.osm.pbf

Note that for applying diff's you won't need `createdb`, `psql` or the `--create` bits.

### Util ###

The Valhalla library comes with some examples of standard utilities that may or may not be applicable for a given project. They are mostly just intented as an example of what you might use to build a library within autotools. 

## Tests ##
Valhalla comes with a test suite, run with ``make check``. If you find a platform where the tests aren't passing [open an issue](https://github.com/mapzen/valhalla/issues/new).

FROM ubuntu:16.04
MAINTAINER Grant Heffernan <grant@mapzen.com>

ENV TERM vt100

ENV DATA_DIR ${DATA_DIR:-"/data/valhalla"}
ENV CONF_FILE ${CONF_FILE:-"/conf/valhalla.json"}

RUN apt-get update -y
RUN apt-get install \
      wget \
      osmosis \
      osmctools \
      pigz \
      parallel \
      awscli \
      supervisor \
      software-properties-common \
      -y

ADD ./scripts /scripts
RUN /scripts/install_from_source.sh

ADD ./conf /conf

RUN valhalla_build_config \
      --mjolnir-tile-dir ${DATA_DIR} \
      >${CONF_FILE}

RUN apt-get clean && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

EXPOSE 8002
CMD ["/scripts/start_valhalla.sh"]

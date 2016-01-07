CC=g++

LIB_PATH=/usr/local/lib
FLAGS_TEST=-std=c++11 -Wl,-rpath=$(LIB_PATH) -Wall
ifdef DEBUG
	FLAGS=-std=c++11 -Wl,-rpath=$(LIB_PATH) -Wall -g
else
	FLAGS=-std=c++11 -Wl,-rpath=$(LIB_PATH) -Wall -O3 -DNDEBUG
endif
LIBS=-lvalhalla_midgard -lvalhalla_baldr -lvalhalla_sif


.PHONY: all
all: commands tests


.PHONY: commands
commands: simple_matcher edge_search service psqlmatcher attacher stat


.PHONY: tests
tests: test_queue test_viterbi_search test_grid_range_query test_sp


.PHONY: run_tests
run_tests: tests
	./test_queue
	./test_viterbi_search
	./test_grid_range_query
	./test_sp


stat: stat.cc
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lsqlite3


attacher: attacher.cc map_matching.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lsqlite3


psqlmatcher: psqlmatcher.cc simple_matcher
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lpqxx -lpq -lgeos -lsqlite3


service: service.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lprime_server -lpthread


edge_search: edge_search.cc edge_search.h candidate.h grid_range_query.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


simple_matcher: simple_matcher.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


test_sp: test_sp.cc sp.h
	$(CC) $(FLAGS_TEST) $< -o $@ -lvalhalla_midgard -lvalhalla_baldr -lvalhalla_sif


test_grid_range_query: test_grid_range_query.cc grid_range_query.h
	$(CC) $(FLAGS_TEST) $< -o $@ -lvalhalla_midgard


test_queue: test_queue.cc queue.h
	$(CC) $(FLAGS_TEST) $< -o $@


test_viterbi_search: test_viterbi_search.cc viterbi_search.h queue.h
	$(CC) $(FLAGS_TEST) $< -o $@

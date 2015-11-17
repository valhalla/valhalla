CC=g++

LIB_PATH=/usr/local/lib
FLAGS_TEST=-std=c++11 -Wl,-rpath=$(LIB_PATH) # -g -pg
ifdef DEBUG
	FLAGS=$(FLAGS_TEST)
else
	FLAGS=-std=c++11 -Wl,-rpath=$(LIB_PATH) -O3 -DNDEBUG
endif

LIBS=-lvalhalla_midgard -lvalhalla_baldr -lvalhalla_sif


.PHONY: all
all: commands tests


.PHONY: commands
commands: map_matching edge_search service psqlmatcher attacher


.PHONY: tests
tests: test_queue test_viterbi_search test_grid_range_query test_sp


.PHONY: run_tests
run_tests: tests
	./test_queue
	./test_viterbi_search
	./test_grid_range_query
	./test_sp


attacher: attacher.cc map_matching.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lsqlite3


psqlmatcher: psqlmatcher.cc map_matching
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lpqxx -lpq -lgeos -lsqlite3


service: service.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lprime_server -lpthread


edge_search: edge_search.cc edge_search.h candidate.h grid_range_query.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


map_matching: map_matching.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


test_sp: test_sp.cc sp.h
	$(CC) $(FLAGS_TEST) $< -o $@ -lvalhalla_midgard -lvalhalla_baldr


test_grid_range_query: test_grid_range_query.cc grid_range_query.h
	$(CC) $(FLAGS_TEST) $< -o $@ -lvalhalla_midgard


test_queue: test_queue.cc queue.h
	$(CC) $(FLAGS_TEST) $< -o $@


test_viterbi_search: test_viterbi_search.cc viterbi_search.h queue.h
	$(CC) $(FLAGS_TEST) $< -o $@

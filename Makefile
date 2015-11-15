CC=g++
FLAGS=-std=c++11 -O3 -DNDEBUG
LIBS=-lvalhalla_midgard -lvalhalla_baldr -lvalhalla_sif


.PHONY: compile_tests


compile_tests: test_queue test_viterbi_search test_map_matching test_edge_search test_grid_range_query test_sp test_service psqlmatcher


psqlmatcher: psqlmatcher.cc
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lpqxx -lpq -lgeos -lsqlite3


test_service: service.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS) -lprime_server -lpthread


test_sp: test_sp.cc sp.h
	$(CC) $(FLAGS) $< -o $@ -lvalhalla_midgard -lvalhalla_baldr


test_edge_search: test_edge_search.cc edge_search.h candidate.h grid_range_query.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


test_grid_range_query: test_grid_range_query.cc grid_range_query.h
	$(CC) $(FLAGS) $< -o $@ -lvalhalla_midgard


test_map_matching: test_map_matching.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h costings.h sp.h
	$(CC) $(FLAGS) $< -o $@ $(LIBS)


test_queue: test_queue.cc queue.h
	$(CC) $(FLAGS) $< -o $@


test_viterbi_search: test_viterbi_search.cc viterbi_search.h queue.h
	$(CC) $(FLAGS) $< -o $@

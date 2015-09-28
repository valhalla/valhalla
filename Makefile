CC=clang++
FLAGS=-std=c++11
LIBS=-lvalhalla_midgard -lvalhalla_baldr -lvalhalla_loki -lvalhalla_sif -lvalhalla_thor


.PHONY: compile_tests


compile_tests: test_queue test_viterbi_search test_map_matching test_edge_search test_grid_range_query


test_grid_range_query: test_grid_range_query.cc grid_range_query.h
	$(CC) $(FLAGS) -lvalhalla_midgard $< -o $@


test_map_matching: test_map_matching.cc map_matching.h edge_search.h viterbi_search.h queue.h candidate.h
	$(CC) $(FLAGS) $(LIBS) $< -o $@


test_edge_search: test_edge_search.cc edge_search.h candidate.h
	$(CC) $(FLAGS) $(LIBS) $< -o $@


test_queue: test_queue.cc queue.h
	$(CC) $(FLAGS) $< -o $@


test_viterbi_search: test_viterbi_search.cc viterbi_search.h queue.h
	$(CC) $(FLAGS) $< -o $@

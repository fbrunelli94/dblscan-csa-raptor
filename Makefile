HDRS:=$(wildcard src/*.hh)
CTRS:=$(wildcard src/*.cc)
LIBS:=-lz

main: dblscan.o
	@echo "This is a compilation test of dblscan.cc"

test: _data/London hl-csa-raptor.o
	./hl-csa-raptor.o -nq=5 $<

#test: _data/London dblscan.o
#	./dblscan.o -nq=5 $<

%.o: src/%.cc $(HDRS)
	g++ -std=c++17 -O3 -v -pthread -o $@ $< $(LIBS)


REPO:=https://files.inria.fr/gang/graphs/public_transport

_data/%:
	mkdir -p _data/$*
	cd _data/$*; \
	for f in queries-unif.csv stop_times.csv.gz transfers.csv.gz walk_and_transfer_inhubs.gr.gz walk_and_transfer_outhubs.gr.gz; do \
		curl -o $$f $(REPO)/$*/$$f ;\
	done

clean:
	rm -fr *~ */*~ _* *.o

.PHONY: all test clean

DIM:="2d"
WALL:="--wall"
BOUNDARY_X:=10
BOUNDARY_Y:=10
BOUNDARY_Z:=10
PROB:=0.1
MAP_NUM:=1

run:
	@echo "Begin to run algorithms"
	@export PYTHONPATH=".:src/" && pipenv run python tools/automata.py \
		-d ${DIM} ${WALL} \
		-b ${BOUNDARY_X} ${BOUNDARY_Y} ${BOUNDARY_Z} \
		--block-prob ${PROB} -n ${MAP_NUM}
	@echo "Algorithm Running Finished!"

test:
	@echo "Begin to test algorithms"
	@sh tools/test.sh
	@echo "Test Finished!"

clean:
	@echo "Cleaning the output files"
	@rm -rf ./output/*
	@echo "Cleaning the log files"
	@rm -rf ./logs/*
	@echo "Cleaning the map files"
	@rm -rf ./RRT/config/map/test_*.yaml
	@echo "Cleaning Finished!"

# Building CPP Targets

## Tests
To build the tests, run `make tests`

## Running Tests
To run the tests, run `make run_tests`

## CPython Dependencies
To build the CPython dependencies, run `make cpython`

## Preparing the Python test harnesses
To access the imports needed run the test harnesses, you must copy 
over the dependency files to the `cpp/` directory or modify the 
dependicies to point to the other directory. 

A quick and dirty hack is to run `cp python/*/* cpp/` in the root directory 
of this repository.

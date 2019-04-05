CC=clang++
#Used for auto generating dependencies
SHELL= /bin/bash 
CSTD= -std=c++11

#Store the directories of needed 
INCLUDEDIRS = -I./

CXXFLAGS= -Wall -c $(CSTD) $(INCLUDEDIRS) -fPIC -O3 #-Wfatal-errors
DEBUGFLAGS = -Wall $(CSTD) -c $(INCLUDEDIRS) -fPIC -DDEBUG -ggdb
PROFILEFLAGS := $(CXXFLAGS) -pg

#used for linker, gives libraries to link
LDLIBS = 
LDFLAGS = -pthread -lgtest -lboost_program_options -lyaml-cpp
LDPROFILEFLAGS := $(LDFLAGS) -pg

CYTHONTOCPPFLAGS :=
CYTHONFLAGS := -pthread -fPIC -fwrapv -fno-strict-aliasing -I/usr/include/python2.7
# whole-archive forces the shared library to contain all of the objects in the
# static libraries I link against
CYTHONLDFLAGS = -pthread -Wl,--no-as-needed $(LDLIBS) -Wl,--as-needed

EXECUTEABLE=main
# C++ source files
SOURCES=policy.cpp col_checker.cpp od_mstar.cpp grid_policy.cpp grid_planning.cpp epemstar.cpp mstar_utils.cpp main.cc
HEADERS= 

# Sources for python bindings
CYTHONSOURCES=cython_od_mstar.pyx

# Executeable for testing purposes
TEST_DIR=tests/
TESTEXECUTEABLE=test
TESTSOURCES=$(addprefix $(TEST_DIR), test.cpp test_col_set.cpp test_policy.cpp test_col_checker.cpp test_od_mstar.cpp test_epemstar.cpp)

all: $(EXECUTEABLE)

tests: $(TESTEXECUTEABLE)

depend: .depend

cython:$(CYTHONSOURCES:.pyx=.so)

# The -MM option is only supposed to generate dependencies on headers that
# are not system headers.  For some reason, the python headers don't count,
# and I get weird behavior unless I filter them out
.depend: .testdepend $(SOURCES)
	rm -f ./.depend
	# Iterate over everything except for .testdepend
	for i in $$(sed 's:$<::' <<< "$^"); do \
		target=./.depend.$$(sed 's/.cpp//' <<< $$i);\
		$(CC) $(CXXFLAGS) -I/usr/include/python2.7 -MM $$i -MF $$target;\
		cat $$target >> .depend;\
		 rm $$target;\
	done
	cat .depend | sed 's:/usr[^ ]*::g' | grep -E [a-zA-Z] > .depend.temp; mv .depend.temp .depend
	cat .testdepend >> .depend

.testdepend: $(TESTSOURCES)
	rm -f $@
	for i in $^; do\
		target=$@.$$(sed 's:.cpp::' <<< $$i | sed 's:$(TEST_DIR)::');\
		$(CC) $(CXXFLAGS) -MM $$i | sed 's:^\([^ ]\):$(TEST_DIR)\1:' > $$target;\
		cat $$target >> $@;\
		rm $$target;\
	done

include .depend

debug: export CXXFLAGS=$(DEBUGFLAGS)
debug: all $(TESTEXECUTEABLE)

profile: export CXXFLAGS=$(PROFILEFLAGS)
profile: export LDFLAGS=$(LDPROFILEFLAGS)
profile: all $(TESTEXECUTEABLE)

$(TESTEXECUTEABLE):  $(SOURCES:.cpp=.o) $(TESTSOURCES:.cpp=.o)
	$(CC) $^ $(LDFLAGS) -o $@

$(EXECUTEABLE): $(SOURCES:.cpp=.o)
	$(CC) $^ $(LDFLAGS) -o $@

debug_cython: export CYTHONTOCPPFLAGS:=$(CYTHONTOCPPFLAGS) --gdb
debug_cython: debug
debug_cython: cython

$(CYTHONSOURCES:.pyx=.cpp): %.cpp:%.pyx
	cython --cplus $(CYTHONTOCPPFLAGS) $(@:.cpp=.pyx)

$(CYTHONSOURCES:.pyx=.o): %.o : %.cpp 
	$(CC) $(CXXFLAGS) $(CYTHONFLAGS) -c $<

$(CYTHONSOURCES:.pyx=.so): %.so : %.o $(SOURCES:.cpp=.o)
	g++ -shared -pthread -fPIC -fwrapv -O3 -Wall -fno-strict-aliasing $(CYTHONLDFLAGS) $^ -o $@ 

run_tests: $(TESTEXECUTEABLE)
	./$(TESTEXECUTEABLE)

debug_tests: debug
	gdb $(TESTEXECUTEABLE)

clean:
	rm -f $(EXECUTEABLE) $(TESTEXECUTEABLE) .depend .testdepend *.o $(TEST_DIR)*.o

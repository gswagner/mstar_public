Native python and C++ code is availble.  Only the C++ code was written with the intention of being made public, so should be a bit cleaner.  Supports combining M* with MA-CBS only in python

Overview
====
* `cpp/` C++ implementation
  * `test/` Source for C++ unittests
* `python/` python implementation
  * `cbs/` MA-CBS
  * `mstar/` basic M*
  * `utils/` utillity code, including parallel and distributed map functions

Required Packages (Python)
----
* `networkx`
* `matplotlib`
* `ipdb`
* `pp`

Required Packages (C++)
----
* `g++`
* `libgtest-dev`
* `libboost-graph-dev`

Required Packages (Python bindings for C++ code)
* `cython` (for python bindings.  must be relatively recent, v0.20 works)	
* `libpython-dev`

Note: libgtest must be made manually.  This requires cmake.  Then run
* `cd /usr/src/gtest`
* `sudo cmake .`
* `sudo make`
* `sudo mv libg* /usr/lib/`

#!/usr/bin/python

# Provides an implementation of a distributed map similar to parmap
# Based on the python-pp package
# All nodes must be running the same version of pp. Thus, I would
# recommend installing pp via pip on 12.04 based systems

import pp
import subprocess
import random
import time
from parmap import _read_cache
import cPickle as pickle
import os
import socket
import itertools
import pdb
import Queue
import multiprocessing


# Try to use a pip subprocess32 package that supports timeouts for
# spawning processes, otherwise, use the default subprocess package
STARTUP_TIMEOUT = 2
try:
    import subprocess32

    def _call(*args, **kwargs):
        kwargs['timeout'] = STARTUP_TIMEOUT
        try:
            return subprocess32.call(*args, **kwargs)
        except subprocess32.TimeoutExpired:
            print 'Error starting node', args[0]
except ImportError:
    def _call(*args, **kwargs):
        print "Falling back on subprocess.  Install subprocess32 for timeouts"
        return subprocess.call(*args, **kwargs)


SLEEP_TIME = 0.1  # Time to sleep between checking for finished jobs
# Command for killing worker processes
WORKER_PIDS = (
    '$(ps aux | grep ppworker | grep -v grep | ' +
    'sed \'s/^[^[:space:]]\\+[[:space:]]\\+\\([[:digit:]]\\+\\).*/\\1/\')')


def dist_map(func, *args, **kwargs):
    """Runs a distributed map

    Does not like lambda functions, because pp literally copies the
    source code for func and passes the source to the remote processing
    nodes.  This doesn't work with lambdas, or closures in general.
    However, func can make use of closures or lambda internally

    Will preferentially provide jobs to the local workers, when possible

    Errors result in the code returning None.  Can't do the same error
    handling I did in parmap because I can't use closures

    Errors in func will be reported as being in the file "<string>" in
    func at line #.  # is the position of the error in func, with the
    line 'def func' numbered 1

    To avoid memory issues, queues a limited number of jobs at a given
    time, equal to the total number of compute nodes * job_factor.  If
    the user doesn't specify the number of workers per host/local
    machine, the code just assumes that 4 workers are used

    func           - function to map
    *args          - iterables overwhich func will be mapped.  The
                     output will have the length of the shortest input
                     iterable
    hosts          - name of hosts to which work will be distributed,
                     defaults to ('python', 'cobra')
    nworkers       - number of workers per remote host, pass None to use
                     as many cores as are present
    nlocal_workers - number of local workers to spawn on local host,
                     defaults to using as many workers as there are
                     cores
    depfuncs       - tuple of local functions called by func.  Functions
                     referenced via their modules, i.e. foo.bar(), do
                     not need to be in depfuncs, as they will be
                     imported from their
    modules
    modules        - tuple of module names that must be imported for
                     func to operate
    restart        - restart worker processes after each job
    cache_file     - file to which partial results are written.  Will
                     automatically try to load results from this file.
                     Will remove the file just before returning.
                     If None, perform no caching
    job_factor     - how many jobs per node should be queued.  Default 3

    returns:
    map(func, *args)
    """
    _validate_kwargs(**kwargs)
    hosts = kwargs.get('hosts', ('python', 'cobra'))
    nworkers = kwargs.get('nworkers', None)
    nlocal_workers = kwargs.get('nlocal_workers', 'autodetect')
    depfuncs = kwargs.get('depfuncs', ())
    modules = kwargs.get('modules', ())
    restart = kwargs.get('restart', False)
    cache_file = kwargs.get('cache_file', None)
    job_factor = kwargs.get('job_factor', 3)
    # Handle loading cache
    if cache_file is None:
        res = {}
    else:
        res, partial_file = _read_cache(cache_file)

    # Need to feed jobs on a as-needed basis to avoid masive memory
    # consumption, so need to get the total number of nodes in use.
    total_nodes = 0
    if nworkers is not None:
        total_nodes += nworkers * len(hosts)
    else:
        # assume 4 workers
        total_nodes += 4 * len(hosts)
    if nlocal_workers == 'autodetect':
        # assume 4 workers
        total_nodes += 4
    else:
        total_nodes += nlocal_workers
    # track number of jobs that have been submitted, but whose results
    # have not been received
    queued_jobs = 0
    # have all jobs been sent to the queue
    all_queued = False

    # Need a non-trivial string secret to make stuff work
    secret = str(random.random())
    with ServerManager(hosts, nworkers, secret) as manager:
        job_server = pp.Server(secret=secret, ppservers=hosts,
                               ncpus=nlocal_workers, restart=restart)
        manager.register_server(job_server)
        jobs = {}
        arg_iterator = enumerate(itertools.izip(*args))
        while (not all_queued) or (len(jobs) > 0):
            if not all_queued:
                # Feed jobs onto the queue on an as-needed basis
                try:
                    while queued_jobs < total_nodes * job_factor:
                        dex, arg = arg_iterator.next()
                        if dex in res:
                            continue
                        jobs[dex] = job_server.submit(
                            func, args=arg, depfuncs=depfuncs, modules=modules)
                        queued_jobs += 1
                except StopIteration:
                    # all jobs have been submitted
                    all_queued = True
            finished = []
            # process jobs one at a time so we can do caching, etc
            for dex, job in jobs.iteritems():

                if job.finished:
                    res[dex] = job()
                    queued_jobs -= 1
                    if cache_file is not None:
                        pickle.dump((dex, res[dex]), partial_file, protocol=-1)
                        partial_file.flush()
                    finished.append(dex)
            for f in finished:
                del jobs[f]
            if len(jobs) > 0:
                time.sleep(SLEEP_TIME)
    out = [res[i] for i in sorted(res.keys())]
    if cache_file is not None:
        try:
            partial_file.close()
            os.remove(cache_file)
        except OSError:
            # cache file doesn't exist for some reason
            pass
    return out


class ServerManager(object):
    """Ensures that servers are set up and killed properly

    cleans up errant local worker processes, so must be run before the
    server is generated"""

    def __init__(self, hosts, nworkers, secret):
        """
        hosts    - host names or ips where nodes will be started
        nworkers - number of worker processes per node
        secret   - string used for authentication
        """
        self._hosts = hosts
        self._nworkers = nworkers
        self._secret = secret
        self._server = None
        self._server_connections = []

    def __enter__(self):
        """Start the remote nodes"""
        # Need to kill any local worker threads
        _call('kill ' + WORKER_PIDS + ' > /dev/null 2>&1', shell=True)
        for host in self._hosts:
            # Kill any existing server instances before running a new
            # one
            _call(map(str, ['ssh', host, 'killall ppserver.py', '-q']))
            _call(['ssh', host, 'kill', WORKER_PIDS, ' > /dev/null 2>&1'])
            # source ~/.profile is needed to intialize environment
            # variables.  Note that this means environment variable
            # declarations should be in .profile instead of .bashrc
            command = map(str, ['ssh', host, 'source ~/.profile;',
                                'ppserver.py', '-s', self._secret])
            if self._nworkers:
                command.extend(map(str, ('-w', self._nworkers)))
            self._server_connections.append(
                subprocess.Popen(map(str, command), close_fds=True))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup remote nodes and local worker processes"""
        for host in self._hosts:
            # Cleanup any servers
            _call(map(str, ['ssh', host, 'killall ppserver.py', '-q']))
            _call(['ssh', host, 'kill', WORKER_PIDS, ' > /dev/null 2>&1'])
        # shut down the local server
        if self._server is not None:
            self._server.destroy()
        # kill the local workers, as I don't trust the destroy method
        # TODO: figure out what is failing here
        _call('kill ' + WORKER_PIDS + ' > /dev/null 2>&1', shell=True)
        for connection in self._server_connections:
            connection.terminate()

    def register_server(self, server):
        """Register a pp.Server so it can be shutdown

        Needed to prevent worker processes being spawned after we try to
        kill them
        """
        self._server = server


def _validate_kwargs(**kwargs):
    """Checks that there are no unknown keyword arguments"""
    keywords = ['hosts', 'nworkers', 'nlocal_workers', 'depfuncs',
                'modules', 'restart', 'cache_file', 'job_factor']
    for key in kwargs:
        if not key in keywords:
            raise TypeError('dist_map got an unexpected keyword argument \'' +
                            key + '\'')
    return True


def timeout_function(func, args, timeout, run_id=None):
    """Wrapper function to run a function in subprocess with timeout

    func - function to wrap
    args - tuple of arguments
    timeout - timeout in seconds
    run_id  - optional identifier of the run, used for printing

    return:
    result of func(*args)
    None if the process ran away
    """
    import traceback
    import sys

    def inner_func(args, queue):
        try:
            queue.put(func(*args))
        except Exception as e:
            e_type, e_val, e_traceback = sys.exc_info()
            e.traceback = traceback.format_tb(e_traceback)
            queue.put(e)
    ret_queue = multiprocessing.Queue()
    inner_proc = multiprocessing.Process(target=inner_func,
                                         args=(args, ret_queue))
    inner_proc.start()
    try:
        # Raises Queue.Empty if timeout is exceeded
        val = ret_queue.get(timeout=timeout)
        if isinstance(val, Exception):
            for l in val.traceback:
                print l
            raise val
        return val
    except Queue.Empty:
        if run_id is not None:
            print 'Test {0} ran away'.format(run_id)
        # Any processes started by func will be orphaned
        inner_proc.terminate()
        return None


def main():
    # Need a non-trivial string secret to make stuff work
    def foo(x):
        time.sleep(1)
        print x, socket.gethostname()
        return x ** 2
    print dist_map(foo, range(100),
                   modules=('socket', 'time', 'random'),
                   cache_file='foo.baz')


if __name__ == '__main__':
    main()

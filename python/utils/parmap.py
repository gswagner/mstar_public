# My own implementation of parallel map, to address the limitations of
# the multiprocessing pool (i.e. not working in interactive sessions)
# Note that Numpy arrays cannot be pickled, because they use a raw
# memory array.  However, they can be inherited, so just write a wrapper
# function which captures the numpy array from the defining namespace,
# and if necessary, iterate over a dummy iterator

from Queue import Empty
import Queue as queue
from multiprocessing import Process, Queue, Manager, cpu_count, JoinableQueue
from multiprocessing import Event
from threading import Thread
import itertools
import sys
import traceback
import cPickle as pickle
import os

# When processes periodically refresh in parmap, need a timeout for how
# long to wait for a result before checking whether a process needs a
# refresh. Hopefully if this matters, the characteristic time between a
# process shutting down will be greater than 1/10 of a second
WAIT_TIME = 0.1


def parmap(func, *args, **kwargs):
    """Simple process-based parallel map.

    Can be used in interactive sessions without pickling issues.
    Multiple iterables can be passed in as *args, in which case one item
    from each iterable will be passed as function arguments, in order.
    This is the same behaviour as map

    arguments and return values are pickled

    func             - function to apply
    iterable         - iterable to map over.

    keyword only:

    num_procs        - number of processes to use, defaults to total
                       number of cpus
    skip_on_error    - skips results that throw errors, and return any
                       non-error results
    max_process_jobs - restart processes after performing this number of
                       jobs defaults to 0, i.e. never restart.  Intended
                       to ensure that resources are freed when dealing
                       with code that has issues, like isomap
    callback         - callback function to call on the results returned
                       by the worker processes before storing in the
                       return list.  This is a bit silly, but can be
                       useful if there is a function you want to run
                       exclusively in a single process, such as saving
                       partial results to a file. Takes in the output
                       value of func and returns the new return value
    cache_file       - file to which partial results are written.  Will
                       automatically try to load results from this file.
                       Will remove the file just before returning

    returns:
    The result of mapping func onto args
    """
    num_procs = kwargs.get('num_procs', 0)
    if not num_procs:
        num_procs = cpu_count()
    skip_on_error = kwargs.get('skip_on_error', False)
    max_process_jobs = kwargs.get('max_process_jobs', 0)
    callback = kwargs.get('callback', None)
    check_timeout = WAIT_TIME  # How often to check for the kill signal
    cache_file = kwargs.get('cache_file', None)

    # Exploit short circuiting behavior of and to avoid cases
    skip_on_error = 'skip_on_error' in kwargs and kwargs['skip_on_error']
    kill_event = Event()

    arg_queue = JoinableQueue()
    res_queue = Queue()

    worker_closure = lambda: Process(
        target=lambda: _worker(arg_queue, res_queue, func, kill_event,
                               check_timeout, max_process_jobs))
    processes = [worker_closure() for i in xrange(num_procs)]

    if cache_file is None:
        out_list = {}
    else:
        out_list, partial_file = _read_cache(cache_file)

    with ProcessManager(processes, worker_closure, kill_event) as manager:
        data_points = 0
        # Want to load data incrementally to minimize memory usage
        arg_iterator = enumerate(itertools.izip(*args))
        while data_points < num_procs * 2:
            # Try to populate the argument queue with enough to keep it
            # going
            try:
                item = arg_iterator.next()
                # Skip items that were loaded from the cache
                if not item[0] in out_list:
                    arg_queue.put(item)
                    data_points += 1
            except StopIteration:
                break
        # Can't use res_queue.empty(), as the multiprocessing
        # documentation wasn't kidding when it was described as
        # unreliable
        received = 0
        while received < data_points:
            if max_process_jobs:
                # If processes can die, need to periodically check
                # whether any process has died, as there may be a long
                # period of time  between jobs being finished
                try:
                    dex, val = res_queue.get(timeout=WAIT_TIME)
                except Empty:
                    manager.refresh_pool()
                    continue
            else:
                # If worker processes never refresh, then can just wait
                # until a finished job becomes available
                dex, val = res_queue.get()
            received += 1
            if isinstance(val, Exception):
                temp = str(val)
                for l in val.traceback:
                    print l
                # For some reason printing val directly causes a hang,
                # but this works.  The advantage of printing here
                # instead of relying on the raised error is that it puts
                # the name of the exception next to the traceback
                print temp
                if not skip_on_error:
                    for p in processes:
                        p.terminate()
                    raise val
            else:
                if callback:
                    # Have a callback to run before storing the result.
                    val = callback(val)
                out_list[dex] = val
                if cache_file is not None:
                    pickle.dump((dex, val), partial_file)
                    partial_file.flush()
                if max_process_jobs:
                    manager.refresh_pool()
            try:
                # Try to push a new data point onto the list, skip if
                # nothing left in the iterator
                while True:
                    item = arg_iterator.next()
                    # iterate until we find an entry that was not read
                    # from the cache file
                    if not item[0] in out_list:
                        arg_queue.put(item)
                        data_points += 1
                        break
            except StopIteration:
                continue
    # return out_list
    keys = sorted(out_list.keys())
    if cache_file is not None:
        try:
            partial_file.close()
            os.remove(cache_file)
        except OSError:
            # cache file doesn't exist for some reason
            pass
    return [out_list[key] for key in keys]


def threadmap(func, *args, **kwargs):
    """Simple thread-based parallel map.

    Can be used in interactive sessions without pickling issues.
    Multiple iterables can be passed in as *args, in which case one item
    from each iterable will be passed as function arguments, in order.
    This is the same behaviour as map

    parallelizes using threads, so arguments and return values are never
    pickled

    func             - function to apply
    iterable         - iterable to map over.

    keyword only:

    num_threads      - number of threads to use, defaults to total
                       number of cpus
    skip_on_error    - skips results that throw errors, and return any
                       non-error results
    max_process_jobs - restart processes after performing this number of
                       jobs defaults to 0, i.e. never restart.  Intended
                       to ensure that resources are freed when dealing
                       with code that has issues, like isomap
    callback         - callback function to call on the results returned
                       by the worker processes before storing in the
                       return list.  This is a bit silly, but can be
                       useful if there is a function you want to run
                       exclusively in a single process, such as saving
                       partial results to a file. Takes in the output
                       value of func and returns the new return value
    cache_file       - file to which partial results are written.  Will
                       automatically try to load results from this file.
                       Will remove the file just before returning

    returns:
    The result of mapping func onto args
    """
    num_threads = kwargs.get('num_threads', 0)
    if not num_threads:
        num_threads = cpu_count()
    skip_on_error = kwargs.get('skip_on_error', False)
    max_process_jobs = kwargs.get('max_process_jobs', 0)
    callback = kwargs.get('callback', None)
    check_timeout = WAIT_TIME  # How often to check for the kill signal
    cache_file = kwargs.get('cache_file', None)

    # Exploit short circuiting behavior of and to avoid cases
    skip_on_error = 'skip_on_error' in kwargs and kwargs['skip_on_error']
    kill_event = Event()

    # use Queue.Queue, so not pickling the output, so can return objects
    # with useful handles
    arg_queue = queue.Queue()
    res_queue = queue.Queue()

    worker_closure = lambda: Thread(
        target=lambda: _worker(arg_queue, res_queue, func, kill_event,
                               check_timeout, max_process_jobs))
    threads = [worker_closure() for i in xrange(num_threads)]

    if cache_file is None:
        out_list = {}
    else:
        out_list, partial_file = _read_cache(cache_file)

    # note that ProcessManager works with threads if worker_closure
    # generates thread objects instead
    with ProcessManager(threads, worker_closure, kill_event) as manager:
        data_points = 0
        # Want to load data incrementally to minimize memory usage
        arg_iterator = enumerate(itertools.izip(*args))
        while data_points < num_threads * 2:
            # Try to populate the argument queue with enough to keep it
            # going
            try:
                item = arg_iterator.next()
                # Skip items that were loaded from the cache
                if not item[0] in out_list:
                    arg_queue.put(item)
                    data_points += 1
            except StopIteration:
                break
        # Can't use res_queue.empty(), as the multiprocessing
        # documentation wasn't kidding when it was described as
        # unreliable
        received = 0
        while received < data_points:
            if max_process_jobs:
                # If threads can die, need to periodically check
                # whether any process has died, as there may be a long
                # period of time  between jobs being finished
                try:
                    dex, val = res_queue.get(timeout=WAIT_TIME)
                except Empty:
                    manager.refresh_pool()
                    continue
            else:
                # If worker threads never refresh, then can just wait
                # until a finished job becomes available
                dex, val = res_queue.get()
            received += 1
            if isinstance(val, Exception):
                temp = str(val)
                for l in val.traceback:
                    print l
                # For some reason printing val directly causes a hang,
                # but this works.  The advantage of printing here
                # instead of relying on the raised error is that it puts
                # the name of the exception next to the traceback
                print temp
                if not skip_on_error:
                    # threads will be shutdown by the manager
                    raise val
            else:
                if callback:
                    # Have a callback to run before storing the result.
                    val = callback(val)
                out_list[dex] = val
                if cache_file is not None:
                    pickle.dump((dex, val), partial_file)
                    partial_file.flush()
                if max_process_jobs:
                    manager.refresh_pool()
            try:
                # Try to push a new data point onto the list, skip if
                # nothing left in the iterator
                while True:
                    item = arg_iterator.next()
                    # iterate until we find an entry that was not read
                    # from the cache file
                    if not item[0] in out_list:
                        arg_queue.put(item)
                        data_points += 1
                        break
            except StopIteration:
                continue
    # return out_list
    keys = sorted(out_list.keys())
    if cache_file is not None:
        try:
            partial_file.close()
            os.remove(cache_file)
        except OSError:
            # cache file doesn't exist for some reason
            pass
    return [out_list[key] for key in keys]


def _worker(in_queue, out_queue, func, kill_event, check_timeout,
            max_process_jobs):
    """Worker function

    Assumed that most of the arguments will be passed by closure

    in_queue         - queue object to provide jobs
    out_queue        - queue object to collect output
    func             - function to execute
    kill_event       - Event object used to terminate the process
    check_timeout    - time to wait for a new job
    max_process_jobs - maximum number of jobs to perform in a single
                       process
    """
    job_counter = 0
    while True:
        # If the number of iterables is very high, much better to
        try:
            if kill_event.is_set():
                return
            dex, args = in_queue.get(block=True, timeout=check_timeout)
            val = func(*args)
            out_queue.put((dex, val))
            in_queue.task_done()
            job_counter += 1
            if max_process_jobs and job_counter >= max_process_jobs:
                return
        except Empty:
            if kill_event.is_set():
                return
        except KeyboardInterrupt:
            print 'Received keyboard kill signal'
            return
        except Exception as e:
            e_type, e_val, e_traceback = sys.exc_info()
            e.traceback = traceback.format_tb(e_traceback)
            out_queue.put((dex, e))


def _read_cache(cache_file):
    """Reads in partial results from a cache file

    cache_file - name of cache file to use

    returns: partial_results, cache_file
    partial_results - dictionary of partial results, indexed by index in
                      the argument list
    cache_file      - file object to which partial results should be
                      written
    """
    out = {}
    try:
        with open(cache_file, 'r') as f:
            reader = pickle.Unpickler(f)
            try:
                while True:
                    dex, val = reader.load()
                    out[dex] = val
            except EOFError:
                pass
    except IOError:
        # cache file didn't exist already
        pass
    if len(out) > 0:
        print 'Read ' + str(len(out)) + ' entries from cache'
    return out, open(cache_file, 'a')


class ProcessManager(object):
    """Guarantees that the processes will be killed

    This thing is side effecting processes in a bad way.  I really
    should split the pool refreshing code from the managing code,
    requiring the monitoring state to be split out as well, but that
    can wait.  I just want it working for now
    """

    def __init__(self, processes, worker_generator, kill_event):
        """I have been having issues with trying to kill processes
        that haven't been started, so explicitly track which
        processes have been started, and only kill those
        """
        self.started = []
        self.processes = processes
        self.worker_generator = worker_generator
        self.kill_event = kill_event

    def __enter__(self):
        """the with statement uses the return value of __enter__ for the
        as parameter
        """
        for p in self.processes:
            p.start()
            self.started.append(p)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        for p in self.started:
            self.kill_event.set()
            try:
                # p.terminate()
                p.join()
            except AttributeError as e:
                print 'Error closing workers ' + str(e)

    def refresh_pool(self):
        if not self.kill_event.is_set():
            dex = 0
            while dex < len(self.started):
                p = self.started[dex]
                if not p.is_alive():
                    self.started.pop(dex)
                    p = self.worker_generator()
                    p.start()
                    self.started.append(p)
                    self.processes.append(p)
                else:
                    dex += 1

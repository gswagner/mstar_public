try:
    import ipdb as pdb
except ImportError:
    import pdb
import traceback
from functools import wraps
import sys
import cProfile
import unittest


DEBUG = True


# I am not happy with how debugging works with nosetests, trying a decorator
# approach from stack overflow
# Uses a global variable DEBUG to control debugging
def debug_on(*skip_exceptions):
    """
    skip_exceptions - exception types to ignore
    """
    if not skip_exceptions:
        skip_exceptions = ()

    def decorator(f):
        global DEBUG

        @wraps(f)
        def wrapper(*args, **kwargs):
            if not DEBUG:
                return f(*args, **kwargs)
            try:
                return f(*args, **kwargs)
            except Exception as e:
                for skip in skip_exceptions:
                    if isinstance(e, skip):
                        raise e
                print '\n'
                for line in traceback.format_tb(sys.exc_info()[2]):
                    print line
                print str(e.__class__.__name__) + ': ' + str(e) + '\n'
                pdb.post_mortem(sys.exc_info()[2])
                raise e
        return wrapper

    return decorator


def DebugMetaClass(*skip_exceptions):
    """
    skip_exceptions - exception types to ignore
    """
    class DebugMeta(type):
        """Meta-class for tagging all methods starting with test_ with
        debug_on

        Used by setting the __metaclass__ attribute of the class in question
        to DebugMeta
        """

        def __new__(cls, name, parents, dct):
            """Runs all functions starting with test through debug_on"""
            for key, attr in dct.iteritems():
                if key.startswith('test_'):
                    dct[key] = debug_on(*skip_exceptions)(dct[key])
            return super(DebugMeta, cls).__new__(cls, name, parents, dct)
    return DebugMeta


DebugMeta = DebugMetaClass(unittest.SkipTest, KeyboardInterrupt)


# To simplify the profiling process, a decorator can be used
# Decorator code from:cprofile
# https://zapier.com/engineering/profiling-python-boss/
def do_cprofile(sort_key='time'):
    """Decorator to produce the profile of a test case

    Takes one argument, which determines how the results should be
    sorted.

    sort_key - the value to use in sorting the profile results.
               Possible values are
               'time'       - internal time
               'cumulative' - cumulative time
    """
    def decorator(func):

        @wraps(func)
        def profiled_func(*args, **kwargs):
            profile = cProfile.Profile()
            try:
                profile.enable()
                result = func(*args, **kwargs)
                profile.disable()
                return result
            finally:
                profile.print_stats(sort_key)
        return profiled_func

    return decorator

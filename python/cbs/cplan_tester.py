#!/usr/bin/python
import cbs
import multiprocessing
import pickle
import sys
import independence_detection
import os
import time
import gc
import  subprocess
import  argparse
from Queue import Empty
from col_set_addition import OutOfTimeError
import  NoSolutionError

def path_cost(path, goals):
    cost = 0
    for point in path:
        for i in xrange(len(point)):
            if point[i] != goals[i]:
                cost += 1
    return cost

def inner_worker(args, output):
    try:
        out = test_func(args)
        output.put(out)
    except RuntimeError as e:
        print "Error %s" %(repr(e))
        print "World %d" %(args['test_num'])
        output.put(repr(e))
    except AttributeError as e:
        print "Attribute Error %s" %(repr(e))
        print "World %d" %(args['test_num'])
        output.put(repr(e))
    # except AssertionError as e:
    #     print "Assertion Error %s" %(repr(e))
    #     print "World %d" %(args['test_num'])
    #     output.put(repr(e))
    except ValueError as e:
        print "Value Error: %s" %(repr(e))
        print "World %d" %(args['test_num'])
        output.put(repr(e))
    # except Exception as e:
    #     print 'Unexpected Error %s' %(repr(e))
    #     print "World %d" %(args['test_num'])
    #     output.put(repr(e))

def worker(input, output, file_name):
    for args in iter(input.get, 'STOP'):
        out_file = open(file_name, 'a')
        p = pickle.Pickler(out_file, protocol=-1)
        results = None
        inner_queue = multiprocessing.Queue()
        start_time = time.time()
        inner_proc = multiprocessing.Process(target=inner_worker,
                                             args=(args, inner_queue))
        inner_proc.start()
        try:
            results = inner_queue.get(timeout = args['time_limit'] + 5)
        except Empty:
            print 'Runaway Timeout'
            inner_proc.terminate()
            res = args_to_dict(args)
            res.update({'time':time.time() - start_time, 
                        'fail_flag':'Out Of Time', 'num_nodes':-1,
                        'corrected_mem_usage':-1, 'final_init_pos':None,
                        'path_cost':-1})
            if args['return_path']:
                res.update({'paths':None, 'obs_map':args['obs_map']})
            p.dump(res)
            out_file.flush()
            os.fsync(out_file.fileno())
            out_file.close()
            output.put('Finished')
            gc.collect(2)
            continue
        if isinstance(results, str):
            #Got some arbitrary error to sort out
            res = args_to_dict(args)
            res.update({'time':time.time() - start_time, 'fail_flag':results,
                        'num_nodes':-1, 'corrected_mem_usage':-1,
                        'final_init_pos':None, 'path_cost':-1})
            if args['return_path']:
                res.update({'paths':None, 'obs_map':args['obs_map']})
            p.dump(res)
        else:
            p.dump(results)
        out_file.flush()
        os.fsync(out_file.fileno())
        out_file.close()
        output.put('Finished')
        gc.collect(2)
    output.put('Process Finished')

def args_to_dict(args):
    '''Reads in the arguments, and returns the dictionary containing the 
    appropriate  portions, to which planning results can be added later'''
    to_skip = ['return_path', 'obs_map']
    output = {k:v for k, v in args.iteritems() if k not in to_skip}
    # output = {'init_pos':args['init_pos'],'goals':args['goals'],
    #           'inflation':args['inflation'],'time_limit':args['time_limit'],
    #           'test_num':args['test_num'],'recursive':args['recursive'], 
    #           'prio':args['prio'], 'policy_opt':args['opt'],'cbs':args['cbs'],
    #           'merge_thresh':args['merge_thresh'],
    #           'op_decomp':args['op_decomp'],'murty':args['murty'],
    #           'swap':args['swap'],'opt_reassign':args['opt_reassign'],
    #           'interior_swap':args['interior_swap'],
    #           'connect_8':args['connect_8'],'brute_force':args['brute_force']}
    #           #,'revision':args['revision']}
    if args['return_path']:
        output['obs_map'] = args['obs_map']
    return output

def test_func(args):
    paths = None
    fail_flag = ''
    start_time = time.clock()
    sol_time = 0
    num_nodes = -1
    corrected_mem = -1
    final_init_pos = None
    cost = -1
    try:
        paths = None
        if args['recursive'] and not args['astar']:
            # print 'od_rmstar'
            planner = 'od_rmstar'
        elif not args['recursive'] and not args['astar']:
            # print 'od_mstar'
            planner = 'od_mstar'
        elif args['recursive'] and args['astar']:
            # print 'rmstar'
            planner = 'rmstar'
        elif not args['recursive'] and args['astar']:
            # print 'mstar'
            planner = 'mstar'
        else:
            planner = None

        if args['cbs']:
            if args['merge_thresh'] > -1:
                meta_agents = True
            else:
                meta_agents = False
            if args['epermstar']:
                planner = 'epermstar'
            else:
                planner = 'od_rmstar'
            if True:
                cbs_planner = cbs.CBS_Planner(args['obs_map'], args['goals'],
                                              conn_8=args['connect_8'],
                                              meta_agents=meta_agents,
                                             merge_thresh=args['merge_thresh'],
                                              meta_planner=planner)
                paths = cbs_planner.find_solution(
                    args['init_pos'], 
                    time_limit = time.clock() - start_time + args['time_limit'])
        sol_time = time.clock() - start_time
    except NoSolutionError:
        print 'No solution'
        print args['test_num']
        fail_flag = 'No Solution'
        path = None
        sol_time = time.clock() - start_time
    except OutOfTimeError:
        print 'Out of time'
        fail_flag = 'Out Of Time'
        path = None
        sol_time = time.clock() - start_time
    if paths != None:
        cost = path_cost(paths, args['goals'])
    res = args_to_dict(args)
    res.update({'time':sol_time, 'fail_flag':fail_flag, 'num_nodes':num_nodes,
                'corrected_mem_usage':corrected_mem,
                'final_init_pos':final_init_pos, 'path_cost':cost})
    if args['return_path']:
        res.update({'paths':paths, 'obs_map':args['obs_map']})
    return res
               
              
def main(argv=sys.argv):
    '''Performs the system test, intended to use world_gen as the primary 
    source'''
    reuse_partial_results = True
    #Create use argparser to process arguments
    parser = argparse.ArgumentParser(description='Runs trials for M* ' + 
                                     'and associated algorithms.  Using' +
                                     'independence detection by default, or' +
                                     'optionally using cbs/ma-cbs')
    parser.add_argument('map_file', 
                        help='Specifies the file from which to draw' +  
                        'the world specifications')
    parser.add_argument('output_file', help='File where output will be saved')
    parser.add_argument('num_processors', type=int, action='store',
                          help='Number of processes to run')
    parser.add_argument('-8', dest='connect_8', action='store_true',
                        help='Build an 8-connected graph' + 
                        'rather than a 4 connected graph')
    parser.add_argument('-r', dest='recursive', action='store_true',
                          help='Uses rM*')
    parser.add_argument('--epermstar', dest='epermstar', action='store_true',
                        help='Use EPErM* as the coupled planner for MA-CBS')
    parser.add_argument('-c', dest='cbs', action='store_true', help='Use cbs')
    parser.add_argument('-m', dest='merge_thresh', action='store', type=int,
                        default = -1, metavar="MERGE_THRESH", help=
                        'Sets the threshold for merging agents in meta-CBS.  '+
                        'Defaults to -1, for basic CBS.  Higher values for ' +
                        'meta cbs.  The default coupled planner is od_rmstar' +
                        '. Passing the -o option results in using op_decomp')
    parser.add_argument('-i', action='store', default=1, type=float,
                        help='Set the inflation factor for the heuristic, '+
                        'defaults to 1',
                        metavar='INF', dest='inflation')
    parser.add_argument('-t', action='store', type=float, default=5*60,
                        help='Set time limit for finding paths',
                        metavar='TIME_LIMIT', dest='time_limit')
    parser.add_argument('-s', action='store_true',
                        help='Remember the collision set used to reach a ' +
                        'given node, to improve cache utillization',
                        dest='use_source_memory')
    parser.add_argument('-p', action='store_true',
                        help='Also make use of the coupled priority planner',
                        dest='prio')
    parser.add_argument('-po', action='store_true',
                        help='Use policy optimization for the M* variants', 
                        dest='opt')
    parser.add_argument('--path', action='store_true',
            help='Stores the paths in the output file, results in large files',
                         dest='return_path')
    parser.add_argument('--astar', action='store_true',
                        help='Uses basic A* instead of operator decomposition',
                        dest='astar')
    arguments = parser.parse_args()
    num_processors = int(argv[3])
    worlds = pickle.load(open(arguments.map_file))
    # revision = subprocess.check_output('svn info | grep Revision | grep -Eo [0-9]+', shell=True).strip()
    revision = subprocess.check_output(
        'git log | head -n 1 | sed s/commit\s*//', shell=True).strip)
    for i in xrange(len(worlds)):
        worlds[i]['test_num'] = i
        worlds[i]['recursive'] = arguments.recursive
        worlds[i]['connect_8'] = arguments.connect_8
        worlds[i]['epermstar'] = arguments.epermstar
        worlds[i]['time_limit'] = arguments.time_limit
        worlds[i]['return_path'] = arguments.return_path
        worlds[i]['astar'] = arguments.astar
        worlds[i]['use_source_memory'] = arguments.use_source_memory
        worlds[i]['opt'] = arguments.opt
        worlds[i]['merge_thresh'] = arguments.merge_thresh
        worlds[i]['revision'] = revision
        assert arguments.inflation >= 1.0
        worlds[i]['inflation'] = arguments.inflation
            
    task_queue = multiprocessing.Queue()
    done_queue = multiprocessing.Queue()
    precomputed_trials = []
    if reuse_partial_results:
        dat = []
        try:
            #Check if we've already established a partial results cache
            dat = pickle.load(open(arguments.output_file + 
                                   '.old_results_cache'))
            print 'Loaded old cache: %d trials'  %(len(dat))
            precomputed_trials = map(lambda x:x['test_num'], dat)
        except IOError:
            #Haven't cached previous results, so don't need to do anything
            pass
        for i in xrange(arguments.num_processors):
            print 'Loading old data from process %d' %(i)
            try:
                p = pickle.Unpickler(open(arguments.output_file + '.' + str(i)))
                try:
                    while True:
                        temp = p.load()
                        precomputed_trials.append(temp['test_num'])
                        dat.append(temp)
                except EOFError:
                    print 'Found end of file: %d trials found' %(len(dat))
            except IOError:
                print 'No cached results'
                break
        if len(dat) > 0:
            # Results files can get rather big, especially if you return 
            # the paths, so cache the results in a temp file
            tempfile = open(arguments.output_file + '.old_results_cache', 'w')
            pickle.dump(dat, tempfile, protocol = -1)
            tempfile.flush()
            del dat
            gc.collect()
        else:
            reuse_partial_results = False
        #Ignore worlds we've already done
        worlds = filter(lambda x:x['test_num'] not in precomputed_trials, 
                        worlds)
    #Clear out the files that will hold the results
    for i in xrange(arguments.num_processors):
        f = open(arguments.output_file + '.' + str(i), 'w')
        f.close()
    #Start the worker process
    for i in xrange(arguments.num_processors):
        multiprocessing.Process(target=worker, 
                                args=(task_queue, done_queue, 
                                      arguments.output_file + '.' + 
                                      str(i))).start()
    for i in worlds:
        task_queue.put(i)
    for i in xrange(arguments.num_processors):
        task_queue.put('STOP')
    #Save the results
    jobs = 0
    processes_finished = 0
    print "waiting for job 1 %d" %(len(worlds))
    for i in xrange(len(worlds) + (arguments.num_processors)):
        temp = done_queue.get()
        if temp == 'Finished':
            print 'got_job'
            jobs += 1
            if jobs < len(worlds):
                print 'waiting for job %d %d' %(jobs + 1, len(worlds))
        elif temp == 'Process Finished':
            processes_finished += 1
            print 'Process terminated: %d %d' %(processes_finished, 
                                                arguments.num_processors)
    print 'Gathering Data'
    dat = []
    if reuse_partial_results:
        print 'old_cache'
        dat = pickle.load(open(arguments.output_file + '.old_results_cache'))
        print 'trials found: %d' %(len(dat))
    for i in xrange(arguments.num_processors):
        print 'Merging data from process %d' %(i)
        p = pickle.Unpickler(open(arguments.output_file + '.' + str(i)))
        try:
            while True:
                dat.append(p.load())
        except EOFError:
            print 'Found end of file: %d trials found' %(len(dat))
        try:
            #Pull out the name of the file itself, so we can move the partial
            #results to old_partials
            tempname = arguments.output_file.split('/')[-1]
            os.rename(arguments.output_file + '.' + str(i), 'old_partials/' + 
                      tempname + '.' + str(i))
        except Exception:
            pass
    print 'total trials found: %d' %(len(dat))
    print 'sorting trials'
    dat.sort(key=lambda x:x['test_num'])
    pickle.dump(dat, open(arguments.output_file, 'w'), protocol=-1)

            
if __name__ == '__main__':
    main()

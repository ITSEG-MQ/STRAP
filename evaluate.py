from prioritization import prioritize_by_semantic_coverage, prioritize_by_rsc, prioritize_by_ch, prioritize_by_random
from utils import get_record_length
import os
from segment_split import record2vec
import numpy as np
import pickle


def top_k(seg_order, seg_diff):
    top_k = None
    for i, seg in enumerate(seg_order):
        if seg_diff[seg]:
            top_k = i + 1
            break

    if not top_k:
        top_k = -1

    return top_k


def apfd(seg_order, seg_diff):
    n = len(seg_order)
    # m = n - sum(seg_diff.values())
    bug_orders = []

    for i, seg in enumerate(seg_order):
        if seg_diff[seg]:
            bug_orders.append(i + 1)

    m = len(bug_orders)

    try:
        apfd = 1 - float(sum(bug_orders)) / (n * m) + 1.0 / (2 * n)
    except:
        apfd = -1

    return round(apfd, 2)

def time_consumption():
    maps = ['cubetown', 'shalun', 'gomentum']

    len_ori = 0
    len_reduced = {'signal': 0}
    for map in maps:
        len, _, _ = get_record_length("data/records/{}.record".format(map))
        len_ori += len

        for module in len_reduced.keys():
            segments = os.listdir("data/record_segments/{}/{}/reduced".format(map, module))
            for segment in segments:
                len, _, _ = get_record_length("data/record_segments/{}/{}/reduced/{}".format(map, module, segment))
                len_reduced[module] += len


def fault_evaluate(module, map, benchmark, mode='reduced'):
    faults = {}
    for segment_name in os.listdir(
            "data/record_segments/{}/{}/{}".format(map, module, mode)):
        try:
            segment_ori = record2vec("data/record_segments/{}/{}/{}/{}".format(map, module, mode, segment_name),
                                     map, test_system=module)

            if module == 'signal':

                segment_reg = record2vec(
                    "results/{}/{}/benchmark{}/{}/reg/{}".format(module, map, benchmark, mode,
                                                                 segment_name),
                    map, test_system=module)
                # segment_reduce_reg_vecs = segment_reduce_reg['scene_vec']

                start_ind_ori = 0
                start_ind_reg = 0
                if '_0' not in segment_name:
                    for j in range(len(segment_reg['time'])):
                        if segment_reg['time'][j] > 1:
                            start_ind_reg = j
                            break
                    for j in range(len(segment_ori['time'])):
                        if segment_ori['time'][j] > 1:
                            start_ind_ori = j
                            break

                segment_reduce_ori_vec = segment_ori['scene_vec'][start_ind_ori + 1]
                segment_reduce_reg_vecs = segment_reg['scene_vec'][start_ind_reg + 1:]

                mismatch = 0

                for vec in segment_reduce_reg_vecs:
                    if vec != segment_reduce_ori_vec:
                        mismatch += 1

                if float(mismatch) / len(segment_reduce_reg_vecs) > 0.1:
                    faults[segment_name] = 1
                else:
                    faults[segment_name] = 0
        except:
            pass

    return faults


def fault_compare():
    maps = ['cubetown']
    modules = ['signal']
    faults_all = {'signal': {}, 'obstacle': {}, 'planning': {}, 'prediction': {}}
    faults_reduced = {'signal': {}, 'obstacle': {}, 'planning': {}, 'prediction': {}}

    num_benchmarks = 9
    for module in modules:
        for map in maps:
            faults_all[module][map] = {}
            faults_reduced[module][map] = {}
            for benchmark in range(0, num_benchmarks):
                faults_reduced_benchmark = fault_evaluate(module, map, benchmark, 'reduced')
                faults_all_benchmark = fault_evaluate(module, map, benchmark, 'all')
                faults_reduced[module][map][benchmark] = faults_reduced_benchmark
                faults_all[module][map][benchmark] = faults_all_benchmark

    for module in modules:
        fault_total = 0
        fault_covered = 0
        for map in maps:
            for benchmark in range(0, num_benchmarks):
                segments = [segment for segment in faults_reduced[module][map][benchmark] if
                            segment in faults_all[module][map][benchmark]]
                for segment in segments:
                    if faults_all[module][map][benchmark][segment] == 1:
                        fault_total += 1
                        if faults_reduced[module][map][benchmark][segment] == 1:
                            fault_covered += 1

    save_path = 'results/faults.pickle'
    with open(save_path, 'wb') as handle:
        pickle.dump(faults_reduced, handle, protocol=pickle.HIGHEST_PROTOCOL)


def prioritization(prioritization_method='sc'):
    maps = ['cubetown']
    modules = ['signal']
    num_benchmarks = 9
    file = open('results/faults.pickle', 'rb')
    faults_result = pickle.load(file)
    file.close()

    apfd_results = {'signal': {}, 'obstacle': {}, 'planning': {}, 'prediction': {}}
    top_k_results = {'signal': {}, 'obstacle': {}, 'planning': {}, 'prediction': {}}
    for module in modules:
        for map in maps:
            apfd_results[module][map] = {}
            top_k_results[module][map] = {}
            apfd_vals = []
            top_k_vals = []
            if prioritization_method == 'rd':
                for benchmark in range(2, num_benchmarks):
                    apfd_benchmark = []
                    top_k_benchmark = []
                    for i in range(100):
                        sorted_segments_all = prioritize_by_random(map, module)
                        fault_result = {}
                        for k, v in faults_result[module][map][benchmark].items():
                            fault_result[int(k.split('.')[0].split('_')[1])] = v

                        print('fault result: ', fault_result)
                        sorted_segments_reduced = []
                        for segment_id in sorted_segments_all:
                            if segment_id in fault_result:
                                sorted_segments_reduced.append(segment_id)


                        apfd_val = apfd(sorted_segments_reduced, fault_result)
                        top_k_val = top_k(sorted_segments_reduced, fault_result)
                        apfd_benchmark.append(apfd_val)
                        top_k_benchmark.append(top_k_val)

                    apfd_vals.append(np.mean(apfd_benchmark))
                    top_k_vals.append(np.mean(top_k_vals))

            else:
                if prioritization_method == 'sc':
                    sorted_segments_all = prioritize_by_semantic_coverage(map, module)
                elif prioritization_method == 'rsc':
                    sorted_segments_all = prioritize_by_rsc(map, module)
                elif prioritization_method == 'ch':
                    sorted_segments_all = prioritize_by_ch(map, module)

                for benchmark in range(2, num_benchmarks):
                    fault_result = {}
                    for k,v in faults_result[module][map][benchmark].items():
                        fault_result[int(k.split('.')[0].split('_')[1])] = v

                    sorted_segments_reduced = []
                    for segment_id in sorted_segments_all:
                        if segment_id in fault_result:
                            sorted_segments_reduced.append(segment_id)

                    apfd_val = apfd(sorted_segments_reduced, fault_result)
                    top_k_val = top_k(sorted_segments_reduced, fault_result)
                    print('apfd: ', apfd_val)
                    print('top_k: ', top_k_val)
                    apfd_vals.append(apfd_val)
                    top_k_vals.append(top_k_val)

            print(module, map, np.mean(apfd_vals), np.mean(top_k_vals))

time_consumption()
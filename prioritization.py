import pickle
import numpy as np
import random

def get_mutant_functions(mutant_file):
    mutant_log = open(mutant_file, 'r')
    lines = mutant_log.readlines()

    mutant_file = []
    mutant_pos = []
    for line in lines:
        if 'source file' in line:
            mutant_file.append(line.split(' ')[-1][:-1])
        elif '==>' in line:
            mutant_pos.append(line.split(' ')[-1][:-1])

    mutants = [(a, b) for a, b in zip(mutant_file, mutant_pos)]
    return mutants


def prioritize_by_random(map, module):
    segment_inds = prioritize_by_ch(map, module)
    random.shuffle(segment_inds)
    return segment_inds

def prioritize_by_fun_call(map, module):
    pass

def prioritize_by_ch(map, module):
    file = open('data/record_data/original/{}_segments.pickle'.format(map), 'rb')
    segments_data = pickle.load(file)
    file.close()

    return list(range(len(segments_data[module]['vecs'])))


def prioritize_by_rsc(map, module):
    file = open('data/record_data/original/{}_segments.pickle'.format(map), 'rb')
    segments_data = pickle.load(file)
    file.close()

    vec_total = np.sum(np.array(segments_data[module]['vecs']), axis=0)
    # vec_weight = [1] * len(vec_total)

    vec_weight = []
    for num in vec_total:
        if num != 0:
            vec_weight.append(float(np.sum(vec_total)) / num)
        else:
            vec_weight.append(0)

    print(vec_weight)
    vec_weight_new = []
    for weight in vec_weight:
        vec_weight_new.append(weight / sum(vec_weight))

    vec_weight = vec_weight_new
    print(vec_weight)


    vec_weight = np.array(vec_weight)

    segment_importance = []
    for vec in segments_data[module]['vecs']:
        vec_importance = np.dot(vec_weight, np.array(vec))
        # print(vec, vec_importance)
        segment_importance.append(vec_importance)


    seg_order_data = np.argsort(segment_importance)[::-1]


    return seg_order_data


def prioritize_by_semantic_coverage(map, module):
    file = open('data/record_data/original/{}_segments.pickle'.format(map), 'rb')
    segments_data = pickle.load(file)
    file.close()

    vec_total = np.sum(np.array(segments_data[module]['vecs']), axis=0)
    vec_weight = [1] * len(vec_total)

    vec_weight = np.array(vec_weight)

    segment_importance = []
    for vec in segments_data[module]['vecs']:
        vec_importance = np.dot(vec_weight, np.array(vec))
        # print(vec, vec_importance)
        segment_importance.append(vec_importance)


    seg_order_data = np.argsort(segment_importance)[::-1]


    return seg_order_data


def prioritize_fun(map, module, mutants_file):
    file = open('data/record_data/original/{}_segments.pickle'.format(map), 'rb')
    segments_info = pickle.load(file)
    file.close()

    mutants = get_mutant_functions(mutants_file)


def prioritize(map, module):
    prioritize_fun(map, module, 'benchmark/{}/mutant_record_{}.txt'.format(module, module))

import time
from mutate import mutate
import os
from shutil import copyfile
import subprocess




def findAllFile(base):
    files = []
    for root, ds, fs in os.walk(base):
        for f in fs:
            if '.cc' in f and 'test' not in f:
                files.append(os.path.join(root, f))
    print(files)
    return files


def add_mutants(mutant_system='prediction', num_mutants=5):
    if mutant_system == 'prediction':
        TARGET_SYSTEM = '../modules/prediction'
    elif mutant_system == 'obstacle':
        TARGET_SYSTEM = '../modules/perception/lidar'
    elif mutant_system == 'signal':
        TARGET_SYSTEM = '../modules/perception/camera/lib/traffic_light'



    c_files = findAllFile(TARGET_SYSTEM)
    # mutation_records = []

    if not os.path.exists('benchmark/{}'.format(mutant_system)):
        os.mkdir('benchmark/{}'.format(mutant_system))
        os.mkdir('benchmark/{}/original'.format(mutant_system))
        os.mkdir('benchmark/{}/mutants'.format(mutant_system))


    f = open('benchmark/{}/mutant_record_{}.txt'.format(mutant_system, mutant_system), 'w')

    mutant_no = 0
    for file in c_files:
        if mutant_no >= num_mutants:
            break
        while True:
            print(file)
            # backup original file
            copyfile(file, 'benchmark/{}/original/{}_{}'.format(mutant_system, mutant_no, file.split('/')[-1]))
            mutant_info = mutate(file, os.path.join('benchmark', mutant_system, 'mutants',
                                                    '{}_{}'.format(mutant_no, file.split('/')[-1])))
            if mutant_info == 'skip':
                os.remove('benchmark/{}/original/mutants/{}_{}'.format(mutant_system, mutant_no, file.split('/')[-1]))
                break
            # replace mutant file with original one
            elif mutant_info != 'fail':
                copyfile(os.path.join('benchmark',mutant_system, 'mutants',
                                                    '{}_{}'.format(mutant_no, file.split('/')[-1])), file)
                # batcmd = '../apollo.sh build_opt_gpu'
                result = subprocess.Popen(["../apollo.sh", "build_opt_gpu"],
                                          stdout=subprocess.PIPE).communicate()[0]
                result = result.split('[')[-1]
                # recover original file
                copyfile('benchmark/{}/original/{}_{}'.format(mutant_no, file.split('/')[-1]), file)
                if '1' in result:
                    print(result)
                    f.write('source file: ' + file + '\n')
                    f.write('backup file: ' + 'benchmark/{}/original/{}_{}'.format(mutant_no, file.split('/')[-1]) + '\n')
                    f.write('change info: ' + mutant_info + '\n')
                    # change_record = {'source file': file, 'backup': 'mutation_testing/original/' + file.split('/')[-1],
                    #                  'change info': mutant_info}
                    mutant_no += 1
                    break


def check_mutants():
    f = open('mutant_record.txt', 'r')
    source_files = []
    replace_files = []
    log = f.readlines()
    for line in log:
        if 'source file' in line:
            replace_files.append(line.split(' ')[-1][:-1])
        elif 'backup file' in line:
            source_files.append(line.split(' ')[-1][:-1])

    # for (source_file, replace_file) in zip(source_files, replace_files):
    #     print(replace_file, source_file)
    #     copyfile(replace_file, source_file)

    modifiedPath = "mutation_testing/mutants"
    originalPath = "mutation_testing/original"

    if not os.path.exists(modifiedPath):
        os.mkdir(modifiedPath)

    if not os.path.exists(originalPath):
        os.mkdir(originalPath)

    # benchmark = 150
    failed = []
    for (source_file, replace_file) in zip(source_files, replace_files):
        source_file = source_file.split('/')[-1]
        print(source_file, replace_file)
        copyfile(os.path.join(modifiedPath, source_file), replace_file)
        time.sleep(1)
        result = subprocess.Popen(["../apollo.sh", "build_opt_gpu"],
                                  stdout=subprocess.PIPE).communicate()[0]
        result = result.split('[')[-1]
        if '1' not in result:
            failed.append(source_file)

        copyfile(os.path.join(originalPath, source_file), replace_file)
        time.sleep(1)

    print('failed: ', failed)


# time.sleep(10)
# add_mutants('signal')


# check_mutants()

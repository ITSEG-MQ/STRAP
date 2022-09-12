import os

def collect_data(mutant_system='obstacle', mode='ori', segments_folder='all'):
    f = open('benchmark/{}/mutant_record_{}.txt'.format(mutant_system, mutant_system), 'r')
    source_files = []
    replace_files = []
    log = f.readlines()
    for line in log:
        if 'source file' in line:
            replace_files.append(line.split(' ')[-1][:-1])
        elif 'backup file' in line:
            source_files.append(line.split(' ')[-1][:-1])

    benchmark = 0

    for i in range(benchmark, 9):
        source_file = source_files[i]
        replace_file = replace_files[i]
        source_file = source_file.split('/')[-1]
        print(source_file, replace_file)
        os.system("./exp_{}.sh {} {} {} {} {}".format(mode, mutant_system, i, source_file, replace_file, segments_folder))



collect_data('signal', mode='reg', segments_folder='all')
collect_data('signal', mode='reg', segments_folder='reduced')
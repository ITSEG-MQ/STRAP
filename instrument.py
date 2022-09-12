import os
import re
import shutil
import time

from dw import Connection


def de_instrument(source_folder):
    g = os.walk(source_folder)
    for path, dir_list, file_list in g:
        for file_name in file_list:
            if '.cc' in file_name:
                fileName = os.path.join(path, file_name)
                print(fileName)
                if fileName[-3:] == '.cc':
                    print ('============================================')
                    print (fileName + ' is Working.')
                    print ('============================================')
                    f = open(fileName, 'r')
                    # print ('Import stdout')
                    #
                    lines = f.readlines()
                    fileContent = ""
                    for line in lines:
                        if 'AINFO << __FILE__' not in line:
                            fileContent += line

                    f.close()
                    f = open(fileName, 'w')
                    f.write(fileContent)
                    f.close()


function_rule = '([a-zA-Z_][a-zA-Z0-9_]*)([\n\r\s]+)([a-zA-Z_][a-zA-Z0-9_]*)::([a-zA-Z_][a-zA-Z0-9_]*)\(([\s\S]*?)([\n\r\s]+)\{'


# inputStr = '\n\t static bool once = true; \n\t if (once) \n\t { \n\t  AINFO << "execute: " <<  __FILE__ << " " << __func__ << " " << __LINE__;\n\t } \n\t once = false;'

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


def instrument_file(file_name, pos):
    inputStr = '\n\t AINFO << "execute: " <<  __FILE__ << " " << __func__ << " " << {};\n\t'.format(pos)
    f = open(file_name, 'r')
    lines = f.readlines()
    fileContent = ""
    mod_build = False
    add_pos = None

    for j in range(int(pos)-2, 0, -1):
        if ';' in lines[j] or '{' in lines[j]:
            add_pos = j
            break

    print(pos, add_pos)
    for i, line in enumerate(lines):
        if 'namespace' in line and 'namespace' not in fileContent and 'cyber/common/log.h' not in fileContent:
            fileContent += '#include "cyber/common/log.h"\n'
            mod_build = True
        #
        if i == add_pos+1:
            fileContent += inputStr

        fileContent += line


    f.close()

    f = open(file_name, 'w')
    f.write(fileContent)
    f.close()

    return mod_build

def instrument(source_folder):
    inputStr = '\n\t AINFO << "execute: " <<  __FILE__ << " " << __func__ << " " << __LINE__;\n\t'
    g = os.walk(source_folder)
    build_files = []
    # tree = ['../modules/planning/navi_planning.cc']
    for path, dir_list, file_list in g:
        # print(path)
        # if 'math' not in path and 'common' not in path and 'tasks' not in path and 'tuning' not in path:
        for file_name in file_list:
            if '.cc' in file_name:
                fileName = os.path.join(path, file_name)
                print(fileName)
                if fileName[-3:] == '.cc':
                    print ('============================================')
                    print (fileName + ' is Working.')
                    print ('============================================')
                    f = open(fileName,'r')
                    # print ('Import stdout')
                    #
                    lines = f.readlines()
                    fileContent = ""
                    for line in lines:
                        if 'namespace' in line and 'namespace' not in fileContent and 'cyber/common/log.h' not in fileContent:
                            fileContent += '#include "cyber/common/log.h"\n'
                        fileContent += line

                    print ('============================================')
                    print ('============================================')
                    rule = re.compile(function_rule)
                    matchStr = rule.finditer(fileContent)
                    currentEnd = 0
                    beforeEnd = 0
                    outputStr = ''
                    i = 0
                    for matchArray in matchStr:
                        currentEnd = matchArray.end()
                        outputStr += fileContent[beforeEnd:currentEnd] + inputStr
                        beforeEnd = currentEnd
                        i += 1

                    outputStr += fileContent[beforeEnd:]

                    print(i)
                        # print (outputStr)
                    f.close()
                    # f = open(fileName,'w')
                    # f.write(outputStr)
                    # f.close()


def collect_log_data(map, test_system='obstacle'):
    dw = Connection()
    if map == "san":
        hdmap = "San Francisco"
    else:
        hdmap = map
    dw.set_hd_map(hdmap)
    time.sleep(3)
    record_logs = []

    record_path = 'data/records/{}.record'.format(map)
    if test_system == 'obstacle':
        module = 'Perception'
    elif test_system == 'signal':
        module = 'Traffic Light'
    elif test_system == 'planning':
        module = 'Planning'
    elif test_system == 'prediction':
        module = 'Prediction'

    dw.enable_module(module)
    if test_system == 'obstacle':
        os.system('cyber_recorder play -f {} -k /apollo/perception/obstacles  /apollo/prediction'.format(record_path))
    elif test_system == 'signal':
        os.system('cyber_recorder play -f {} -k /apollo/perception/traffic_light'.format(record_path))
    elif test_system == 'planning':
        os.system('cyber_recorder play -f {} -k /apollo/planning'.format(record_path))
    elif test_system == 'prediction':
        os.system('cyber_recorder play -f {} -k /apollo/prediction'.format(record_path))



    dw.disable_module(module)
    # files = os.listdir('../data/log')
    # files.sort(key=lambda x: os.path.getmtime(os.path.join('../data/log', x)), reverse=True)


    # for file_name in files:
    #     if 'mainboard.log' in file_name:
    #         record_log = file_name
    #         break
        # if test_system == 'obstacle' and 'perception.log' in file_name:
        #     record_log = file_name
        #     break
        # elif test_system == 'signal' and 'perception.log' in file_name:
        #     record_log = file_name
        #     break


    # shutil.copyfile(os.path.join('../data/log', record_log),
    #                 'data/running_log/{}/{}.txt'.format(test_system, map))

    time.sleep(3)


# instrument('../modules/perception/lidar')

def instrument_module(module):
    mutants = get_mutant_functions('benchmark/{}/mutant_record_{}.txt'.format(module, module))
    print(mutants)
    # for i, (file_name, pos) in enumerate(mutants):
    #     print(file_name, pos)
    #     if i <= 9:
    #         if '.cc' in file_name:
    #             instrument_file(file_name, pos)
    #     else:
    #         break

if __name__ == '__main__':
    collect_log_data('gomentum', 'prediction')
    # instrument_module('prediction')
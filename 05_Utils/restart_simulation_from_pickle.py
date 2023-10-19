import os

def restart_simulation_from_pickle(case_route, case_name, pickle_file_path):
    os.system('sharpy ' + case_route + case_name + '.sharpy -r ' + pickle_file_path)


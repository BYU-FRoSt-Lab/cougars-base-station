import os
import os.path

path_to_python_script = os.path.dirname(__file__)
new_mission_dir= path_to_python_script + '/new_mission'
os.system('cd ' + new_mission_dir + '&& rm -rf *') 

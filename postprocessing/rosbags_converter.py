
import os

from sys import argv

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

import pandas as pd



def generate_typestore(
        msgs_dirs:list[str], 
        base_version = Stores.ROS2_HUMBLE,
        verbose:bool = True
):
    """
    A Typestore 
    """
    typestore = get_typestore(base_version)
    for msgs_dir in msgs_dirs:
        msgs_source = Path(msgs_dir)
        for root, dirs, files in os.walk(msgs_source):
            # Ignore messages in build and install folders
            if "build" in dirs: dirs.remove("build")
            if "install" in dirs: dirs.remove("install")

            if os.path.basename(root) == 'msg':
                if verbose: print(f"Registering Messages in {os.path.abspath(root)}")
                
                msg_name_prefix = os.path.basename(Path(root).parent) + "/msg/"

                for file in files:
                    if(file.endswith(".msg")):
                        filepath = Path(root) / file
                        # if verbose: print(f"\t{os.path.abspath(filepath)}")
                        text = filepath.read_text()
                        name = msg_name_prefix + file[:-4]
                        typestore.register(get_types_from_msg(text, name))
                        if verbose: print(f"\tRegistered {name}")
    return typestore


def convert_rosbags(bags_dirs:list[str], typestore):
    """
    Converts rosbags into pandas dataframes that can be easily manipulated with a python workflow.
    """
    dataframes: dict[pd.DataFrame|dict] = []

    for bags_dir in bags_dirs:
        for root, dirs, files in os.walk(bags_dir):
            with AnyReader([bagpath], default_typestore=typestore) as reader:
                pass    

    return dataframes


def save_as_csv(dataframes, target):
    """
    Saves pandas dataframes into csv files
    """



if __name__ == '__main__':

    rosbags_dir = ""
    msgs_dir = ""
    out_dir = ""
    for arg in argv:
        if arg.startswith("bagsDir:="):
            rosbags_dir = arg[9:]
        elif arg.startswith("msgsDir:="):
            msgs_dir = arg[9:]
        elif arg.startswith("outDir:="):
            out_dir = arg[8:]    

    typestore = generate_typestore([msgs_dir])



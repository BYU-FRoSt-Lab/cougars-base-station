
import os

from sys import argv

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

import pandas as pd



def generate_typestore(
        msgs_dirs:list[str] = [], 
        ros_distro = Stores.ROS2_HUMBLE,
        verbose:bool = True
):
    """
    Creates a Typestore that has all the ros messages for a base ros distribution
    as well as any custom messages you want to add

    args:
     - msgs_dirs: A list of directories that contain packages with custom ros messages.
     - ros_distro: The base distribution to generate the Typestore from.
     - verbose: If true, prints updates as messages are registered

    return:
     - typestore: The Typestore with the base and custom ros message types

    The method loops through all subdirectories of "msgs_dirs" until it finds a package with a 
    'msgs' subdirectory. Then it registers all the the "*.msg" files in that directory.
    It ignores anything found in the "build" and "install" directories.
    """
    typestore = get_typestore(ros_distro)
    if verbose: print(f"Initialized Typestore with {ros_distro.name} base messages")
    for msgs_dir in msgs_dirs:
        msgs_source = Path(msgs_dir)
        if verbose: print(f"Searching {os.path.abspath(msgs_dir)} for ROS messages...")
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


def rosmsg_generator(
        bags_dir:str, 
        typestore, 
        topics:list[str]|None = None, 
        verbose:bool=False
):
    """
    generates ros messages of the given topics
    """
    stack = [os.path.basename(bags_dir)]
    for root, dirs, files in os.walk(bags_dir):
        path = Path(root)

        # process rosbag
        if "metadata.yaml" in files:
            if verbose: print(f"Unpacking {os.path.abspath(path)}")
            msgs: dict[str, pd.DataFrame] = dict()
            with AnyReader([path], default_typestore=typestore) as reader:
                if topics is None:
                    connections = [x for x in reader.connections]
                else:
                    connections = [x for x in reader.connections if x.topic in topics]

                for connection, timestamp, rawdata in reader.messages(connections=connections):
                    rosmsg = reader.deserialize(rawdata, connection.msgtype)
                    yield connection, rosmsg, path
                    


def convert_rosbags(bags_dir:str, typestore, verbose:bool=False):
    """
    Converts rosbags into pandas dataframes that can be easily manipulated with a python workflow.
    """
    def values_generator(msg, prefix=""):
        for attr, val in msg.__dict__.items():
            if attr=='__msgtype__': continue
            name = f"{prefix}.{attr}"
            if name[0]=='.': name = name[1:]
            if hasattr(val, "__dict__"):
                yield from values_generator(val, name)
            else:
                yield name, val

    dataframes: dict[str, dict[str, pd.DataFrame]] = dict()
    topics = None
    for connection, msg, path in rosmsg_generator(bags_dir, typestore, verbose=verbose):        
        if path not in dataframes.keys():
            dataframes[path] = dict()
            topics = dataframes[path]
        topicname = connection.topic.replace('/', '.')
        if topicname[0]=='.': topicname = topicname[1:]
        if topicname not in topics:
            topics[topicname] = list()
        data = topics[topicname]
        data.append(dict(values_generator(msg)))

    for path, topics in dataframes.items():
        print(f"Converting {os.path.abspath(Path(bags_dir)/path)}")
        for topic in topics.keys():
            topics[topic] = pd.DataFrame(topics[topic])
            # print(topics[topic])

    return dataframes


        # if path != newpath:
        #     # add path to dataframes tree
        #     path = newpath
        #     bag_dfs = dataframes
        #     dirs = path.relative_to(bags_dir).parts
        #     for dir in dirs[:-1]:
        #         if dir not in bag_dfs:
        #             bag_dfs[dir] = dict()
        #         bag_dfs = bag_dfs[dir]
        #     converted_name = "converted_" + dirs[-1]
        #     if converted_name not in bag_dfs:
        #         bag_dfs[converted_name] = dict()
        #     bag_dfs = bag_dfs[converted_name]

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

    typestore = generate_typestore([msgs_dir], verbose=True)

    dataframes = convert_rosbags(rosbags_dir, typestore, verbose=True)
    # print(dataframes)



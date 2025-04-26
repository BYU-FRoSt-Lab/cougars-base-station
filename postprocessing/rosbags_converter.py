
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
                print(topics)
                if topics is None:
                    connections = [x for x in reader.connections]
                else:
                    connections = [x for x in reader.connections if x.topic in topics]
                for connection, timestamp, rawdata in reader.messages(connections=connections):
                    rosmsg = reader.deserialize(rawdata, connection.msgtype)
                    yield connection, rosmsg, path
                    


def convert_rosbags(bags_dir:str, typestore, topics, verbose:bool=False):
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
    topic_data = None
    for connection, msg, path in rosmsg_generator(bags_dir, typestore, topics=topics, verbose=verbose):
        if connection.topic=='/rosout': continue
        relpath = path.relative_to(bags_dir)       
        if relpath not in dataframes.keys():
            dataframes[relpath] = dict()
            topic_data = dataframes[relpath]
        topic = connection.topic
        if topic not in topic_data:
            topic_data[topic] = list()
        data = topic_data[topic]
        data.append(dict(values_generator(msg)))

    for path, topic_data in dataframes.items():
        print(f"Converting {path}")
        for topic in topic_data.keys():
            topic_data[topic] = pd.DataFrame(topic_data[topic])

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

def save_to_csv(
        dataframes:dict[str, dict[str, pd.DataFrame]],
        out_dir:str,
        verbose:bool = False
):
    """
    Saves pandas dataframes into csv files
    """
    outpath = Path(out_dir)
    for relpath, topics in dataframes.items():
        relpath = relpath.parent / ("converted_"+os.path.basename(relpath))
        if verbose: print(f"Saving {os.path.abspath(outpath / relpath)}")
        for topic, dataframe in topics.items():
            topicname = topic.replace('/', '.') + ".csv"
            if topicname[0]=='.': topicname = topicname[1:]
            fullpath = outpath / relpath / topicname
            fullpath.parent.mkdir(parents=True, exist_ok=True)
            dataframe.to_csv(os.path.abspath(fullpath))



if __name__ == '__main__':

    rosbags_dir = "."
    msgs_dir = "."
    out_dir = None
    topics = None
    for arg in argv:
        if arg.startswith("bagsDir:="):
            rosbags_dir = arg[9:]
        elif arg.startswith("msgsDir:="):
            msgs_dir = arg[9:]
        elif arg.startswith("outDir:="):
            out_dir = arg[8:]
        elif arg.startswith("topics:="):
            topics = arg[8:].split(',')

    print(topics)

    if out_dir == None:
        out_dir = Path(rosbags_dir) / "converted_bags"

    typestore = generate_typestore([msgs_dir], verbose=True)
    dataframes = convert_rosbags(rosbags_dir, typestore, topics=topics, verbose=True)
    save_to_csv(dataframes, out_dir, verbose=True)



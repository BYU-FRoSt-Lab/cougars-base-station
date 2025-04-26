import os
from sys import argv
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

import pandas as pd


"""
converts rosbags into pandas DataFrames and csv files.

Usage:
python3 rosbags_converter.py bagsDir:=/path/to/rosbags msgsDir:=/path/to/custom/message/definitions outDir:=/path/to/output/dir topics:=/list,/of,/topics

required libraries:
    rosbags
    pandas

Make sure the list of topics has no spaces.
'msgsDir' is typically the src directory of your ros workspace.
If 'topics' is undefined, all topics will be converted.
If 'outDir' is undefined, it defaults to <bagsDir>/converted_bags.
'bagsDir' and 'msgsDir' defaults to the current working directory.
For easy batch processing, rosbags_converter recursively searches for 
rosbags in 'bagsDir' and keeps the same file structure in 'outDir'.
You can use relative paths (such as ~/mydir, ../../mydir, etc.)

You can also import this module and use it directly in your code.
"""


def generate_typestore(
        msgs_dirs:list[str] = [], 
        ros_distro = Stores.ROS2_HUMBLE,
        verbose:bool = True
):
    """
    Creates a Typestore that includes all the ros messages for a base ros distribution
    as well as any custom messages you want to add

    args:
        msgs_dirs: A list of directories that contain packages with custom ros messages.
        ros_distro: The base distribution to generate the Typestore from.
        verbose: If true, prints updates as messages are registered

    return:
        typestore: The Typestore with the base and custom ros message types

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
        bags_dirs:list[str|Path], 
        typestore, 
        topics:list[str]|None = None, 
        verbose:bool=False
):
    """
    Generates ros messages from a directory containing rosbags.

    args:
        bags_dirs: a list of directorys containing rosbags
        typestore: the typestore of the message types in your rosbags
        topics: the list of topics to report. If 'None', reports all topics
        verbose: if true, prints updates as rosbags are unpacked
    
    yields:
        connection: metadata for the ros message
        msg: a ros message
        path (pathlib.Path): the path to the rosbag from which this message was unpacked
    """
    for bags_dir in bags_dirs:
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
                        msg = reader.deserialize(rawdata, connection.msgtype)
                        yield connection, msg, path
                        


def convert_rosbags(bags_dir:str, typestore, topics, verbose:bool=False):
    """
    Converts rosbags into pandas DataFrames.

    args:
        bags_dir: a directory containing rosbags
        typestore: the typestore of the message types in your rosbags
        topics: the list of topics to report. If 'None', reports all topics
        verbose: if true, prints updates as rosbags are unpacked

    returns:
        dataframes: a dictionary from relative rosbag paths to topics, where 
            topics is a dictionary from topic names to pandas DataFrames
    """
    def values_generator(msg, prefix=None):
        for attr, val in msg.__dict__.items():
            if attr=='__msgtype__': continue
            name = attr if prefix is None else f"{prefix}.{attr}"
            if hasattr(val, "__dict__"):
                yield from values_generator(val, name)
            else:
                yield name, val

    dataframes: dict[Path, dict[str, pd.DataFrame]] = dict()
    topic_data = None
    for connection, msg, path in rosmsg_generator([bags_dir], typestore, topics=topics, verbose=verbose):
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


def save_to_csv(
        dataframes:dict[str, dict[str, pd.DataFrame]],
        out_dir:str,
        verbose:bool = False
):
    """
    Saves pandas DataFrames generate from rosmsgs to csv files. 

    args:
        dataframes: a dictionary from relative rosbag paths to topics, where 
            topics is a dictionary from topic names to pandas DataFrames.
        out_dir: the directory to save the csv files to
        verbose: if true, prints updates as dataframes are saved
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

    def pathof(string:str):
        return Path.home()/string[2:] if string[0]=='~' else Path(string)

    rosbags_dir = Path.cwd()
    msgs_dirs = [Path.cwd()]
    out_dir = None
    topics = None
    for arg in argv:
        if arg.startswith("bagsDir:="):
            rosbags_dir = pathof(arg[9:])
        elif arg.startswith("msgsDir:="):
            msgs_dirs = [pathof(s) for s in arg[9:].split(',')]
        elif arg.startswith("outDir:="):
            out_dir = pathof(arg[8:])
        elif arg.startswith("topics:="):
            topics = arg[8:].split(',')

    if out_dir == None:
        out_dir = Path(rosbags_dir) / "converted_bags"

    typestore = generate_typestore(msgs_dirs, verbose=True)
    dataframes = convert_rosbags(rosbags_dir, typestore, topics=topics, verbose=True)
    save_to_csv(dataframes, out_dir, verbose=True)



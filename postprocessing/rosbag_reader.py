
import os

from sys import argv

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def generate_typestore(
        workspace_sources:list[str], 
        base_version = Stores.ROS2_HUMBLE,
        verbose:bool = True
):
    """
    A Typestore 
    """
    typestore = get_typestore(base_version)
    for workspace_source in workspace_sources:
        for root, dirs, files in os.walk(workspace_source):
            if os.path.basename(root) == 'msg':
                if verbose: print(f"Registering messages in {root}")
                typestore.register(get_types_from_msg(root))
    return typestore


def convert_rosbags(bagpaths, typestore):
    """
    Converts rosbags into pandas dataframes that can be easily manipulated with a python workflow.
    """
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        pass

def save_as_csv(dataframes, target):
    """
    Saves pandas dataframes into csv files
    """


if __name__ == '__main__':
    bagpath = Path(argv[1])



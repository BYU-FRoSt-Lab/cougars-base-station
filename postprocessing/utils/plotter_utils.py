from . import rosbags_converter as rc

def get_dataframes(
        rosbags_dir:str,
        rosmsgs_dir:str,
        csv_dir:str,
        keywords:list[str],
        topics:list[str]=None,
        verbose:bool=True,
):
    print("converting rosbags")
    typestore = rc.generate_typestore(rosmsgs_dir)
    dataframes = rc.convert_rosbags(rosbags_dir, typestore, 
                                    keywords=keywords,verbose=verbose,
                                    topics=topics)
    rc.save_to_csv(dataframes, csv_dir, verbose=verbose)
    dataframes = rc.load_dataframes(csv_dir, keywords=keywords)
    return dataframes
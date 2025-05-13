# Import statements
import argparse
import sys
import numpy as np
import math
import os
import yaml
import re
from datetime import date

# Assuming src.inputparse exists and is correctly structured
import src.inputparse as inputparse


EARTH_CIRCUMFRENCE_METERS = 40075000
EARTH_RADIUS_METERS       = 6371000
METERS_PER_DEGREE_LON     = EARTH_CIRCUMFRENCE_METERS / 360 # Not used in Haversine directly
POINTS_LINE_BASE          = "	           points = "
DEPTH_LINE                = "             depth = " # Not used directly, but good constant
DEFAULT_HEADING_LINE      = "                heading = " # Not used
DEFAULT_DEPTH_LINE        = "                depth = " # Not used
MOOS_COMMUNITY_LINE       = "Community    = "
OUTPUTS_DIR               = 'moos_converter_output/'
# Default origins (will be overridden by mission_params.yaml)
# LONG_ORIGIN = 40.23849638603186 # Example: This seems like LAT
# LAT_ORIGIN  = -111.73956745065406 # Example: This seems like LON
# These will be loaded from YAML

# Converts a point's lat/long into meters using Haversine's formula
# Input:  The set of reference cordinates (refLong, refLat), the set of point cordinates (pointLong, pointLat)
# Output: The x & y cordinates of the point in meters relative to the reference
def CalculateHaversine(refLong, refLat, pointLong, pointLat):
    # convert GPS coordinates to radians
    ref_lat_rad     = math.radians(float(refLat))
    ref_long_rad    = math.radians(float(refLong))
    point_lat_rad   = math.radians(float(pointLat))
    point_lon_rad   = math.radians(float(pointLong))

    # calculate distance and direction from reference point to GPS coordinate
    delta_lon = point_lon_rad - ref_long_rad
    delta_lat = point_lat_rad - ref_lat_rad
    a = math.sin(delta_lat/2)**2 + math.cos(ref_lat_rad) * math.cos(point_lat_rad) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = EARTH_RADIUS_METERS * c # Distance in meters

    # Calculate bearing (theta)
    # Adjusted for standard Cartesian coordinates where X is East, Y is North
    # MOOS often uses X as forward/East, Y as port/North from an East-facing start if not georeferenced
    # However, for geodetic to local tangent plane, a common convention is X=East, Y=North
    y_intermediate = math.sin(delta_lon) * math.cos(point_lat_rad)
    x_intermediate = math.cos(ref_lat_rad) * math.sin(point_lat_rad) - \
                     math.sin(ref_lat_rad) * math.cos(point_lat_rad) * math.cos(delta_lon)
    theta = math.atan2(y_intermediate, x_intermediate) # Angle from North, positive eastward

    # Convert polar (d, theta) to Cartesian (x, y)
    # x = East, y = North
    x_local = d * math.sin(theta)
    y_local = d * math.cos(theta)
    
    # Your original code had:
    # x = d * math.cos(theta)
    # y = -d * math.sin(theta)
    # This implies a different coordinate system for theta or the final axes.
    # If MOOS expects X forward, Y left (NED-like from a specific heading), then further rotation might be needed.
    # For now, using standard X=East, Y=North. If MOOS expects X,Y differently (e.g. pMarineViewer default view),
    # ensure the points align with that. Typically, MOOS waypoints are (x,y) in a local Cartesian grid.
    return x_local, y_local


def CopyTemplateFile(args, template_file_name, printingString='', mission_name_hint='', mission_output_dir=''):
    base_template_name, template_extension = os.path.splitext(template_file_name)
    target_file_name_only = ""

    if template_file_name.endswith('_deploy.sh'):
        target_file_name_only = mission_name_hint + '_deploy.sh'
    elif template_file_name.endswith('description.txt'):
        target_file_name_only = mission_name_hint + '_description.txt'
    else: # For .bhv, .moos, etc.
        if args.output: # User specified an output name via -o
            target_file_name_only = os.path.splitext(os.path.basename(args.output))[0] + template_extension
        else: # Default: use input file's basename + template's extension
            input_base = os.path.splitext(os.path.basename(args.input))[0]
            target_file_name_only = input_base + template_extension
            if not target_file_name_only: # Failsafe if input had no extension somehow
                 target_file_name_only = args.input + template_extension
    
    if not mission_output_dir:
        print("FATAL ERROR: mission_output_dir not provided to CopyTemplateFile!")
        return None # Critical failure

    outputFilePath = os.path.join(mission_output_dir, target_file_name_only)

    # Determine full path to the template file itself
    full_template_path = ""
    if template_file_name.startswith('template_') and (template_file_name.endswith('.sh') or template_file_name.endswith('.txt')):
        # These are special template files (e.g., 'template_deploy.sh') that are not in 'template_files/' folder
        # And their content is entirely replaced by printingString
        full_template_path = template_file_name # Assuming it's directly accessible or path is complete
    elif not template_file_name.endswith('.sh') and not template_file_name.endswith('.txt'):
        full_template_path = os.path.join('template_files', template_file_name)
    else: # For other .sh or .txt files not prefixed with 'template_' or if path is already full
        full_template_path = template_file_name


    outputStream = None
    try:
        outputStream = open(outputFilePath, "x") # Try exclusive creation
    except FileExistsError:
        print(f"{outputFilePath} already exists. Override? (y/n)")
        while True:
            user_choice = sys.stdin.readline().strip().lower()
            if user_choice == "y":
                print("Overwriting")
                outputStream = open(outputFilePath, "w") # Open in write mode
                break
            elif user_choice == "n":
                print(f"Skipping overwrite for {outputFilePath}.")
                return outputFilePath # Return path, but indicate it wasn't (re)written
            else:
                print("Invalid input. Please enter 'y' or 'n'.")
    except Exception as e:
        print(f"Error opening file {outputFilePath} for writing: {e}")
        return None

    if outputStream is None: # Means user chose 'n' for overwrite
        return outputFilePath

    # Write content based on template type
    try:
        if template_file_name.startswith('template_') and (template_file_name.endswith('.sh') or template_file_name.endswith('.txt')):
            outputStream.write(printingString)
        else: # Files based on a template from 'template_files/'
            try:
                with open(full_template_path, 'r') as templateFile:
                    lines = templateFile.readlines()
            except FileNotFoundError:
                print(f"ERROR: Template file '{full_template_path}' not found.")
                outputStream.close()
                # Clean up the file created by 'x' mode if template is missing and file is empty
                if os.path.exists(outputFilePath):
                    try:
                        if os.stat(outputFilePath).st_size == 0:
                            os.remove(outputFilePath)
                            print(f"Removed empty file: {outputFilePath}")
                    except OSError as e_rem:
                        print(f"Error removing empty file {outputFilePath}: {e_rem}")
                return None

            for currentLine in lines:
                if template_file_name.endswith(".bhv") and "points =" in currentLine:
                    outputStream.write(printingString + '\n')
                elif template_file_name.endswith(".moos") and "behaviors" in currentLine and "pShare" not in currentLine: # Avoid pShare line
                    outputStream.write(printingString + '\n') # printingString is the behavior_file_line
                elif template_file_name.endswith(".moos") and "COMMUNITY_NAME" in currentLine:
                    moos_community = input(f'What is the name of this moos community for {target_file_name_only} (example: coug1)? ')
                    outputStream.write(MOOS_COMMUNITY_LINE + moos_community + '\n\n')
                else:
                    outputStream.write(currentLine)
    finally:
        if outputStream:
            outputStream.close()

    print(f"Successfully wrote to {outputFilePath}")
    return outputFilePath


def main():
    today = date.today()

    answer = input('Have you updated your shared origin in mission_params.yaml for this multi-agent mission? (y/n) ')
    if answer.lower() != 'y':
        print('Please update the mission_params.yaml file, then try running this script again.')
        return 1

    lat_origin = None
    long_origin = None
    try:
        with open('../mission_params.yaml', 'r') as file:
            parameters = yaml.safe_load(file)
        lat_origin = float(parameters['/**']['ros__parameters']['origin.latitude'])
        long_origin = float(parameters['/**']['ros__parameters']['origin.longitude'])
        print(f"Using origin: Latitude={lat_origin}, Longitude={long_origin}")
    except FileNotFoundError:
        print("ERROR: ../mission_params.yaml not found. Cannot determine origin.")
        return 1
    except (KeyError, TypeError) as e:
        print(f"ERROR: Could not read origin from mission_params.yaml. Check format. Details: {e}")
        return 1

    parser = argparse.ArgumentParser(description="Converts a .waypoints file to MOOS mission files (.bhv, .moos, scripts).")
    parser.add_argument("input", help="The .waypoints file (e.g., myroute.waypoints)")
    parser.add_argument("-o", "--output", help="Base name for the output files and mission directory (e.g., 'MyMission1'). Extensions are added automatically.", default="")
    args = parser.parse_args()

    # Determine base output name for the directory and files
    if args.output:
        mission_base_name = os.path.splitext(args.output)[0] # Remove extension if user accidentally added one
    else:
        mission_base_name = os.path.splitext(os.path.basename(args.input))[0]

    # Ensure the main output directory (e.g., moos_converter_output/) exists
    if not os.path.exists(OUTPUTS_DIR):
        try:
            os.makedirs(OUTPUTS_DIR)
            print(f"Created base output directory: {OUTPUTS_DIR}")
        except OSError as e:
            print(f"Error creating base output directory {OUTPUTS_DIR}: {e}")
            return 1
            
    # Create the specific mission directory (e.g., moos_converter_output/MyMission1)
    mission_specific_output_path = os.path.join(OUTPUTS_DIR, mission_base_name)
    if not os.path.exists(mission_specific_output_path):
        try:
            os.makedirs(mission_specific_output_path)
            print(f"Created mission directory: {mission_specific_output_path}")
        except OSError as e:
            print(f"Error creating mission directory {mission_specific_output_path}: {e}")
            return 1
    else:
        print(f"Mission directory {mission_specific_output_path} already exists. Files may be overwritten if you choose 'y'.")

    #================#
    # PHASE 1: INPUT #
    #================#
    Waypoints = inputparse.parseInputs(args) # Ensure parseInputs can handle args object
    if not Waypoints:
        print("Failed to parse waypoints from input file. Exiting.")
        return 1
    description_text = input('Please type up a sufficient description for this mission: ')

    #====================================#
    # PHASE 2: Convert LAT/LONG to X/Y   #
    #====================================#
    coords = []
    for waypoint_obj in Waypoints: # Assuming Waypoints is a list of objects
        # Ensure waypoint_obj has attributes: latitude, longitude, depth
        try:
            x, y = CalculateHaversine(long_origin, lat_origin, waypoint_obj.longitude, waypoint_obj.latitude)
            x = round(x)
            y = round(y)
            coords.append([x, y, waypoint_obj.depth])
        except AttributeError as e:
            print(f"Error accessing waypoint attributes (latitude, longitude, depth): {e}. Please check your .waypoints parser.")
            return 1
        except ValueError as e:
            print(f"Error converting waypoint data (lat/lon/depth probably not numbers): {e}")
            return 1


    numCoords = len(coords)
    if numCoords == 0:
        print("No coordinates were processed. Exiting.")
        return 1

    # Creating the MOOS behavior string for waypoints
    waypoint_printing_string = POINTS_LINE_BASE
    for i, currentCoord in enumerate(coords):
        waypoint_printing_string += str(currentCoord[0]) + "," + str(currentCoord[1])
        if i < numCoords - 1:
            waypoint_printing_string += ":"
    
    # Align depths string under points string for readability in the .bhv file
    depth_string_padding = ' ' * (len(POINTS_LINE_BASE) - len("depths = ") -1) # Adjust padding as needed
    waypoint_printing_string += '\n' + depth_string_padding + 'depths = '
    for i, currentCoord in enumerate(coords):
        waypoint_printing_string += str(currentCoord[2]) # Assuming depth is the third element
        if i < numCoords - 1:
            waypoint_printing_string += ":"

    #=================#
    # PHASE 3: OUTPUT #
    #=================#
    print(f"\nGenerating mission files in: {mission_specific_output_path}")

    # Create .bhv file
    # The 'args' object is passed to CopyTemplateFile; its 'output' attribute guides the naming.
    generated_bhv_filepath = CopyTemplateFile(args, "template.bhv", printingString=waypoint_printing_string, mission_output_dir=mission_specific_output_path)
    if not generated_bhv_filepath:
        print("Failed to generate .bhv file. Aborting further file generation.")
        return 1
    
    mission_name_for_scripts = os.path.splitext(os.path.basename(generated_bhv_filepath))[0]

    # Create .moos file
    behavior_file_line_for_moos = f'  behaviors  = {os.path.basename(generated_bhv_filepath)}\n'
    generated_moos_filepath = CopyTemplateFile(args, "template.moos", printingString=behavior_file_line_for_moos, mission_name_hint=mission_name_for_scripts, mission_output_dir=mission_specific_output_path)
    if not generated_moos_filepath:
        print("Failed to generate .moos file.")
        # Decide if you want to continue or return, e.g., return 1

    # Create deploy script
    # The mission_name_hint ensures the script is named after the primary mission file (e.g., MyMission1_deploy.sh)
    moos_file_basename_for_script = os.path.basename(generated_moos_filepath) if generated_moos_filepath else mission_name_for_scripts + '.moos'
    deploy_moos_str = f"#!/bin/bash\n\n# Deploy script for {mission_name_for_scripts}\n# Generated on {today.strftime('%Y-%m-%d')}\n\nuPokeDB {moos_file_basename_for_script} DEPLOY=true MOOS_MANUAL_OVERRIDE=false\n"
    CopyTemplateFile(args, "template_deploy.sh", printingString=deploy_moos_str, mission_name_hint=mission_name_for_scripts, mission_output_dir=mission_specific_output_path)

    # Create description file
    full_description_content = f"Mission Description for: {mission_name_for_scripts}\n"
    full_description_content += f"Generated on: {today.strftime('%Y-%m-%d')}\n"
    full_description_content += f"Input .waypoints file: {args.input}\n"
    full_description_content += f"Origin Latitude: {lat_origin}\nOrigin Longitude: {long_origin}\n\n"
    full_description_content += f"User-provided description:\n{description_text}\n"
    CopyTemplateFile(args, "template_description.txt", printingString=full_description_content, mission_name_hint=mission_name_for_scripts, mission_output_dir=mission_specific_output_path)

    print(f"\nAll mission files generated in: {mission_specific_output_path}")

    # #=========================================#
    # # PHASE 4: Docker Operations (Optional) #
    # #=========================================#
    # # If you need to copy this directory to Docker:
    # # name_of_container = "your_docker_container_name"  # Define this
    # # path_in_container = "/path/inside/container"      # Define this
    # # docker_cp_command = f'docker cp "{mission_specific_output_path}" "{name_of_container}:{path_in_container}"'
    # # print(f"\nTo copy the mission directory to Docker, you can run:\n{docker_cp_command}")
    # # # print("Attempting to copy to Docker...")
    # # # status = os.system(docker_cp_command)
    # # # if status == 0:
    # # #     print("Successfully copied to Docker.")
    # # # else:
    # # #     print(f"Failed to copy to Docker. Exit status: {status}")

    return 0


if __name__ == "__main__":
    exitCode = main()
    print(f"\nProcess finished with code {str(exitCode)}")
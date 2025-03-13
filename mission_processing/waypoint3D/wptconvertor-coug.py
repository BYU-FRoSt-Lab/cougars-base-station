# Import statements
import argparse
import sys

import src.inputparse as inputparse
import numpy as np
import math
import os
import yaml
import re
from datetime import date


EARTH_CIRCUMFRENCE_METERS = 40075000
EARTH_RADIUS_METERS       = 6371000
METERS_PER_DEGREE_LON     = EARTH_CIRCUMFRENCE_METERS / 360
POINTS_LINE_BASE          = "	           points = "
DEPTH_LINE                = "             depth = "
DEFAULT_HEADING_LINE      = "                heading = "
DEFAULT_DEPTH_LINE        = "                depth = "
MOOS_COMMUNITY_LINE       = "Community    = "
OUTPUTS_DIR               = 'outputs/'
LONG_ORIGIN = 40.23849638603186
LAT_ORIGIN  = -111.73956745065406

# Converts a point's lat/long into meters using Haversine's formula
# Input:  The set of reference cordinates, the set of point cordinates
# Output: The x & y cordinates of the point
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
    d = EARTH_RADIUS_METERS * c
    theta = math.atan2(math.sin(delta_lon) * math.cos(point_lat_rad), math.cos(ref_lat_rad) * math.sin(point_lat_rad) - math.sin(ref_lat_rad) * math.cos(point_lat_rad) * math.cos(delta_lon))

    # convert distance and direction to xy MOOS coordinates in meters
    x = d * math.cos(theta)
    y = -d * math.sin(theta)
    return x, y

def CopyTemplateFile(args, template_file_name, printingString = '', mission_name = ''):
    # Split the file name and get the extension
    base_name, extension = os.path.splitext(template_file_name)
    
    
    if not template_file_name.endswith('.sh') and not template_file_name.endswith('.txt'):
        template_file_name = 'template_files/' + template_file_name
        # First, read in the template file
        templateFile = open(template_file_name)
        lines = []
        for currentLine in templateFile:
            lines.append(currentLine)
        templateFile.close()

        # Set up the output file name
        outputFileName = args.output
        # if template_file_name.endswith(".bhv"):
        if outputFileName == "":
            inputWithoutExtension = args.input[0:-10]
            outputFileName = inputWithoutExtension + extension
            del inputWithoutExtension
        elif not outputFileName.endswith(extension):
            outputFileName = outputFileName + extension


        # Open the file and Check if the output file already exists
        try:
            outputStream = open(OUTPUTS_DIR + outputFileName, "x")
        except FileExistsError:
            print(outputFileName + " already exists.  Override? (y/n)")
            while True:
                c = sys.stdin.read(1)
                if c == "y" or c == "Y":
                    print("Overwriting")
                    outputStream = open(OUTPUTS_DIR + outputFileName, "w")
                    break
                if c == "n" or c == "N":
                    print("Canceling operation")
                    return 3

        for currentLine in lines:
            if template_file_name.endswith(".bhv") and currentLine.find("points") != -1:
                outputStream.write(printingString + '\n')
            elif template_file_name.endswith(".moos") and currentLine.find("behaviors") != -1:
                outputStream.write(printingString + '\n')
            elif template_file_name.endswith(".moos") and currentLine.find("COMMUNITY_NAME") != -1:
                outputStream.write(MOOS_COMMUNITY_LINE + input('What is the name of this moos community (example: coug1) ') + '\n\n')
            else:
                outputStream.write(currentLine)
        
        #Close up
        outputStream.close()
    else:
        # create a simple deploy script
        if template_file_name.endswith('_deploy.sh'):
            outputFileName =  mission_name + '_deploy.sh'

        elif template_file_name.endswith('description.txt'):
            outputFileName = mission_name + '_description.txt'
        try:
            outputStream = open(OUTPUTS_DIR + outputFileName, "x")
        except FileExistsError:
            print(outputFileName + " already exists.  Override? (y/n)")
            while True:
                c = sys.stdin.read(1)
                if c == "y" or c == "Y":
                    print("Overwriting")
                    outputStream = open(OUTPUTS_DIR + outputFileName, "w")
                    break
                if c == "n" or c == "N":
                    print("Canceling operation")
                    return 3
        outputStream.write(printingString)
        outputStream.close()

    
    return outputFileName



def main():

    today = date.today()


    # Set up the argument parsing

    answer = input('Have you updated your shared origin in for this multi-agent mission? This script should only be run if you have. (y/n) ')
    if answer.lower() == 'n':
        ('please update the parameter files for this mission, then try running this script again')
        return

    # get origin for the mission
    with open('../mission_params.yaml', 'r') as file:
        parameters = yaml.safe_load(file)
    lat_origin = float(parameters['/**']['ros__parameters']['origin.latitude'])
    long_origin = float(parameters['/**']['ros__parameters']['origin.longitude'])
    print(lat_origin, long_origin)


    parser = argparse.ArgumentParser(description="A simple script that converts a .waypoints file (made by Ardupilot) to a put into a moos behavior file")
    parser.add_argument("input", help="The .waypoints file")
    parser.add_argument("-o", "--output", help="A specified name of the output file.  If no name is specified, then the same name will be used as the input file.  If the name given doesn't end with .wpt, the .wpt will be appended", default="")
    args = parser.parse_args()
  
    #================#
    # PHASE 1: INPUT #
    #================#

    Waypoints = inputparse.parseInputs(args)
    description = input('Please type up a sufficient description for this mission: ')
    
    #====================================#
    # PHASE 2: Convertor LAT/LONG to X/Y #
    #====================================#

    numCoords = 0
    coords = []
    for waypoint in Waypoints:
        x, y = CalculateHaversine(lat_origin, long_origin, waypoint.latitude, waypoint.longitude)
        x = round(x)
        y = round(y)
        coords.append([x, y, waypoint.depth])
        numCoords += 1

    # Creating the string
    printingString = POINTS_LINE_BASE
    numCoordsRead = 0
    # x-y part
    for currentCoord in coords:
        numCoordsRead += 1
        printingString += str(currentCoord[0]) + "," + str(currentCoord[1])
        if not numCoordsRead == numCoords:
            printingString += ":"
    printingString += '\n             depths = '
    numCoordsRead = 0
    for currentCoord in coords:
        numCoordsRead +=1
        printingString += str(currentCoord[2])
        if not numCoordsRead == numCoords:
            printingString += ":"

    #=================#
    # PHASE 3: OUTPUT #
    #=================#


    # first create directory to store files

    # create .bhv, .moos file, and a handy deploy script

    # The way this works is that we read in the template.bhv file in the src directory.
    behaviorFileName = CopyTemplateFile(args, "template.bhv", printingString=printingString)
    mission_name, extension = os.path.splitext(behaviorFileName)

    # add a mission file - template.moos
    # currently, we are just using the same template .moos file for every mission
    # so only copying each line exactly is happening below
    beahvior_file_line = f'  behaviors  = {behaviorFileName}\n' 
    missionFileName = CopyTemplateFile(args, "template.moos", printingString= beahvior_file_line, mission_name=mission_name)

    # copy the deploy script
    deploy_moos_str = f"#!/bin/bash\n\nuPokeDB {missionFileName} DEPLOY=true, MOOS_MANUAL_OVERIDE=false"
    # deploy_script_name = f'{mission_name}_deploy.sh'
    # create_file_command = f'echo \'{deploy_moos_str}\' >outputs/{deploy_script_name}'
    # os.system(create_file_command)

    deploy_script_name = CopyTemplateFile(args, "template_deploy.sh", printingString=deploy_moos_str, mission_name=mission_name)

    description_file_name = CopyTemplateFile(args, "template_description.txt", printingString=description, mission_name=mission_name)
   
    # #=================#
    # # PHASE 4: Copy New Mission
    # #          Directory to Docker 
    # #          Container
    # #=================#

    # create a new mission directory
    new_directory_name = mission_name + '_mission'
    mkdir = 'mkdir ' + new_directory_name
    os.system(mkdir)

    # put .bhv, .moos, deploy script, and a picture if you want
    list_of_file_strings = []
    list_of_file_strings.append(OUTPUTS_DIR + behaviorFileName)
    list_of_file_strings.append(OUTPUTS_DIR + missionFileName)
    list_of_file_strings.append(OUTPUTS_DIR + deploy_script_name)
    list_of_file_strings.append(OUTPUTS_DIR + description_file_name)
    # answer = input("do you have a picture to send over also? y/n ")
    # if answer == 'y' or answer == 'Y':
    #     picture_name = input("what is the file name of the picture that you placed in the pictures/ directory (example: waypoints.png) ")

    #     picture_dir = os.path.dirname(os.path.abspath(__file__)) + '/pictures/'
    #     file_path = os.path.join(picture_dir, picture_name)
    #     # Check if the file exists
    #     if os.path.isfile(file_path):
    #         print(f"Found file:'{picture_name}' -- SUCCESS.")
    #         list_of_file_strings.append('pictures/' + picture_name)
    #     else:
    #         print(f"The file '{picture_name}' does NOT exist in the directory: {picture_dir}")

    mv_new_dir_command = 'echo \'Cleaning Up\''
    for file in list_of_file_strings:
        mv_new_dir_command += f' && cp {file} {new_directory_name}' 

    os.system(mv_new_dir_command) 

    # docker_cp_directory = f'docker cp {new_directory_name}/ {name_of_container}:{path_in_container}' 
    # os.system(docker_cp_directory)

    return 0


if __name__ == "__main__":
    exitCode = main()
    print("Process finished with code " + str(exitCode))

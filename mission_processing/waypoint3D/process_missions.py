# Import statements
import argparse
import sys

import src.parse_waypoints as inputparse
import numpy as np
import math
import os
import yaml
import re
from datetime import date
from pathlib import Path
import os.path
from datetime import datetime



EARTH_CIRCUMFRENCE_METERS = 40075000
EARTH_RADIUS_METERS       = 6371000
METERS_PER_DEGREE_LON     = EARTH_CIRCUMFRENCE_METERS / 360
POINTS_LINE_BASE          = "	           points = "
DEPTH_LINE                = "             depth = "
DEFAULT_HEADING_LINE      = "                heading = "
DEFAULT_DEPTH_LINE        = "                depth = "
MOOS_COMMUNITY_LINE       = "Community    = "
# OUTPUTS_DIR               = 'outputs/'
# LONG_ORIGIN = 40.23849638603186
# LAT_ORIGIN  = -111.73956745065406
NUM_AGENTS = 6

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

def CopyTemplateFile(args, template_file_name, printingString = '', mission_name = '', output_dir = '', vehicle_id=''):
    # Split the file name and get the extension
    base_name, extension = os.path.splitext(template_file_name)
    
    
    if not template_file_name.endswith('.sh') and not template_file_name.endswith('.txt'):

        template_file_name = output_dir + '../' + 'template_files/' + template_file_name
        # First, read in the template file
        templateFile = open(template_file_name)
        lines = []
        for currentLine in templateFile:
            lines.append(currentLine)
        templateFile.close()

        # Set up the output file name
        outputFileName = vehicle_id + extension
        # if template_file_name.endswith(".bhv"):
        # if outputFileName == "":
        #     inputWithoutExtension = args.input[0:-10]
        #     outputFileName = inputWithoutExtension + extension
        #     del inputWithoutExtension
        # elif not outputFileName.endswith(extension):
        #     outputFileName = outputFileName + extension


        # Open the file and Check if the output file already exists
        try:
            outputStream = open(output_dir + outputFileName, "x")
        except FileExistsError:
            print(outputFileName + " already exists.  Override? (y/n)")
            while True:
                c = sys.stdin.read(1)
                if c == "y" or c == "Y":
                    print("Overwriting")
                    outputStream = open(output_dir + outputFileName, "w")
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
                outputStream.write(MOOS_COMMUNITY_LINE + 'multi-agent-coug' + '\n\n')
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
            outputStream = open(output_dir + outputFileName, "x")
        except FileExistsError:
            print(outputFileName + " already exists.  Override? (y/n)")
            while True:
                c = sys.stdin.read(1)
                if c == "y" or c == "Y":
                    print("Overwriting")
                    outputStream = open(output_dir + outputFileName, "w")
                    break
                if c == "n" or c == "N":
                    print("Canceling operation")
                    return 3
        outputStream.write(printingString)
        outputStream.close()

    
    return outputFileName



def main():

    parser = argparse.ArgumentParser(description="A simple script that converts a .waypoints file (made by Ardupilot) to a put into a moos behavior file")
    parser.add_argument("vehicle_id", help="the vehicle identifier 0-5, or \'-a\' for all vehicles)")
    parser.add_argument("mission_directory_descriptor", help="should be brief but descriptive and in the waypoint3D/missions/ directory already")
    args = parser.parse_args()
    path_to_python_script = os.path.dirname(__file__) # path independent of shell python script is called from
    # make mission directory if it does not already exist
    # print(path_to_python_script)
    mission_dir = path_to_python_script + '/missions/' + str(date.today()) + '_' + str(int(datetime.now().timestamp())) + '_' + args.mission_directory_descriptor
    mkdir = 'mkdir -p ' + mission_dir
    
    os.system(mkdir)
    new_mission_dir= path_to_python_script + '/new_mission'
    mkdir = 'mkdir -p ' + new_mission_dir
    os.system(mkdir)

    outputs_dir = mission_dir + '/../../outputs/'
    # print(args.vehicle_id)
    # print('')

    if args.vehicle_id != 'sim' and args.vehicle_id == 'all':
        for i in range(NUM_AGENTS):
            # print(str(i), args.vehicle_id[-1])
            # if args.vehicle_id == 'all' or str(i + 1) == args.vehicle_id[-1]:
            mkdir = 'mkdir -p ' + mission_dir  + "/" + 'coug' + str(i)
            os.system(mkdir)
    elif args.vehicle_id != 'sim':
        mkdir = 'mkdir -p ' + mission_dir  + "/" + args.vehicle_id
        os.system(mkdir) 
    else:
        mkdir = 'mkdir -p ' + mission_dir  + "/" + 'sim' 
        os.system(mkdir)

    # Set up the argument parsing
    # answer = input('Have you updated your shared origin in for this multi-agent mission? This script should only be run if you have. (y/n) ')
    # if answer.lower() == 'n':
    #     ('please update the parameter files for this mission, then try running this script again')
    #     return

    # get origin for the mission
    with open(mission_dir + '/../../../mission_params.yaml', 'r') as file:
        parameters = yaml.safe_load(file)
    lat_origin = float(parameters['/**']['ros__parameters']['origin.latitude'])
    long_origin = float(parameters['/**']['ros__parameters']['origin.longitude'])
    # print(lat_origin, long_origin)


    for i in range(NUM_AGENTS - 1):
        if args.vehicle_id == 'all' or str(i) == args.vehicle_id[-1]:
            mkdir = 'mkdir -p ' + mission_dir + '/coug' + str(i)
            os.system(mkdir)

            #================#
            # PHASE 1: INPUT #
            #================#
            vehicle_id = 'coug' + str(i)
            Waypoints = inputparse.parseInputs(args, vehicle_id)
            if Waypoints is None:
                continue
            description = input(f'\n\nPlease type up a sufficient description of the behavior for the vehicle "{vehicle_id}" in the mission "{args.mission_directory_descriptor}": ')

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

            # #=================#
            # # PHASE 3: OUTPUT #
            # #=================#


            # first create directory to store files

            # create .bhv, .moos file, and a handy deploy script

            # The way this works is that we read in the template.bhv file in the src directory.
            behaviorFileName = CopyTemplateFile(args, "template.bhv", printingString=printingString, output_dir=outputs_dir, vehicle_id=vehicle_id)
            mission_name, extension = os.path.splitext(behaviorFileName)

            # add a mission file - template.moos
            # currently, we are just using the same template .moos file for every mission
            # so only copying each line exactly is happening below
            beahvior_file_line = f'  behaviors  = {behaviorFileName}\n' 
            missionFileName = CopyTemplateFile(args, "template.moos", printingString= beahvior_file_line, mission_name=mission_name, output_dir=outputs_dir, vehicle_id=vehicle_id)

            # copy the deploy script
            deploy_moos_str = f"#!/bin/bash\n\nuPokeDB {missionFileName} DEPLOY=true, MOOS_MANUAL_OVERIDE=false"
            # deploy_script_name = f'{mission_name}_deploy.sh'
            # create_file_command = f'echo \'{deploy_moos_str}\' >outputs/{deploy_script_name}'
            # os.system(create_file_command)

            deploy_script_name = CopyTemplateFile(args, "template_deploy.sh", printingString=deploy_moos_str, mission_name=mission_name, output_dir=outputs_dir, vehicle_id=vehicle_id)

            description_file_name = CopyTemplateFile(args, "template_description.txt", printingString=description, mission_name=mission_name, output_dir=outputs_dir, vehicle_id=vehicle_id)
        
            # #=================#
            # # PHASE 4: Put into directories
            # #=================#

            list_of_file_strings = []
            list_of_file_strings.append(outputs_dir + behaviorFileName)
            list_of_file_strings.append(outputs_dir + missionFileName)
            list_of_file_strings.append(outputs_dir + deploy_script_name)
            list_of_file_strings.append(outputs_dir + description_file_name)
        

            mv_new_dir_command = 'echo \'Cleaning Up\''
            for file in list_of_file_strings:
                mv_new_dir_command += f' && cp {file} {mission_dir + '/' + vehicle_id}' 

            os.system(mv_new_dir_command) 
            os.system('cd ' + outputs_dir + '&& rm -f *') 

    os.system('cp -R ' + mission_dir+ ' '+ new_mission_dir)
    return 0 

         
    


if __name__ == "__main__":
    exitCode = main()
    print("Process finished with code " + str(exitCode))
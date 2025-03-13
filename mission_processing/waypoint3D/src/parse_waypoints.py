import os.path

#  A simple class that holds all the information we want from a line in a .waypoints file
class Waypoint:
    def __init__(self, latitude, longitude, depth ):
        self.latitude = latitude
        self.longitude = longitude
        self.depth     = depth

def parseInputs(args, vehicle_id):
    # Establish that the name is valid
    waypoint_dir = os.path.dirname(__file__) + '/../../waypoint-files/' + args.mission_directory_descriptor + '/'
    # inputFileName = '../waypoint-files/' + args.vehicle_id
    # print(waypoint_dir)

    inputFileName = ''
 
    for root, dirs, files in os.walk(waypoint_dir):
        for file in files: 
            if file.startswith(vehicle_id):
                inputFileName = waypoint_dir + str(file)
                # print(inputFileName)

    if inputFileName == '':
        return None

    if not inputFileName.endswith(".waypoints"):
        print("The input file must be a .waypoints file and be located in the waypoint_files directory")
        raise NameError
    
    
    
    # Open the file and make sure it is a valid file
    try:
        inputFileStream = open(inputFileName, "r")
    except FileNotFoundError:
        print("The input file " + inputFileName + " doesn't exist")
        raise RuntimeError
    else:
        print(inputFileName + " opened succesfully")
    
    # Setting up the reading loop
    print("Reading input")
    numLines = 0
    goodLines = 0
    print("Line " + str(numLines), end="")
    currentLine = inputFileStream.readline()

    Waypoints = []
    # Starting the loop
    while not currentLine == "":
        numLines += 1
        print("\rLine " + str(numLines), end="")
        currentLine = inputFileStream.readline()
        data = currentLine.split("\t")
        
        # Check that the line is a data line
        if not len(data) == 12:
            continue
        
        # Check that the line is a move-to-location line
        # The command for moving to a location is 16
        # All other commands are irrelevant to us
        if not int(data[3]) == 16:
            continue
        
        # Read the lat, long, and depth values
        Waypoints.append(Waypoint(data[8], data[9], data[10]))
        goodLines += 1

    # Cleaning up the loop
    print("\rRead " + str(numLines) + " total lines                                                                          ")
    print(" and " + str(goodLines) + " movement lines")

    # Close the file and remove the variable
    inputFileStream.close()
    del inputFileStream

    

    return Waypoints

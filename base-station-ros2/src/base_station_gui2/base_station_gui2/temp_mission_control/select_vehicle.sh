#!/bin/bash

# Function to display the menu and get user input
select_vehicle() {
    echo "Select a vehicle ID:" >&2
    echo "1) coug1" >&2
    echo "2) coug2" >&2
    echo "3) coug3" >&2
    echo "4) coug4" >&2
    echo "5) coug5" >&2
    echo "0) Vehicle 0 (Simulator)" >&2
    
    read -p "Enter the number (0-5): " vehicle_option >&2

    case $vehicle_option in
        1) echo "coug1" ;;
        2) echo "coug2" ;;
        3) echo "coug3" ;;
        4) echo "coug4" ;;
        5) echo "coug5" ;;
        0) echo "coug0" ;;
        *) echo "Invalid option. Exiting." >&2; exit 1 ;;
    esac
}

# Call the function and output the result
select_vehicle
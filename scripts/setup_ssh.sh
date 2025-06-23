#!/bin/bash

# Prompt the user for where they want to send the SSH key
echo "Enter the vehicle number (0–9) or an IP address/hostname:"
read -p "Enter your choice: " choice

# Check if the input is a number between 0 and 9
if [[ "$choice" =~ ^[0-9]$ ]]; then
    IP_ADDRESS="192.168.0.10${choice}"
else
    IP_ADDRESS="$choice"
fi

# Define SSH key path
SSH_KEY_PATH="/home/frostlab/.ssh/id_rsa"

# Check if the SSH key exists
if [ ! -f "$SSH_KEY_PATH" ]; then
    echo "No SSH key found at $SSH_KEY_PATH."

    # Ask if user wants to generate a new key
    read -p "Do you want to generate a new SSH key? (y/n): " generate_key

    if [[ "$generate_key" =~ ^[Yy]$ ]]; then
        # Generate a new SSH key
        echo "Generating new SSH key..."
        ssh-keygen -t rsa -b 2048 -f "$SSH_KEY_PATH" -N ""  # No passphrase
    else
        echo "SSH key not generated. Exiting script."
        exit 1
    fi
fi

# Remove old SSH key (if it exists) to prevent key mismatch issues
echo "Removing old SSH key for $IP_ADDRESS..."
ssh-keygen -R "$IP_ADDRESS"

# Test the SSH connection to accept the new key if prompted
echo "Copying SSH key to $IP_ADDRESS..."
ssh-copy-id -i "$SSH_KEY_PATH.pub" frostlab@"$IP_ADDRESS"

# Test the SSH connection
echo "Testing SSH connection to $IP_ADDRESS..."
ssh -o StrictHostKeyChecking=accept-new frostlab@"$IP_ADDRESS" "echo 'SSH connection successful to $IP_ADDRESS'" &> /dev/null

if [ $? -eq 0 ]; then
    echo "✅ SSH connection to $IP_ADDRESS was successful!"
else
    echo "❌ SSH connection to $IP_ADDRESS failed!"
fi

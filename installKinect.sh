#!/bin/bash
set -e

# Import Ubuntu security key
sudo gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3B4FE6ACC0B21F32
sudo gpg --export --armor 3B4FE6ACC0B21F32 | sudo tee /etc/apt/trusted.gpg.d/ubuntu-security.asc

# Backup sources.list
sudo cp /etc/apt/sources.list /etc/apt/sources.list.old
echo "Backed up /etc/apt/sources.list to /etc/apt/sources.list.old"

# Replace with custom bionic sources.list
sudo cp sources.list /etc/apt/sources.list
echo "Overwrote /etc/apt/sources.list with sources.list"

# Add Microsoft repo for Ubuntu 18.04
wget -q https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb

# Remove any jammy or 20.04 repo entries accidentally added
sudo sed -i '/ubuntu\/20.04/d' /etc/apt/sources.list.d/*.list || true
sudo sed -i '/jammy/d' /etc/apt/sources.list.d/*.list || true

# Update apt cache
sudo apt-get update

# Install Kinect packages
sudo apt-get install \
    libsoundio1 \
    libk4a1.4 \
    libk4a1.4-dev \
    k4a-tools
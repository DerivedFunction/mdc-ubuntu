# Installing Azure-Kinect-SDK 1.4 on Ubuntu 22.04
# Based on: https://github.com/juancarlosmiranda/azure_kinect_notes/

# Step 1: Install Essential Tools
sudo apt-get install -y build-essential
sudo apt-get install -y cmake
sudo apt-get install -y libgtk2.0-dev
sudo apt-get install -y libusb-1.0
sudo apt-get install -y ffmpeg
sudo apt-get install -y mlocate
sudo apt-get install -y locate
sudo apt install -y curl

# Step 2: Add Microsoft Repositories for Ubuntu 20.04 and 18.04
# Ubuntu 20.04
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc
sudo apt-add-repository https://packages.microsoft.com/ubuntu/20.04/prod

# Ubuntu 18.04
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -

# Step 3: Install Azure Kinect Packages
sudo apt-get update
sudo apt-get install -y libk4a1.4
sudo apt install -y libk4a1.4-dev

# Step 4: Install Dependency libsoundio1
# Note: libsoundio1 is a dependency for k4a-tools
# https://packages.ubuntu.com/focal/amd64/libsoundio1/download
wget mirrors.kernel.org/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb
sudo dpkg -i libsoundio1_1.1.0-1_amd64.deb
sudo apt install -y k4a-tools

# Step 5: Copy Udev Rules so no root / sudo is needed to access the kinect device
sudo cp 99-k4a.rules /etc/udev/rules.d/

# Expected Output:
# Bus 004 Device 006: ID 045e:097c Microsoft Corp. Azure Kinect Depth Camera
# Bus 004 Device 005: ID 045e:097d Microsoft Corp. Azure Kinect 4K Camera
# Bus 004 Device 004: ID 045e:097a Microsoft Corp. 4-Port USB 3.0 Hub
# Bus 003 Device 008: ID 045e:097e Microsoft Corp. Azure Kinect Microphone Array
# Bus 003 Device 006: ID 045e:097b Microsoft Corp. 4-Port USB 2.0 Hub

# Step 7: Run k4aviewer
# k4aviewer 
# Instructions: Select ‚Open Device' -> Click ‚Start'
# Image Should Appear
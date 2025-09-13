# PowerShell script to attach all available USB devices to WSL
# Requires usbipd-win installed and WSL 2 running
# Run this script in an elevated (Administrator) PowerShell prompt

# Ensure usbipd is installed
if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Write-Error "usbipd-win is not installed. Please install it using 'winget install --exact dorssel.usbipd-win' or download from https://github.com/dorssel/usbipd-win/releases"
    exit 1
}

# Ensure WSL is running
$wslStatus = wsl --status
if (-not $wslStatus) {
    Write-Error "WSL is not running or not installed. Please ensure WSL 2 is set up and a distribution is running."
    exit 1
}

# Get the list of USB devices
$devices = usbipd list | Select-String -Pattern "^\s*(\d+-\d+)\s+([0-9a-fA-F:]+)\s+(.+?)\s+(Not shared|Shared|Attached)"

# Check if any devices were found
if (-not $devices) {
    Write-Output "No USB devices found."
    exit 0
}

# Process each device
foreach ($device in $devices) {
    $busId = $device.Matches.Groups[1].Value
    $state = $device.Matches.Groups[4].Value

    # Bind the device if it's not already shared or attached
    if ($state -eq "Not shared") {
        Write-Output "Binding USB device with BusID $busId..."
        usbipd bind --busid $busId
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Failed to bind device with BusID $busId."
            continue
        }
    }

    # Attach the device to WSL if it's not already attached
    if ($state -ne "Attached") {
        Write-Output "Attaching USB device with BusID $busId to WSL..."
        usbipd attach --wsl --busid $busId
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Failed to attach device with BusID $busId to WSL."
            continue
        }
    } else {
        Write-Output "USB device with BusID $busId is already attached to WSL."
    }
}

Write-Output "All available USB devices have been processed."
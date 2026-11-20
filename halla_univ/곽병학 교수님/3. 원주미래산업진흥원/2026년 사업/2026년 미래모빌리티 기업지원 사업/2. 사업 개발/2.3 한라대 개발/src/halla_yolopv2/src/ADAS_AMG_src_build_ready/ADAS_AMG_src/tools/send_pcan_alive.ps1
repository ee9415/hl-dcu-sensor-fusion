param(
    [string]$Channel = "PCAN_USBBUS1",
    [string]$Bitrate = "500K",
    [string]$Id = "0x300",
    [string]$Data = "00 00 00 00 00 00 00 00",
    [double]$CycleMs = 50,
    [double]$ReconnectMs = 200,
    [double]$Duration = 0,
    [string]$Dll = ""
)

$script = Join-Path $PSScriptRoot "send_pcan_alive.py"

$arguments = @(
    $script,
    "--channel", $Channel,
    "--bitrate", $Bitrate,
    "--id", $Id,
    "--data", $Data,
    "--cycle-ms", "$CycleMs",
    "--reconnect-ms", "$ReconnectMs"
)

if ($Duration -gt 0) {
    $arguments += @("--duration", "$Duration")
}

if ($Dll -ne "") {
    $arguments += @("--dll", $Dll)
}

python @arguments

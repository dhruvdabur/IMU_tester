param(
    [string]$BnoCom = "",
    [switch]$NoBridge,
    [switch]$Restart,
    [switch]$NoBrowser
)

$ErrorActionPreference = "Stop"

$Root = $PSScriptRoot
$LogDir = Join-Path $Root "logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Stop-PortOwner {
    param([int]$Port)

    Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
        Select-Object -ExpandProperty OwningProcess -Unique |
        ForEach-Object {
            Stop-Process -Id $_ -Force -ErrorAction SilentlyContinue
        }
}

function Test-PortListening {
    param([int]$Port)

    $listener = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
        Select-Object -First 1

    return $null -ne $listener
}

function Get-LanAddress {
    $ip = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object {
            $_.IPAddress -notlike "127.*" -and
            $_.PrefixOrigin -ne "WellKnown" -and
            $_.InterfaceAlias -notmatch "Loopback|Bluetooth"
        } |
        Sort-Object InterfaceMetric |
        Select-Object -First 1 -ExpandProperty IPAddress

    if (-not $ip) {
        $ip = "127.0.0.1"
    }

    return $ip
}

function Find-Stm32ComPort {
    $devices = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Status -eq "OK" -and
            ($_.InstanceId -match "VID_0483&PID_5740" -or $_.FriendlyName -match "STM32|Virtual ComPort")
        }

    foreach ($device in $devices) {
        if ($device.FriendlyName -match "\(COM\d+\)") {
            return $Matches[0].Trim("(", ")")
        }
    }

    return ""
}

function Start-LoggedProcess {
    param(
        [string]$Name,
        [string]$FilePath,
        [string[]]$ArgumentList,
        [string]$OutLog,
        [string]$ErrLog
    )

    Clear-Content $OutLog -ErrorAction SilentlyContinue
    Clear-Content $ErrLog -ErrorAction SilentlyContinue

    $process = Start-Process `
        -FilePath $FilePath `
        -ArgumentList $ArgumentList `
        -WorkingDirectory $Root `
        -RedirectStandardOutput $OutLog `
        -RedirectStandardError $ErrLog `
        -PassThru

    Write-Host "$Name started. PID $($process.Id)"
    return $process
}

function Start-ExpoWindow {
    $escapedRoot = $Root.Replace("'", "''")
    $command = "Set-Location -LiteralPath '$escapedRoot'; Write-Host 'Expo QR code will appear below. Scan it with Expo Go.'; Write-Host ''; & npx.cmd expo start --lan --clear --port 8081"

    $process = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList @("-NoExit", "-ExecutionPolicy", "Bypass", "-Command", $command) `
        -WorkingDirectory $Root `
        -PassThru

    Write-Host "Expo Metro started in a new PowerShell window. PID $($process.Id)"
    Write-Host "Scan the QR code in that Expo window with Expo Go."
    return $process
}

function Start-BridgeWindow {
    param([string]$ComPort)

    $escapedRoot = $Root.Replace("'", "''")
    $escapedComPort = $ComPort.Replace("'", "''")
    $command = "Set-Location -LiteralPath '$escapedRoot'; Write-Host 'BNO055 bridge output will appear below.'; Write-Host ''; & py -u scripts\bno055_serial_bridge.py '$escapedComPort'"

    $process = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList @("-NoExit", "-ExecutionPolicy", "Bypass", "-Command", $command) `
        -WorkingDirectory $Root `
        -PassThru

    Write-Host "BNO055 bridge ($ComPort) started in a new PowerShell window. PID $($process.Id)"
    return $process
}

function Show-QuickExit {
    param(
        [string]$Name,
        [System.Diagnostics.Process]$Process,
        [string]$OutLog,
        [string]$ErrLog
    )

    Start-Sleep -Seconds 2
    $Process.Refresh()

    if ($Process.HasExited) {
        Write-Host "$Name exited early with code $($Process.ExitCode)."
        if (Test-Path $OutLog) {
            Get-Content $OutLog -Tail 20
        }
        if (Test-Path $ErrLog) {
            Get-Content $ErrLog -Tail 20
        }
    }
}

if ($Restart) {
    Stop-PortOwner -Port 65000
    Stop-PortOwner -Port 8081
    Get-CimInstance Win32_Process |
        Where-Object {
            $_.CommandLine -match "bno055_serial_bridge.py" -or
            $_.CommandLine -match "imu-json-server.js"
        } |
        ForEach-Object {
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
    Start-Sleep -Seconds 1
}

if (Test-PortListening -Port 65000) {
    Write-Host "JSON host already listening on port 65000."
} else {
    $serverProcess = Start-LoggedProcess `
        -Name "JSON host" `
        -FilePath "npm.cmd" `
        -ArgumentList @("run", "server") `
        -OutLog (Join-Path $LogDir "imu-json-server.out.log") `
        -ErrLog (Join-Path $LogDir "imu-json-server.err.log")
    Show-QuickExit `
        -Name "JSON host" `
        -Process $serverProcess `
        -OutLog (Join-Path $LogDir "imu-json-server.out.log") `
        -ErrLog (Join-Path $LogDir "imu-json-server.err.log")
}

if (Test-PortListening -Port 8081) {
    Write-Host "Expo Metro already listening on port 8081."
    Write-Host "If you need the QR window again, re-run with: .\launch.ps1 -Restart"
} else {
    $expoProcess = Start-ExpoWindow
}

if (-not $NoBridge) {
    if (-not $BnoCom) {
        $BnoCom = Find-Stm32ComPort
    }

    if ($BnoCom) {
        $bridgeAlreadyRunning = Get-CimInstance Win32_Process |
            Where-Object {
                $_.CommandLine -match "bno055_serial_bridge.py" -and
                $_.CommandLine -match [regex]::Escape($BnoCom)
            } |
            Select-Object -First 1

        if ($bridgeAlreadyRunning) {
            Write-Host "BNO055 bridge already running for $BnoCom. PID $($bridgeAlreadyRunning.ProcessId)"
        } else {
            $bridgeProcess = Start-BridgeWindow -ComPort $BnoCom
        }
    } else {
        Write-Host "No STM32 CDC COM port detected. Re-run with: .\launch.ps1 -BnoCom COM9"
    }
}

Start-Sleep -Seconds 3

$LanAddress = Get-LanAddress
$VisualizerUrl = "http://${LanAddress}:65000/visualizer"
$DriftJsonUrl = "http://${LanAddress}:65000/drift.json"
$ExpoUrl = "exp://${LanAddress}:8081"

if (-not $NoBrowser) {
    Start-Process $VisualizerUrl
    Start-Process "http://localhost:8081"
}

Write-Host ""
Write-Host "Launch complete."
Write-Host "Visualizer: $VisualizerUrl"
Write-Host "Drift JSON:  $DriftJsonUrl"
Write-Host "Expo Go:     $ExpoUrl"
Write-Host "App host:    http://${LanAddress}:65000"
Write-Host ""
Write-Host "Logs:"
Write-Host "  $LogDir"

# TurtleBot3 Nav Monitor — Windows launcher
# Requires: Docker Desktop, VcXsrv (https://sourceforge.net/projects/vcxsrv/)
#
# Usage:
#   $env:WORLD = "obstacles"; .\scripts\run_windows.ps1                                           # SLAM mapping
#   $env:WORLD = "obstacles"; $env:MODE = "amcl"; .\scripts\run_windows.ps1                    # AMCL adaptive (default)
#   $env:WORLD = "house"; $env:MODE = "amcl"; $env:NAV_CONFIG = "baseline"; .\scripts\run_windows.ps1               # baseline
#   $env:WORLD = "house"; $env:MODE = "amcl"; $env:NAV_CONFIG = "params_a_adaptive"; .\scripts\run_windows.ps1       # custom config + adaptive
#   $env:WORLD = "house"; $env:MODE = "amcl"; $env:NAV_CONFIG = "params_a_baseline"; $env:NAV_ADAPTIVE = "false"; .\scripts\run_windows.ps1  # custom config + no adaptive
#
# Mapping workflow:
#   1. Set WORLD and run this script (SLAM mode, default)
#   2. In a new terminal: docker exec -it tbot3_nav_monitor-sim-1 `
#        ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   3. Drive until map is complete, then:
#        docker exec tbot3_nav_monitor-sim-1 bash /ros2_ws/scripts/save_map.sh <world>
#   4. Ctrl-C, then relaunch with $env:MODE = "amcl"

param(
    [string]$World         = $env:WORLD,
    [string]$Mode          = "slam",
    [string]$DashboardPort = "8080",
    [string]$RosDomainId   = "0",
    [string]$NavConfig     = $env:NAV_CONFIG,
    [string]$NavAdaptive   = $env:NAV_ADAPTIVE
)

if ($env:MODE)          { $Mode          = $env:MODE }
if ($env:DASHBOARD_PORT){ $DashboardPort = $env:DASHBOARD_PORT }
if ($env:ROS_DOMAIN_ID) { $RosDomainId   = $env:ROS_DOMAIN_ID }
if ($env:NAV_CONFIG)    { $NavConfig     = $env:NAV_CONFIG }
if ($env:NAV_ADAPTIVE)  { $NavAdaptive   = $env:NAV_ADAPTIVE }

# ── Validate ───────────────────────────────────────────────────────────────
if (-not $World) {
    Write-Error "WORLD not set. Example: `$env:WORLD = 'obstacles'; .\scripts\run_windows.ps1"
    exit 1
}

if ($World -notin @("obstacles", "house", "narrow")) {
    Write-Error "Unknown world '$World'. Choose: obstacles | house | narrow"
    exit 1
}

# ── Check / Launch VcXsrv ──────────────────────────────────────────────────
$vcxsrvExe = "C:\Program Files\VcXsrv\vcxsrv.exe"
$vcxsrvRunning = Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue
if (-not $vcxsrvRunning) {
    if (Test-Path $vcxsrvExe) {
        Write-Host "[run_windows] Starting VcXsrv..."
        Start-Process -FilePath $vcxsrvExe -ArgumentList ":0", "-multiwindow", "-clipboard", "-noprimary", "-wgl", "-ac"
        Start-Sleep -Seconds 2
    } else {
        Write-Warning "VcXsrv not found at: $vcxsrvExe"
        Write-Warning "Download from: https://sourceforge.net/projects/vcxsrv/"
        Write-Warning "Install it, then re-run this script - it will start automatically."
    }
} else {
    Write-Host "[run_windows] VcXsrv already running."
}

# ── Environment ────────────────────────────────────────────────────────────
$env:WORLD          = $World
$env:MODE           = $Mode
$env:DASHBOARD_PORT = $DashboardPort
$env:ROS_DOMAIN_ID  = $RosDomainId
$env:DISPLAY        = "host.docker.internal:0.0"
if ($NavConfig)   { $env:NAV_CONFIG   = $NavConfig   }
if ($NavAdaptive) { $env:NAV_ADAPTIVE = $NavAdaptive }

# ── Build ──────────────────────────────────────────────────────────────────
Write-Host "[run_windows] Building image..."
$env:DOCKER_BUILDKIT = "1"
docker compose build --no-cache=false

# ── Launch ─────────────────────────────────────────────────────────────────
try {
    if ($Mode -eq "amcl") {
        $cfgLabel = if ($NavConfig) { $NavConfig } else { "adaptive" }
        $adaptLabel = if ($NavAdaptive -eq "false") { "adaptive OFF" } else { "adaptive ON" }
        Write-Host "[run_windows] Demo mode - world: $World | config: $cfgLabel ($adaptLabel) | Dashboard: http://localhost:$DashboardPort"
        docker compose up
    } else {
        Write-Host "[run_windows] Mapping mode - world: $World"
        Write-Host "[run_windows] When done, save map with:"
        Write-Host "  docker exec tbot3_nav_monitor-sim-1 bash /ros2_ws/scripts/save_map.sh $World"
        docker compose up sim
    }
} finally {
    Write-Host ""
    Write-Host "[run_windows] Shutting down and cleaning up..."
    docker compose down
}


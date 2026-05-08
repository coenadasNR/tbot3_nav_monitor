# TurtleBot3 Nav Monitor — Windows launcher
# Requires: Docker Desktop, VcXsrv (https://sourceforge.net/projects/vcxsrv/)
#
# Usage:
#   $env:WORLD = "obstacles"; .\scripts\run_windows.ps1             # SLAM mapping
#   $env:WORLD = "obstacles"; $env:MODE = "amcl"; .\scripts\run_windows.ps1  # AMCL demo
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
    [string]$RosDomainId   = "0"
)

if ($env:MODE)          { $Mode          = $env:MODE }
if ($env:DASHBOARD_PORT){ $DashboardPort = $env:DASHBOARD_PORT }
if ($env:ROS_DOMAIN_ID) { $RosDomainId   = $env:ROS_DOMAIN_ID }

# ── Validate ───────────────────────────────────────────────────────────────
if (-not $World) {
    Write-Error "WORLD not set. Example: `$env:WORLD = 'obstacles'; .\scripts\run_windows.ps1"
    exit 1
}

if ($World -notin @("obstacles", "house", "narrow")) {
    Write-Error "Unknown world '$World'. Choose: obstacles | house | narrow"
    exit 1
}

# ── Check VcXsrv ───────────────────────────────────────────────────────────
if (-not (Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue)) {
    Write-Warning "VcXsrv does not appear to be running."
    Write-Warning "Download from: https://sourceforge.net/projects/vcxsrv/"
    Write-Warning "Launch XLaunch and make sure 'Disable access control' is ticked."
}

# ── Environment ────────────────────────────────────────────────────────────
$env:WORLD          = $World
$env:MODE           = $Mode
$env:DASHBOARD_PORT = $DashboardPort
$env:ROS_DOMAIN_ID  = $RosDomainId
$env:DISPLAY        = "host.docker.internal:0.0"

# ── Build ──────────────────────────────────────────────────────────────────
Write-Host "[run_windows] Building image..."
docker compose build

# ── Launch ─────────────────────────────────────────────────────────────────
try {
    if ($Mode -eq "amcl") {
        Write-Host "[run_windows] Demo mode - world: $World | Dashboard: http://localhost:$DashboardPort"
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


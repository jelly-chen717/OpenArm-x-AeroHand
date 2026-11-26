[CmdletBinding(SupportsShouldProcess)]
param()

$ErrorActionPreference = 'Stop'
$port = 8092
$projectRoot = Split-Path -Parent $PSScriptRoot
$relativeEntry = 'apps\openarm_monitor.py'
$absoluteEntry = Join-Path $projectRoot $relativeEntry
$projectPythonPaths = @(
    (Join-Path $projectRoot 'openarm\.venv\Scripts\python.exe')
    (Join-Path $projectRoot 'openarm\.python311\python.exe')
)

function Get-ListenerProcessIds {
    Get-NetTCPConnection -State Listen |
        Where-Object LocalPort -eq $port |
        Select-Object -ExpandProperty OwningProcess -Unique
}

$listenerIds = @(Get-ListenerProcessIds)
if ($listenerIds.Count -eq 0) {
    Write-Host "端口 $port 已空闲。"
    return
}
if ($listenerIds.Count -ne 1) {
    throw "端口 $port 有多个监听者。进程 PID 为 $($listenerIds -join ', ')。"
}

$listenerId = $listenerIds[0]
$process = Get-Process -Id $listenerId
# Hold a handle to the identified process even if it exits and Windows reuses its ID.
$null = $process.Handle
$processInfo = Get-CimInstance Win32_Process -Filter "ProcessId = $listenerId"
$entryPattern = '^(?:"[^"]+"|\S+)(?:\s+-(?:B|u))*\s+(?:"(?<script>[^"]+)"|(?<script>\S+))(?=\s|$)'

$isProjectMonitor = $false
if ($processInfo.ExecutablePath -and
    [IO.Path]::GetFileName($processInfo.ExecutablePath) -ieq 'python.exe' -and
    $processInfo.CommandLine -match $entryPattern) {
    $entry = $Matches.script.Replace('/', '\')
    if ([IO.Path]::IsPathRooted($entry)) {
        $isProjectMonitor = [IO.Path]::GetFullPath($entry) -ieq $absoluteEntry
    } else {
        # The project launchers set their working directory before using a relative entry.
        $relativeMonitor = $entry -ieq $relativeEntry -or $entry -ieq ".\$relativeEntry"
        $isProjectMonitor = $relativeMonitor -and $processInfo.ExecutablePath -in $projectPythonPaths
    }
}
if ($process.HasExited -or -not $isProjectMonitor) {
    throw "无法确认端口 $port 的 PID $listenerId 是本项目控制台。请核对该进程。"
}

if ($PSCmdlet.ShouldProcess("OpenArm monitor PID $listenerId, TCP $port", 'Stop')) {
    Stop-Process -InputObject $process -Confirm:$false
    Wait-Process -InputObject $process -Timeout 5
    $remainingIds = @(Get-ListenerProcessIds)
    if ($remainingIds.Count -gt 0) {
        throw "旧服务已结束，但端口 $port 当前由 PID $($remainingIds -join ', ') 监听。"
    }
    Write-Host "已结束控制台 PID $listenerId，端口 $port 已释放。"
}

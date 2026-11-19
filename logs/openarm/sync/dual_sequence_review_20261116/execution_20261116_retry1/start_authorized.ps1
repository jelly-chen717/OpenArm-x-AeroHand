$ErrorActionPreference = 'Stop'
$taskRoot = 'D:\桌面\open CyMon\弹钢琴'
$runDir = $PSScriptRoot
$requestPath = Join-Path $runDir 'request.authorized.json'
$requestData = Get-Content -LiteralPath $requestPath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($requestData.authorization_status -ne 'USER_EXPLICITLY_AUTHORIZED') { throw 'Missing user authorization' }
if (Test-Path -LiteralPath (Join-Path $runDir 'process.json')) { throw 'This run was already started; no automatic repeat' }
if (Test-Path -LiteralPath (Join-Path $runDir 'cancel.stop')) { throw 'Cancel already requested' }
$launchArgs = @('-u', (Join-Path $taskRoot 'apps/drive_openarm_from_sim.py'), $requestPath,
    '--mode','hardware','--action','execute','--sequence','--allow-dual-arm','--batch-mit','--hold-after',
    '--joint-settings',(Join-Path $runDir 'settings.json'),'--limit-profile','rated',
    '--left-channel','0x51','--right-channel','0x52','--confirm-bus-ownership',
    '--period','0.04','--max-lateness','0.08','--max-error','0.08726646259971647',
    '--cancel-file',(Join-Path $runDir 'cancel.stop'),'--log',(Join-Path $runDir 'execution.jsonl'))
$quotedArgs = $launchArgs | ForEach-Object { '"' + $_ + '"' }
$controller = Start-Process -FilePath (Join-Path $taskRoot 'openarm/.venv/Scripts/python.exe') -ArgumentList $quotedArgs -WorkingDirectory $taskRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $runDir 'stdout.txt') -RedirectStandardError (Join-Path $runDir 'stderr.txt') -PassThru
[ordered]@{ launcher_pid=$controller.Id; launched_at=(Get-Date -Format o); arguments=$launchArgs; user_authorization='按此表执行'; cancel_file=(Join-Path $runDir 'cancel.stop') } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runDir 'process.json') -Encoding UTF8
Write-Output "Started authorized controller launcher PID $($controller.Id)"

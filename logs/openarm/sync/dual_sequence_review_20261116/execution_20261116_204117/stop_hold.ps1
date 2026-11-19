$ErrorActionPreference = 'Stop'
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'cancel.stop') -Value 'User requested stop and disable' -Encoding UTF8
Write-Output 'Stop requested. Wait for STOPPED and all 14 motors state=0 in execution.jsonl before handing over CAN.'

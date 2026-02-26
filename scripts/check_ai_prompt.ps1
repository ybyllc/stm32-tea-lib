param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

function Add-Result {
    param(
        [System.Collections.Generic.List[object]]$List,
        [string]$Name,
        [bool]$Ok,
        [string]$Detail
    )
    $List.Add([pscustomobject]@{
            Name   = $Name
            Ok     = $Ok
            Detail = $Detail
        }) | Out-Null
}

$results = New-Object 'System.Collections.Generic.List[object]'

$requiredFiles = @(
    "AGENTS.md",
    "AI_PROMPT.md",
    "AI_PROMPT_L2.md",
    "AI_TASK_TEMPLATE.md",
    "AI_PROMPT_CHANGELOG.md"
)

foreach ($file in $requiredFiles) {
    $path = Join-Path $Root $file
    Add-Result -List $results -Name "File exists: $file" -Ok (Test-Path $path) -Detail $path
}

$l1Path = Join-Path $Root "AI_PROMPT.md"
$l2Path = Join-Path $Root "AI_PROMPT_L2.md"
$agentsPath = Join-Path $Root "AGENTS.md"
$taskTemplatePath = Join-Path $Root "AI_TASK_TEMPLATE.md"
$changelogPath = Join-Path $Root "AI_PROMPT_CHANGELOG.md"

if (Test-Path $l1Path) {
    $l1 = Get-Content -Raw -Path $l1Path
    $l1Lines = (Get-Content -Path $l1Path).Count

    Add-Result -List $results -Name "L1 enable rule text exists" -Ok ($l1 -match "L1 \+ L2-") -Detail "AI_PROMPT.md"
    Add-Result -List $results -Name "L1 build command exists" -Ok ($l1 -match "UV4\.exe -b F401_TEST\.uvprojx") -Detail "AI_PROMPT.md"
    Add-Result -List $results -Name "L1 has self maintenance section" -Ok ($l1 -match "PROMPT_SELF_MAINTENANCE") -Detail "AI_PROMPT.md"
    Add-Result -List $results -Name "L1 line count <= 180" -Ok ($l1Lines -le 180) -Detail "Current lines: $l1Lines"
}

if (Test-Path $agentsPath) {
    $agents = Get-Content -Raw -Path $agentsPath
    Add-Result -List $results -Name "AGENTS requires L1+L2 echo" -Ok ($agents -match "L1 \+ L2-") -Detail "AGENTS.md"
    Add-Result -List $results -Name "AGENTS has priority rule" -Ok ($agents -match "L1 > L2") -Detail "AGENTS.md"
    Add-Result -List $results -Name "AGENTS mentions self maintenance" -Ok ($agents -match "PROMPT_SELF_MAINTENANCE") -Detail "AGENTS.md"
}

if (Test-Path $l2Path) {
    $l2 = Get-Content -Raw -Path $l2Path
    Add-Result -List $results -Name "L2 has tunable parameters section" -Ok ($l2 -match "long_press_ms" -and $l2 -match "servo_deadzone_deg") -Detail "AI_PROMPT_L2.md"
}

if (Test-Path $taskTemplatePath) {
    $taskTemplate = Get-Content -Raw -Path $taskTemplatePath
    Add-Result -List $results -Name "Task template has report format" -Ok ($taskTemplate -match "commit id ->") -Detail "AI_TASK_TEMPLATE.md"
    Add-Result -List $results -Name "Task template has distill fields" -Ok ($taskTemplate -match "精华沉淀" -and $taskTemplate -match "沉淀文件") -Detail "AI_TASK_TEMPLATE.md"
}

if (Test-Path $changelogPath) {
    $changelog = Get-Content -Raw -Path $changelogPath
    Add-Result -List $results -Name "Changelog has format section" -Ok ($changelog -match "## 记录格式") -Detail "AI_PROMPT_CHANGELOG.md"
    Add-Result -List $results -Name "Changelog has at least one record" -Ok ($changelog -match "### 20\d{2}-\d{2}-\d{2}") -Detail "AI_PROMPT_CHANGELOG.md"
}

$failed = $results | Where-Object { -not $_.Ok }

foreach ($item in $results) {
    if ($item.Ok) {
        Write-Host "[OK]   $($item.Name) - $($item.Detail)" -ForegroundColor Green
    }
    else {
        Write-Host "[FAIL] $($item.Name) - $($item.Detail)" -ForegroundColor Red
    }
}

if ($failed.Count -gt 0) {
    Write-Host "`nCheck failed: $($failed.Count) item(s)." -ForegroundColor Red
    exit 1
}

Write-Host "`nCheck passed: $($results.Count) item(s)." -ForegroundColor Green
exit 0

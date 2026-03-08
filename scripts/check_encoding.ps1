param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string[]]$Path,
    [switch]$FromGitChanged,
    [switch]$Strict
)

$ErrorActionPreference = "Stop"

function Test-Utf8Bytes {
    param([byte[]]$Bytes)
    try {
        $utf8Strict = New-Object System.Text.UTF8Encoding($false, $true)
        [void]$utf8Strict.GetString($Bytes)
        return $true
    }
    catch {
        return $false
    }
}

function Has-NonAsciiByte {
    param([byte[]]$Bytes)
    foreach ($b in $Bytes) {
        if ($b -gt 127) { return $true }
    }
    return $false
}

function Get-ChangedPaths {
    param([string]$Base)
    $out = git -C $Base status --porcelain
    $list = New-Object 'System.Collections.Generic.List[string]'
    foreach ($line in $out) {
        if ([string]::IsNullOrWhiteSpace($line) -or $line.Length -lt 4) { continue }
        $raw = $line.Substring(3).Trim()
        if ([string]::IsNullOrWhiteSpace($raw)) { continue }
        $full = Join-Path $Base $raw
        if (Test-Path $full -PathType Leaf) {
            $list.Add((Resolve-Path $full).Path) | Out-Null
        }
    }
    return $list
}

$textExt = @(".c", ".h", ".md", ".txt", ".ps1", ".json", ".yml", ".yaml")
$targets = New-Object 'System.Collections.Generic.List[string]'

if ($Path) {
    foreach ($p in $Path) {
        $resolved = Resolve-Path $p -ErrorAction Stop
        foreach ($r in $resolved) {
            if ((Get-Item $r.Path).PSIsContainer) {
                Get-ChildItem -Path $r.Path -Recurse -File | ForEach-Object {
                    $targets.Add($_.FullName) | Out-Null
                }
            }
            else {
                $targets.Add($r.Path) | Out-Null
            }
        }
    }
}

if ($FromGitChanged) {
    $changed = Get-ChangedPaths -Base $Root
    foreach ($c in $changed) { $targets.Add($c) | Out-Null }
}

if ($targets.Count -eq 0) {
    Write-Host "No target files. Use -Path or -FromGitChanged." -ForegroundColor Yellow
    exit 2
}

$files = $targets | Select-Object -Unique | Where-Object {
    Test-Path $_ -PathType Leaf
} | Where-Object {
    $ext = [System.IO.Path]::GetExtension($_).ToLowerInvariant()
    $textExt -contains $ext
}

$failed = New-Object 'System.Collections.Generic.List[string]'

foreach ($f in $files) {
    $bytes = [System.IO.File]::ReadAllBytes($f)
    if ($bytes.Length -eq 0) { continue }

    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        $failed.Add("$f : UTF-8 BOM") | Out-Null
        continue
    }

    if (-not (Has-NonAsciiByte -Bytes $bytes)) {
        continue
    }

    if (Test-Utf8Bytes -Bytes $bytes) {
        $failed.Add("$f : UTF-8 (no BOM)") | Out-Null
    }
}

if ($failed.Count -gt 0) {
    Write-Host "Found likely UTF-8 files with non-ASCII content:" -ForegroundColor Yellow
    foreach ($line in $failed) {
        Write-Host "  - $line" -ForegroundColor Yellow
    }
    if ($Strict) { exit 1 }
    exit 0
}

Write-Host "Encoding check passed." -ForegroundColor Green
exit 0

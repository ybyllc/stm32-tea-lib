param(
    [Parameter(Mandatory = $true)]
    [string[]]$Path,

    [switch]$NoBackup
)

$ErrorActionPreference = "Stop"
$utf8Strict = New-Object System.Text.UTF8Encoding($false, $true)
$cp936 = [System.Text.Encoding]::GetEncoding(936)

function Resolve-Text {
    param([byte[]]$Bytes)

    if ($Bytes.Length -ge 3 -and $Bytes[0] -eq 0xEF -and $Bytes[1] -eq 0xBB -and $Bytes[2] -eq 0xBF) {
        return [System.Text.Encoding]::UTF8.GetString($Bytes, 3, $Bytes.Length - 3)
    }

    try {
        return $utf8Strict.GetString($Bytes)
    }
    catch {
        throw "Input is not valid UTF-8. Skip convert to avoid corruption."
    }
}

foreach ($p in $Path) {
    $resolved = Resolve-Path $p -ErrorAction Stop
    foreach ($r in $resolved) {
        $full = $r.Path
        $bytes = [System.IO.File]::ReadAllBytes($full)
        $text = Resolve-Text -Bytes $bytes

        if (-not $NoBackup) {
            Copy-Item -Path $full -Destination ("$full.bak") -Force
        }

        [System.IO.File]::WriteAllBytes($full, $cp936.GetBytes($text))
        Write-Host "Converted: $full"
    }
}

param(
    [string]$Arg1, 
    [string]$Arg2  
)

# --- CONFIGURATION ---
$DefaultProject = "00Registers" 
$MakefileNames = @("STM32Make.make", "Makefile", "makefile")
# ---------------------

function Get-Makefile {
    param([string]$Path)
    foreach ($name in $MakefileNames) {
        if (Test-Path (Join-Path $Path $name)) { return $name }
    }
    return $null
}

$FoundMakefile = Get-Makefile -Path "."
$ValidActions = "clean", "release", "flash-release", "upload", "all"

# --- SMART ARGUMENT PARSING ---

if ($FoundMakefile) {
    # SCENARIO A: You are ALREADY inside a project folder
    $ProjectName = "."
    $TargetMakefile = $FoundMakefile

    # Default to "all"
    $Action = "all"

    if ($ValidActions -contains $Arg1) {
        # Case: "stm32 clean"
        $Action = $Arg1
    }
    elseif ($Arg1 -eq ".") {
        # Case: "stm32 ." or "stm32 . clean"
        if ($ValidActions -contains $Arg2) {
            $Action = $Arg2
        }
        # If Arg2 is empty, Action stays "all" (Ignores the dot)
    }
}
else {
    # SCENARIO B: You are in the Root folder
    $ProjectName = $Arg1
    $Action = $Arg2

    if ([string]::IsNullOrEmpty($ProjectName) -or $ProjectName -eq ".") {
        $ProjectName = $DefaultProject
        $Action = "all"
    }
    elseif ($ValidActions -contains $ProjectName) {
        # Case: "stm32 clean" (implies default project)
        $Action = $ProjectName
        $ProjectName = $DefaultProject
    }

    if ([string]::IsNullOrEmpty($Action)) { $Action = "all" }
    
    if (-not (Test-Path $ProjectName)) {
        Write-Host "Error: Folder '$ProjectName' not found!" -ForegroundColor Red
        exit 1
    }
    $TargetMakefile = Get-Makefile -Path $ProjectName
}

# --- VALIDATION ---
if ([string]::IsNullOrEmpty($TargetMakefile)) {
    Write-Host "Error: No valid Makefile found inside '$ProjectName'." -ForegroundColor Red
    exit 1
}

# --- EXECUTION ---
Push-Location $ProjectName
$CurrentDirName = (Get-Item .).Name
Write-Host ">> Building: $CurrentDirName" -ForegroundColor Cyan
Write-Host ">> Using: $TargetMakefile | Action: $Action" -ForegroundColor DarkCyan

switch ($Action) {
    "clean" { 
        # PowerShell Clean (Fixes 'rm' error on Windows)
        if (Test-Path "build") {
            Write-Host "   Deleting build folder..." -ForegroundColor Gray
            Remove-Item -Path "build" -Recurse -Force
        }
        # Run make clean silently just in case
        & make -f $TargetMakefile clean 2>$null
    }
    "release" { & make -j16 -f $TargetMakefile all DEBUG=0 OPT=-O3 }
    "flash-release" { & make -f $TargetMakefile flash DEBUG=0 OPT=-O3 }
    "upload" { & make -f $TargetMakefile flash }
    Default { & make -j16 -f $TargetMakefile all }
}

Pop-Location
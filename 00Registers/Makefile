param(
    [string]$Arg1, 
    [string]$Arg2  
)

# --- CONFIGURATION ---
$DefaultProject = "00Registers" 
$MakefileNames = @("STM32Make.make", "Makefile", "makefile")
# ---------------------

# Helper: Find Makefile
function Get-Makefile {
    param([string]$Path)
    foreach ($name in $MakefileNames) {
        if (Test-Path (Join-Path $Path $name)) { return $name }
    }
    return $null
}

# 1. INTELLIGENT DETECTION
# Check if we are ALREADY inside a project folder
$FoundMakefile = Get-Makefile -Path "."
$ValidActions = "clean", "release", "flash-release", "upload", "all"

if ($FoundMakefile) {
    # Scenario A: We are inside a project folder
    $ProjectName = "."
    $TargetMakefile = $FoundMakefile

    # Argument Logic for "Inside Folder"
    # Did user type: "stm32 clean" OR "stm32 . clean"?
    
    if ($ValidActions -contains $Arg1) {
        # User typed: stm32 clean
        $Action = $Arg1
    }
    elseif ($Arg1 -eq "." -and ($ValidActions -contains $Arg2)) {
        # User typed: stm32 . clean
        $Action = $Arg2
    }
    elseif (-not [string]::IsNullOrEmpty($Arg1) -and $Arg1 -ne ".") {
        # User typed: stm32 clean (but Arg1 captured it)
        $Action = $Arg1
    }
    else {
        $Action = "all"
    }
}
else {
    # Scenario B: We are in Root folder
    $ProjectName = $Arg1
    $Action = $Arg2

    # Default Logic
    if ([string]::IsNullOrEmpty($ProjectName)) {
        $ProjectName = $DefaultProject
        $Action = "all"
    }
    elseif ($ValidActions -contains $ProjectName) {
        # User typed: stm32 clean (Implying default project)
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
        # FIX: Use PowerShell to clean instead of 'make clean'
        # This solves the "rm not found" error on Windows
        if (Test-Path "build") {
            Write-Host "   Deleting build folder..." -ForegroundColor Gray
            Remove-Item -Path "build" -Recurse -Force
        }
        # Also try standard make clean just in case it does other things, but ignore errors
        & make -f $TargetMakefile clean 2>$null
    }
    "release" { & make -j16 -f $TargetMakefile all DEBUG=0 OPT=-O3 }
    "flash-release" { & make -f $TargetMakefile flash DEBUG=0 OPT=-O3 }
    "upload" { & make -f $TargetMakefile flash }
    Default { & make -j16 -f $TargetMakefile all }
}

Pop-Location
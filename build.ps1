param(
    [string]$ProjectName,
    [string]$Action = "all"
)

if ([string]::IsNullOrEmpty($ProjectName)) {
    Write-Host "Usage: ./build.ps1 <Folder> [all/clean/upload/release/flash-release]" -ForegroundColor Yellow
    exit 1
}

# Enter the project directory
Push-Location $ProjectName
$MakeFile = "STM32Make.make"

switch ($Action) {
    "clean" {
        & make -f $MakeFile clean
    }
    "release" {
        & make -j16 -f $MakeFile all DEBUG=0 OPT=-O3
    }
    "flash-release" {
        & make -f $MakeFile flash DEBUG=0 OPT=-O3
    }
    "upload" {
        & make -f $MakeFile flash
    }
    Default {
        & make -j16 -f $MakeFile all
    }
}

Pop-Location
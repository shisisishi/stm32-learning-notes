# 编译 + 运行 + 画图（Windows / PowerShell）
# 用法： .\build.ps1
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

$cc = $null
foreach ($c in 'gcc', 'clang', 'cc') {
    $p = Get-Command $c -ErrorAction SilentlyContinue
    if ($p) { $cc = $p.Source; break }
}
if (-not $cc) {
    Write-Host "没找到 C 编译器。Windows 上可以装 w64devkit（绿色免安装）或 MSYS2。" -ForegroundColor Red
    exit 1
}

Write-Host "=== 编译 (使用 $cc) ===" -ForegroundColor Cyan
& $cc -O2 -std=c99 -Wall -Wextra -o pidsim.exe sim.c pid.c plant.c -lm
if ($LASTEXITCODE -ne 0) { Write-Host "编译失败" -ForegroundColor Red; exit 1 }
Write-Host "编译通过（零警告）" -ForegroundColor Green

Write-Host ""
Write-Host "=== 运行仿真 ===" -ForegroundColor Cyan
& .\pidsim.exe

if (Get-Command python -ErrorAction SilentlyContinue) {
    Write-Host ""
    Write-Host "=== 画图 ===" -ForegroundColor Cyan
    $env:PYTHONIOENCODING = 'utf-8'
    & python plot.py
} else {
    Write-Host "没找到 python，跳过画图（不影响数据生成）" -ForegroundColor Yellow
}

# Publish committed changes by pushing to origin main, triggering GitHub Pages deployment.
param()
$ErrorActionPreference = 'Stop'
$contentRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
Push-Location $contentRoot
try {
    if ((git branch --show-current) -ne 'main') { throw 'Switch to the main branch before publishing.' }
    if (git status --porcelain) { throw 'Commit the intended content changes before publishing.' }
    git push origin main
    if ($LASTEXITCODE -ne 0) { throw 'Push failed; deployment was not triggered.' }
    Write-Output 'Pushed to origin main successfully. GitHub Actions deployment triggered.'
    Write-Output 'https://github.com/langxin11/langxin11.github.io/actions/workflows/deploy.yml'
} finally { Pop-Location }

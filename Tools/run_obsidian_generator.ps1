$path = Join-Path $PSScriptRoot 'generate_obsidian_code_notes.ps1'
$source = [System.IO.File]::ReadAllText($path, [System.Text.Encoding]::UTF8)
& ([System.Management.Automation.ScriptBlock]::Create($source))

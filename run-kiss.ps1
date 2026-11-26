param(
    [Parameter(Mandatory)][string]$Bag,
    [Parameter(Mandatory)][string]$Output,
    [string]$Topic = '/utlidar/cloud_base',
    [double]$Seconds,
    [switch]$NoDeskew,
    [string]$Image = 'kiss-icp-hackathon:humble'
)

$ErrorActionPreference = 'Stop'
$bagPath = (Resolve-Path -LiteralPath $Bag).Path
if (-not (Test-Path -LiteralPath (Join-Path $bagPath 'metadata.yaml') -PathType Leaf)) {
    throw 'Укажите папку записи с файлом metadata.yaml.'
}
$outputPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Output)
$bagPrefix = $bagPath.TrimEnd('\', '/') + '\'
$outputPrefix = $outputPath.TrimEnd('\', '/') + '\'
if ($outputPrefix.StartsWith($bagPrefix, [StringComparison]::OrdinalIgnoreCase) -or
    $bagPrefix.StartsWith($outputPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Папки записи и результатов должны быть отдельными и не вложенными друг в друга.'
}
New-Item -ItemType Directory -Path $outputPath -Force | Out-Null
$argsForRun = @(
    'run', '--rm', '--network', 'none',
    '--mount', "type=bind,source=$bagPath,target=/bag,readonly",
    '--mount', "type=bind,source=$outputPath,target=/out",
    $Image, 'python3', '/opt/go2_playback/run_kiss.py', 'run',
    '--bag', '/bag', '--topic', $Topic, '--output', '/out'
)
if ($PSBoundParameters.ContainsKey('Seconds')) {
    $argsForRun += '--seconds', $Seconds.ToString([Globalization.CultureInfo]::InvariantCulture)
}
if ($NoDeskew) { $argsForRun += '--no-deskew' }
& docker @argsForRun
exit $LASTEXITCODE

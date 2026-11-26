param(
    [string]$Image = 'kiss-icp-hackathon:humble',
    [string]$Display = 'host.docker.internal:0'
)

$ErrorActionPreference = 'Stop'
docker run --rm --init -e "DISPLAY=$Display" -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 `
    --mount "type=bind,source=$PSScriptRoot,target=/viewer,readonly" `
    $Image python3 /viewer/run_kiss.py view --output /viewer/demo/mapping `
    --frame odom --rviz-config /viewer/demo/mapping/guided.rviz
exit $LASTEXITCODE

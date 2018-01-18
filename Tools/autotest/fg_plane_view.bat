set AUTOTESTDIR="%~dp0\aircraft"

set FGDIR=FlightGear 2017.1.2
echo "Using FlightGear %FGDIR%"
D:
cd "D:\Program Files\%FGDIR%\bin"
fgfs ^
    --native-fdm=socket,in,10,,5503,udp ^
    --fdm=external ^
    --aircraft=Rascal110-JSBSim ^
    --fg-aircraft=%AUTOTESTDIR% ^
    --airport=TNCS ^
    --geometry=650x550 ^
    --bpp=32 ^
    --disable-anti-alias-hud ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --wind=0@0
pause

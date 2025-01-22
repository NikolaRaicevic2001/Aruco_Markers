# ArucoMarkers
The package integrates all the ArucoMarker related computations

## Aruco Marker Generator
    https://chev.me/arucogen/

## Installation 
git clone --recursive https://github.com/NikolaRaicevic2001/ArucoMarkers.git
git submodule update
git submodule foreach 'git checkout $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo main)'

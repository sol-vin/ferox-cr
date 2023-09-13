git clone https://github.com/c-krit/ferox
Copy-Item -Path "ferox/include" -Destination "rsrc/include" -Recurse -Force 
Copy-Item -Path "ferox/src" -Destination "rsrc/src" -Recurse -Force 
Remove-Item -Path "ferox" -Recurse -Force

cl /c /Irsrc/include /Irsrc/src rsrc\ferox.c
lib ferox.obj

$ErrorActionPreference= 'silentlycontinue'
Remove-Item -Path "rsrc\include" -Recurse -Force
Remove-Item -Path "rsrc\src" -Recurse -Force
Remove-Item -Path "ferox.obj" -Force
mkdir C:\ferox
Copy-Item -Path "ferox.lib" -Destination "C:\ferox\ferox.lib" -Force 
Move-Item -Path "ferox.lib" "rsrc\ferox.lib" -Force
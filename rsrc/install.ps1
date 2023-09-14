git clone https://github.com/c-krit/ferox

cd ferox
nmake -f NMakefile

$ErrorActionPreference= 'silentlycontinue'

mkdir C:\ferox
Copy-Item -Path "lib/ferox.lib" -Destination "C:\ferox\ferox.lib" -Force 
Copy-Item -Path "lib/ferox.lib" -Destination "..\rsrc\ferox.lib" -Force 
cd ..
Remove-Item -Path "ferox" -Recurse -Force

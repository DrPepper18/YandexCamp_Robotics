#!/bin/bash
# Run after EVERY Unity Linux build. Copies the gRPC native lib to where
# ML-Agents looks for it and removes debug junk.
set -e
cd "$(dirname "$0")"
cp Build_Linux/learning_Data/Plugins/AnyCPU/libgrpc_csharp_ext.x64.so Build_Linux/learning_Data/Managed/
rm -rf "Build_Linux/My project_BurstDebugInformation_DoNotShip"
echo "OK: gRPC lib in place, junk removed"
ls -la Build_Linux/learning_Data/Managed/libgrpc_csharp_ext.x64.so

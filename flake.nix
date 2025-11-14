{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/ros1-25.05";  # Use the correct branch for ROS 1!
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "ROS Noetic dev shell";
          packages = [
            pkgs.colcon
            # Add ROS packages required by mmWave (message generation + runtime + catkin toolchain)
            (with pkgs.rosPackages.noetic; buildEnv {
              paths = [
                ros-core
                catkin
                roscpp
                rospy
                std-msgs
                message-generation
                message-runtime
              ];
            })
            # Python packages needed for mmWave
            (pkgs.python3.withPackages (ps: with ps; [
              numpy
              opencv4
              pyserial
            ]))
          ];
          shellHook = ''
            export ROS_DISTRO=noetic
            # Put local workspace (if created) on ROS_PACKAGE_PATH automatically when in dev shell
            if [ -d "$PWD/ws/src" ]; then
              export ROS_PACKAGE_PATH="$PWD/ws/src:$ROS_PACKAGE_PATH"
            fi
            # Make sure the shared library is findable
            if [ -d "$PWD/ws/devel/lib" ]; then
              export LD_LIBRARY_PATH="$PWD/ws/devel/lib:$LD_LIBRARY_PATH"
            fi
            echo "ROS Noetic environment loaded (ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH)"
          '';
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}

# NavInfo custom packages
This directory contains the infrastructure that is used to build Navinfo custom Debian packages.

## Structure
Many package build scripts contain automatic dependency discovery. To avoid accidental discovery of dependencies we
build packages in a bare environment supplemented only by explicitly specified dependencies.

The 'bare' environment is provided by a Docker image of Ubuntu 18.04, supplemented with CUDA 10.0, OpenGL support and
some basic dependencies required to build.

There is a single build script (build.sh) that builds and runs the docker image and dispatches building of a specific 
package to a script that resides in the 'src' directory. The name of the script in the 'src' directory is identical
to the package name. All Debian packages that are created are placed in the 'deb' directory; These Debian package files
may then be distributed, for instance with Puppet, or via our local apt mirror: aptmirror.navinfo.eu

To support inter-dependencies between packages, the 'deb' directory is also mounted in the root of the docker image. 

## How to build a package
Specify the name(s) of the package(s) that you want to build as parameter(s) of the build.sh script, for example:

```console
user@host:~$ ./build.sh libeigen3-dev ceres-solver libopencv-dev-desktop
```
Directory 'deb' will now contain the generated Debian packages, directory 'log' will contain the log of the build
process. 

Note that if multiple packages are specified to the build script, the order is important only if custom packages depend
on other custom packages.

## List of custom packages
Note: Available version refers to Ubuntu 18.04 
 
| Package Name                  | Custom Version  | Available Version | Reason for custom build                    |
| :---------------------------- | :-------------: | :---------------: | :----------------------------------------- |
| navinfo-ceres-solver          | 1.14.0          | 1.13.0+dfsg0      | Newer version                              |
| navinfo-libdate-dev           | 2.4.1           | -                 | Not available, will be part of C++20       |
| navinfo-libeigen3-dev         | 3.3.7           | 3.3.4             | Newer version, dependency for ceres-solver |
| navinfo-liblas-dev            | 1.8.1-patched   | 1.8.1             | Unknown                                    |
| navinfo-libnanoflann-dev      | 1.3.1           | -                 | Not available                              |
| navinfo-libopencv-dev-desktop | 4.5.0           | 3.2.0+dfsg        | Many WITH_ flags, includes GTK support     |
| navinfo-libopencv-dev-server  | 4.5.0           | 3.2.0+dfsg        | Many WITH_ flags, no GTK support           |
| navinfo-libpdal-dev           | 2.0.1           | 1.6.0             | Newer version, I/O problems with 1.6.0     |
| navinfo-libprotobuf-dev       | 3.6.0           | 3.0.0             | Newer version                              |

Packages that are not yet converted to conform to this infrastructure

| Package Name          | Custom Version  | Available Version | Notes / Reason for custom build     |
| :-------------------- | :-------------: | :---------------: | :---------------------------------- |
| libabseil-dev         | ?               | -                 | Not available                       |
| tensorflow            | ?               | -                 | Unknown reason                      |

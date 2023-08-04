Gorgon ROS
==========

This library exists to extend the gorgon mutations library
into common ROS messages. Using this library, 
ros msgs can be passed by reference to be mutated in place. 

Gorgon ROS aims to provide C++ and Python bindings to allow for mutation
in both ROScpp and ROSpy

Note that, in order to deal with serialized messages, Gorgon ROS
has an additional MutateSerialized method for serialized messages. 
Please use the RosWrapperMutator wrapper class around a base gorgon mutator for
compatibility. The RosMutatorFactory automatically does this.

Additionally, when using the MutateSerializedAPI, no message can change size.
Otherwise, Gorgon-ros would not be able to write the mutation into the same
memory.

INSTALLATION
============

REQUIREMENTS
------------

Gorgon ROS requires both the base Gorgon Mutations Library and
ROS. Additionally, the Mutation Factory requires NLohmann's cpp json
for runtime creation of Mutators. 

1. [Install ROS](http://wiki.ros.org/ROS/Installation). 
  * Gorgon ROS currently has only been tested on Melodic, but should work
on most versions with the same message structure. 

2. [Download and Install the Gorgon Mutations Library](https://bitbucket.rec.ri.cmu.edu/projects/GOR/repos/gorgon-mutations-lib/). 
  * CPP Installation is recomended via a local install, but systemwide
    methods should also work.

3. (Optional) [include NLohmann's cpp json](https://github.com/nlohmann/json).
  * This is only required for the CPP mutation Factory API

INSTALL INSTRUCTIONS
--------------------

Installation follows a similar procedure as the Gorgon Mutations Library.

The current shared library is compiled from source via cmake:

`cmake -B build`  
`cmake --build build`  

### c++  

Global C++ installation for a user is run via:

`sudo make -C build install`

Then update the system packages by running 

`sudo ldconfig`

The file can be linked at compile time with  
`-lgorgon`  


Alternatively, the library can also be included via cmake:
```cmake
find_package(gorgon REQUIRED)
find_package(gorgon_ros REQUIRED)
...
add_library(foo ...)
...
target_link_libraries(foo PRIVATE gorgon::gorgon gorgon_ros::gorgon_ros)
```

After Installation, the library can be used freely:  
```cpp
#include <gorgon_ros/mutators/point_msgs.h>

...

PointMsg::AnisotropicGaussianNoiseMutator MyMutator(0.0, 5.0, 0.0, 5.0);
```   
or by using any other headerfile for a type defined with mutators.


### Local Install

When using docker containers, the default install location `/usr/local`
may not be mounted. In this situation, install the library locally using:

`cmake -DCMAKE_INSTALL_PREFIX=./lib -B build`  
`cmake --build build`  
`sudo make -C build install`  
 
The library can then be added to a cmake package with a local path
as in the following:

```cmake
find_package(gorgon_ros 
            NAMES gorgon_ros 
            HINTS "../gorgon_ros/lib/"
)
...
add_library(foo ...)
...
target_link_libraries(foo gorgon_ros::gorgon_ros)

```

C++ files can then include gorgon via local paths:

```cpp
#include <gorgon_ros/mutators/point_msgs.h>
...
PointMsg::AnisotropicGaussianNoiseMutator MyMutator(0.0, 5.0, 0.0, 5.0);
```


UNINSTALL
---------

To uninstall, use the `install_manifest.txt` in the build directory:

`sudo xargs rm < install_manifest.txt`  



DOCUMENTATION
=============

Documentation can be configured using the `-DBUILD_DOCS` argument:

`cmake -DBUILD_DOCS=ON -B build`

Once installed, Documentation can be generated with the following:

`cmake --build build --target docs`

Doxygen will generate html output in `build/docs/html`


TESTING
===========

Testing is implemented via GoogleTest in the tests directory.
Currently, tests are not installed by default. 

To install, use the `-DBUILD_TESTING` argument:

`cmake -DBUILD_TESTING=ON -B build`

Upon installing, tests can be run with:

`cmake --build build -t test`   

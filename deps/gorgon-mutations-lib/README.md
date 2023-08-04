Gorgon Mutation Library
=======================

This repository contains a mutation library for fault injection
robustness testing. Development aims to provide a centralized, extensible 
library to apply various 'mutations' to data that imitate real failure modes.

Gorgon is middleware agnostic and runs in C++. Mutations are 
organized via altered data type and should run in-place for 
the C++ implementation. 


ROADMAP
=======

Gorgon is a C++ library defining Mutator Classes, ojects which can be 
included and called on incoming data to alter it. 

An additional MutatorFactory is included in the Utils header to create 
appropriate mutators in datastreams defined at runtime.

Additionally, Gorgon is extensible with Middleware specific packages
for required data types. Custom libraries  extend Gorgon
Mutators and Utils and allow seamless usage of the baseline library.


INSTALLATION
===========

REQUIREMENTS
------------
Gorgon Requires [NLohmann's json cpp](https://github.com/nlohmann/json#cmake)
for reading and building mutators from json configurations. 

Additionally, Gorgon Optionally requires GTest for unit testing.


NLohmann's json can be included via cmake fetch-content:

```cmake
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(foo PRIVATE nlohmann_json::nlohmann_json)
```
For other options, see the 
[nlohmann/json repo](https://github.com/nlohmann/json#integration)

Nlohmann's Json and GoogleTest can also be supplied using a local install.
See [Using Local Libraries](#USING-LOCAL-LIBRARIES)


INSTALL INSTRUCTIONS
--------------------

Currently, the C++ source can be compiled into a shared library and included 
as needed. Docker containers that don't mount /usr/local should follow the
usage instructions for a  local install (see header below)

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


Alternatively, the library can also be includable via cmake:
```cmake
find_package(gorgon REQUIRED)
...
add_library(foo ...)
...
target_link_libraries(foo PRIVATE gorgon::gorgon)
```

After Installation, the library can be included via:  
```cpp
#include <gorgon/mutators/floats.h>

...

GaussianNoiseMutator MyMutator(10, 1);
```   
or by using any other headerfile for a type defined with mutators.

### Local Install <a name="Local-Install"></a>

When using docker containers, the default install location `/usr/local`
may not be mounted. In this situation, install the library locally using:

`cmake -DCMAKE_INSTALL_PREFIX=./lib -B build`  
`cmake --build build`  
`sudo make -C build install`  
 
The library can then be added to a cmake package with a local path
as in the following:

```cmake
find_package(gorgon 
            NAMES gorgon 
            HINTS "../../../gorgon-mutations-lib/lib/"
)
...
add_library(foo ...)
...
target_link_libraries(foo gorgon::gorgon)

```

C++ files can then include gorgon via local paths:

```cpp
#include <gorgon/mutators/floats.h>

...

GaussianNoiseMutator MyMutator(10, 1);
```

UNINSTALL 
---------

To uninstall, use the `install_manifest.txt` in the build directory:

`sudo xargs rm < install_manifest.txt`  


USING LOCAL LIBRARIES <a name="USING-LOCAL-LIBRARIES"></a>
---------------------

Gorgon Also allows users to define local installations of Nlohmann's Json and
Google Test for better integration with potential version differences.

For a different Json path, build cmake with:  
`-DLOCAL_JSON_LIB=</path/to/json/>`  

For a different Google Test Path, build with:  
`-DLOCAL_GTEST_LIB=</path/to/gtest/> -BUILD_TESTING=ON`

Both options are compatible.

EXTENSION
=========

When defining custom type mutators, follow these specific steps:
1. Create a TypeMutator that derives from the Gorgon BaseMutator
    - BaseMutator requires the mutateRef method to be defined. This
    should simply cast the given void pointer to a pointer to the desired
    type, then call a virtual mutate function
2. Create Custom mutators derived from the TypeMutator
    - this will ensure the custom mutator can be used in Mutator containers
3. (Optional) Create mutateByReferenceSeeded implementation
    - this is only necessary if the mutator utilizes a random number generator.
    - otherwise, the method will call mutateRef by default
3. Extend the MutatorFactory with a Defined custom mutator blueprint
    - Create a subclass of the MutatorFactory and add custom
    mutator creation blueprint lambdas to the m_MutatorMap:
```cpp
// set Mutator Creation Lambda
    m_MutatorMap["MyCustomMutator"] = 
        [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator>
        { 
            return std::make_shared<MyCustomMutator>(args.at("arg1").get<type>(),
                                                    args.at("arg2").get<type>(),
                                                    args.at("arg3").get<type>()  
                                                    seed  
            ); 
        };
```


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

See [Using Local Libraries](#USING-LOCAL-LIBRARIES) to use a local install
of Google Test.

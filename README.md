# AVE - Autonomous Vehicle Control Toolbox

The Autonomous Vehicle Control Toolbox (**AVE** for short) is a development of the Chair of Control Engineering at the University of Rostock.
This toolbox contains MATLAB packages and Simulink libraries that are largely universal and frequently required for the software development of autonomous vehicles (see [Design Philosophy](#design-philosophy)).
The AVE Toolbox is based on the standard MATLAB/Simulink libraries.

> **MATLAB Version**\
> R2024a


## How To Use
To use the AVE toolbox, add the `AVE.prj` project file as reference project to your own simulink project.
This will add the `library` and `packages` directories to your project path automatically.


## Design Philosophy
The following points describe the purpose of the AVE Toolbox. If the toolbox is to be expanded, the design philosophy of the toolbox must be taken into account.

**The more generic, the better**\
MATLAB functions should be kept as universal as possible. A function should be application-independent.

**SI units and data types**\
The SI system of units should always be used for calculations.
Anglular values should be in a symmetrical range `[-pi, +pi)`.
For floating point numbers `double` should be used as the default data type.
Bytes should be represented with `uint8` and countable quantities with `int32` or `uint32`.

**No hardware dependency**\
The AVE Toolbox is not intended to provide driver blocks for hardware.
The reason for this is that the corresponding hardware becomes obsolete over time and is replaced by new hardware.
In addition, a particular sensor or actuator, for example, is only installed in one vehicle.
A driver block in a universal library is then practically not used at all and remains in the library, even if the associated hardware component is never used again.
Separate hardware-specific libraries are suitable for such developments.

**New development instead of redevelopment**\
Functions that are already available as standard in MATLAB or Simulink should not be developed again.
However, if functions in MATLAB are not available in Simulink, a corresponding Simulink block can be added to the library.
If an existing function is not suitable for code generation or contains too much overhead, the desired functionality can also be redeveloped.

**No dependency on third-party software**\
Simulink blocks in particular should not depend on third-party libraries, as it cannot be assumed that every user of the AVE Toolbox has the required third-party software.
It should not be necessary to install third-party software in order to use the AVE Toolbox.

**Package + Block**\
If possible, for new developments a function should be provided in the MATLAB package.
In the simulink library, a block can provide this functionality using an embedded MATLAB function.
In this way, the function can be used both in MATLAB and Simulink and is only defined in one place. 

**Standardized naming**\
The naming of functions, for example, should be as uniform as possible.
Meaningful English-language names should be used as function names.
In this context, it may be better to use rather long function names if this makes the functionality behind the function easier to recognize.
Special characters should always be avoided.
For example, Simulink blocks should not be named like `v = s [m] / t [s]`, as the path name to blocks can then lead to problems, e.g. `Model/Subsystem 1/v = s [m] / t [s]/Gain 1`.
When writing functions, classes, variables and signals, camel-case notation should be used, with functions and classes starting with capital letters and variables and signals starting with small letters.
A function definition such as `closestPoint = GetClosestPoint(listOfPoints, queryPoint)` is then much more intuitive than `result = find_pnt(a, b)`.
The uniform naming is not a law, but a recommendation.

**Documentation**\
Both functions in MATLAB packages and Simulink blocks in the library should always be documented.
In the function, a comment describing the purpose of the function and the inputs and outputs is sufficient.
In a block, this description can be stored in the block mask.


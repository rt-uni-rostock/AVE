# Example: Automatic Bus Serialization
This example demonstrates how to generate subsystem reference models for packing and unpacking bus signals.
All bus signals to be considered are defined by callback functions that are stored in the [Messages](Messages/) directory.
These callbacks return a single structure.

Run the MATLAB script `Example_CreateBusSerialization.m` to generate the subsystem reference models.
For each callback function in the Messages directory, two subsystem reference models are generated: One for packing and one for unpacking the message.
The output directory `Serialization` is created automatically.
All simulink objects like bus objects are going to be stored in a simulink data dictionary `MessageData.sldd`.

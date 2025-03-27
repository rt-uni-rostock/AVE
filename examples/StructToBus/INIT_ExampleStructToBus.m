structExample = struct();
structExample.timestamp            = 1;
structExample.position             = [2;3;4];
structExample.velocity             = [5;6;7];
structExample.types.u8_scalar      = uint8(8);
structExample.types.i32_matrix     = int32([9 10; 11 12]);
structExample.nested_bus.timestamp = 13;
structExample.multivar_bus(1).id   = uint8(100);
structExample.multivar_bus(1).x    = single(101);
structExample.multivar_bus(1).y    = single(102);
structExample.multivar_bus(1).z    = single(103);
structExample.multivar_bus(2).id   = uint8(200);
structExample.multivar_bus(2).x    = single(201);
structExample.multivar_bus(2).y    = single(202);
structExample.multivar_bus(2).z    = single(203);

ave.Struct2Bus(structExample, 'busExample');

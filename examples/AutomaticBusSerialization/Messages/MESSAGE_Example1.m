function msg = MESSAGE_Example1()
    msg = struct();
    msg.timestamp    = 0.0;
    msg.position     = zeros(3,1);
    msg.nested.value = uint8(0);
end


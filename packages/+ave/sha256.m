function hash = sha256(bytes, numBytes)
    %ave.sha256 Compute the SHA256 for given binary data.
    % 
    % PARAMETERS
    % bytes    ... Array of bytes of type uint8 containing the binary data for which to compute the hash.
    % numBytes ... Number of bytes to be considered for the hash computation.
    % 
    % RETURN
    % hash ... The 256-bit hash value, represented by a 32-by-1 vector of datatype uint8.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20250308    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    % ensure correct inputs
    assert(isa(bytes,'uint8'), 'Input "bytes" must be of type uint8!');
    assert(isscalar(numBytes), 'Input "numBytes" must be scalar!');
    numBytes = uint64(numBytes);

    % default output
    hash = zeros(32,1,'uint8');

    % initialize SHA 256 context
    ctx_data = zeros(64,1,'uint8');
    ctx_datalen = uint32(0);
    ctx_bitlen = uint64(0);
    ctx_state = uint32([0x6a09e667; 0xbb67ae85; 0x3c6ef372; 0xa54ff53a; 0x510e527f; 0x9b05688c; 0x1f83d9ab; 0x5be0cd19]);

    % update
    for i1 = uint64(1):numBytes
        ctx_data(ctx_datalen + uint32(1)) = bytes(i1);
        ctx_datalen = ctx_datalen + uint32(1);
        if(uint32(64) == ctx_datalen)
            ctx_state = sha256_transform(ctx_state, ctx_data);
            ctx_bitlen = ctx_bitlen + uint64(512);
            ctx_datalen = uint32(0);
        end
    end

    % final
    i = ctx_datalen;
    if(ctx_datalen < uint32(56))
        ctx_data(i + uint32(1)) = uint8(0x80);
        i = i + uint32(1);
        while(i < uint32(56))
            ctx_data(i + uint32(1)) = uint8(0x00);
            i = i + uint32(1);
        end
    else
        ctx_data(i + uint32(1)) = uint8(0x80);
        i = i + uint32(1);
        while(i < uint32(64))
            ctx_data(i + uint32(1)) = uint8(0x00);
            i = i + uint32(1);
        end
        ctx_state = sha256_transform(ctx_state, ctx_data);
        ctx_data(1:56) = zeros(56,1,'uint8');
    end
    ctx_bitlen = ctx_bitlen + uint64(ctx_datalen) * uint64(8);
    ctx_data(63+1) = uint8(bitand(uint64(255), ctx_bitlen));
	ctx_data(62+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -8)));
	ctx_data(61+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -16)));
	ctx_data(60+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -24)));
	ctx_data(59+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -32)));
	ctx_data(58+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -40)));
	ctx_data(57+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -48)));
	ctx_data(56+1) = uint8(bitand(uint64(255), bitshift(ctx_bitlen, -56)));
    ctx_state = sha256_transform(ctx_state, ctx_data);
    for i = uint32(0):uint32(3)
        hash(i+uint32(1))      = uint8(bitand(uint32(255), bitshift(ctx_state(1), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 4))  = uint8(bitand(uint32(255), bitshift(ctx_state(2), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 8))  = uint8(bitand(uint32(255), bitshift(ctx_state(3), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 12)) = uint8(bitand(uint32(255), bitshift(ctx_state(4), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 16)) = uint8(bitand(uint32(255), bitshift(ctx_state(5), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 20)) = uint8(bitand(uint32(255), bitshift(ctx_state(6), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 24)) = uint8(bitand(uint32(255), bitshift(ctx_state(7), -(int32(24) - int32(i) * int32(8)))));
        hash(i+uint32(1 + 28)) = uint8(bitand(uint32(255), bitshift(ctx_state(8), -(int32(24) - int32(i) * int32(8)))));
    end
end

function ctx_state = sha256_transform(ctx_state_in, ctx_data)
    sha256_k = uint32([0x428a2f98; 0x71374491; 0xb5c0fbcf; 0xe9b5dba5; 0x3956c25b; 0x59f111f1; 0x923f82a4; 0xab1c5ed5; 0xd807aa98; 0x12835b01; 0x243185be; 0x550c7dc3; 0x72be5d74; 0x80deb1fe; 0x9bdc06a7; 0xc19bf174; 0xe49b69c1; 0xefbe4786; 0x0fc19dc6; 0x240ca1cc; 0x2de92c6f; 0x4a7484aa; 0x5cb0a9dc; 0x76f988da; 0x983e5152; 0xa831c66d; 0xb00327c8; 0xbf597fc7; 0xc6e00bf3; 0xd5a79147; 0x06ca6351; 0x14292967; 0x27b70a85; 0x2e1b2138; 0x4d2c6dfc; 0x53380d13; 0x650a7354; 0x766a0abb; 0x81c2c92e; 0x92722c85; 0xa2bfe8a1; 0xa81a664b; 0xc24b8b70; 0xc76c51a3; 0xd192e819; 0xd6990624; 0xf40e3585; 0x106aa070; 0x19a4c116; 0x1e376c08; 0x2748774c; 0x34b0bcb5; 0x391c0cb3; 0x4ed8aa4a; 0x5b9cca4f; 0x682e6ff3; 0x748f82ee; 0x78a5636f; 0x84c87814; 0x8cc70208; 0x90befffa; 0xa4506ceb; 0xbef9a3f7; 0xc67178f2]);
    ctx_state = ctx_state_in;
    tmp_m = zeros(64,1,'uint32');
    tmp_i = uint32(0);
    tmp_j = uint32(0);
    while(tmp_i < uint32(16))
        tmp_m(tmp_i+uint32(1)) = bitor(bitor(bitshift(uint32(ctx_data(tmp_j+uint32(1))), 24), bitshift(uint32(ctx_data(tmp_j+uint32(2))), 16)), bitor(bitshift(uint32(ctx_data(tmp_j+uint32(3))), 8), uint32(ctx_data(tmp_j+uint32(4)))));
        tmp_i = tmp_i + uint32(1);
        tmp_j = tmp_j + uint32(4);
    end
    while(tmp_i < uint32(64))
        tmp_m(tmp_i+uint32(1)) = u32_add(u32_add(SIG1(tmp_m(tmp_i-uint32(1))), tmp_m(tmp_i-uint32(6))), u32_add(SIG0(tmp_m(tmp_i-uint32(14))), tmp_m(tmp_i-uint32(15))));
        tmp_i = tmp_i + uint32(1);
    end
	tmp_a = ctx_state(1);
	tmp_b = ctx_state(2);
	tmp_c = ctx_state(3);
	tmp_d = ctx_state(4);
	tmp_e = ctx_state(5);
	tmp_f = ctx_state(6);
	tmp_g = ctx_state(7);
	tmp_h = ctx_state(8);
    tmp_i = uint32(0);
    while(tmp_i < uint32(64))
        tmp_t1 = u32_add(u32_add(u32_add(tmp_h, EP1(tmp_e)), u32_add(CH(tmp_e,tmp_f,tmp_g), sha256_k(tmp_i+uint32(1)))), tmp_m(tmp_i+uint32(1)));
		tmp_t2 = u32_add(EP0(tmp_a), MAJ(tmp_a,tmp_b,tmp_c));
		tmp_h = tmp_g;
		tmp_g = tmp_f;
		tmp_f = tmp_e;
		tmp_e = u32_add(tmp_d, tmp_t1);
		tmp_d = tmp_c;
		tmp_c = tmp_b;
		tmp_b = tmp_a;
		tmp_a = u32_add(tmp_t1, tmp_t2);
        tmp_i = tmp_i + uint32(1);
    end
	ctx_state(1) = u32_add(ctx_state(1), tmp_a);
	ctx_state(2) = u32_add(ctx_state(2), tmp_b);
	ctx_state(3) = u32_add(ctx_state(3), tmp_c);
	ctx_state(4) = u32_add(ctx_state(4), tmp_d);
	ctx_state(5) = u32_add(ctx_state(5), tmp_e);
	ctx_state(6) = u32_add(ctx_state(6), tmp_f);
	ctx_state(7) = u32_add(ctx_state(7), tmp_g);
	ctx_state(8) = u32_add(ctx_state(8), tmp_h);
end

function ret = u32_add(a,b)
    ret = uint32(mod(uint64(a) + uint64(b), uint64(0x100000000)));
end

function ret = ROTRIGHT(a,b)
    ret = bitor(bitshift(a,-int64(b)), bitshift(a,int64(32)-int64(b)));
end

function y = NOT(x)
    y = bitxor(x, uint32(0xFFFFFFFF));
end

function ret = CH(x,y,z)
    ret = bitxor(bitand(x,y), bitand(NOT(x),z));
end

function ret = MAJ(x,y,z)
    ret = bitxor(bitxor(bitand(x,y), bitand(x,z)), bitand(y,z));
end

function y = EP0(x)
    y = bitxor(bitxor(ROTRIGHT(x,2), ROTRIGHT(x,13)), ROTRIGHT(x,22));
end

function y = EP1(x)
    y = bitxor(bitxor(ROTRIGHT(x,6), ROTRIGHT(x,11)), ROTRIGHT(x,25));
end

function y = SIG0(x)
    y = bitxor(bitxor(ROTRIGHT(x,7), ROTRIGHT(x,18)), bitshift(x, -3));
end

function y = SIG1(x)
    y = bitxor(bitxor(ROTRIGHT(x,17), ROTRIGHT(x,19)), bitshift(x, -10));
end

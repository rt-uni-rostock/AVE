s = 'Hello';

bytes = uint8(s);
numBytes = numel(bytes);
hash = ave.sha256(bytes, numBytes);

fprintf('sha256:  ');
for i=1:numel(hash), fprintf('%02x',hash(i)); end
fprintf('\n');


% Hello: 185f8db32271fe25f561a6fc938b2e264306ec304eda518007d1764826381969
fprintf('correct: 185f8db32271fe25f561a6fc938b2e264306ec304eda518007d1764826381969\n');

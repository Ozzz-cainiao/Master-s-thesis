function [cc, dd] = testMulti(aa, bb)
% if (size(aa,2) == size(bb, 1))
%     cc = aa * bb;
% else
%     cc = zeros(5,0);
% end   
cc = aa * bb;
dd = inv(bb);
end

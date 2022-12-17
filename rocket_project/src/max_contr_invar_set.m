function [max_invar_set] = max_contr_invar_set(P, A, B, G, g)
%returns the maximum controlled invariant set of the polytope P
%   input:  - P: polytope of the set {x | Hx <= h}
%           - A, B: system variables xdot = Ax + Bu (B is optional)
%           - G, g: constraints of the input {u | Gu <= g} 
%                   (both optional)
%   output: - max_invar_set: a polytope that is the maximal controlled
%                            invariant set of P

loopcount = 0;

while loopcount < 100
    [newH, newh] = double(P);
    
    if nargin > 2
        preMatrix = [newH*A newH*B; zeros(size(G,1)) G];
        preVector = [newh; g];
        preP3D = polytope(preMatrix, preVector);
        preP = preP3D.projection(1:size(A,1));
    else
        preP = polytope(newH*A, newh);
    end
    max_invar_set = intersect(preP, P);

    try
        if P == max_invar_set; break; end
    catch
        error("Cannot compare different dimensions")
    end
    P = max_invar_set;
    loopcount = loopcount + 1;
end

end
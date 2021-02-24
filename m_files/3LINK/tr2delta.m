function delta = tr2delta(A, B)

    % handle arguments, convert all to 4x4 matrices
    if nargin > 0
        disp("here")
        if isa(A, 'SE3')
            T1 = A.double
        elseif ishomog(A)
            T1 = A;
        else
            error('SMTB:tr2delta:badarg', 'T1 should be a homogeneous transformation');
        end
        T0 = eye(4,4);
    end
    if nargin > 1
        disp("here >1")
        T0 = T1;
        if isa(B, 'SE3')
            disp("double?")
            T1 = B.double
        elseif ishomog(B)
            T1 = B;
        else
                error('SMTB:tr2delta:badarg', 'T0 should be a homogeneous transformation');
        end
    end
    
    % compute incremental transformation from T0 to T1 in the T0 frame
    TD = inv(T0) * T1

    % build the delta vector
    delta = [transl(TD); vex(t2r(TD) - eye(3,3))]
    
    
%    R0 = t2r(T0); R1 = t2r(T1);
%    % in world frame
%    %[th,vec] = tr2angvec(R1*R0');
%    dR = vex(R1*R0');
%    %delta = [ (T1(1:3,4)-T0(1:3,4)); th*vec' ];
%    delta = [ (T1(1:3,4)-T0(1:3,4)); dR];

% same as above but more complex
%    delta = [	T1(1:3,4)-T0(1:3,4);
%        0.5*(	cross(T0(1:3,1), T1(1:3,1)) + ...
%            cross(T0(1:3,2), T1(1:3,2)) + ...
%            cross(T0(1:3,3), T1(1:3,3)) ...
%        )];
end

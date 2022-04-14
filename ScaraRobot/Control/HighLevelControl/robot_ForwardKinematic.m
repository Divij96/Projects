function [T, RT] = robot_ForwardKinematic( DH, varargin )
% Compute Symbolic Forward Kinematic of Robot Manipulator based on DH
% Matrix. Important: Input Arg are Symbolic!
% Input:
%   DH: Matrix containing all Denavit-Hartenberg params (thera, d, a, alpha)
%       First n/2 rows = Joints, next n/2 rows = center of masses
%   T0_W: Transformation from Base Frame 0 to World Frame
%   CM: True if DH also contains information about Center of Masses
% Return:
%   T_W: Matrix 4x4xn that contains all Joint to World Tranformations in
%   the first n/2 entires and all CMs to World Tranformations in next n/2
%   entires

    %% Parse Input
    p = inputParser;
    addRequired(p, 'DH');
    addOptional(p, 'T0_W', sym(eye(4)));    % init World Frame = Base Frame as default
    addOptional(p, 'CM', true);
    parse(p, DH, varargin{:});

    T0_W = p.Results.T0_W;
	N = size(DH, 1);            % size of DH Table
    
    if(p.Results.CM == true)
        N_joint = floor(N/2);   % number of joints
        N_cm = floor(N/2);      % number of center of masses
    else
        N_joint = N;            % number of joints
        N_cm = 0;               % number of center of masses        
    end

    %% Compute all Relative Homogenous Transformations
    
    % T1_0, T2_1, ... and Tcm1_0, Tcm2_1, ...
    RT = sym( zeros(4,4,N) );
    for i = 1 : N
        RT(:,:,i) =D2H( DH(i,:) ) ;
    end
    
    %% Compute all absolut Homogenous Transformations form Joints to World Frame
    
    % Initalize first Transformation with H1_W
    T = sym( zeros(4,4,N_joint) );
    T(:,:,1) = simplify( T0_W * RT(:,:,1));
    
    % multiply Transformation matrices in succession
    % Hi_W = Hi-1_W * Hi_i-1
    for i = 2 : N_joint
       T(:,:,i) = simplify( T(:,:,i-1) * RT(:,:,i) );
    end

    %% Compute all absolute Homogenous Transformations form CMs to World Frame
    
    if( N_cm ~= 0)
    
        % Initalize first Transformation with Hcm1_W
        Tcm = sym( zeros(4,4,N_cm) );
        Tcm(:,:,1) = simplify( T0_W * RT(:,:,N_joint+1));

        % multiply Cm Transformations with Joint Transformations
        % Hcmi_W = Hi-1_W * Hcmi_i-1
        for i = 2 : N_cm
            Tcm(:,:,i) = simplify( T(:,:,i-1) * RT(:,:,N_joint+i) );
        end
        
        % combine Transfomrations into Transformation stack
        T = cat(3, T, Tcm);
    end
    
end


function H = D2H(D)
% Computes a Homogeneous Transformation H form a set of DH Parameters
% Input:
%   D: vector with DH Parameters between two joint i and joint i-1
%      with the following order D = [thera, d, a, alpha]
% Ouput:
%   H: Relative Homogenous Tranfromation Hi_i-1 between both joints

% extract variables
theta = D(1);
d = D(2);
a = D(3);
alpha = D(4);

% construct Relative Homogenous Transformation Ti_i-1
H = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
    0 sin(alpha) cos(alpha) d;...
    0 0 0 1];

end
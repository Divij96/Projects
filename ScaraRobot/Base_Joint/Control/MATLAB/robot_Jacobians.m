function [J] = robot_Jacobians(T, type, varargin)
% Compute all Jacobian Matices for Robot Kinematics
% Important: Input Arg are Symbolic!
% Input:
%   T_W: 3d Matrix containing all Absolute Joint Transformations in first
%   n/2 entires and all Absolute CM Transformations in next n/2 entires
%   type: String containing 'r' or 'p' to describe type of each joint
%   Tcm_W: 3d Matrix containing all Absolute CM Transformations
%   CM: True if T also contains information about Center of Masses
% Return:
%   J_W: Matrix 6x4xn that contains all Jacobians of Joints
%   Jcm_W: Matrix 6x4xn that contains all Jacobians of CMs

    %% Parse Input
    p = inputParser;
    addRequired(p, 'T_W');
    addRequired(p, 'type');
    addOptional(p, 'CM', true);
    addOptional(p, 'T0_W', sym(eye(4)));
    parse(p, T, type, varargin{:});

    T0_W = p.Results.T0_W;
    type = p.Results.type;
    
    N = size(T, 3);
    
    if(p.Results.CM == true)
        N_joint = floor(N/2);   % number of joints
        CM = 1;                 % cm flag
    else
        CM = 0;               	% cm flag 
        N_joint = N;
    end

    %% Compute all Jacobians for Joints
    J = sym( zeros(6, N_joint, N) );

    % Combine all Absolut Joint Transformation and World Transf
    % T0_W, T1_W, T2_W, ..., Tn_W
    T_cat = cat(3, T0_W, T);
    
    % for Joints (cm = 0) and CMs (cm = 1) do
    for cm = 0 : CM
        k = cm*N_joint; % index

        % for every joint or CM do
        for i = 1 : N_joint

            % for every colum in Jacobian (Matrix i) do
            for j = 1 : i

                z = T_cat(1:3, 3, j);
                t_start = T_cat(1:3, 4, j);
                t_dest = T(1:3, 4, k+i);

                if type(j) == 'r'
                    % Revolute Joint
                    J(:,j,k+i) = simplify( [cross(z, (t_dest - t_start)); z] );
                else
                    % Prismatic Joint
                    J(:,j,k+i) = simplify( [z; [0;0;0]] );
                end

            end

        end
        
    end

end


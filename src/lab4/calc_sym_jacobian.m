clear
syms q1(t) q2(t) q3(t) q4(t);

% Joints are in radians
joints = [q1,q2,q3,q4];
fks = joints2fk(joints);
last_fk = simplify(fks(:,:,4));

% Convert the fractions to decimals with 5 sig figs
approx_last_fk = vpa(last_fk, 5)
approx_pos = vpa(last_fk(1:3,4), 5)

% Differentiate
d_last_fk = simplify(diff(last_fk,t));

% Convert the fractions to decimals with 5 sig figs
approx_d_last_fk = vpa(d_last_fk, 5)
approx_d_pos = vpa(d_last_fk(1:3,4), 5)

% Extract the derivative
A = subs(approx_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {1,0,0,0});
B = subs(approx_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {0,1,0,0});
C = subs(approx_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {0,0,1,0});
D = subs(approx_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {0,0,0,1});

jacob = vpa(simplify([A B C D]),5)

% Verify solution
joint_d = [diff(q1(t),t); diff(q2(t),t); diff(q3(t),t); diff(q4(t),t)];
solved_d_pos = vpa(simplify(jacob * joint_d),5)

original = vpa(subs(subs(approx_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {1,1,1,1}), {q1(t),q2(t),q3(t),q4(t)}, {1,2,3,4}),5)
solved = vpa(subs(subs(solved_d_pos, {diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t)}, {1,1,1,1}), {q1(t),q2(t),q3(t),q4(t)}, {1,2,3,4}),5)

%% Functions stolen from Robot
% Gets a DH table using the current joint positions
% Takes in a 1x4 vector of joint angles in deg
function dhTable = getDHTable(joint_pos)
    % Initialize the generalized DH table
    dhTable = sym([0,             60.25, 0,                -pi/2;
                  -atan2(128,24), 0      sqrt(24^2+128^2), 0;
                   atan2(128,24), 0,     124,              0;
                   0,             0,     133.4,            0]);

    % Add the joint angles
    dhTable(:, 1) = dhTable(:, 1) + transpose(joint_pos);
end % getDHTable

% Takes a 1x4 array of DH parameters in form: [theta, d, a, alpha]
% Returns a 4x4 frame transformational matrix  
function dhMat = dh2mat(DHrow)
    theta = DHrow(1,1);
    d = DHrow(1,2);
    a = DHrow(1,3);
    alpha = DHrow(1,4);

    dhMat = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta); 
             sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); 
             0           sin(alpha)             cos(alpha)            d;
             0           0                      0                     1];
end % dh2mat

% Takes in an nx4 array corresponding to the n rows of the full DH parameter table
% Each joint gets a 4x4 homogenous matrix that gives the global orientation and position of the joint
% Outputs a (4x4)xn array where “n” is the number of joints
function dhfk_mats = dh2fk(DHparams)
    % Number of joints
    n = size(DHparams, 1);

    % Initialize array to hold transformation matrices
    dhfk_mats = sym(zeros(4, 4, n)); 

    % Initialize the transformation as an identity matrix
    dhfk_prev = eye(4);
    initialTranslation = [0, 0, 36.076];
    dhfk_prev(1:3, 4) = initialTranslation;
                
    for joint_index = 1:n
        % Compute the transformation matrix for the current joint
        dhfk_current = dh2mat(DHparams(joint_index, :));

        % Accumulate the transformation
        dhfk_prev = dhfk_prev * dhfk_current;

        % Store the transformation in the output array
        dhfk_mats(:, :, joint_index) = dhfk_prev;
    end
end % dh2fk

% Takes a 1x4 vector of joint angles
% Outputs the full transformations from the reference frame
function fk = joints2fk(joint_pos)
    fk = dh2fk(getDHTable(joint_pos));
end % joints2fk
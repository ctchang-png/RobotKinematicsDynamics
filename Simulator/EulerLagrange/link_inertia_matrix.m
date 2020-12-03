function [mu,...
          m,...
          mu_translational,...
          mu_rotational,...
          b,...
          mu_rotational_link] = link_inertia_matrix(link_vector,link_radius)
% Construct an inertia matrix for a cylindrical link, as a function of the
% link vector (which defines the direction and length of the cylinder axis
% as seen in the link frame, and the radius of the link
%
% Inputs:
%
%   link_vector: A 3x1 link vector
%   link_radius: A scalar containing the radius of the link
%
% Output:
%
%   mu: The 6x6 inertia matrix for the link with respect to translations
%       of the center of the link and rotations of the link frame
%
%
% Additional output (These are intermediate variables. Having the option
%   to return them as outputs lets our automatic code checker tell you
%   where problems are in your code):
%
%   m: The mass of the link mu_translational: The inertia matrix of the
%       link with respect to translations 
%   mu_rotational: The inertia matrix of the link with respect to rotations
%       about its center of mass, in a frame aligned with the link vector
%       (the cylinder's main axis)
%   b: A matrix of orthogonal basis vectors, the first of which is aligned
%       with the link vector
%   mu_rotational_link: The inertia matrix of the link with respect to
%       rotations about its center, in the link frame

    

    % A. Take the mass of the link as being proportional to its length
    % times the square of its radius (we're dropping a factor of pi
    % here for conveniece). Call this mass 'm'
    m = norm(link_vector)*link_radius^2;
    % B. Make a 3x3 matrix with this mass on the diagonal entries, and
    % zero for all the off-diagonal entries. Call this matrix
    % 'mu_translational'
    mu_translational = [m 0 0; 0 m 0; 0 0 m];

    % C. Construct the moment of inertia matrix for a cylinder rotating
    % around its center, with the axis of the cylinder being the length of
    % the link and corresponding to the first vector in the
    % cylinder-aligned frame. Call this inertia matrix 'mu_rotational'
    Ixx = 0.5*m*link_radius^2;
    Izz = (1/12)*m*(3*link_radius^2 + norm(link_vector)^2);
    Iyy = Izz;
    mu_rotational = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
    % D. Generate an orthonormal basis from the link vector, and save
    % it in a matrix 'b'
    b = orthonormal_basis_from_vector(link_vector);
    % E. Convert both mu_translational and mu_rotational from the
    % cylinder-aligned frame to the link-aligned frame by
    % pre-multiplying them by 'b' and post-multiplying them by
    % 'inv(b)'. Call the resulting matrices 'mu_translational_link' and
    % 'mu_rotational_link'
    mu_translational_link = b*mu_translational*inv(b);
    mu_rotational_link = b*mu_rotational*inv(b);
    % F. Construct the full inertia matrix in the link-aligned
    % frame by taking the translational and rotational mu components as
    % blocks in a block-diagonal matrix. Call this composite matrix
    % 'mu'
    Z = zeros(3,3);
    mu = [mu_translational_link Z; Z mu_rotational_link];




end


    
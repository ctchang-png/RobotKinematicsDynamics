function q = draw_vectors_at_point(p,V,ax)
% Draw the columns of V as arrows based at point p, in axis ax
%
% Inputs:
%
%   p: a 3x1 vector designating the location of the vectors
%   V: a 3xn matrix, each column of which is a 3x1 vector that should be
%       drawn at point p
%   ax: the axis in which to draw the vectors
%
% Output:
%
%   q: a cell array of handles to the quiver objects for the drawn arrows

    % First use hold(ax,'on') so that when we call quiver, it does not
    % delete the plot
    hold(ax, 'on')
    % Now create an empty cell array with one row and as many columns as V
    q = cell(1, size(V, 2));
    % Loop over the columns of V
        
        % Use quiver3 to plot an arrow at p, with vector components taken
        % as the first three rows of the (idx)th column of V. Store the
        % output as the (idx)th element of q
    for idx = 1:numel(q)
        q{idx} = quiver3(ax, p(1), p(2), p(3), V(1,idx), V(2,idx), V(3,idx));
    end
                
                 
    
    % Use hold(ax,'off') to return the axis to its normal behavior
    hold(ax, 'off')
end

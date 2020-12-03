function [ax,f] = create_subaxes(fignum,m,n,p)
% Clear out a specified figure and create a clean set of axes in that
% figure with equal-axis aspect ratio
%
% Input:
%
%   fignum: The number of the figure (or a figure handle) in which to
%       create the axes 
%   m: the number of rows of subplots to create
%   n: the number of columns of subplots to create
%   p: the number of subplots to create (should be less than or equal to
%       m*n)
%
% Outputs:
%
%   ax: A cell array of handles to the created subplot axes
%   f: A handle to the figure that was created

    %%%%%%%
    % Use the 'figure' command to make sure that the figure in the input
    % exists. The output to figure provides a handle to the figure
    f = figure(fignum);
    %%%%%%%
    % Use the 'clf' command with the 'reset' option to clean out any
    % existing contents in the figure. Use the figure handle to make sure
    % that the 'clf' command targets this figure
    clf(f, 'reset')
    
    % Create an empty 1xp cell named ax to hold the axis handles
    ax = cell(1, p);
    %%%%%%%
    % Loop over the number of subplots requested
    for idx = 1:p
        %%%%%%%
        % Use the 'subplot' command to create the (idx)th subplot axis for
        % plotting. Use the 'Parent' option with the figure handle to make
        % sure that the axes are created in that figure. The output of
        % 'subplot' provides a handle to the axis. Store this this handle
        % as 'ax{idx}'
        ax{idx} = subplot(m, n, idx, 'Parent', f);
        %%%%%%
        % Use the 'axis' command with the 'equal' option to make sure that all
        % the axes of the suplot have the same length scale. Use the handle of
        % the axis to make sure that this action applies to the axis you want
        % it to apply to
        axis(ax{idx}, 'equal')

        %%%%%%
        % Use the 'box' command with the 'on' option to have all edges of the
        % plotting axis marked Use the handle of the axis to make sure that
        % this action applies to the axis you want it to apply to
        box(ax{idx}, 'on')
    end

end
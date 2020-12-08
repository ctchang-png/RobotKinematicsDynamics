function wire =  interpolate_wire_points(wire_segments, wire_segment_types, points_per_seg, file_name)
% interpolate_wire_points generates points along the wire specified by
% wire_segments
%
% Given a wire with s segments
% wire_segments: 1 x s cell array, with each element 3 x n array of points
% in the segment.
% wire_segment_types: 1 x s cell array, with each element a character 'l'
% for linear segment, 'x' for constant-x spline, 'y' for constant-y spline
% and 'z' for constant-z spline
% points_per_seg (optional): Number of points to interpolate per segment. 
% Default is 50.
% file_name: string filename to save wire points to. Should be a *.mat 
%
% Returns
% wire: 3 x (s * points_per_seg) matrix of interpolated points along the wire

if (nargin>2)
    n = points_per_seg;
else
    n = 50;
end

wire = []
for i=1:numel(wire_segments)
    seg_points = wire_segments{i};
    switch(wire_segment_types{i})
        case 'l'
            P1 = seg_points(:, 1);
            P2 = seg_points(:, 2);
            t = linspace(0,1,n);
            P = P1*(1-t)+ P2*t;
            wire = [wire P];
        case 'x'
            % yz plane. x is constant, y is dominant axis, z is interpolated.
            xs = ones(1, n) * seg_points(1, 1);
            ys = linspace(seg_points(2, 1), seg_points(2, end), n);
            zs = spline(seg_points(2, :), seg_points(3, :), ys);
            spline_points = [xs; ys; zs];
            wire = [wire spline_points];
        case 'y'
            % xz plane. y is constant, z is dominant, x is interpolated.
            ys = ones(1, n) * seg_points(2, 1);
            zs = linspace(seg_points(3, 1), seg_points(3, end), n);
            xs = spline(seg_points(3, :), seg_points(1, :), zs);
            spline_points = [xs; ys; zs];
            wire = [wire spline_points];
        case 'z'
            % xy plane. z is constant, x is dominant, y is interpolated.
            zs = ones(1, n) * seg_points(3, 1);
            xs = linspace(seg_points(1, 1), seg_points(1, end), n);
            ys = spline(seg_points(1, :), seg_points(2, :), xs);
            spline_points = [xs; ys; zs];
            wire = [wire spline_points];
    end
end

if (nargin>3)
    save(file_name, "wire");
end
end


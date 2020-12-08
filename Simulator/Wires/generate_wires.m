%% WIRE 1:
p0 = [0, 0, 2]';
p1 = [0, 0, 3]';
p2 = [0, 1, 3.5]';
p3 = [0, 2, 3]';
p4 = [1, 2.5, 3]';
p5 = [2, 2, 3]';

s1 = [p0 p1];
s2 = [p1 p2 p3];
s3 = [p3 p4 p5];

wire_segments = {s1, s2, s3};
wire_segment_types = {'l', 'x', 'z'};
n = 60;
filename = "Wires/wire1.mat";

wire = interpolate_wire_points(wire_segments, wire_segment_types, n, filename);
figure(1);
plot3(wire(1,:), wire(2, :), wire(3, :))
axis([-5,5,-5,5,0,5])
%% WIRE 2: 
p7 = [0, 0, 2]';
p0 = [.2, .2, 4]';
p1 = [ 2, 2, 4]';
p2 = [3, 0, 4]';
p3 = [ 0, -4, 4]';
p4 = [ -3, 0, 4]';
p5 = [ -2, 2, 4]';
p6 = [ -.2, -.2, 4]';

s0 = [p7 p0];
s1 = [p0 p1 p2];
s2 = [p2 p3];
s3 = [p3 p4];
s4 = [p4, p5, p6];

wire_segments = {s0, s1, s2, s3, s4};
wire_segment_types = {'l', 'z','l', 'l', 'z'};
n = 60;
filename = "Wires/wire2.mat";

wire = interpolate_wire_points(wire_segments, wire_segment_types, n, filename);
figure(2);
plot3(wire(1,:), wire(2, :), wire(3, :))
axis([-5,5,-5,5,0,5])
%% WIRE 3
wire = [];
% given values
pos = [0 0 ;    % startpoint
       3.5 3.5] ;  % endpoint
nturns = 4 ;    % number of turns (integer value)
% engine
dp = diff(pos,1,1) ;
R = hypot(dp(1), dp(2)) ;
phi0 = atan2(dp(2), dp(1)) ;
phi = linspace(0, nturns*2*pi, 10000) ; % 10000 = resolution
r = linspace(0, R, numel(phi)) ;
x = pos(1,1) + r .* cos(phi + phi0) ;
y = pos(1,2) + r  .* sin(phi + phi0) ;

z = linspace(5,2,numel(phi));
figure(3)
plot3(x,y,z)
axis([-5,5,-5,5,0,5])
spline_points = [x; y; z];
wire = [wire spline_points];
save("Wires/wire3.mat", "wire");

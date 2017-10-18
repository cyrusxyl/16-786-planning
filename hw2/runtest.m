function[numofmoves, caught] = runtest(mapfile, armstart, armgoal, planner_id)

LINKLENGTH_CELLS=10;
envmap = load(mapfile);

close all;

%draw the environment
image(envmap'*255);
hold on;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 
armplan = armplanner(envmap, armstart, armgoal, planner_id);

fprintf(1, 'plan of length %d was found\n', size(armplan,1));

N = 10;
Xi = N*(1:size(armplan,1));
Xo = N:Xi(end);
armplan = interp1(Xi,armplan,Xo);

%draw the plan
midx = size(envmap,2)/2;
x = zeros(length(armstart)+1,1);
x(1) = midx;
y = zeros(length(armstart)+1,1);
for i = 1:size(armplan)
    for j = 1:length(armstart)
        x(j+1) = x(j) + LINKLENGTH_CELLS*cos(armplan(i,j));
        y(j+1) = y(j) + LINKLENGTH_CELLS*sin(armplan(i,j));
    end;
    if mod(i,10) == 1
        plot(x,y, 'r-');
    else
        plot(x,y, 'c-');
    end
    pause(0.1);
end;

%armplan
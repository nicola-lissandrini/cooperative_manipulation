A = diag ([2, 2]);

[xgrid, ygrid] = meshgrid (-5:0.07:5);

obst = [1; 3];
r = 1;
margin = 1;

i = 1;
j = 1;
z = 0 * xgrid;
for x = -5:0.07:5
    for y = -5:0.07:5
        v = [x; y];
        z(i,j) = v' * A * v + costCollide (v, obst, r, margin);
        j = j + 1;
    end
    i = i + 1;
    j = 1;
end

surf (xgrid, ygrid, z)
axis ([0 5 -1 3 0 60])
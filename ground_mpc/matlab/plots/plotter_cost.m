A = diag ([2, 2]);

[xgrid, ygrid] = meshgrid (-1:0.09:5);

obst = [1; 2.5];
r = 0.7;
margin = 2;

i = 1;
j = 1;
z = 0 * xgrid;
for x = -1:0.09:5
    for y = -1:0.09:5
        v = [x; y];
        z(i,j) = v' * A * v + costCollide (v, obst, r, margin);
        z(i,j) = min (z(i,j), 61);
        j = j + 1;
    end
    i = i + 1;
    j = 1;
end


%% Plots

h = figure ('rend','painters','Position',[100, 100, 550, 350], 'PaperPositionMode','auto');
% plots 
co = get (gca,'ColorOrder');



surf (xgrid, ygrid, z)
axis ([-1 5 -1 3 0 60])

view([-20, 30])

xlabel ('$x$','Interpreter','latex');
ylabel ('$y$','Interpreter','latex');
zlabel ('Cost term');

%  l = legend ('$\chi(\bf{x})$','r = 1','m = 3');
%  l.Interpreter = 'latex';
%  l.Location = 'northeast';

grid on
grid minor

pos = get(h,'Position');

set(h,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3)*0.75, pos(4)*0.8])
print(h,'excost_obst.pdf','-dpdf','-r0')
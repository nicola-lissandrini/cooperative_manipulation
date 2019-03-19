%% Plotter 

d = 0:0.1:10;
m = 3;
eps = 0.1;
lambda = -log(eps)/m;
r = 1;
chi = exp (-lambda * (d - r));




%% Plots

h = figure ('rend','painters','Position',[100, 100, 500, 250], 'PaperPositionMode','auto');
% plots 
co = get (gca,'ColorOrder');

plot (d, chi)
hold on
plot ([1,1],[0,2],'--k')
plot ([4,4],[0,2],':k')

xlabel ('Distance  $d = \|\bf{x} - \bf{c}\|$','Interpreter','latex');
ylabel ('$\chi (\bf{x})$','Interpreter','latex');

 l = legend ('$\chi(\bf{x})$','r = 1','m = 3');
 l.Interpreter = 'latex';
 l.Location = 'northeast';

axis ([0 5 0 2])

grid on
grid minor

pos = get(h,'Position');

set(h,'PaperPositionMode','Auto','PaperUnits','Points','PaperSize',[pos(3)*0.75, pos(4)*0.8])
print(h,'chi_Exp.pdf','-dpdf','-r0')
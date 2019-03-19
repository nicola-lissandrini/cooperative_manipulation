data = csvread ('/home/nicola/stateLog');

%% Plot joints

plot (data(:,6))
hold on
grid on 
grid minor
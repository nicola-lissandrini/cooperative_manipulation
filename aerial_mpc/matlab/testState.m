

aerialParams = csvread ('../config/parameters.csv');

direc = @(q, params)[q(3)+params(3).*cos(q(6))-params(4).*sin(q(6))+cos(q(6)).*(params(6).*cos(q(1))+params(1).*sin(q(1)))-params(2).*cos(q(6)).*sin(q(1)).*(1.0./2.0)+sqrt(3.0).*params(2).*cos(q(6)).*cos(q(1)).*(1.0./2.0);q(4)+params(4).*cos(q(6))+params(3).*sin(q(6))+sin(q(6)).*(params(6).*cos(q(1))+params(1).*sin(q(1)))-params(2).*sin(q(6)).*sin(q(1)).*(1.0./2.0)+sqrt(3.0).*params(2).*cos(q(1)).*sin(q(6)).*(1.0./2.0);params(5)+q(5)-params(1).*cos(q(1))+params(2).*cos(q(1)).*(1.0./2.0)+params(6).*sin(q(1))+sqrt(3.0).*params(2).*sin(q(1)).*(1.0./2.0)];

p_o = direc (q, aerialParams);

%% from log
close all
stateLog = csvread ('/home/nicola/stateLogAerial');
q = stateLog (:,4:end);
sz = size(stateLog);

clear estim

for i=1:sz(1)
    estim(i,:) = direc (q(i,:), aerialParams)';
    
end

plot3 (stateLog(:,1),stateLog(:,2), stateLog(:,3))
hold on
plot3 (estim(:,1), estim(:,2), estim(:,3));
plot3 (stateLog (:,6),stateLog (:,7),stateLog (:,8));


axis equal
grid on
grid minor


groundParams = csvread ('../config/parameters.csv');
state = [-0.027015593286887726, -0.009280480546768352, 0.4058632486138665, 0.012370102055404564, -0.0023284837240095725, -0.9999207762311401, -0.0008901481594517499, 0.9999968667783226, -0.002339673009644465, 0.9999230911482108, 0.0009190196324652919, 0.012367990601631407, 0.5490212099352814, 1.9635840265244822, 0.0009824752260723102, -0.3804312096858702, -0.33968264008516397, 0.018508268611259728, -0.5481493835247582];
q = state(13:end);
p_o = getGroundKinematics (q, groundParams);

%% from log
close all
stateLog = csvread ('/home/nicola/stateLog');
q = stateLog (:,4:end);
sz = size(stateLog);

for i=1:sz(1)
    estim(i,:) = getGroundKinematics (q(i,:), groundParams)';
end

axis vis3d
plot3 (stateLog(:,1),stateLog(:,2), stateLog(:,3))
hold on
plot3 (estim(:,1), estim(:,2), estim(:,3));


grid on
grid minor
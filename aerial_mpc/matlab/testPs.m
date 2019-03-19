
 
getP1 = @(q, params) [q(3)+params(3).*cos(q(6))-params(4).*sin(q(6))+params(1).*cos(q(6)).*sin(q(1));q(4)+params(4).*cos(q(6))+params(3).*sin(q(6))+params(1).*sin(q(6)).*sin(q(1));params(5)+q(5)-params(1).*cos(q(1))];

q = [-pi/2, pi/2, 0.016738657528454468, -0.010929570078473453, 0.251422237132109, -0.00851756869197796];

aerialParams = csvread('../config/parameters.csv');
getAerialKinematics (q, aerialParams)
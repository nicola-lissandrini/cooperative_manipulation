function f = costCollide (p, obst, r, margin)
    epsilon = 0.01;
    d = norm (p - obst);
    lambda =  -log (epsilon) / margin;
    f = 100 * exp (-lambda*(d - r));
end
function f = collision (point, center, a, b, c, delta)
    A = diag ([1/(a + delta)^2, 1/(b + delta)^2, 1/(c + delta)^2]);
    f = (point - center)'  * (point - center) - delta;
end
function R = getAerialAttitude (q)
    R = reshape([sin(q(1)+q(2)).*cos(q(6)),sin(q(1)+q(2)).*sin(q(6)),-cos(q(1)+q(2)),cos(q(1)+q(2)).*cos(q(6)),cos(q(1)+q(2)).*sin(q(6)),sin(q(1)+q(2)),sin(q(6)),-cos(q(6)),0.0],[3,3]);
end
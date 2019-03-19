function R = getGroundAttitude (q)
    R = reshape([cos(q(2)+q(3)+q(4)).*cos(q(7)+q(1)),cos(q(2)+q(3)+q(4)).*sin(q(7)+q(1)),-sin(q(2)+q(3)+q(4)),sin(q(2)+q(3)+q(4)).*cos(q(7)+q(1)),sin(q(2)+q(3)+q(4)).*sin(q(7)+q(1)),cos(q(2)+q(3)+q(4)),sin(q(7)+q(1)),-cos(q(7)+q(1)),0.0],[3,3])
end

r = 0.2;
margin = 0.2;
i = 1;
func=zeros(100,1)
for d = 0:0.01:1
    func(i) = costCollide (d, 0, r, margin);
    i = i + 1;
end
plot (0:0.01:1, func)
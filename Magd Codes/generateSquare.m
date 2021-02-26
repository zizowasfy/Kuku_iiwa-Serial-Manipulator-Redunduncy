function [x,y] = generateSquare(xs, ys, dim, n)

x1 = linspace(xs,xs+dim,n);
x2 = (xs+dim) * ones(1,n);
x3 = linspace(xs+dim,xs,n);
x4 = xs * ones(1,n);

y1 = ys * ones(1,n);
y2 = linspace(ys,ys+dim,n);
y3 = (ys+dim) * ones(1,n);
y4 = linspace(ys+dim,ys,n);

x = [x1 x2 x3 x4];
y = [y1 y2 y3 y4];

end 
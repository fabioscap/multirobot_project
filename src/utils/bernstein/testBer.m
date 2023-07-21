
int = [0,1];
p = [1 2 3 2 1 0 -1 -2 -3 -2 -1 0 ];
pd = diffBernstein(p, int);
pdd = diffBernstein(pd, int);

sq = sqNormBernstein(pdd);
t = linspace(int(1), int(end) , 100);

values = zeros(size(t));
eff = zeros(size(t));
for i=1:length(values)
    values(i) = deCasteljau(p, t(i), int);
    eff(i) = sqrt(deCasteljau(sq,t(i),int));
end

f = plot(t, values, "r"); hold on;
plot(t, eff, "b")

trapz(t, eff)
intBernstein(eff, int)
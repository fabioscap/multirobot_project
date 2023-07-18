function ab = eq8(n_samples, plot)
arguments
    n_samples=500; % how many samples for regression
    plot=false
end
%EQ8 fit two parameters in order to approximate a complex function
% to an ellipsoid
% equation 8 in Avalanche Vitcim Search ...

theta = linspace(-pi,pi,n_samples);

gt = target(theta); % real function

ab = nlinfit(theta,gt,@model,[1;1]);
if plot 
    theta = linspace(-pi,pi,100);
    gt = target(theta); % real function
    polarplot(theta,gt,"b"); hold on;
    pred = model(ab, theta);
    polarplot(theta,pred,"r");
end

end

function y = target(theta)
    y = (1+3*cos(theta).^2).^(-  1/3);
end 

function y = model(c, theta)
    y = (cos(theta)/c(1)).^2 + (sin(theta)/c(2)).^2;
end
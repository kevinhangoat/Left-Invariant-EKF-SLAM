function filter = filter_initialization(sys, initialStateMean,initialStateCov, filter_name)
switch filter_name
    %only InEKF 
    case "InEKF"
        init.mu = eye(5);
        init.mu(1,5) = initialStateMean(1); % position x
        init.mu(2,5) = initialStateMean(2); % position y
        init.mu(3,5) = initialStateMean(3); % position z
        init.Sigma = initialStateCov;
        filter = InEKF(sys, init);
end
end
function  T = RotD(around,q)   % use this function if working with Degrees
%     T = eye(4); 
    if around == "z" || around == "Z"          % Rotation around z
        T =          [cosd(q) -sind(q) 0 0;
                      sind(q) cosd(q)  0 0;
                      0         0      1 0;
                      0         0      0 1];
                  
    elseif around == "y" || around == "Y"      % Rotation around y
        T =          [cosd(q)   0   sind(q) 0;
                      0         1     0     0;
                      -sind(q)  0   cosd(q) 0;
                      0         0     0     1];
                  
    elseif around == "x" || around == "X"      % Rotation around x
        T =          [1      0         0       0;
                      0     cosd(q)   -sind(q) 0;
                      0     sind(q)   cosd(q)  0;
                      0      0         0       1];
                  
    elseif around == "dz" || around == "dZ"    % Rotation around z
        T =          [-sind(q) -cosd(q)  0 0;
                      cosd(q)  -sind(q)  0 0;
                      0         0        0 0;
                      0         0        0 0];
                  
    elseif around == "dy" || around == "dY"    % Rotation around y
    T =          [-sind(q)   0   cosd(q) 0;
                  0         0     0      0;
                  -cosd(q)  0   -sind(q) 0;
                  0         0     0      0];
              
    elseif around == "dx" || around == "dX"     % Rotation around x
        T =          [0      0         0        0;
                      0     -sind(q)   -cosd(q) 0;
                      0     cosd(q)   -sind(q)  0;
                      0      0         0        0];
    else 
        disp(' Not Valid Rotation ')
        
    end
end
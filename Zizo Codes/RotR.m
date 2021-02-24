function  T = RotR(around,q)   % use this function if working with Radians
    if around == "z" || around == "Z"        % Rotation around z
        T =          [cos(q) -sin(q)   0 0;
                      sin(q) cos(q)    0 0;
                      0         0      1 0;
                      0         0      0 1];
    elseif around == "y" || around == "Y"    % Rotation around y
        
        T =          [cos(q)   0   sin(q)   0;
                      0         1     0     0;
                      -sin(q)  0   cos(q)   0;
                      0         0     0     1];
                  
    elseif around == "x" || around == "X"     % Rotation around x
        T =          [1      0         0       0;
                      0     cos(q)   -sin(q)   0;
                      0     sin(q)   cos(q)    0;
                      0      0         0       1];
                  
    elseif around == "dz" || around == "dZ"    % Rotation around z
        T =          [-sin(q) -cos(q)  0 0;
                      cos(q)  -sin(q)  0 0;
                      0         0      0 0;
                      0         0      0 0];
                  
    elseif around == "dy" || around == "dY"    % Rotation around y
        T =          [-sin(q)  0   cos(q)  0;
                     0         0     0     0;
                     -cos(q)   0   -sin(q) 0;
                     0         0     0     0];
              
    elseif around == "dx" || around == "dX"     % Rotation around x
        T =          [0      0         0       0;
                      0     -sin(q)   -cos(q)  0;
                      0     cos(q)   -sin(q)   0;
                      0      0         0       0];
    else 
        disp(' Not Valid Rotation ')
        
    end
end
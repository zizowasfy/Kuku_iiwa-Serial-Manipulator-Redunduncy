function T = Transl(along,q)
    if along == "z" || along == "Z"   % Translation along z-axis
        T = [1  0   0   0;
             0  1   0   0;
             0  0   1   q;
             0  0   0   1];
    elseif along == "y" || along == "Y"  % Translation along y-axis
        T = [1  0   0   0;
             0  1   0   q;
             0  0   1   0;
             0  0   0   1];
    elseif along == "x" || along == "X"   % Translation along x-axis
        T = [1  0   0   q;
             0  1   0   0;
             0  0   1   0;
             0  0   0   1];
         
    elseif along == "dz" || along == "dZ"   % Translation along z-axis
        T = [0  0   0   0;
             0  0   0   0;
             0  0   0   1;
             0  0   0   0];
    elseif along == "dy" || along == "dY"  % Translation along y-axis
        T = [0  0   0   0;
             0  0   0   1;
             0  0   0   0;
             0  0   0   0];
    elseif along == "dx" || along == "dX"   % Translation along x-axis
        T = [0  0   0   1;
             0  0   0   0;
             0  0   0   0;
             0  0   0   0];
    else 
        disp(' Not Valid Translation ')
        
    end 
end
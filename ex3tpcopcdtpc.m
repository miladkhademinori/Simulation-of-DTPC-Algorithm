% Milad Khademi Nori 95123012
% Exercise 3
% Dynamic Target-SINR Tracking Power Control

% Cell coverage area = 100m * 100m
% Background noise power DELTA2= e-10W
% OPC and DTPC constant Ettai = e-3
% Path gain hi = 0.09d^(-3)
% Simulate the system under the above conditions. The users should be uniformly distributed in the cell.
% X = rand (1, Nu) * 100;  %1:Length/Nu:Length;
% Y = rand (1, Nu) * 100;  %1:Length/Nu:Length;
% XYVector = [X;Y]';
XYVector = [24.0707 25.4790 ; 67.6122 22.4040 ; 28.9065 66.7833 ; 67.1808 84.4392 ; 69.5140 34.4462 ; 6.7993 78.0520];

% 1. Plot SINR and power of each user versus the number of iterations (a measure of time).

Nu = 6;
R = zeros(Nu ,1);
Rth = zeros(Nu ,1);
whichUpdateFunction = 'DTPC'; % TPC or OPC or DTPC
TargetSINRGama = 0.2 * ones( Nu , 1);
Length = 100;
Width = 100;
DeltaPower2 = 1e-10 * ones( Nu , 1);
Ni = 0.001;
NiVec = Ni * ones( Nu , 1);
PositionOfBS = [ 0 , 0 ];
DistanceFromBS = pdist2(XYVector,PositionOfBS,'euclidean');
PathGainh = 0.09*DistanceFromBS.^(-3);
MaxOfIteration = 2000;
PowerVector = zeros( Nu , 1);
GamaVector = zeros( Nu , 1);
PowerVectorHistory = [];
GamaVectorHistory = [];
PreGamaVector = zeros( Nu , 1);
PostGamaVector = zeros( Nu , 1);
PrePowerVector = zeros( Nu , 1);
PostPowerVector = zeros( Nu , 1);

for k = 1:MaxOfIteration
    PowerVectorHistory = [PowerVectorHistory;PowerVector'];
    GamaVectorHistory = [GamaVectorHistory;GamaVector'];
    PreGamaVector = PostGamaVector;
    PrePowerVector = PostPowerVector;
    for i = 1:Nu
        if (strcmp(whichUpdateFunction,'TPC'))
        
        PowerVector( i , 1 ) =  TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
        elseif (strcmp(whichUpdateFunction,'OPC'))
            
        PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
                
        elseif (strcmp(whichUpdateFunction,'DTPC'))
            
            R (i ,1) = ( PowerVector' * PathGainh - PowerVector( i , 1 ) * PathGainh( i , 1 ) + ...
                DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));
            Rth (i ,1) = sqrt(NiVec( i , 1)/TargetSINRGama( i , 1 )); 
            if (R (i ,1) >= Rth (i ,1))   %
                PowerVector( i , 1 ) =  (TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) ))/(PathGainh( i , 1 ));
                 
             else
                PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));
                 
            end
                GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
            
        end
    end
    PostGamaVector = GamaVector;
    PostPowerVector = PowerVector;
    if (abs(PostGamaVector - PreGamaVector) <= 0.000000000001) 
        if (abs(PostPowerVector - PrePowerVector) <= 0.000000000001)
        break;
        end
    end

end

figure
subplot(2,1,1);
plot(GamaVectorHistory);
title('GamaVectorHistory');
subplot(2,1,2);
plot(PowerVectorHistory);
title('PowerVectorHistory');
%% QUESTION7
% Milad Khademi Nori 95123012
% Exercise 3
% Dynamic Target-SINR Tracking Power Control

% Cell coverage area = 100m * 100m
% Background noise power DELTA2= e-10W
% OPC and DTPC constant Ettai = e-3
% Path gain hi = 0.09d^(-3)
% Simulate the system under the above conditions. The users should be uniformly distributed in the cell.
% X = rand (1, Nu) * 100;  %1:Length/Nu:Length;
% Y = rand (1, Nu) * 100;  %1:Length/Nu:Length;
% XYVector = [X;Y]';
XYVector = [24.0707 25.4790 ; 67.6122 22.4040 ; 28.9065 66.7833 ; 67.1808 84.4392 ; 69.5140 34.4462 ; 6.7993 78.0520];

% 1. Plot SINR and power of each user versus the number of iterations (a measure of time).

Nu = 6;
R = zeros(Nu ,1);
Rth = zeros(Nu ,1);
whichUpdateFunction = 'DTPC'; % TPC or OPC or DTPC
TargetSINRGama = [1.5;0.5;0.75;1.25;1;1.5];
Length = 100;
Width = 100;
DeltaPower2 = 1e-10 * ones( Nu , 1);
Ni = 0.001;
NiVec = Ni * ones( Nu , 1);
PositionOfBS = [ 0 , 0 ];
DistanceFromBS = pdist2(XYVector,PositionOfBS,'euclidean');
PathGainh = 0.09*DistanceFromBS.^(-3);
MaxOfIteration = 10000;
PowerVector = zeros( Nu , 1);
GamaVector = zeros( Nu , 1);
PowerVectorHistory = [];
GamaVectorHistory = [];
PreGamaVector = zeros( Nu , 1);
PostGamaVector = zeros( Nu , 1);
PrePowerVector = zeros( Nu , 1);
PostPowerVector = zeros( Nu , 1);

for k = 1:MaxOfIteration
    PowerVectorHistory = [PowerVectorHistory;PowerVector'];
    GamaVectorHistory = [GamaVectorHistory;GamaVector'];
    PreGamaVector = PostGamaVector;
    PrePowerVector = PostPowerVector;
    if k >= 2000
        TargetSINRGama( 1 , 1 ) = 0;
        NiVec( 1 , 1) = 0;
    end
    if k >= 4000
        TargetSINRGama( 2 , 1 ) = 0;
        NiVec( 2 , 1) = 0;
        TargetSINRGama( 6 , 1 ) = 0;
        NiVec( 6 , 1) = 0;
    end
    if k >= 6000
        TargetSINRGama( 3 , 1 ) = 0;
        NiVec( 3 , 1) = 0;
    end
    if k >= 8000
        TargetSINRGama( 4 , 1 ) = 0;
        NiVec( 4 , 1) = 0;
    end
    if k >= 10000
        TargetSINRGama( 5 , 1 ) = 0;
        NiVec( 5 , 1) = 0;
    end
    for i = 1:Nu
        if (strcmp(whichUpdateFunction,'TPC'))
        
        PowerVector( i , 1 ) =  TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
        elseif (strcmp(whichUpdateFunction,'OPC'))
            
        PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
                
        elseif (strcmp(whichUpdateFunction,'DTPC'))
            
            R (i ,1) = ( PowerVector' * PathGainh - PowerVector( i , 1 ) * PathGainh( i , 1 ) + ...
                DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));
            Rth (i ,1) = sqrt(NiVec( i , 1)/TargetSINRGama( i , 1 )); 
            if (R (i ,1) >= Rth (i ,1))   %
                PowerVector( i , 1 ) =  (TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) ))/(PathGainh( i , 1 ));
                 
             else
                PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));
                 
            end
                GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
            
        end
        if PowerVector( i , 1 ) >= 100000000
           PowerVector( i , 1 ) = 100000000; 
                GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
        end
    end
    PostGamaVector = GamaVector;
    PostPowerVector = PowerVector;
%     if (abs(PostGamaVector - PreGamaVector) <= 0.000000000001) 
%         if (abs(PostPowerVector - PrePowerVector) <= 0.000000000001)
%         break;
%         end
%     end

end

figure
subplot(2,1,1);
plot(GamaVectorHistory);
title('GamaVectorHistory');
subplot(2,1,2);
plot(PowerVectorHistory);
title('PowerVectorHistory');

%% Part 2  Constrained
XYVector = [24.0707 25.4790 ; 67.6122 22.4040 ; 28.9065 66.7833 ; 67.1808 84.4392 ; 69.5140 34.4462 ; 6.7993 78.0520];

% 1. Plot SINR and power of each user versus the number of iterations (a measure of time).

Nu = 6;
R = zeros(Nu ,1);
Rth = zeros(Nu ,1);
whichUpdateFunction = 'DTPC'; % TPC or OPC or DTPC
TargetSINRGama = 0.2 * ones( Nu , 1);
Length = 100;
Width = 100;
DeltaPower2 = 1e-10 * ones( Nu , 1);
Ni = 0.001;
NiVec = Ni * ones( Nu , 1);
PositionOfBS = [ 0 , 0 ];
DistanceFromBS = pdist2(XYVector,PositionOfBS,'euclidean');
PathGainh = 0.09*DistanceFromBS.^(-3);
MaxOfIteration = 2000;
PowerVector = zeros( Nu , 1);
GamaVector = zeros( Nu , 1);
PowerVectorHistory = [];
GamaVectorHistory = [];
PreGamaVector = zeros( Nu , 1);
PostGamaVector = zeros( Nu , 1);
PrePowerVector = zeros( Nu , 1);
PostPowerVector = zeros( Nu , 1);

for k = 1:MaxOfIteration
    PowerVectorHistory = [PowerVectorHistory;PowerVector'];
    GamaVectorHistory = [GamaVectorHistory;GamaVector'];
    PreGamaVector = PostGamaVector;
    PrePowerVector = PostPowerVector;
    for i = 1:Nu
        if (strcmp(whichUpdateFunction,'TPC'))
        
        PowerVector( i , 1 ) =  TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
        elseif (strcmp(whichUpdateFunction,'OPC'))
            
        PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));

        GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
        - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
                
        elseif (strcmp(whichUpdateFunction,'DTPC'))
            
            R (i ,1) = ( PowerVector' * PathGainh - PowerVector( i , 1 ) * PathGainh( i , 1 ) + ...
                DeltaPower2( i , 1 ) )/(PathGainh( i , 1 ));
            Rth (i ,1) = sqrt(NiVec( i , 1)/TargetSINRGama( i , 1 )); 
            if (R (i ,1) >= Rth (i ,1))   %
                PowerVector( i , 1 ) =  (TargetSINRGama( i , 1 ) * ( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) ))/(PathGainh( i , 1 ));
                 
             else
                PowerVector( i , 1 ) =  NiVec( i , 1) / (( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) )/(PathGainh( i , 1 )));
                 
            end
                GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
            
        end
         if PowerVector( i , 1 ) >= 0.001
           PowerVector( i , 1 ) = 0.001; 
                GamaVector( i , 1 ) = ( PowerVector( i , 1 ) * PathGainh( i , 1 ))/( PowerVector' * PathGainh ...
                - PowerVector( i , 1 ) * PathGainh( i , 1 ) + DeltaPower2( i , 1 ) );
        end
    end
    PostGamaVector = GamaVector;
    PostPowerVector = PowerVector;
    if (abs(PostGamaVector - PreGamaVector) <= 0.000000000001) 
        if (abs(PostPowerVector - PrePowerVector) <= 0.000000000001)
        break;
        end
    end

end

figure
subplot(2,1,1);
plot(GamaVectorHistory);
title('GamaVectorHistory');
subplot(2,1,2);
plot(PowerVectorHistory);
title('PowerVectorHistory');

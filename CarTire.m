classdef CarTire < handle
    
    properties
        MaxForwardAcceleration  % Max non turning acceleration on GG curve
        MaxBrakingAcceleration
        ForwardAccelerationMap
        BrakingAccelerationMap
        RegenAccelerationMap
        MaxLateralAcceleration % Max turning acceleration on GG curve
        LateralAccelerationMap % Max Lateral acceleration for at different radii
        RollingResistance % Constant rolling resistance coefficient
        SpringRate
        J % Rotational inertia (lbf in^2)
        Radius % Effective radius (in)
        RawTireModel
        MaxPossibleAcceleration
        GripScalingFactor
        Name = 'Tire property';
    end
    
    methods
        function T = CarTire(TM,K,R,Resistance,J,Gripfactor)
            % CarTire Constructor method
            %
            % This method constructs an object of type CarTire.  To define
            % an object of this class, input a maximum forward
            % acceleration, maximum lateral acceleration, a radius, a
            % rolling resistance coefficient, a weight, a center of
            % gravity, a moment of inertia about the CG, and a rotational
            % moment of inertia about the center of the tire.
            %
            % INPUTS
            % Name          Type          Units   Description
            %**************************************************************
            % MaxFA         float         G's     Maximum forward or
            %                                     reverse acceleration of
            %                                     the tire.
            %
            % R             float         in      Effective tire radius
            %
            % Resistance    float         N/A     Rolling resistance
            %                                     coefficient.
            %
            % CG            1x3 Array     in      Center of gravity
            %                                     location on vehicle
            %
            % J             float         lb in^2 Mass moment of inertia
            %                                     acted on by wheel torque
            %
            % OUTPUTS
            % Name          Type          Units   Description
            %**************************************************************
            % T             CarTire       N/A     CarTire Object
            %
            % VARIABLES
            % Name          Type          Units   Description
            %**************************************************************
            % NONE
            %
            % FUNCTIONS
            % Name          Location         Description
            %**************************************************************
            % NONE
            
            % Assigns values to tire object properties
            T.SpringRate = K;
            T.Radius = R;
            T.RollingResistance = Resistance;
            T.J = J;
            T.RawTireModel = TM;
            T.MaxPossibleAcceleration = 3.0;
            T.GripScalingFactor = Gripfactor;
        end
        function [F, S] = TireModel(T, Fz, Mode)
            [F, S] = T.RawTireModel(Fz, Mode);
            F = F * T.GripScalingFactor;
        end
        function CalculateLateralGMap(T,CarObject,TrackObject)
            TrackCornerRadii = TrackObject.CornerRadii();
            TrackCornerRadii = unique(TrackCornerRadii);
            T.LateralAccelerationMap = struct('accelerations', arrayfun(@(radius)(LateralGCalculator(T,CarObject,'',radius)), TrackCornerRadii),'radii', TrackCornerRadii);
            T.MaxLateralAcceleration = max(T.LateralAccelerationMap.accelerations);
        end
        function lateralG = LateralGCalculator(T,CarObject,balance,Radius)
            Ws = CarObject.SprungWeight;
            Wfus = CarObject.Suspension.UnsprungWeight(1);
            Wrus = CarObject.Suspension.UnsprungWeight(2);
            Tf = CarObject.TrackWidth(1);
            Tr = CarObject.TrackWidth(2);
            hfus = CarObject.Suspension.UnsprungHeight(1);
            hrus = CarObject.Suspension.UnsprungHeight(2);
            hfrc = CarObject.Suspension.RollCenters(1);
            hrrc = CarObject.Suspension.RollCenters(2);
            hCG = CarObject.CG(3) - (hfrc + hrrc)/2;
            b = CarObject.CG(1) / CarObject.Wheelbase;
            a = 1 - b;
            FR = [ a b ];
            
            K1F = CarObject.Suspension.LinearSpring(1);
            K1R = CarObject.Suspension.LinearSpring(2);
            KarbF = CarObject.Suspension.ARB(1);
            KarbR = CarObject.Suspension.ARB(2);
            K2  = T.SpringRate;
            
            Kf = ((1/(K1F*Tf^2/2 + KarbF) + 2/(K2*Tf^2))^-1);
            Kr = ((1/(K1R*Tr^2/2 + KarbR) + 2/(K2*Tr^2))^-1);
            
            Kf_initial = Kf;
            Kr_initial = Kr;
            
            Gs = (0:0.01:T.MaxPossibleAcceleration)';
            Velocity = sqrt(Gs * 32.2 * 12 * Radius); %in/s
            Fz_aero_delta = CarObject.CalculateAeroEffects(Velocity);
            
            while true
                Fz = LateralWeightTransfer( Gs,Ws,Wfus,Wrus,FR,Tf,Tr,Kf,Kr,hCG,hfus,hrus,hfrc,hrrc ) + Fz_aero_delta;
                
                % Set wheel forces less than zero to zero
                I = Fz < 0;
                Fz(I) = 1;
                
                [Fy, SA] = T.TireModel(Fz,'Lateral');
                
                Fy = Fy .* cosd(SA);  % Find Lateral Force Eff.
                
                % Remove any NaN wheel forces.
                NaNI = isnan(Fy);
                Fy(NaNI) = 0;
                
                FyFront = Fy(:,1) + Fy(:,2);
                FyRear  = Fy(:,3) + Fy(:,4);
                
                W = Ws + Wrus + Wfus;
                Wf = W*FR(1);
                Wr = W*FR(2);
                
                FrontGs = FyFront/Wf;
                RearGs  = FyRear/Wr;
                
                OutGs = zeros(length(FrontGs),1);
                
                I = FrontGs > RearGs;
                OutGs(I) = RearGs(I);
                I = RearGs >= FrontGs;
                OutGs(I) = FrontGs(I);
                
                Difference = OutGs - Gs;
                I1 = find(Difference >= 0,1,'last');
                I2 = find(Difference < 0, 1,'first');
                
                Diff1 = abs(Difference(I1));
                Diff2 = abs(Difference(I2));
                
                if Diff1 > Diff2
                    I = I2;
                else
                    I = I1;
                end
                break
            end
            %             change these
            %             if Fz(171,1) <= 0
            %                 disp('Warning, car fails tilt test')
            %             elseif Fz(I,1) <= 0
            %                 disp('Warning, car flips before max lateral acceleration acheived')
            %             end
            lateralG = OutGs(I);
            CarObject.Suspension.LinearSpring(1) = CarObject.Suspension.LinearSpring(1) - (Kf_initial - Kf);
            CarObject.Suspension.LinearSpring(2) = CarObject.Suspension.LinearSpring(2) - (Kr_initial - Kr);
            
        end
        function CalculateLongitudinalGMap(T, CarObject)
            Velocities = 0:100:3600;
            [ForwardGs, BrakingGs, RegenGs] = arrayfun(@(velocity)(LongitudinalGCalculator(T, CarObject, velocity)), Velocities);
            T.ForwardAccelerationMap = struct('accelerations', ForwardGs, 'velocities', Velocities);
            T.BrakingAccelerationMap = struct('accelerations', BrakingGs, 'velocities', Velocities);
            T.RegenAccelerationMap = struct('accelerations', RegenGs, 'velocities', Velocities);
            
            T.MaxForwardAcceleration = max(T.ForwardAccelerationMap.accelerations);
            T.MaxBrakingAcceleration = max(T.BrakingAccelerationMap.accelerations);
        end
        function [forwardG, brakingG, regenG] = LongitudinalGCalculator(T,CarObject, Velocity)
            
            Kf = CarObject.Suspension.LinearSpring(1);
            Kr = CarObject.Suspension.LinearSpring(2);
            Kt = T.SpringRate;
            Ws = CarObject.SprungWeight;
            Wfus = CarObject.Suspension.UnsprungWeight(1);
            Wrus = CarObject.Suspension.UnsprungWeight(2);
            hCG = CarObject.CG(3);
            PC = CarObject.Suspension.PitchCenter;
            L = CarObject.Wheelbase;
            b = CarObject.CG(1)/L;
            a = 1 - b;
            FR = [ a b ];
            
            % Acceleration
            Gs = (0:0.01:T.MaxPossibleAcceleration)';
            
            Fz = LongitudinalWeightTransfer( Kf, Kr, Kt, Gs, Ws, Wfus, Wrus, hCG, PC, FR, L );
            Fz_aero_deltas = CarObject.CalculateAeroEffects(Velocity);
            
            Fz(:,1) = Fz(:,1) + Fz_aero_deltas(1);
            Fz(:,2) = Fz(:,2) + Fz_aero_deltas(2);
            Fz(:,3) = Fz(:,3) + Fz_aero_deltas(3);
            Fz(:,4) = Fz(:,4) + Fz_aero_deltas(4);
            
            [Fx, ~] = T.TireModel(Fz,'Longitudinal');
            
            FxRear  = Fx(:,3) + Fx(:,4);
            
            W = Ws + Wrus + Wfus;
            RearGs = FxRear/W;
            
            Difference = RearGs - Gs;
            I1 = find(Difference >= 0,1,'last');
            I2 = find(Difference < 0, 1,'first');
            
            Diff1 = abs(Difference(I1));
            Diff2 = abs(Difference(I2));
            
            if Diff1 > Diff2
                I = I2;
            else
                I = I1;
            end
            
            forwardG = RearGs(I);
            
            % Braking
            Gs = -(0:0.01:T.MaxPossibleAcceleration)';
            
            Fz = LongitudinalWeightTransfer( Kf, Kr, Kt, Gs, Ws, Wfus, Wrus, hCG, PC, FR, L );
            
            I = find(Fz(:,3:4) < 0);
            
            if I
                Fz = Fz(1:I(1)-1,:);
                Gs = Gs(1:I(1)-1);
            end
            
            [Fx, ~] = T.TireModel(Fz,'Longitudinal');
            
            FxTotal = Fx(:,1) + Fx(:,2) + Fx(:,3) + Fx(:,4);
            
            OutGs = -FxTotal/W;
            
            Difference = OutGs - Gs;
            I1 = find(Difference >= 0,1,'last');
            I2 = find(Difference < 0, 1,'first');
            
            Diff1 = abs(Difference(I1));
            Diff2 = abs(Difference(I2));
            
            if Diff1 > Diff2
                I = I2;
            else
                I = I1;
            end
            
            if I
                
                brakingG = OutGs(I);
                
            else
                disp('Warning, car flips before brake lockup')
                brakingG = OutGs(end);
                
            end
            
            
            % Regen
            Gs = -(0:0.01:T.MaxPossibleAcceleration)';
            
            Fz = LongitudinalWeightTransfer( Kf, Kr, Kt, Gs, Ws, Wfus, Wrus, hCG, PC, FR, L );
            Fz_aero_deltas = CarObject.CalculateAeroEffects(Velocity);
            
            Fz(:,1) = Fz(:,1) + Fz_aero_deltas(1);
            Fz(:,2) = Fz(:,2) + Fz_aero_deltas(2);
            Fz(:,3) = Fz(:,3) + Fz_aero_deltas(3);
            Fz(:,4) = Fz(:,4) + Fz_aero_deltas(4);
            
            [Fx, ~] = T.TireModel(Fz,'Longitudinal');
            
            FxRear  = Fx(:,3) + Fx(:,4);
            
            W = Ws + Wrus + Wfus;
            RearGs = -FxRear/W;
            
            Difference = RearGs - Gs;
            I1 = find(Difference >= 0,1,'last');
            I2 = find(Difference < 0, 1,'first');
            
            Diff1 = abs(Difference(I1));
            Diff2 = abs(Difference(I2));
            
            if Diff1 > Diff2
                I = I2;
            else
                I = I1;
            end
            
            regenG = RearGs(I);
            
        end
        function LongA = GGCurve(T,LateralA,BrakeThrottle, Velocity, BrakingMode)
            % CarTire GGCurve Method
            %
            % This method returns an array of possible longitudinal
            % accelerations that are possible from a given array of lateral
            % accelerations.  Currently assumes GG curve is symmetric
            % ellipse.
            %
            % INPUTS
            % Name          Type          Units   Description
            %**************************************************************
            % T             TireObject    N/A     Tire Object for the given
            %                                     GG curve
            %
            % LateralA      Nx1 array     G's     Lateral Acceleration for
            %                                     which a maximum forward/
            %                                     backward acceleration is
            %                                     desired from the car tire
            % OUTPUTS
            % Name          Type          Units   Description
            %**************************************************************
            % LongA         Nx1 array     G's     Maximum longitudinal
            %                                     acceleration for given
            %                                     lateral acceleration
            %
            % VARIABLES
            % Name          Type          Units   Description
            %**************************************************************
            % NONE
            %
            % FUNCTIONS
            % Name          Location         Description
            %**************************************************************
            % NONE
            
            radii = ((Velocity ./ 12).^2)./LateralA; % Will result in NaN for velocities/Lat A's of zero.
            maxLateralAs = interp1(T.LateralAccelerationMap.radii, T.LateralAccelerationMap.accelerations, radii, 'linear');
            
            if strcmp(BrakeThrottle,'Throttle')
                maxForwardA = interp1(T.ForwardAccelerationMap.velocities, T.ForwardAccelerationMap.accelerations, Velocity, 'linear');
                InsideSqrt = 1-(LateralA./maxLateralAs).^2;
                OverMax = InsideSqrt < 0;
                
                LongA = maxForwardA.*sqrt(InsideSqrt);
                LongA(OverMax) = 0.01;
                I = isnan(LongA);
                LongA(I) = maxForwardA(I);
            elseif strcmp(BrakeThrottle,'Brake')
                if strcmp(BrakingMode, 'Hydraulic')
                    maxBrakeA = interp1(T.BrakingAccelerationMap.velocities, T.BrakingAccelerationMap.accelerations, Velocity, 'linear');
                elseif strcmp(BrakingMode, 'Regen')
                    maxBrakeA = interp1(T.RegenAccelerationMap.velocities, T.RegenAccelerationMap.accelerations, Velocity, 'linear');
                end
                
                InsideSqrt = 1-(LateralA./maxLateralAs).^2;
                OverMax = InsideSqrt < 0;
                
                LongA = maxBrakeA.*sqrt(InsideSqrt);
                LongA(OverMax) = 0.01;
                
                I = isnan(LongA);
                LongA(I) = maxBrakeA(I);
                
                LongA = abs(LongA);
            end
        end
    end
end


function [ NormalLoad ] = LateralWeightTransfer( Gs,Ws,Wfus,Wrus,FR,Tf,Tr,Kf,Kr,hCG,hfus,hrus,hfrc,hrrc )

            Weight = Ws + Wfus + Wrus;
            WeightF = Weight*FR(1);
            WeightR = Weight*FR(2);
            
            SprungWeightF = WeightF - Wfus;
            SprungWeightR = WeightR - Wrus;
            
            FRSprung = [ SprungWeightF/Ws, SprungWeightR/Ws ];
            
            UnsprungWTF = Wfus*Gs*hfus/Tf;
            UnsprungWTR = Wrus*Gs*hrus/Tr;
            
            GeometricWTF = Ws*Gs*FRSprung(1)*hfrc/Tf;
            GeometricWTR = Ws*Gs*FRSprung(2)*hrrc/Tr;
            
            ElasticWTF = Ws*Gs*hCG*(Kf/(Kf+Kr))/Tf; % hCG is with respect to roll axis
            ElasticWTR = Ws*Gs*hCG*(Kr/(Kf+Kr))/Tr;
            
            StaticWFR = -Weight*FR(1)/2;
            StaticWFL = -Weight*FR(1)/2;
            StaticWRR = -Weight*FR(2)/2;
            StaticWRL = -Weight*FR(2)/2;
            
            NormalWFR = -(StaticWFR + UnsprungWTF + GeometricWTF + ElasticWTF);
            NormalWFL = -(StaticWFL - UnsprungWTF - GeometricWTF - ElasticWTF);
            
            NormalWRR = -(StaticWRR + UnsprungWTR + GeometricWTR + ElasticWTR);
            NormalWRL = -(StaticWRL - UnsprungWTR - GeometricWTR - ElasticWTR);
            
            %               InsideF  OutsideF   InsideR   OutsideR
            NormalLoad = [ NormalWFR NormalWFL NormalWRR NormalWRL ];
            
        end
function [ NormalLoad ] = LongitudinalWeightTransfer( Kf, Kr, Kt, Gs, Ws, Wfus, Wrus, hCG, PC, FR, L )
    
    a = FR(2)*L;
    b = FR(1)*L;
    o = PC(2) - b;
    h = hCG - PC(1);
    
    F = Gs*(Ws + Wfus + Wrus);

    S0 = Kf*Kt^2*a^2 + Kr*Kt^2*b^2 + Kf*Kt^2*o^2 + Kr*Kt^2*o^2 +...
    Kf*Kr*Kt*a^2 + Kf*Kr*Kt*b^2 + 2*Kf*Kr*Kt*o^2 - 2*Kf*Kt^2*a*o +...
    2*Kr*Kt^2*b*o - 2*Kf*Kr*Kt*a*o + 2*Kf*Kr*Kt*b*o;

    Pitch = F*h*(Kf+Kt)*(Kr+Kt)/S0;
    
    DynamicCGh = Pitch*o + hCG;
    
% 
%     WeightTransferF = -F*h*(a-o)*(Kt*2)*(Kf*(Kr + Kt))/S0;
%     WeightTransferR =  F*h*(b+o)*(Kt*2)*(Kr*(Kf + Kt))/S0;



    WeightTransferF = -DynamicCGh./L*(Ws+Wfus+Wrus).*Gs;
    WeightTransferR = DynamicCGh./L*(Ws+Wfus+Wrus).*Gs;
    
    StaticWeightF = Ws*b/(a+b) + Wfus;
    StaticWeightR = Ws*a/(a+b) + Wrus;
    
    FrontLoad = StaticWeightF + WeightTransferF;
    RearLoad = StaticWeightR + WeightTransferR;
    
    NormalLoad = [ FrontLoad/2 FrontLoad/2 RearLoad/2 RearLoad/2 ];
    
end
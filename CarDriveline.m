classdef CarDriveline < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Efficiency
        J
        FinalDriveRatio
        OutputCurve % [AxleRPM, AxleTorque, MotorRPM, MotorTorque, MotorEfficiency, GearNumber]
        OriginalOutputCurve
        CurrentTF
        CurrentRPMLimit
        GearRatios
    end
    methods
        function D = CarDriveline(GearRatios,Efficiency,J,FinalDriveRatio,MotorOutputCurve)
            D.GearRatios = GearRatios;
            D.Efficiency = Efficiency;
            D.J = J;
            D.FinalDriveRatio = FinalDriveRatio;
            D.CurrentTF = NaN;
            D.CurrentRPMLimit = NaN;
            D.CalculateOutputCurve(MotorOutputCurve);
            D.OriginalOutputCurve = D.OutputCurve;
        end
        function CalculateOutputCurve(D, MotorOutputCurve)
                  maxAxleRPM = round(max(MotorOutputCurve(:,1)) / D.FinalDriveRatio);
                D.OutputCurve = zeros(maxAxleRPM + 1, 6);
                D.OutputCurve(:,1) = 0:maxAxleRPM;
                D.OutputCurve(:, 2) = interp1(MotorOutputCurve(:,1), MotorOutputCurve(:,2), D.OutputCurve(:,1) * D.FinalDriveRatio)*D.Efficiency*D.FinalDriveRatio;
                D.OutputCurve(:, 3) = D.OutputCurve(:,1) * D.FinalDriveRatio;
                D.OutputCurve(:, 4) = D.OutputCurve(:,2) / D.FinalDriveRatio;
                D.OutputCurve(:, 5) = interp1(MotorOutputCurve(:,1), MotorOutputCurve(:,3), D.OutputCurve(:,1));
                D.OutputCurve(:, 6) = 1;
%                  D.OutputCurve(:,1) = D.OutputCurve(:,1) / D.FinalDriveRatio;
%                  D.OutputCurve(:,2) = D.OutputCurve(:,2) * D.FinalDriveRatio *  D.Efficiency;
%             if ~isnan(D.CurrentRPMLimit)
%                 D.SetRPMLimit(D.CurrentRPMLimit);
%             end
%             
%             if ~isnan(D.CurrentTF)
%                 D.SetTorqueFactor(D.CurrentTF);
%             end
%             
%         end
%          function SetTorqueFactor(D, TF)
%             if isnan(D.CurrentTF)
%                 D.CurrentTF = 1;
%             end
%             
%             D.OutputCurve(:,2) = D.OutputCurve(:,2) * TF / D.CurrentTF;
%             D.OutputCurve(:,4) = D.OutputCurve(:,4) * TF / D.CurrentTF;
%             D.CurrentTF = TF;
%         end
%         
%         function SetRPMLimit(D, RPMLimit)
%             TF = 1;
%             
%             if ~isnan(D.CurrentTF)
%                 TF = D.CurrentTF;
%             end
%             
%             D.ResetTorqueCurve();
%             D.CurrentRPMLimit = RPMLimit;
%             if RPMLimit < size(D.OutputCurve,1)
%                 D.OutputCurve(RPMLimit+1:end,:) = [];
%                 D.OutputCurve(end+1,:) = [RPMLimit,1,D.OutputCurve(end,3),1,.9,D.OutputCurve(end,6)];
%             end
%             D.SetTorqueFactor(TF);
        end
    end
end

classdef TrackSection < handle
    %TrackSection is an object class that is used to construct a TestTrack
    %Object.  It is defined by its type (Curved/Straight), angle,
    %length, and radius.
    %   TrackSection objects are constructed by defining length for
    %   straight sections and angle and radius for curved sections.
    
    properties
        Type  % Desribes the type of track section (Curved or Straight)
        Angle % The angle of a curved section, 0 for a straight (radians)
        Length % The length of the section (feet)
        Radius % The radius of a curved section, 0 for a straight (feet)
        Number % Track Section number
        AccTable = []; % Look up table for throttling around turn
        DecTable = []; % Look up table for braking around turn
        AccCurve = []; % Acceleration curve for turn
        DecCurve = []; % Braking curve for turn
    end %properties
    
    methods
        function TS = TrackSection(Length,Radius,Number)
            % TrackSection Constructor method
            %
            % This method constructs an object of type TrackSection.  To
            % define a straight section, input the desired length and enter
            % a 0 for radius.  To define a curved section, input a desired
            % angle (defined in radians) and a desired radius. Positive
            % radiuses define CCW turns and negative radiuses define CW 
            % turns.
            %
            % INPUTS
            % Name          Type          Units   Description            
            %**************************************************************
            % Length_Angle  float         in/rad  Angle for curved 
            %                                     sections and Length for 
            %                                     straight sections.
            % Radius        float         in      Radius for curved 
            %                                     sections.
            % Number        int           N/A     Track Section Number
            %
            % OUTPUTS
            % Name          Type          Units   Description            
            %**************************************************************
            % TS            TrackSection  N/A     TrackSection Object
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
            
            if Radius  % If radius is not zero, then create curved portion
                TS.Type = 'Curved';  %Define TS Type
                TS.Length = Length; %Define TS Angle
                TS.Angle = abs(TS.Length/Radius);%Calculate path length
            else % Otherwise, create straight portion
                TS.Type = 'Straight'; %Define TS Type
                TS.Angle = 0; %Define TS ArcLength (0 for straight)
                TS.Length = Length; %Define TS Length
            end
            TS.Radius = Radius; %Define TS Radius
            TS.Number = Number;
        end %constructor
    end
end

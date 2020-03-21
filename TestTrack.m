classdef TestTrack < handle
    %TestTrack is a class for a track object that a car class object will
    %be run on for the FE SAE lap simulator.  It is constructed of
    %several track section objects
    %   TestTrack, is constructed of several straight sections defined by
    %   length and several curved sections defined by length and radius.
    %   All dimensions are in inches and radians unless otherwise
    %   specified. Currently contains a light check for track geometric 
    %   consistency. The coder is trusted to input logical geometric 
    %   information when editing or creating track code.
    
    properties
        Track % Array of track section objects
        Length % Total length of the track
        Sections % Number of track sections
        CurrentSection % Current section that the car is on
        CornerRadii % Array of corner radii the car will negotiate
        MinAutoXTime
        MinSkidpadTime
        MinAccelerationTime
        MinEndLapTime
        MinEndEnergy
        MaxEndEnergy
        MinEndEnergyFactor
        MaxEndEnergyFactor
    end % properties
    
    methods
        function TT = TestTrack(SectionArray)
            % TestTrack Constructor method
            %
            % This method constructions an object of class TestTrack which
            % is defined by an array of TrackSection Objects.  Constructor
            % will make a basic geometry consistency check.
            %
            % INPUTS
            % Name          Type          Units   Description            
            %**************************************************************
            % SectionArray  [Nx1] array   N/A     Array of TrackSection
            %                                     objects.
            %
            % OUTPUTS
            % Name          Type          Units   Description            
            %**************************************************************
            % TT            TestTrack     N/A     TestTrack Object
            %
            % VARIABLES
            % Name          Type          Units   Description            
            %**************************************************************
            % i             int           N/A     Indexing variable 
            %
            % FUNCTIONS
            % Name          Location         Description            
            %**************************************************************
            % GeometryCheck GeometryCheck.m  Checks the geometry
            %                                consistency of the given test 
            %                                track
            
            TT.Track = SectionArray; % Define TT sections
            TT.CurrentSection = 1; % Initializes section index
            TT.Sections = length(SectionArray); % Defines the number of sections
            TT.Length = 0; % Initializes TT length as 0
            for i = 1:TT.Sections % For each section in the track
                % Adds individual section lengths to each other
                TT.Length = TT.Length + TT.Track(i).Length;
            end
            
%             % Calls geometry check function
%             if ~GeometryCheck(TT)
%                 disp('Warning: Track geometry possibly inconsistent.')
%             end
            
            % Initialize Corner Radius Array
            sectionIsCorner = arrayfun(@(section)(section.Radius ~= 0), TT.Track);
            numCorners = sum(sectionIsCorner);
            TT.CornerRadii = zeros(numCorners,1);
            
            cornerIndex = 1;
            for sectionIndex = 1:TT.Sections
                if TT.Track(sectionIndex).Radius ~= 0
                    TT.CornerRadii(cornerIndex) = TT.Track(sectionIndex).Radius;
                    cornerIndex = cornerIndex + 1;
                end
            end
            
        end % constructor function
    end
end

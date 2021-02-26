% This function takes angular values and ensures that they always fall
% within the range of -180 to +180 deg.
% It simply takes a vector "Angle" containing angular values as input

function Angle = AngleLim(Angle)
% For each angle in the "Angle" vector
for i = 1:length(Angle)
    % Start loop that will end only if the angle has been reduced to the
    % -180 to +180 deg range
    while 1
        if Angle(i) <= 180
            break;
        else
            Angle(i) = Angle(i) - 360;
        end
    end
    
    % Start loop that will end only if the angle has been reduced to the
    % -180 to +180 deg range
    while 1
        if Angle(i) >= -180
            break;
        else
            Angle(i) = Angle(i) + 360;
        end
    end
end

function Angle = AngleLim(Angle)
for i = 1:length(Angle)
    while 1
        if Angle(i) <= 180
            break;
        else
            Angle(i) = Angle(i) - 360;
        end
    end
    while 1
        if Angle(i) >= -180
            break;
        else
            Angle(i) = Angle(i) + 360;
        end
    end
end

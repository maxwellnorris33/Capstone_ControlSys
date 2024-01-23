function coordinate = convert_coordinates(deg, min, sec)
%takes coordinates in degree, min, seconds format to degree decimal format
coordinate = deg + min/60 + sec/3600;
end
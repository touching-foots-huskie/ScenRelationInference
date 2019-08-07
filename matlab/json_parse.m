function val = json_parse(file_name)
% Parse Json File
fid = fopen(file_name); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

end


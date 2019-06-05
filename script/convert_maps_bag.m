% odoms_array: time, linear velocity, angular velocity
% scans_array_info: time, min angle, max angle, angle increment
% scans_array: ranges

bag = rosbag('../raw_data/dual_hector_slam.bag');
map1_Sel = select(bag,'Topic','/ABot1/map');
map2_Sel = select(bag,'Topic','/ABot2/map');

maps1 = readMessages(map1_Sel);
maps2 = readMessages(map2_Sel);

maps1_array = [];
maps2_array = [];
for ii = 1:length(maps1)
   maps1_array(ii,:) = maps1{ii,1}.Data;
end

for ii = 1:length(maps2)
   maps2_array(ii,:) = maps2{ii,1}.Data;
end


csvwrite('maps1_array.csv',maps1_array);
csvwrite('maps2_array.csv',maps2_array);

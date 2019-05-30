% odoms_array: time, linear velocity, angular velocity
% scans_array_info: time, min angle, max angle, angle increment
% scans_array: ranges

bag = rosbag('../raw_data/pcl.bag');
scan_Sel = select(bag,'Topic','/output_point_cloud');
odom_Sel = select(bag,'Topic','/odom');
scans = readMessages(scan_Sel);
odoms = readMessages(odom_Sel);

odoms_array = [];
scans_array = [];
scans_array_info = [];
for ii = 1:length(odoms)
   odoms_array(ii,1) = odoms{ii,1}.Header.Stamp.Sec + odoms{ii,1}.Header.Stamp.Nsec*10^-9;
   odoms_array(ii,2) = odoms{ii,1}.Twist.Twist.Linear.X;
   odoms_array(ii,3) = odoms{ii,1}.Twist.Twist.Angular.Z;
end

for ii = 1:length(scans)
   scans_array_info(ii) = scans{ii,1}.Header.Stamp.Sec + scans{ii,1}.Header.Stamp.Nsec*10^-9;
   for jj = 1:length(scans{ii,1}.Points)
       scans_array(ii,jj,:) = double([scans{ii,1}.Points(jj,1).X, scans{ii,1}.Points(jj,1).Y, scans{ii,1}.Points(jj,1).Z]);
   end
   
end

csvwrite('odoms_array.csv',odoms_array);
csvwrite('scans_array.csv',scans_array);
csvwrite('scans_array_info.csv',scans_array_info);
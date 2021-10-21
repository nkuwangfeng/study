clear all
close all

calibrate_imu = true;
calibrate_gyro = true;
global only_bias;

only_bias = 0;
%% ACC calibration
if calibrate_imu
    disp("calibrate accel,please wait ...")
    acc_folders = {'acc/zyx_0_0_0','acc/zyx_0_0_180','acc/zyx_0_90_0','acc/zyx_0_-90_0','acc/zyx_0_0_90','acc/zyx_0_0_-90'};
    Y = [];
    F = [];
    for i=1:size(acc_folders,2)
        acc_folder = acc_folders{1,i};
%         disp(['process ',acc_folder,' .....']);
        [acc,ref_acc] = readFolder(acc_folder);
        [y,f] = getYF(acc,ref_acc);
        Y = [Y;y];
        F = [F;f];
    end

    % 
    FTF = (F'*F);
    FTY = (F'*Y);
    disp('##### imu accel calibration result is #####')
    x = (inv(FTF)*FTY)';
    scal_matrix_acc = [x(1),x(4),x(5);x(6),x(2),x(7);x(8),x(9),x(3)]
    bias_acc = x(10:12)
end
%% GYRO calibration
if calibrate_gyro
    disp("calibrate gyro,please wait ...")
    gyro_scale_folders = {'gyro/scale/zyx_90_0_0','gyro/scale/zyx_-90_0_0',...
               'gyro/scale/zyx_0_90_0','gyro/scale/zyx_0_-90_0',...
               'gyro/scale/zyx_0_0_90','gyro/scale/zyx_0_0_-90'};

    disp('##### imu gyro calibration result is #####')
    row1 = gyro_k_scale_calib(gyro_scale_folders{5},gyro_scale_folders{6});
    row2 = gyro_k_scale_calib(gyro_scale_folders{3},gyro_scale_folders{4});
    row3 = gyro_k_scale_calib(gyro_scale_folders{1},gyro_scale_folders{2});
    
    scal_matrix_gyro = [row1;row2;row3]
    
    gyro_bias_folders = {'gyro/bias/zyx_0_0_0','gyro/bias/zyx_180_0_0',...
               'gyro/bias/zyx_0_0_0','gyro/bias/zyx_0_180_0',...
               'gyro/bias/zyx_0_0_0','gyro/bias/zyx_0_0_180'};
    bias_angle1 = gyro_bias_calib(gyro_bias_folders{1},gyro_bias_folders{2});
    bias_angle2 = gyro_bias_calib(gyro_bias_folders{1},gyro_bias_folders{4});
    bias_gyro = bias_angle1'
end

%% some function
function [acc,ref_acc] = readFolder(file_path)
  global only_bias;
  ref_acc = csvread([file_path,'/ref_accel.csv'],1,0);
  if only_bias == 1
      acc = csvread([file_path,'/accel_only_with_bias-0.csv'],1,0);
  else
      acc = csvread([file_path,'/accel-0.csv'],1,0);
  end
end

function [y,f] = getYF(acc,ref_acc)
y = [];
f = [];
a_ref = mean(ref_acc);
% disp([' acc_ref is: ',num2str(a_ref)])
for i=1:size(acc,1)
    acc_i = acc(i,:);
    a_ref_x = a_ref(1);
    a_ref_y = a_ref(2);
    a_ref_z = a_ref(3);
    f_i = [a_ref_x 0 0 a_ref_y a_ref_z 0 0 0 0 1 0 0;...
           0 a_ref_y 0 0 0 a_ref_x a_ref_z 0 0 0 1 0;...
           0 0 a_ref_z 0 0 0 0 a_ref_x a_ref_y 0 0 1];
    y = [y;[acc_i(1);acc_i(2);acc_i(3)]];
    f = [f;f_i];
end
end


function bias_angle = gyro_bias_calib(file_path_1,file_path_2)
  global only_bias;
  if only_bias == 1
      gyro_data_1 = csvread([file_path_1,'/gyro_only_with_bias-0.csv'],1,0);
      gyro_time_1 = csvread([file_path_1,'/time.csv'],1,0);
      angle_1 = integration(gyro_data_1,gyro_time_1);

      gyro_data_2 = csvread([file_path_2,'/gyro_only_with_bias-0.csv'],1,0);
      gyro_time_2 = csvread([file_path_2,'/time.csv'],1,0);
      angle_2 = integration(gyro_data_2,gyro_time_1);

      bias_angle = (angle_1 + angle_2)/2;
      time = min([max(gyro_time_1),max(gyro_time_2)]);
      bias_angle = bias_angle/time;
  else
      gyro_data_1 = csvread([file_path_1,'/gyro-0.csv'],1,0);
      gyro_time_1 = csvread([file_path_1,'/time.csv'],1,0);
      angle_1 = integration(gyro_data_1,gyro_time_1);

      gyro_data_2 = csvread([file_path_2,'/gyro-0.csv'],1,0);
      gyro_time_2 = csvread([file_path_2,'/time.csv'],1,0);
      angle_2 = integration(gyro_data_2,gyro_time_1);

      bias_angle = (angle_1 + angle_2)/2;
      time = min([max(gyro_time_1),max(gyro_time_2)]);
      bias_angle = bias_angle/time;
  end
end


function calib_data = gyro_k_scale_calib(file_path_1,file_path_2)
  global only_bias;
  if only_bias == 1
      gyro_data_1 = csvread([file_path_1,'/gyro_only_with_bias-0.csv'],1,0);
      gyro_ref_data_1 = csvread([file_path_1,'/ref_gyro.csv'],1,0);
      gyro_time_1 = csvread([file_path_1,'/time.csv'],1,0);

      angle_1 = integration(gyro_data_1,gyro_time_1);
      ref_angle_1 = integration(gyro_ref_data_1,gyro_time_1);

      gyro_data_2 = csvread([file_path_2,'/gyro_only_with_bias-0.csv'],1,0);
      gyro_ref_data_2 = csvread([file_path_2,'/ref_gyro.csv'],1,0);
      gyro_time_2 = csvread([file_path_2,'/time.csv'],1,0);
      angle_2 = integration(gyro_data_2,gyro_time_1);
      ref_angle_2 = integration(gyro_ref_data_2,gyro_time_1);

      delta_angle = (angle_1 - angle_2);
      delta_ref_angle = (ref_angle_1 - ref_angle_2);
      calib_data = delta_angle/max(delta_ref_angle);
  else
      gyro_data_1 = csvread([file_path_1,'/gyro-0.csv'],1,0);
      gyro_ref_data_1 = csvread([file_path_1,'/ref_gyro.csv'],1,0);
      gyro_time_1 = csvread([file_path_1,'/time.csv'],1,0);

      angle_1 = integration(gyro_data_1,gyro_time_1);
      ref_angle_1 = integration(gyro_ref_data_1,gyro_time_1);

      gyro_data_2 = csvread([file_path_2,'/gyro-0.csv'],1,0);
      gyro_ref_data_2 = csvread([file_path_2,'/ref_gyro.csv'],1,0);
      gyro_time_2 = csvread([file_path_2,'/time.csv'],1,0);
      angle_2 = integration(gyro_data_2,gyro_time_1);
      ref_angle_2 = integration(gyro_ref_data_2,gyro_time_1);

      delta_angle = (angle_1 - angle_2);
      delta_ref_angle = (ref_angle_1 - ref_angle_2);
      calib_data = delta_angle/max(delta_ref_angle);
  end
end

function angle = integration(gyro,time)
   angle = [0,0,0];
   for i=2:size(gyro,1)
       dt = time(i)-time(i-1);
       delta_angle = (gyro(i,:) + gyro(i-1,:))/2 * dt;
       angle = angle + delta_angle;
   end
end

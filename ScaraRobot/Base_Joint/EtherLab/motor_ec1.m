%
% Master 0, Slave 0, "LAN9252-2_motor_Master"
%
function rv = slave0()

% Slave configuration

rv.SlaveConfig.vendor = hex2dec('000004D8');
rv.SlaveConfig.product = hex2dec('00000002');
rv.SlaveConfig.description = 'LAN9252-2_motor_Master';
rv.SlaveConfig.sm = { ...
    {0, 0, {
        {hex2dec('1a00'), [
            hex2dec('3101'), hex2dec('01'),  32; ...
            hex2dec('3102'), hex2dec('01'),  32; ...
            hex2dec('3103'), hex2dec('01'),  32; ...
            hex2dec('3104'), hex2dec('01'),  32; ...
            hex2dec('3105'), hex2dec('01'),  32; ...
            hex2dec('3106'), hex2dec('01'),  32; ...
            hex2dec('3107'), hex2dec('01'),  32; ...
            hex2dec('3108'), hex2dec('01'),  32; ...
            hex2dec('3109'), hex2dec('01'),  32; ...
            hex2dec('310a'), hex2dec('01'),  32; ...
            hex2dec('310b'), hex2dec('01'),  32; ...
            hex2dec('310c'), hex2dec('01'),  32; ...
            hex2dec('310d'), hex2dec('01'),  32; ...
            hex2dec('310e'), hex2dec('01'),  32; ...
            hex2dec('310f'), hex2dec('01'),  32; ...
            hex2dec('3110'), hex2dec('01'),  1; ...
            hex2dec('3111'), hex2dec('01'),  1; ...
            ]}, ...
        }}, ...
    {1, 1, {
        {hex2dec('1600'), [
            hex2dec('3001'), hex2dec('01'),  32; ...
            hex2dec('3002'), hex2dec('01'),  16; ...
            hex2dec('3003'), hex2dec('01'),  32; ...
            hex2dec('3004'), hex2dec('01'),  32; ...
            hex2dec('3005'), hex2dec('01'),  16; ...
            hex2dec('3006'), hex2dec('01'),  32; ...
            hex2dec('3007'), hex2dec('01'),  32; ...
            hex2dec('3008'), hex2dec('01'),  16; ...
            hex2dec('3009'), hex2dec('01'),  32; ...
            ]}, ...
        }}, ...
    };

% Port configuration

rv.PortConfig.input(1).pdo = [0, 0, 0, 0];
rv.PortConfig.input(1).pdo_data_type = 3032;

rv.PortConfig.input(2).pdo = [0, 0, 1, 0];
rv.PortConfig.input(2).pdo_data_type = 3032;

rv.PortConfig.input(3).pdo = [0, 0, 2, 0];
rv.PortConfig.input(3).pdo_data_type = 3032;

rv.PortConfig.input(4).pdo = [0, 0, 3, 0];
rv.PortConfig.input(4).pdo_data_type = 3032;

rv.PortConfig.input(5).pdo = [0, 0, 4, 0];
rv.PortConfig.input(5).pdo_data_type = 3032;

rv.PortConfig.input(6).pdo = [0, 0, 5, 0];
rv.PortConfig.input(6).pdo_data_type = 3032;

rv.PortConfig.input(7).pdo = [0, 0, 6, 0];
rv.PortConfig.input(7).pdo_data_type = 3032;

rv.PortConfig.input(8).pdo = [0, 0, 7, 0];
rv.PortConfig.input(8).pdo_data_type = 3032;

rv.PortConfig.input(9).pdo = [0, 0, 8, 0];
rv.PortConfig.input(9).pdo_data_type = 3032;

rv.PortConfig.input(10).pdo = [0, 0, 9, 0];
rv.PortConfig.input(10).pdo_data_type = 3032;

rv.PortConfig.input(11).pdo = [0, 0, 10, 0];
rv.PortConfig.input(11).pdo_data_type = 3032;

rv.PortConfig.input(12).pdo = [0, 0, 11, 0];
rv.PortConfig.input(12).pdo_data_type = 3032;

rv.PortConfig.input(13).pdo = [0, 0, 12, 0];
rv.PortConfig.input(13).pdo_data_type = 3032;

rv.PortConfig.input(14).pdo = [0, 0, 13, 0];
rv.PortConfig.input(14).pdo_data_type = 3032;

rv.PortConfig.input(15).pdo = [0, 0, 14, 0];
rv.PortConfig.input(15).pdo_data_type = 3032;

rv.PortConfig.input(16).pdo = [0, 0, 15, 0];
rv.PortConfig.input(16).pdo_data_type = 1001;

rv.PortConfig.input(17).pdo = [0, 0, 16, 0];
rv.PortConfig.input(17).pdo_data_type = 1001;

rv.PortConfig.output(1).pdo = [1, 0, 0, 0];
rv.PortConfig.output(1).pdo_data_type = 3032;

rv.PortConfig.output(2).pdo = [1, 0, 1, 0];
rv.PortConfig.output(2).pdo_data_type = 1016;

rv.PortConfig.output(3).pdo = [1, 0, 2, 0];
rv.PortConfig.output(3).pdo_data_type = 3032;

rv.PortConfig.output(4).pdo = [1, 0, 3, 0];
rv.PortConfig.output(4).pdo_data_type = 3032;

rv.PortConfig.output(5).pdo = [1, 0, 4, 0];
rv.PortConfig.output(5).pdo_data_type = 1016;

rv.PortConfig.output(6).pdo = [1, 0, 5, 0];
rv.PortConfig.output(6).pdo_data_type = 3032;

rv.PortConfig.output(7).pdo = [1, 0, 6, 0];
rv.PortConfig.output(7).pdo_data_type = 3032;

rv.PortConfig.output(8).pdo = [1, 0, 7, 0];
rv.PortConfig.output(8).pdo_data_type = 1016;

rv.PortConfig.output(9).pdo = [1, 0, 8, 0];
rv.PortConfig.output(9).pdo_data_type = 3032;



end

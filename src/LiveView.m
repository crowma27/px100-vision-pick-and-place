classdef LiveView < handle
% LIVEVIEW  Janela de video do CoppeliaSim (setup + update no mesmo ficheiro).
%   live = LiveView(sim_interface);   % cria a janela com o 1o frame real
%   live.update(image);               % atualiza so os pixels

    properties (Access = private)
        h_im
    end

    methods
        function obj = LiveView(sim_interface)
            [image_init, ~] = sim_interface.get_camera_information();
            fig = figure('Name', 'Live Video CoppeliaSim', 'NumberTitle', 'off');
            ax  = axes('Parent', fig);
            obj.h_im = imshow(image_init, 'Parent', ax);
            axis(ax, 'image');   % trava a proporcao original da lente
        end

        function update(obj, image)
            if isvalid(obj.h_im)
                set(obj.h_im, 'CData', image);
                drawnow limitrate;
            end
        end
    end
end
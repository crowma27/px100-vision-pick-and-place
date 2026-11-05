classdef VisionDetector < handle
    % VISIONDETECTOR  Deteta blobs de cor por segmentacao HSV + centroid.
    %
    %   det = VisionDetector();
    %   det.thresholds.green = [0.20 0.45; 0.30 1.00; 0.20 1.00];  % afinar
    %   centroid = det.detect(image, 'green');   % [u v] em pixeis, ou [] se nada
    %
    %   centroid(1) = u = coluna = posicao horizontal no image
    %   centroid(2) = v = linha  = posicao vertical   no image

    properties
        thresholds     % struct: .blue, .green, .<whatever>, cada um [3x2]
        min_area = 60  % [pixel^2] area minima para aceitar um blob
    end

    methods
        function obj = VisionDetector(min_area)
            if nargin >= 1 && ~isempty(min_area), obj.min_area = min_area; end
            % Thresholds HSV (H,S,V em [0..1]).
            % Estes valores sao pontos de partida para o CoppeliaSim.
            % Ajusta com det.show_mask(...) ou det.show_debug(...).
            obj.thresholds.blue  = [0.50 0.70; 0.40 1.00; 0.20 1.00];
            obj.thresholds.green = [0.20 0.45; 0.35 1.00; 0.20 1.00];
        end

        % --- Detecao principal: devolve [u v] do maior blob, ou [] ---
        function centroid = detect(obj, image, className)
            if ~isfield(obj.thresholds, className)
                error('Classe de cor desconhecida: %s', className);
            end
            mask = obj.make_mask(image, className);
            centroid = obj.centroid_of_largest(mask);
        end

        % --- Versao debug: devolve mask e centroid para visualizar ---
        function [centroid, mask] = detect_debug(obj, image, className)
            mask = obj.make_mask(image, className);
            centroid = obj.centroid_of_largest(mask);
        end

        % --- Mostra imagem original + mask + centroid ---
        function show_debug(obj, image, className)
            [c, mask] = obj.detect_debug(image, className);
            figure('Name', ['Detection: ' className]);
            subplot(1,2,1); imshow(image);  title('Original');
            subplot(1,2,2); imshow(mask);   title(['Mask: ' className]);
            if ~isempty(c)
                hold on;
                plot(c(1), c(2), 'r+', 'MarkerSize', 30, 'LineWidth', 2);
                hold off;
            end
            if ~isempty(c)
                fprintf('%s centroid: (u=%.1f, v=%.1f)\n', className, c(1), c(2));
            else
                fprintf('%s: no blob detected\n', className);
            end
        end
    end

    methods (Access = private)
        function mask = make_mask(obj, image, className)
            hsv = rgb2hsv(image);
            H = hsv(:,:,1); S = hsv(:,:,2); V = hsv(:,:,3);
            th = obj.thresholds.(className);
            mask = (H >= th(1,1)) & (H <= th(1,2)) & ...
                (S >= th(2,1)) & (S <= th(2,2)) & ...
                (V >= th(3,1)) & (V <= th(3,2));
            mask = bwareaopen(mask, obj.min_area);
        end

        function c = centroid_of_largest(~, mask)
            stats = regionprops(mask, 'Centroid', 'Area');
            if isempty(stats)
                c = [];
                return
            end
            [~, idx] = max([stats.Area]);
            c = stats(idx).Centroid;   % [u v]
        end
    end
end
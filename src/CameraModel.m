classdef CameraModel < handle
    % CAMERAMODEL  Converte pixel -> ponto 3D no mundo, usando o plano da mesa.
    %
    %   cam = CameraModel(sim_interface, cfg.z_table);
    %   P   = cam.pixel_to_world([u v]);            % método geral (sempre OK)
    %   P2  = cam.pixel_to_world_vertical([u v]);   % versão "vertical" simples
    %   uv  = cam.world_to_pixel(P);                % validação: projeção inversa
    %
    %   Convenção: P = [x; y; z] em metros no referencial do mundo.

    properties
        K          % 3x3 intrinsecos
        R_wc       % 3x3 rotação world <- camera
        t_wc       % 3x1 posição da camara no mundo
        z_plane    % plano z (mesa) em metros
        resolution % [W H]
    end

    methods
        function obj = CameraModel(sim_interface, z_plane)
            obj.z_plane = z_plane;

            % --- intrinsecos a partir de FOV + resolução ---
            [res, fov] = sim_interface.get_camera_intrinsics();
            obj.resolution = res;
            f  = (res(1)/2) / tan(fov/2);
            cx = res(1)/2;
            cy = res(2)/2;
            obj.K = [f 0 cx; 0 f cy; 0 0 1];

            % --- pose da camara (extrinsecos) ---
            [obj.R_wc, obj.t_wc] = sim_interface.get_camera_pose();
        end

        % --- Método GERAL: ray-plane. Usar SEMPRE no main. ---
        function P = pixel_to_world(obj, uv)
            % Direção do raio no referencial da camara
            d_cam = obj.K \ [uv(1); uv(2); 1];
            d_cam = d_cam / norm(d_cam);

            % Rodar para o referencial do mundo
            d_world = obj.R_wc * d_cam;

            if abs(d_world(3)) < 1e-8
                error('Raio paralelo ao plano da mesa (sem interseção).');
            end

            % Interseção com z = z_plane
            s = (obj.z_plane - obj.t_wc(3)) / d_world(3);
            if s <= 0
                error('Interseção atrás da camara (s<=0). Verifica R_wc.');
            end

            P = obj.t_wc + s * d_world;
            P(3) = obj.z_plane;   % forçar por causa de erro numerico
        end

        % --- Método SIMPLES: só válido para camera vertical. ---
        % Demonstra a formula d = h·tan(theta) directamente.
        % Assume: camera em (Cx,Cy,h), eixo optico = -z, camera-x = world-x,
        % camera-y = world -y (típico: imagem "para cima" = world +y).
        function P = pixel_to_world_vertical(obj, uv)
            Cx = obj.t_wc(1);
            Cy = obj.t_wc(2);
            h  = obj.t_wc(3) - obj.z_plane;   % altura sobre a mesa
            fx = obj.K(1,1);
            fy = obj.K(2,2);
            cx = obj.K(1,3);
            cy = obj.K(2,3);

            x = Cx + h * (uv(1) - cx) / fx;
            y = Cy - h * (uv(2) - cy) / fy;   % sinal - por causa da imagem
            P = [x; y; obj.z_plane];
        end

        % --- Projeção inversa: P_world -> (u,v). Para VALIDAÇÃO. ---
        function uv = world_to_pixel(obj, P)
            P = P(:);
            % world -> camera frame
            P_cam = obj.R_wc.' * (P - obj.t_wc);
            % camera -> image
            p_img = obj.K * P_cam;
            uv = [p_img(1)/p_img(3); p_img(2)/p_img(3)];
        end

        % --- Debug: imprime a calibração na consola ---
        function show_pose(obj)
            fprintf('--- CameraModel ---\n');
            fprintf('Resolucao: %dx%d\n', obj.resolution(1), obj.resolution(2));
            fprintf('K = \n'); disp(obj.K);
            fprintf('R_wc = \n'); disp(obj.R_wc);
            fprintf('t_wc = [%.4f %.4f %.4f]''\n', obj.t_wc);
            fprintf('z_plane = %.4f\n', obj.z_plane);
        end
    end
end
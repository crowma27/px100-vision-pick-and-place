function scene = capture_scene(sim_interface, cfg, cam_model, detector, opts)
% CAPTURE_SCENE  Captura UMA imagem, deteta os dois cubos, devolve
% coordenadas em mundo. Nao mexe no braco — o chamador decide quando chamar.
%
%   scene = capture_scene(si, cfg, cam, det);
%   if ~scene.ok, fprintf('falhou\\n'); end
%   objectPos = scene.objectPos;   % [x;y;z] centro do azul
%   targetPos = scene.targetPos;   % [x;y;z] base do verde
%
%   Opcional:
%     scene = capture_scene(..., struct('show', true));

if nargin < 5 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'show'), opts.show = false; end

scene.ok        = false;
scene.objectPos = [];
scene.targetPos = [];
scene.c_obj     = [];
scene.c_tgt     = [];

% --- Uma captura, nada mais ---
[img, ~] = sim_interface.get_camera_information();

if isempty(img) || all(img(:) == 0)
    fprintf('ERRO: buffer vazio. Corre step_simulation() antes.\n');
    return
end

% --- Detecao ---
c_obj = detector.detect(img, cfg.colour_object);
c_tgt = detector.detect(img, cfg.colour_target);

if isempty(c_obj) || isempty(c_tgt)
    fprintf('ERRO detecao: blue=%d  green=%d\n', ~isempty(c_obj), ~isempty(c_tgt));
    if opts.show || isempty(c_obj) || isempty(c_tgt)
        figure('Name', 'capture_scene: FALHA');
        imshow(img);
        title(sprintf('blue=%d green=%d', ~isempty(c_obj), ~isempty(c_tgt)));
    end
    return
end

% --- Back-projection ---
P_obj = cam_model.pixel_to_world(c_obj);
P_tgt = cam_model.pixel_to_world(c_tgt);

scene.c_obj     = c_obj;
scene.c_tgt     = c_tgt;
scene.objectPos = [P_obj(1); P_obj(2); cfg.z_table + cfg.cube_half];
scene.targetPos = [P_tgt(1); P_tgt(2); cfg.z_table];
scene.ok        = true;

fprintf('\n--- CAPTURE ---\n');
fprintf('Blue  px (%6.1f,%6.1f) -> world [%.4f  %.4f  %.4f]\n', ...
    c_obj, scene.objectPos);
fprintf('Green px (%6.1f,%6.1f) -> world [%.4f  %.4f  %.4f]\n', ...
    c_tgt, scene.targetPos);
fprintf('Dist XY: %.4f m\n----------------\n\n', ...
    norm(scene.objectPos(1:2) - scene.targetPos(1:2)));

if opts.show
    figure('Name', 'capture_scene: OK');
    imshow(img); hold on;
    plot(c_obj(1), c_obj(2), 'c+', 'MarkerSize', 30, 'LineWidth', 2);
    plot(c_tgt(1), c_tgt(2), 'm+', 'MarkerSize', 30, 'LineWidth', 2);
    hold off;
end
end
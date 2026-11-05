function test_vision_pipeline()
% TEST_VISION_PIPELINE  Valida o pipeline de visão contra ground truth.
%   Corre uma vez, ajusta thresholds, e repete ate o erro ser pequeno.

cfg = config_px100();
[si, e] = simulator_interface(cfg.VISIONWORKING);
if e, return; end
cleanup = onCleanup(@() si.terminate()); 

si.start_simulation();
for k = 1:10
    si.step_simulation();
end
pause(0.3);

% --- Calibração ---
cam = CameraModel(si, cfg.z_table);
cam.show_pose();

% --- Ground truth ---
gt_obj = si.get_target_position(cfg.BLUE_CUBE);
gt_tgt = si.get_target_position(cfg.TARGET_GREEN);
fprintf('\n--- Ground truth ---\n');
fprintf('Blue  : [%.4f %.4f %.4f]\n', gt_obj);
fprintf('Green : [%.4f %.4f %.4f]\n', gt_tgt);



[img, ~] = si.get_camera_information();
figure('Name','Raw camera image'); imshow(img);
fprintf('image size: %dx%dx%d, dtype: %s, range: [%d %d]\n', ...
    size(img,1), size(img,2), size(img,3), class(img), min(img(:)), max(img(:)));


% --- Detecao ---
det = VisionDetector(cfg.vision_min_area);
[img, ~] = si.get_camera_information();

c_obj = det.detect(img, cfg.colour_object);
c_tgt = det.detect(img, cfg.colour_target);

fprintf('\n--- Deteção (pixeis) ---\n');
if ~isempty(c_obj), fprintf('Blue  : (%.1f, %.1f)\n', c_obj); else, fprintf('Blue  : MISSING\n'); end
if ~isempty(c_tgt), fprintf('Green : (%.1f, %.1f)\n', c_tgt); else, fprintf('Green : MISSING\n'); end

% --- Back-projection ---
if ~isempty(c_obj)
    P_obj = cam.pixel_to_world(c_obj);
    e_obj = norm(P_obj(1:2) - gt_obj(1:2).');
    fprintf('\n--- Back-projection (blue) ---\n');
    fprintf('Vision: [%.4f %.4f %.4f]\n', P_obj);
    fprintf('Truth : [%.4f %.4f %.4f]\n', gt_obj);
    fprintf('Erro XY: %.4f m\n', e_obj);
end
if ~isempty(c_tgt)
    P_tgt = cam.pixel_to_world(c_tgt);
    e_tgt = norm(P_tgt(1:2) - gt_tgt(1:2).');
    fprintf('\n--- Back-projection (green) ---\n');
    fprintf('Vision: [%.4f %.4f %.4f]\n', P_tgt);
    fprintf('Truth : [%.4f %.4f %.4f]\n', gt_tgt);
    fprintf('Erro XY: %.4f m\n', e_tgt);
end

% --- Round-trip: projeta ground truth de volta para pixel ---
fprintf('\n--- Round-trip (world -> pixel) ---\n');
uv_obj_gt = cam.world_to_pixel(gt_obj);
uv_tgt_gt = cam.world_to_pixel(gt_tgt);
fprintf('Blue  GT -> (%6.1f, %6.1f) | detected: ', uv_obj_gt);
if ~isempty(c_obj), fprintf('(%6.1f, %6.1f)\n', c_obj); else, fprintf('MISSING\n'); end
fprintf('Green GT -> (%6.1f, %6.1f) | detected: ', uv_tgt_gt);
if ~isempty(c_tgt), fprintf('(%6.1f, %6.1f)\n', c_tgt); else, fprintf('MISSING\n'); end

% --- Visualização opcional ---
if ~isempty(c_obj), det.show_debug(img, cfg.colour_object); end
if ~isempty(c_tgt), det.show_debug(img, cfg.colour_target); end
end
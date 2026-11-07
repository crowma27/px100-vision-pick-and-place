%   main_program.m
%   Programa principal: so orquestra. A logica esta nos outros ficheiros.
%   Autores originais: Luis Louro, Estela Bicho (Copyright 2026)
%--------------------------------------------------------------------------
function main_program()
    clear
    cfg = config_px100();

    % ---------------- Inicializacao ----------------
    [sim_interface, error_sim] = simulator_interface(cfg.VISIONWORKING);
    if error_sim == 1, return; end
    cleanup = onCleanup(@() shutdown_simulation(sim_interface)); 

    [robot_arm, error_man] = arm_interface(sim_interface, cfg.robot_name, cfg.ROSWORKING);
    if error_man == 1
        sim_interface.terminate();
        return;
    end

    sim_interface.start_simulation();
    [~, Links, qmin, qmax] = robot_arm.get_RobotCharacteristics();

    if cfg.DEBUG
        disp(Links)
        disp([qmin(:)'; qmax(:)'])
        fprintf('Alcance max do pulso = %.3f m\n', hypot(Links(3),Links(2)) + Links(4));
    end

    % Pose inicial + mao aberta
    q_cmd = cfg.q_init;
    pause(0.5);
    if robot_arm.set_joints(q_cmd) == 1, return; end
    robot_arm.open_hand();
    sim_interface.step_simulation();

    q_cmd = cfg.q_up;
    if robot_arm.set_joints(q_cmd) == 1, return; end
    for k = 1:30
        sim_interface.step_simulation();
    end
    pause(0.3);     % deixa o render do sensor atualiza
    
    % window de video
    live = [];
    if cfg.VISIONWORKING == 1
        live = LiveView(sim_interface);
        
        detector  = VisionDetector(cfg.vision_min_area);
        cam_model = CameraModel(sim_interface, cfg.z_table);
        if cfg.DEBUG, cam_model.show_pose(); end

        scene = capture_scene(sim_interface, cfg, cam_model, detector, ...
            struct('show', cfg.DEBUG));
        if ~scene.ok
            fprintf('Percecao falhou. A abortar.\n');
            return
        end
        objectPos = scene.objectPos;
        targetPos = scene.targetPos;
    else
        % Fallback: ground truth (so para debug)
        objectPos = sim_interface.get_target_position(cfg.BLUE_CUBE);
        targetPos = sim_interface.get_target_position(cfg.TARGET_GREEN);
    end

    % ---------------- Ciclo principal ----------------
    fsm  = FSM_Machine();
    stop = false;
    told = -1;
    t_loop = tic;
    
    while ~stop
        % Leituras
        q_read    = robot_arm.get_joints();
        %objectPos = sim_interface.get_target_position(cfg.BLUE_CUBE);
        %targetPos = sim_interface.get_target_position(cfg.TARGET_GREEN);

        if cfg.VISIONWORKING == 1
            [image, ~] = sim_interface.get_camera_information();
            live.update(image);
          
        end

        % maquina de estados
        [q_cmd, stop] = fsm.step(cfg, robot_arm, Links, q_read, q_cmd, objectPos, targetPos);

        % Seguranca + envio
        q_cmd = JointUtils.clamp(q_cmd, qmin, qmax);
        if robot_arm.set_joints(q_cmd) == 1, break; end

        % Avancar simulacao e sincronizar relogio
        sim_interface.step_simulation();
        elapsed = toc(t_loop);
        if elapsed < cfg.dt_loop, pause(cfg.dt_loop - elapsed); end
        t_loop = tic;

        t = sim_interface.get_simulation_time();
        if t == told, break; end   % simulacao parou
        told = t;
    end
end

% -------------------------------------------------------------------------
function shutdown_simulation(sim_interface)
% Chamada pelo onCleanup (Ctrl+C, erro ou fim normal).
    disp('stop simulation')
    sim_interface.terminate();
end

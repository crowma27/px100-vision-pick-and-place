function cfg = config_px100()
% CONFIG_PX100  Todos os parametros ajustaveis do projeto num so sitio.
% Se quiseres mudar uma tolerancia e aqui.

% --- Sistema ---
cfg.robot_name    = '/px100';
cfg.ROSWORKING    = 0;
cfg.VISIONWORKING = 1;
cfg.DEBUG         = false;      % true -> imprime Links, limites, etc.

% --- IDs dos objetos no simulador ---
cfg.TARGET_GREEN  = 1;
cfg.BLUE_CUBE     = 2;

% --- Poses [rad] ---
cfg.q_init        = [15 0 0 0] * pi/180;   % pose inicial
cfg.q_home        = [0  0 0 0];            % pose de homing final
cfg.q_up          = [0 -30 -30 0]*pi/180;

% --- Controlo / maquina de estados ---
cfg.tol_joints     = 0.05;   % [rad] erro maximo para dar a junta como "chegou"
cfg.grasp_steps    = 10;     % passos de simulacao com a garra a fechar
cfg.release_steps  = 10;     % passos de simulacao com a garra a abrir
cfg.lift_height    = 0.05;   % [m] subida apos agarrar
cfg.lift_xy_scale  = 0.9;    % fator aplicado a (x,y) no lift (ver nota no texto)
cfg.drop_clearance = 0.02;   % [m] folga acima do alvo verde

% --- Temporizacao do ciclo ---
cfg.dt_loop        = 0.05;   % [s] periodo minimo do ciclo (tempo real)

%--- Visao ---
cfg.z_table           = 0.0;         % altura da mesa no referencial do mundo [m]
cfg.cube_half         = 0.02;        % semi-aresta do cubo [m]
cfg.colour_object     = 'blue';      % nome do threshold no VisionDetector
cfg.colour_target     = 'green';
cfg.vision_min_area   = 60;          % [pixel^2] rejeita ruido
end
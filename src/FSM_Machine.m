classdef FSM_Machine < handle
% FSM_MACHINE  Maquina de estados do pick-and-place (init + step num so ficheiro).
%   Estados: APPROACH -> GRASP -> LIFT -> DROP -> RELEASE -> HOME -> DONE
%
%   fsm = FSM_Machine();                                   % estado inicial
%   [q_cmd, stop] = fsm.step(cfg, robot_arm, Links, ...
%                            q_read, q_cmd, objectPos, targetPos);
%
%   E uma "handle class": o objeto e atualizado no sitio, por isso o step
%   nao precisa de devolver o fsm.

    properties
        state     = 'APPROACH';
        counter   = 0;            % passos de simulacao (garra a fechar/abrir)
        grasp_pos = [0 0 0];      % posicao do cubo no instante do agarre
    end

    methods
        function [q_cmd, stop] = step(obj, cfg, robot_arm, Links,q_read, q_cmd, objectPos, targetPos)
            stop = false;

            switch obj.state

                case 'APPROACH'   % ir ate ao cubo com a mao aberta
                    robot_arm.open_hand();
                    [q_cmd, ~] = FSM_Machine.go_to(Links, objectPos, q_cmd);
                    if JointUtils.reached(q_read, q_cmd, cfg.tol_joints)
                        armPos = robot_arm.get_robot_position();
                        fprintf('ALVO: X=%.4f Y=%.4f | GARRA: X=%.4f Y=%.4f\n',objectPos(1:2), armPos(1:2));
                        obj.state = 'GRASP';
                    end

                case 'GRASP'      % fechar a garra durante N passos
                    robot_arm.close_hand();
                    obj.counter = obj.counter + 1;
                    if obj.counter >= cfg.grasp_steps
                        obj.counter   = 0;
                        obj.grasp_pos = objectPos;
                        obj.state     = 'LIFT';
                    end

                case 'LIFT'       % levantar o cubo
                    liftPos      = obj.grasp_pos;
                    liftPos(1:2) = cfg.lift_xy_scale * obj.grasp_pos(1:2);
                    liftPos(3)   = obj.grasp_pos(3) + cfg.lift_height;
                    [q_cmd, ok] = FSM_Machine.go_to(Links, liftPos, q_cmd);
                    if ~ok, fprintf('Lift inalcancavel, a parar.\n'); stop = true; return; end
                    if JointUtils.reached(q_read, q_cmd, cfg.tol_joints)
                        obj.state = 'DROP';
                    end

                case 'DROP'       % posicionar sobre o alvo verde
                    dropPos    = targetPos;
                    dropPos(3) = targetPos(3) + cfg.drop_clearance;
                    [q_cmd, ok] = FSM_Machine.go_to(Links, dropPos, q_cmd);
                    if ~ok, fprintf('Drop inalcancavel, a parar.\n'); stop = true; return; end
                    if JointUtils.reached(q_read, q_cmd, cfg.tol_joints)
                        obj.state = 'RELEASE';
                    end

                case 'RELEASE'    % abrir a garra durante N passos
                    robot_arm.open_hand();
                    obj.counter = obj.counter + 1;
                    if obj.counter >= cfg.release_steps
                        obj.counter = 0;
                        obj.state   = 'HOME';
                    end

                case 'HOME'       % voltar a pose de repouso
                    q_cmd = cfg.q_home;
                    if JointUtils.reached(q_read, q_cmd, cfg.tol_joints)
                        obj.state = 'DONE';
                    end

                case 'DONE'
                    stop = true;
            end
        end
    end

    methods (Static, Access = private)
        function [q_new, ok] = go_to(Links, pos, q_old)
            % IK; se o ponto for inalcancavel devolve q_old inalterado.
            [t1, t2, t3, t4, ok] = calculate_IK_px100(Links, pos);
            if ok
                q_new = [t1 t2 t3 t4];
            else
                q_new = q_old;
            end
        end
    end
end
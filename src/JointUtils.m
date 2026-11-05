classdef JointUtils
% JOINTUTILS  Funcoes auxiliares das juntas (clamp + chegada) num so ficheiro.
%   q = JointUtils.clamp(q, qmin, qmax);
%   ok = JointUtils.reached(q_read, q_cmd, tol);

    methods (Static)
        function q = clamp(q, qmin, qmax)
            % Limita as 4 primeiras juntas aos limites do robo.
            q = min(max(q(1:4), qmin(1:4)), qmax(1:4));
        end

        function tf = reached(q_read, q_cmd, tol)
            % true se as 4 juntas estao a menos de 'tol' [rad] do comando.
            tf = max(abs(q_read(1:4) - q_cmd(1:4))) < tol;
        end
    end
end
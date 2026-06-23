function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
r_w = 0.33; if isfield(VEH, 'r_w'), r_w = VEH.r_w; end
    t_f = VEH.track_f; t_r = VEH.track_r;
    ratio_f = 0.60; ratio_r = 0.40;

    % 1. 기본 제동 분배
    base_brake = zeros(4, 1);
    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        Fx_total = abs(lonCmd.Fx_total);
        base_brake(1) = (Fx_total * ratio_f / 2) * r_w;
        base_brake(2) = (Fx_total * ratio_f / 2) * r_w;
        base_brake(3) = (Fx_total * ratio_r / 2) * r_w;
        base_brake(4) = (Fx_total * ratio_r / 2) * r_w;
    end

    % 2. ESC 요 모멘트 제어
    diff_brake = zeros(4, 1);
    if isfield(latCmd, 'yawMoment') && latCmd.yawMoment ~= 0
        Mz = latCmd.yawMoment;
        dT_f = (Mz * ratio_f / t_f) * r_w;
        dT_r = (Mz * (1 - ratio_f) / t_r) * r_w;
        diff_brake(1) =  dT_f; diff_brake(2) = -dT_f; 
        diff_brake(3) =  dT_r; diff_brake(4) = -dT_r; 
    end

    total_brake = base_brake + diff_brake;
    abs_brake_reduction = zeros(4,1);

    % 3.  4륜 독립 ABS 제어
    if isfield(lonCmd, 'wheelSlip') && isfield(lonCmd, 'ax')
        if lonCmd.ax < 0
            kappa_target = 0.098; 
            K_abs = 15.0; 
            
            for i = 1:4
                slip_i = abs(lonCmd.wheelSlip(i));
                if slip_i > kappa_target
                    reduce_ratio = (slip_i - kappa_target) * K_abs;
                    abs_brake_reduction(i) = -min(reduce_ratio, 1.0) * LIM.MAX_BRAKE_TRQ;
                end
            end
        end
    end
    % 4. 최종 합산
    final_brake = total_brake + abs_brake_reduction;

    % 제어기가 명령한 마이너스 신호(운전자 브레이크 상쇄)를 0으로 자르지 않고 메인 루프로 통과
    actuatorCmd.brakeTorque = min(final_brake, LIM.MAX_BRAKE_TRQ);

    % 5. 조향 및 서스펜션 할당
    if isfield(latCmd, 'steerAngle')
        actuatorCmd.steerAngle = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);
    else
        actuatorCmd.steerAngle = 0;
    end
    actuatorCmd.dampingCoeff = verCmd;
end


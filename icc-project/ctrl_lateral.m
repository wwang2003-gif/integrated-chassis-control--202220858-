function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intErrorLat, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
% 0. 내부 상태 초기화 (Anti-windup 및 적분기 사용을 위한 설정)
    if ~isfield(ctrlState, 'intErrorLat')
        ctrlState.intErrorLat = 0;
    end
    if ~isfield(ctrlState, 'prevErrorLat')
        ctrlState.prevErrorLat = 0;
    end
    
    errorYaw = yawRateRef - yawRate;

    % 1.  Speed Scheduling 
    
    v_ref = 10.0; % 차량의 전형적인 고속 기준 속도
    f_vx = min(vx / v_ref, 2.0); 

    % 2.  Anti-windup 로직이 포함된 적분 제어
   
    ctrlState.intErrorLat = ctrlState.intErrorLat + errorYaw * dt;
    ctrlState.intErrorLat = max(min(ctrlState.intErrorLat, CTRL.LAT.intMax), -CTRL.LAT.intMax);

    % 3. AFS (보조 조향) - PID 제어기 설계
    
    Kp_afs = CTRL.LAT.Kp * 1.0 * f_vx; 
    Ki_afs = CTRL.LAT.Ki * 0.1 * f_vx; 
    
    raw_steer = Kp_afs * errorYaw + Ki_afs * ctrlState.intErrorLat;
    deltaAdd.steerAngle = max(min(raw_steer, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    % 4. [요구사항 2] ESC 자세 제어
   
    beta_th = LIM.MAX_SLIP_ANGLE * 0.90; 
    K_beta = 8000; 
    
    if abs(slipAngle) > beta_th
        
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th);
    else
        
        deltaAdd.yawMoment = 0;
    end
    
    % 상태 업데이트
    ctrlState.prevErrorLat = errorYaw;
end

   

    


function [fc_frac, batt_cmd, fc_ref_kw] = balanced_ems(soc, p_req_kw, p_aux_kw, speed_mps, fc_ref_prev_kw, params)
% BALANCED_EMS - MATLAB port of rl_hyb_train BalancedEMS policy
%
% Inputs:
%   soc             - Current battery SOC [0,1]
%   p_req_kw        - Traction power request (kW)
%   p_aux_kw        - Auxiliary load (kW)
%   speed_mps       - Current speed (unused, for parity)
%   fc_ref_prev_kw  - Previous FC reference (kW)
%   params          - Parameter struct (with ems_balanced sub-struct)
%
% Returns:
%   fc_frac   - Fuel cell fraction command [0,1]
%   batt_cmd  - Battery command [-1,1]
%   fc_ref_kw - Updated FC reference (kW)

    %#ok<NASGU> % speed currently unused but kept for signature compatibility

    cfg = params.ems_balanced;

    target_soc = cfg.target_soc;
    soc_high = target_soc + cfg.soc_high_margin;

    p_fc_max = params.fc.p_max_kw;
    fc_ramp = params.fc.ramp_kw_per_s;
    dt = params.sim.dt;

    p_batt_dis_max = params.battery.p_max_discharge_kw;
    p_batt_chg_max = params.battery.p_max_charge_kw;

    eta_trac = max(params.driveline.eta_traction, 1e-6);
    p_aux_kw = max(p_aux_kw, 0.0);

    if p_req_kw >= 0.0
        p_dem_kw = p_req_kw / eta_trac + p_aux_kw;  % DC bus demand
    else
        p_dem_kw = p_aux_kw;  % During braking, traction demand handled via regen path
    end

    %% Fuel cell target logic
    if p_req_kw >= 0.0
        if soc < target_soc
            fc_target_kw = min(p_fc_max, max(0.0, p_dem_kw + cfg.charge_boost_kw));
        elseif soc > soc_high
            fc_target_kw = min(p_fc_max, max(p_aux_kw, 0.0));
        else
            fc_target_kw = min(p_fc_max, max(p_dem_kw, 0.0));
        end
    else
        if soc > cfg.regen_clip_soc
            fc_target_kw = 0.0;
        else
            fc_target_kw = min(max(p_aux_kw, 0.0), 0.2 * p_fc_max);
        end
    end

    ramp_delta = fc_ramp * dt;
    fc_ref_kw = min(max(fc_target_kw, fc_ref_prev_kw - ramp_delta), fc_ref_prev_kw + ramp_delta);

    %% Battery command logic
    residual_kw = p_dem_kw - fc_ref_kw;
    batt_kw = residual_kw;

    if p_req_kw < 0.0
        regen_power_kw = abs(p_req_kw);
        batt_kw = -min(p_batt_chg_max, regen_power_kw);
        if soc >= cfg.soc_full
            batt_kw = 0.0;
        end
    else
        if soc > soc_high
            batt_kw = min(p_batt_dis_max, residual_kw + 0.5 * cfg.charge_boost_kw);
        elseif soc < target_soc
            if residual_kw < 0.0
                batt_kw = max(residual_kw, -p_batt_chg_max);
            else
                batt_kw = min(residual_kw, p_batt_dis_max);
            end
        else
            batt_kw = residual_kw;
        end
    end

    batt_kw = max(-p_batt_chg_max, min(batt_kw, p_batt_dis_max));

    %% Map to normalized action
    if p_fc_max > 0.0
        fc_frac = max(0.0, min(1.0, fc_ref_kw / p_fc_max));
    else
        fc_frac = 0.0;
    end

    if batt_kw >= 0.0
        if p_batt_dis_max > 0.0
            batt_cmd = min(1.0, batt_kw / p_batt_dis_max);
        else
            batt_cmd = 0.0;
        end
    else
        if p_batt_chg_max > 0.0
            batt_cmd = max(-1.0, batt_kw / p_batt_chg_max);
        else
            batt_cmd = 0.0;
        end
    end

end

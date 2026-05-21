function next = predict_next_state(state, input, params)
next = state;

v_cmd = min(max(input.v_ref, params.v_min), params.v_max);
w_cmd = min(max(input.w_ref, params.w_min), params.w_max);

next.v = v_cmd;
next.w = w_cmd;

next.x = state.x + next.v * cos(state.psi) * params.dt;
next.y = state.y + next.v * sin(state.psi) * params.dt;
next.psi = wrap_to_pi(state.psi + next.w * params.dt);
end
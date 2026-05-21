function value = clamp_value(value, min_value, max_value)
if value < min_value
    value = min_value;
elseif value > max_value
    value = max_value;
end
end
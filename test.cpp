shader_type spatial;
render_mode cull_disabled, unshaded;

uniform vec4 primary_grid_color : hint_color;
uniform vec4 secondary_grid_color : hint_color;
uniform int primary_grid_steps = 10;

varying vec2 world_pos;
varying vec2 clip_pos;

void vertex() {
	world_pos = (WORLD_MATRIX * vec4(VERTEX, 1.0)).xz;
}

void fragment() {
	ALBEDO = vec3(1.0);
	ALPHA = 0.0;

	vec2 closest_x = vec2(roundEven(world_pos.x), world_pos.y);
	vec2 closest_y = vec2(world_pos.x, roundEven(world_pos.y));

	vec4 closest_x_clip4 = PROJECTION_MATRIX * INV_CAMERA_MATRIX * vec4(closest_x.x, 0.0, closest_x.y, 1.0);
	vec2 closest_x_clip = (closest_x_clip4.xy / closest_x_clip4.w) * 0.5 + 0.5;
	closest_x_clip *= VIEWPORT_SIZE;

	vec4 closest_y_clip4 = PROJECTION_MATRIX * INV_CAMERA_MATRIX * vec4(closest_y.x, 0.0, closest_y.y, 1.0);
	vec2 closest_y_clip = (closest_y_clip4.xy / closest_y_clip4.w) * 0.5 + 0.5;
	closest_y_clip *= VIEWPORT_SIZE;

	vec2 s = SCREEN_UV * VIEWPORT_SIZE;

	//ALPHA += (closest_x - world_pos.x);
	float d1 = distance(s, closest_x_clip);
	//d1 = clamp(d1, 0.0, 1.0);
	d1 = 1.0 - d1;

	ALPHA = d1;


	//ALPHA = closest_x_clip.y;
	//ALPHA = ndc.x;
	//ALPHA = distance(ndc, closest_x_clip);

	//ALPHA = min(ALPHA, 1.0);
}

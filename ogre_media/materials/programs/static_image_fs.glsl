// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

uniform sampler2D rt_scene; // The RTT from the compositor
uniform sampler2D static_image; // The RTT from the compositor
uniform sampler2D orig_static_image; // The RTT from the compositor
uniform int is_background;
uniform vec4 background_color;
uniform int rendering_mode;
uniform vec4 color_mode_color;

const int RM_NORMAL = 0;
const int RM_COLOR = 1;
const int RM_MASK = 2;

float length(vec4 vec)
{
    return  sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z + vec.w * vec.w);
}

void main()
{
    vec2 uv = gl_TexCoord[0].xy;
    vec4 scene_color = texture2D(rt_scene, uv);
    vec4 static_color = texture2D(static_image, uv);
    vec4 orig_static_color = texture2D(orig_static_image, uv);

    vec4 final_static_color;
    if (rendering_mode == RM_NORMAL)
        final_static_color = static_color;
    else if (rendering_mode == RM_COLOR)
        final_static_color = color_mode_color;
    else if (rendering_mode == RM_MASK)
        final_static_color = vec4(1.0, 1.0, 1.0, 1.0);

    if (is_background == 0)
    {
        if (orig_static_color.a < 0.5 || orig_static_color == background_color)
            gl_FragColor = scene_color;
        else
            gl_FragColor = final_static_color;
    }
    else
    {
        if (scene_color.a < 0.5)
            gl_FragColor = final_static_color;
        else
            gl_FragColor = scene_color;
    }
}

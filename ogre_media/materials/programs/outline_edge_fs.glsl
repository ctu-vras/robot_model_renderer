// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

uniform sampler2D rt_scene; // The RTT from the compositor
uniform sampler2D blurred; // The RTT from the compositor

uniform vec4 outline_color;
uniform int use_closest_color;

void main()
{
    vec2 oUv = gl_TexCoord[0].xy;
    vec4 center = texture2D(rt_scene, oUv);

    vec4 closest_color = texture2D(blurred, oUv);
    bool touchesNonTransparent = closest_color.a > 0.0;

    if (touchesNonTransparent && center.a < 0.9)
    {
        if (use_closest_color == 1)
            gl_FragColor = closest_color;
        else
            gl_FragColor = outline_color;
    }
    else
    {
        gl_FragColor = center;
    }
}
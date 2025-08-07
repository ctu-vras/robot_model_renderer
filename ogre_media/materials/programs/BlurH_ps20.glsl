// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: OGRE authors
// SPDX-FileCopyrightText: Czech Technical University in Prague

// Adapted from https://github.com/OGRECave/ogre/blob/master/Samples/Media/materials/programs/GLSL/BlurH_ps20.glsl

//-------------------------------
// BlurH_ps20.glsl
// Horizontal Gaussian-Blur pass
//-------------------------------

uniform sampler2D RT;
uniform vec2 texel_size;
uniform float outline_width;

const int num_pos = 11;
vec2 pos[num_pos];

void main()
{
    const float range = float(num_pos / 2);
    for (int i = 0; i < num_pos; i++)
        pos[i] = vec2((float(i) - range) * (outline_width / range), 0.0);

    vec4 sum = vec4(0);
    vec2 texcoord = gl_TexCoord[0].xy;

    for (int i=0; i < num_pos; i++)
        sum += texture2D(RT, texcoord + (pos[i] * texel_size));

    gl_FragColor = sum / sum.a;
}
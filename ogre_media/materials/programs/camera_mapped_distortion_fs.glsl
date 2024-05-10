// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Mapping of undistorted to distorted uv coordinates.
uniform sampler2D distortionMap;

void main()
{
  vec2 uvDistorted = texture2D(distortionMap, gl_TexCoord[0].xy).xy;

  if (uvDistorted.x < 0.0 || uvDistorted.y < 0.0)
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    gl_FragColor = texture2D(RT, uvDistorted);
}


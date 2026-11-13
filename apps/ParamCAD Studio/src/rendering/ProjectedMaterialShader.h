#pragma once

// Shared GLSL for the projected-light GGX material. The structured-light
// capture backend and the viewport material preview compile exactly the same
// code, so a material cannot look different in the two places. Both users
// supply the same custom uniforms and the same world-space varyings
// (slWorldPosition, slWorldNormal, slSurfaceUV).
namespace shading {

// Declarations are injected as the fragment stage's PositionVC block. A 1x1
// white map plus a constant factor is how a missing material map is expressed,
// so the sampler is always bound.
inline constexpr const char* fragmentDeclarations = R"glsl(
//VTK::PositionVC::Dec
in vec3 slWorldPosition;
in vec3 slWorldNormal;
in vec2 slSurfaceUV;
vec3 slLinear(vec3 c) {
  return mix(pow((c+0.055)/1.055,vec3(2.4)),c/12.92,lessThanEqual(c,vec3(0.04045)));
}
// The capture backend writes linear values because the decoded patterns are
// compared numerically; the viewport needs display encoding or every mid tone
// lands far too dark. slDisplaySpace selects between the two.
vec3 slOutput(vec3 linear) {
  vec3 clamped=clamp(linear,0.0,1.0);
  if (slDisplaySpace < 0.5) return clamped;
  return mix(1.055*pow(clamped,vec3(1.0/2.4))-0.055, clamped*12.92,
             lessThanEqual(clamped,vec3(0.0031308)));
}
)glsl";

// Incident projected light and a GGX material, not an albedo decal. A projector
// Z-buffer in slShadowMap controls visibility.
inline constexpr const char* illumination = R"glsl(
vec3 slBase=slBaseColor*slLinear(texture(slBaseMap,slSurfaceUV).rgb);
float slMetal=clamp(slMetallic*texture(slMetalMap,slSurfaceUV).r,0.0,1.0);
float slRough=clamp(slRoughness*texture(slRoughMap,slSurfaceUV).r,0.04,1.0);
vec3 slRadiance=slAmbient*slBase;
vec3 slP=(slWorldToProjector*vec4(slWorldPosition,1.0)).xyz;
if(slP.z>slNear && slP.z<slFar) {
  vec2 slPixel=vec2(slFx*slP.x/slP.z+slCx,slFy*slP.y/slP.z+slCy);
  vec2 slUV=slPixel/vec2(slWidth,slHeight);
  float slDepth=0.5*((slFar+slNear)/(slFar-slNear)-2.0*slFar*slNear/((slFar-slNear)*slP.z))+0.5;
  vec3 slPackedDepth=texture(slShadowMap,vec2(slUV.x,1.0-slUV.y)).rgb;
  float slShadow=dot(slPackedDepth,vec3(65536.0,256.0,1.0))/65793.0;
  vec3 slN=normalize(slWorldNormal);
  vec3 slL=normalize(slProjectorCenter-slWorldPosition);
  float slNL=max(dot(slN,slL),0.0);
  // A fixed bias cannot cover the depth spread of one shadow texel at grazing
  // angles, which shows up as projector-grid shaped self-shadow acne. The bias
  // grows with the local slope while staying small on front-facing surfaces.
  float slSlope=sqrt(max(1.0-slNL*slNL,0.0))/max(slNL,0.0001);
  float slTexelDepth=(slFar*slNear)/max((slFar-slNear)*slP.z*max(slShadowFy,1.0),1.0);
  float slBias=max(0.00003,2.0*slSlope*slTexelDepth);
  bool slVisible=all(greaterThanEqual(slUV,vec2(0.0))) && all(lessThan(slUV,vec2(1.0))) && slDepth<=slShadow+slBias;
  if(slVisible && slNL>0.0) {
    vec3 slV=normalize(slCameraCenter-slWorldPosition);
    float slNV=max(dot(slN,slV),0.0);
    if(slNV>0.0) {
      vec3 slH=normalize(slL+slV);
      float slNH=max(dot(slN,slH),0.0),slVH=max(dot(slV,slH),0.0);
      float slA=slRough*slRough,slA2=slA*slA;
      float slDen=slNH*slNH*(slA2-1.0)+1.0;
      float slD=slA2/(3.14159265359*slDen*slDen);
      float slK=(slRough+1.0)*(slRough+1.0)/8.0;
      float slG=(slNL/(slNL*(1.0-slK)+slK))*(slNV/(slNV*(1.0-slK)+slK));
      vec3 slF0=mix(vec3(0.04),slBase,slMetal);
      vec3 slF=slF0+(vec3(1.0)-slF0)*pow(1.0-slVH,5.0);
      vec3 slBRDF=(vec3(1.0)-slF)*(1.0-slMetal)*slBase/3.14159265359+slD*slG*slF/max(4.0*slNL*slNV,0.000001);
      float slSignal=texture(slPatternMap,vec2(slUV.x,1.0-slUV.y)).r;
      float slDistance2=dot(slProjectorCenter-slWorldPosition,slProjectorCenter-slWorldPosition);
      slRadiance+=slBRDF*slNL*slSignal*slIntensity*slReference2/max(slDistance2,0.000001);
    }
  }
}
gl_FragData[0]=vec4(slOutput(slRadiance*slExposure),slOpacity);
)glsl";

}  // namespace shading

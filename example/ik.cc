#include "mmd-physics/ik.hpp"
#include "../src/ext.hpp"
#include <glm/gtx/transform.hpp>

using namespace mmd::physics;
using namespace glm;

int main() {
    IK *ik = IK::gradient(0.01f, 1e-7);

    vec3 t0 = vec3(0.0f, 0.0f, 0.0f);
    vec3 t1 = vec3(0.0f, 1.0f, 0.0f);
    vec3 t2 = vec3(0.0f, 2.0f, 0.0f);

    vec3 tt = vec3(0.5f, 0.5f, 0.0f);

    ik->pushBone(t0);
    ik->pushBone(t1);
    ik->pushBone(t2);
    ik->solve(tt);

    LOG << ik->getDirection(0);
    LOG << ik->getDirection(1);

    mat4 m0 = translate(t0) * ik->getRotation(0) * translate(-t0);
    mat4 m1 = translate(t1) * ik->getRotation(1) * translate(-t1);
    LOG << m0 * vec4(t1, 1.0f);
    LOG << m0 * m1 * vec4(t2, 1.0f);

    return 0;
}

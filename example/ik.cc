#include "mmd-physics/ik.hpp"
#include "../src/ext.hpp"
#include <glm/gtx/transform.hpp>

using namespace mmd::physics;
using namespace glm;

int main() {
    IK *ik = IK::gradient(0.1f, 100);

    vec3 t0 = vec3(-1.02279, 3.84852, 1.15599);
    vec3 t1 = vec3(-0.836166, -0.784939, 0.986868);
    vec3 t2 = vec3(-0.778207, -5.58021, 1.49155);

    vec3 tt = vec3(-0.378185, 1.91404, 0.0525511);

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

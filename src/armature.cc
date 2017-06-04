#include "ext.hpp"
#include "mmd-physics/armature.hpp"
#include <glm/gtc/quaternion.hpp>
#include "bone.hpp"

using namespace glm;
using namespace std;

static const float eps = 1e-5f;

namespace mmd {
    namespace physics {

        Armature::~Armature() {}

        class ArmatureImp : public Armature {
            vector<Bone> bones;
            int size;

            void doSolve(int index) {
                const auto &ikBone = bones[index];
                const auto &ik = ikBone.base->IK;
                CHECK(ik.target < size && ik.target >= 0) << "invalid ik target!";
                const auto &last = bones[ik.target];
                vec3 targetPos = ikBone.position();

                for (int i = 0; i < ik.loopCount; ++i) {
                    for (int u = 0; u < ik.links.size(); ++u) {
                        const auto &link = ik.links[u];
                        CHECK(link.bone >= 0 && link.bone < size)
                            << "invalid ik link!";
                        auto &linkBone = bones[link.bone];

                        mat4 invGlobal = inverse(linkBone.global());
                        vec3 lastDir = invGlobal * vec4(last.position(), 1.0f);
                        vec3 targetDir = invGlobal * vec4(targetPos, 1.0f);
                        lastDir = normalize(lastDir);
                        targetDir = normalize(targetDir);

                        float p = dot(lastDir, targetDir);
                        if (p > 1.0f - eps) continue;
                        p = acos(p);
                        p = glm::min(p, ik.constraint);
                        mat4 rot = rotate(p, cross(lastDir, targetDir));

                        if (link.limit) {
                            vec3 euler = eulerAngles(quat_cast(linkBone.transform * rot));
                            euler = clamp(euler, link.min, link.max);
                            linkBone.transform = mat4_cast(quat(euler));
                        } else {
                            linkBone.transform = linkBone.transform * rot;
                        }
                    }
                    if (distance(last.position(), targetPos) < eps) break;
                }
            }

        public:
            void loadModel(pmx::Model *model) {
                size = model->bones.size();
                bones.resize(size);
                for (int i = 0; i < size; ++i) {
                    auto &b = model->bones[i];
                    auto &bone = bones[i];
                    bone.base = &b;
                    if (b.parent >= 0) {
                        bone.parent = &bones[b.parent];
                        bone.transition = translate(b.position
                                        - model->bones[b.parent].position);
                    } else {
                        bone.parent = NULL;
                        bone.transition = translate(b.position);
                    }
                    bone.invBone = translate(-b.position);
                    bone.transform = mat4(1.0f);
                }
            }

            void reset() {
                for (int i = 0; i < size; ++i) {
                    bones[i].transform = mat4(1.0f);
                }
            }

            void applyLocal(int index, const mat4 &m) {
                bones[index].transform = m;
            }

            void solveIK(void) {
                for (int i = 0; i < size; ++i) {
                    if (bones[i].base->flag & pmx::Bone::IsIK) {
                        doSolve(i);
                    }
                }
            }

            mat4 skin(int index) {
                return bones[index].skin();
            }

            int getSize() {
                return size;
            }
        };

        Armature *Armature::create() {
            return new ArmatureImp();
        }

    } /* physics */
} /* mmd */

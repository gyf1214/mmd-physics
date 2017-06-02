#include "mmd-physics/ik.hpp"
#include "../ext.hpp"

using namespace std;
using namespace glm;

namespace mmd {
    namespace physics {
        class Gradient : public IK {
            int size;
            vec3 origin, last;
            vector<float> extents;
            vector<vec2> angles;
            float alpha, eps;

            vec3 getDir(const vec2 &angle) {
                return vec3(
                    cos(angle.x) * cos(angle.y),
                    sin(angle.x) * cos(angle.y),
                    sin(angle.y)
                );
            }

            vec2 getAngle(const vec3 &direction) {
                float pitch = asin(direction.z);
                return vec2(acos(direction.x / cos(pitch)), pitch);
            }

            void forward() {
                last = origin;
                for (int i = 0; i < size - 1; ++i) {
                    last += extents[i] * getDir(angles[i]);
                }
            }

            vec2 backward(int index, const vec3 &loss) {
                vec2 angle = angles[index];
                vec3 d1 = vec3(-sin(angle.x) * cos(angle.y),
                                cos(angle.x) * cos(angle.y), 0.0f);
                vec3 d2 = vec3(-cos(angle.x) * sin(angle.y),
                               -sin(angle.x) * sin(angle.y), cos(angle.y));
                return vec2(dot(d1, loss), dot(d2, loss)) * extents[index];
            }

            void solveAngle(int index, const vec3 &target) {
                vec2 diff;
                do {
                    forward();
                    vec3 loss = last - target;
                    diff = backward(index, loss);
                    angles[index] -= alpha * diff;
                } while (dot(diff, diff) >= eps);
            }

        public:
            Gradient(float alpha, float eps)
                : size(0), alpha(alpha), eps(eps) {}

            void reset() {
                size = 0;
                extents.clear();
                angles.clear();
            }

            void pushBone(const vec3 &position) {
                if (!size++) {
                    last = origin = position;
                } else {
                    vec3 d = position - last;
                    last = position;
                    extents.push_back(length(d));
                    angles.push_back(getAngle(normalize(d)));
                }
            }

            void solve(const vec3 &target) {
                forward();
                vec3 loss = last - target;
                while (dot(loss, loss) >= eps) {
                    for (int i = 0; i < size - 1; ++i) {
                        solveAngle(i, target);
                    }
                    forward();
                    loss = last - target;
                }
            }

            vec3 getDirection(int index) {
                return getDir(angles[index]);
            }

            mat4 getRotation(int index) {
                // TODO
                return mat4(1.0f);
            }
        };

        IK *IK::gradient(float alpha, float eps) {
            return new Gradient(alpha, eps);
        }

    } /* physics */
} /* mmd */

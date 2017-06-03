#ifndef __BONE
#define __BONE

#include "ext.hpp"

namespace mmd {
    namespace physics {

        struct Bone {
            pmx::Bone *base;
            Bone *parent;
            mat4 transform, transition, invBone;

            inline mat4 local() const {
                return transition * transform;
            }

            mat4 global() const {
                mat4 ret = local();
                for (Bone *p = parent; p; p = p->parent) {
                    ret = p->local() * ret;
                }
                return ret;
            }

            vec3 position() const {
                return global()[3];
            }

            inline mat4 skin() const {
                return global() * invBone;
            }
        };

    } /* physics */
} /* mmd */



#endif

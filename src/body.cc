#include "ext.hpp"
#include "mmd-physics/body.hpp"
#include <glm/gtc/quaternion.hpp>

using namespace std;
using namespace glm;

namespace mmd {
    namespace physics {
        inline btVector3 gl2bt(const vec3 &v) {
            return btVector3(v[0], v[1], v[2]);
        }

        inline vec3 bt2gl(const btVector3 &v) {
            return vec3(v[0], v[1], v[2]);
        }

        inline mat4 bt2gl(const btTransform &t) {
            const auto &T = t.getOrigin();
            const auto &M = t.getBasis();

            return mat4(
                M[0][0], M[1][0], M[2][0], 0.0f,
                M[0][1], M[1][1], M[2][1], 0.0f,
                M[0][2], M[1][2], M[2][2], 0.0f,
                T[0]   , T[1]   , T[2]   , 1.0f
            );
        }

        inline btTransform gl2bt(const mat4 &m) {
            btMatrix3x3 basis(
                m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]
            );

            return btTransform(basis, btVector3(m[3][0], m[3][1], m[3][2]));
        }

        struct Rigid {
            const pmx::Rigid *base;
            btRigidBody *bt;
            btCollisionShape *shape;
            btMotionState *motion;

            mat4 w0, s0i, ti;

            Rigid() : bt(NULL), shape(NULL), motion(NULL) {}

            void createRigid() {
                switch (base->shape) {
                    case pmx::Rigid::RigidBox:
                    shape = new btBoxShape(gl2bt(base->size / 2.0f));
                    break;
                    case pmx::Rigid::RigidSphere:
                    shape = new btSphereShape(base->size.x / 2.0f);
                    break;
                    case pmx::Rigid::RigidCapsule:
                    shape = new btCapsuleShape(base->size.x, base->size.y);
                    break;
                }

                btVector3 inert(0.0f, 0.0f, 0.0f);
                motion = new btDefaultMotionState(gl2bt(w0));
                bool kinematic = !base->mode;
                float mass = kinematic ? 0.0f : base->mass;
                if (mass > 0) shape->calculateLocalInertia(mass, inert);
                btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inert);
                bt = new btRigidBody(info);
                bt->setRestitution(base->repulsion);
                bt->setFriction(base->friction);
                bt->setDamping(base->moveDamping, base->rotateDamping);
                if (kinematic) {
                    bt->setCollisionFlags(bt->getCollisionFlags() |
                                          btCollisionObject::CF_KINEMATIC_OBJECT);
                }
                bt->setActivationState(DISABLE_DEACTIVATION);
            }

            void load(const pmx::Model *model, int index) {
                base = &model->rigids[index];

                w0 = translate(base->position) * mat4_cast(quat(base->rotation));
                if (base->bone >= 0) {
                    const auto &bone = model->bones[base->bone];
                    vec3 p = -bone.position;
                    if (bone.parent >= 0) p += model->bones[bone.parent].position;
                    ti = translate(p);
                    s0i = inverse(w0) * translate(bone.position);
                } else {
                    ti = mat4(1.0f);
                    s0i = inverse(w0);
                }

                createRigid();
            }

            void reset() {
                delete bt;
                delete motion;
                delete shape;
            }
        };

        Body::~Body() {}

        class BodyImp : public Body {
            std::vector<Rigid> rigids;
            Armature *armature;

            struct {
                btDiscreteDynamicsWorld *base;
                btCollisionDispatcher *dispatcher;
                btBroadphaseInterface *cache;
                btConstraintSolver *solver;
                btCollisionConfiguration *config;

                void setup() {
                    config = new btDefaultCollisionConfiguration();
                    dispatcher = new btCollisionDispatcher(config);
                    cache = new btDbvtBroadphase();
                    solver = new btSequentialImpulseConstraintSolver();
                    base = new btDiscreteDynamicsWorld(dispatcher, cache,
                                                       solver, config);
                }

                void reset() {
                    delete base;
                    delete solver;
                    delete cache;
                    delete dispatcher;
                    delete config;
                }
            } world;
        public:
            BodyImp() { world.setup(); }

            ~BodyImp() {
                reset();
                world.reset();
            }

            void loadModel(const pmx::Model *model) {
                int n = model->rigids.size();
                rigids.resize(n);
                for (int i = 0; i < n; ++i) {
                    rigids[i].load(model, i);
                    int group = 1 << ((int)rigids[i].base->group);
                    int mask = (int)rigids[i].base->mask;
                    world.base->addRigidBody(rigids[i].bt, group, mask);
                }
            }

            void bindArmature(Armature *armature) {
                this->armature = armature;
            }

            void reset() {
                int n = rigids.size();
                for (int i = 0; i < n; ++i) {
                    world.base->removeRigidBody(rigids[i].bt);
                    rigids[i].reset();
                }
                rigids.clear();
            }
        };

        Body *Body::create() {
            return new BodyImp();
        }

    } /* physics */
} /* mmd */
